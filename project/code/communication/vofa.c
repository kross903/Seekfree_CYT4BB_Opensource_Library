#include "communication/vofa.h"
#include "communication/comm_mux.h"
#include "common/project_headfile.h"
#include "motor_hardware/small_driver_uart_control.h"
#include <math.h>
#include <string.h>

/*
 * VOFA 无线调参模块。
 * UART 接收缓冲、组包过程、解析辅助以及回传模式状态都由本文件私有维护，
 * 不再通过 vofa.h 向外暴露内部实现细节。
 */

/*
 * VOFA 串口调参模块:
 * 负责串口收发、命令解析，以及把上位机调参命令转换成 Flight_Control_* 调用。
 */

#define VOFA_PACKET_MAX_LEN        64
#define VOFA_PACKET_QUEUE_DEPTH    4
#define VOFA_LOOP_OVERRUN_US       20000.0f
#define VOFA_DWT_LAR_UNLOCK        0xC5ACCE55UL
#define VOFA_ESC_DIAG_PERIOD_US    50000UL

static uint8 uart_get_data[VOFA_PACKET_MAX_LEN];
static uint8 fifo_get_data[VOFA_PACKET_MAX_LEN];
static uint8 get_data = 0;
static fifo_struct uart_data_fifo;

static void send_four_date(float date_1, float date_2, float date_3, float date_4,
                           float date_5, float date_6, float date_7, float date_8);
static void send_sixteen_date(float date_1, float date_2, float date_3, float date_4,
                              float date_5, float date_6, float date_7, float date_8,
                              float date_9, float date_10, float date_11, float date_12,
                              float date_13, float date_14, float date_15, float date_16);

/* 保留的回传模式见VOFA_MANUAL.md，send_back_flag只保存当前选择。 */
static int send_back_flag = 0;
static uint8 send_back_command = 'R';

static uint8 vofa_loop_profiler_ready = 0U;
static uint32 vofa_loop_start_cycle = 0U;
static volatile float vofa_loop_last_us = 0.0f;
static volatile float vofa_loop_max_us = 0.0f;
static volatile float vofa_loop_avg_us = 0.0f;
static volatile float vofa_loop_freq_hz = 0.0f;
static volatile uint32 vofa_loop_count = 0U;
static volatile uint32 vofa_loop_overrun_count = 0U;
static uint32 f2_last_cycle = 0U;
static uint32 f2_frame_sequence = 0U;
static uint8 f2_timing_initialized = 0U;
static uint32 fc_last_cycle = 0U;
static uint32 fc_frame_sequence = 0U;
static uint8 fc_timing_initialized = 0U;
static uint32 fe_last_cycle = 0U;
static uint8 fe_timing_initialized = 0U;

static void vofa_loop_profiler_init(void)
{
    if(vofa_loop_profiler_ready)
    {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
    {
        return;
    }

    DWT->LAR = VOFA_DWT_LAR_UNLOCK;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    vofa_loop_profiler_ready = 1U;
}

static uint8 vofa_esc_diag_send_due(void)
{
    uint32 now_cycle;
    uint32 period_cycles;

    vofa_loop_profiler_init();
    if(!vofa_loop_profiler_ready || SystemCoreClock == 0U)
    {
        return 1U;
    }

    now_cycle = DWT->CYCCNT;
    if(!fe_timing_initialized)
    {
        fe_last_cycle = now_cycle;
        fe_timing_initialized = 1U;
        return 1U;
    }

    period_cycles = (uint32)(((uint64)SystemCoreClock * VOFA_ESC_DIAG_PERIOD_US) / 1000000ULL);
    if(period_cycles == 0U)
    {
        period_cycles = 1U;
    }
    if((uint32)(now_cycle - fe_last_cycle) < period_cycles)
    {
        return 0U;
    }

    fe_last_cycle = now_cycle;
    return 1U;
}

void Vofa_MainLoopProfile_Begin(void)
{
    vofa_loop_profiler_init();
    if(!vofa_loop_profiler_ready)
    {
        return;
    }

    vofa_loop_start_cycle = DWT->CYCCNT;
}

void Vofa_MainLoopProfile_End(void)
{
    uint32 cycle_diff = 0U;
    float elapsed_us = 0.0f;

    if(!vofa_loop_profiler_ready || SystemCoreClock == 0U)
    {
        return;
    }

    cycle_diff = DWT->CYCCNT - vofa_loop_start_cycle;
    elapsed_us = ((float)cycle_diff * 1000000.0f) / (float)SystemCoreClock;

    vofa_loop_last_us = elapsed_us;
    if(elapsed_us > vofa_loop_max_us)
    {
        vofa_loop_max_us = elapsed_us;
    }

    if(vofa_loop_count == 0U)
    {
        vofa_loop_avg_us = elapsed_us;
    }
    else
    {
        vofa_loop_avg_us += 0.02f * (elapsed_us - vofa_loop_avg_us);
    }

    vofa_loop_count++;
    if(elapsed_us > VOFA_LOOP_OVERRUN_US)
    {
        vofa_loop_overrun_count++;
    }

    if(elapsed_us > 0.0f)
    {
        vofa_loop_freq_hz = 1000000.0f / elapsed_us;
    }
}

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 把“串口收到字节”和“主循环执行命令”拆开，避免旧命令反复执行。
 * 2. 给 VOFA 命令增加一个很轻量的包队列，尽量保持原有主流程不变。
 */
static uint8 vofa_partial_packet[VOFA_PACKET_MAX_LEN] = {0};
static uint8 vofa_partial_length = 0;
static uint8 vofa_packet_queue[VOFA_PACKET_QUEUE_DEPTH][VOFA_PACKET_MAX_LEN] = {{0}};
static uint8 vofa_packet_queue_head = 0;
static uint8 vofa_packet_queue_tail = 0;
static uint8 vofa_packet_queue_count = 0;
static uint8 vofa_packet_ready = 0;

static float build_vofa_test_channel(void)
{
    static float test_value = 0.0f;
    static float step = 2.5f;

    test_value += step;

    if(test_value >= 100.0f)
    {
        test_value = 100.0f;
        step = -2.5f;
    }
    else if(test_value <= 0.0f)
    {
        test_value = 0.0f;
        step = 2.5f;
    }

    return test_value;
}

static uint8 set_angle_pid_value(uint8 axis, uint8 item, float value)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    Flight_Control_Get_Angle_PID(axis, &kp, &ki, &kd);

    switch(item)
    {
        case '1': kp = value; break;
        case '2': ki = value; break;
        case '3': kd = value; break;
        default:  return 0U;
    }

    Flight_Control_Set_Angle_PID(axis, kp, ki, kd);
    return 1U;
}

static uint8 set_gyro_pid_value(uint8 axis, uint8 item, float value)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float out_limit = 0.0f;
    float int_limit = 0.0f;

    switch(item)
    {
        case '1':
        case '2':
        case '3':
            Flight_Control_Get_Gyro_PID(axis, &kp, &ki, &kd);
            if(item == '1')
            {
                kp = value;
            }
            else if(item == '2')
            {
                ki = value;
            }
            else
            {
                kd = value;
            }
            Flight_Control_Set_Gyro_PID(axis, kp, ki, kd);
            return 1U;

        case '4':
            Flight_Control_Get_Gyro_Limits(axis, &out_limit, &int_limit);
            Flight_Control_Set_Gyro_Limits(axis, out_limit, value);
            return 1U;

        default:
            return 0U;
    }
}

static uint8 set_height_pid_value(uint8 item, float value)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    Flight_Control_Get_Height_PID(&kp, &ki, &kd);

    switch(item)
    {
        case '1': kp = value; break;
        case '2': ki = value; break;
        case '3': kd = value; break;
        default:  return 0U;
    }

    Flight_Control_Set_Height_PID(kp, ki, kd);
    return 1U;
}

static uint8 set_height_vel_pid_value(uint8 item, float value)
{
    switch(item)
    {
        case '1': HeightVelPID.Kp = value; return 1U;
        case '2': HeightVelPID.Ki = value; return 1U;
        case '3': HeightVelPID.Kd = value; return 1U;
        default:  return 0U;
    }
}

/*
 * VOFA 调参: 光流速度闭环 PID。
 *
 * axis = 0 → 惯性系X速度PID，输出X加速度修正
 * axis = 1 → 惯性系Y速度PID，输出Y加速度修正
 *
 * item:
 *   '1' Kp, '2' Ki, '3' Kd, '4' 加速度输出限幅, '5' I项加速度限幅,
 *   '6' 目标角变化率前馈增益, '7' 前馈角速度限幅 (deg/s),
 *   '8' D支路低通截止频率 (Hz)
 *
 * 上层在 USART_PID_Adjust 已经按命令字符 'V'/'W' 区分轴，避免与角度环 R/P/Y 混淆。
 */
static uint8 set_vel_pid_value(uint8 axis, uint8 item, float value)
{
    PID_Controller *pid = NULL;
    float ff_gain = 0.0f;
    float ff_limit_dps = 0.0f;

    if(axis == 0U)
    {
        pid = &vel_pid_x;
    }
    else if(axis == 1U)
    {
        pid = &vel_pid_y;
    }
    else
    {
        return 0U;
    }

    if(!isfinite(value))
    {
        return 0U;
    }

    switch(item)
    {
        case '1': if(value < 0.0f || value > 20.0f) return 0U; pid->Kp = value; break;
        case '2': if(value < 0.0f || value > 10.0f) return 0U; pid->Ki = value; break;
        case '3': if(value < 0.0f || value > 2.0f) return 0U; pid->Kd = value; break;
        case '4': if(value < 20.0f || value > 400.0f) return 0U; pid->output_limit = value; break;
        case '5': if(value < 0.0f || value > 200.0f) return 0U; pid->integral_limit = value; break;
        case '6':
            Velocity_Loop_Get_Rate_Feedforward_Parameters(axis, &ff_gain, &ff_limit_dps);
            return Velocity_Loop_Set_Rate_Feedforward_Parameters(axis, value, ff_limit_dps);
        case '7':
            Velocity_Loop_Get_Rate_Feedforward_Parameters(axis, &ff_gain, &ff_limit_dps);
            return Velocity_Loop_Set_Rate_Feedforward_Parameters(axis, ff_gain, value);
        case '8':
            return Velocity_Loop_Set_D_Filter_Hz(axis, value);
        default:  return 0U;
    }

    return 1U;
}

static uint8 set_position_loop_value(uint8 axis, uint8 item, float value)
{
    PositionLoopTrajectoryConfig_t trajectory = {0};

    if(item == '0')
    {
        Position_Loop_Set_Enabled((value > 0.5f) ? 1U : 0U);
        return 1U;
    }

    if(axis > 1U)
    {
        return 0U;
    }

    switch(item)
    {
        case '1':
            Position_Loop_Get_Trajectory_Config(&trajectory);
            trajectory.position_kp = value;
            return Position_Loop_Set_Trajectory_Config(&trajectory);
        case '2':
        case '3':
        case '4':
        case '5':
            return 0U;
        default:  return 0U;
    }
}

static uint8 set_position_feedforward_value(uint8 item, float value)
{
    PositionLoopTrajectoryConfig_t trajectory = {0};

    /* M1/M2设置X/Y速度前馈增益，M6/M7设置X/Y加速度前馈增益。 */
    if('1' == item)
    {
        return Position_Loop_Set_Car_Velocity_Feedforward_Gain(0U, value);
    }
    if('2' == item)
    {
        return Position_Loop_Set_Car_Velocity_Feedforward_Gain(1U, value);
    }

    Position_Loop_Get_Trajectory_Config(&trajectory);
    switch(item)
    {
        case '3': trajectory.position_kp = value; break;
        case '4': trajectory.correction_speed_max_cms = value; break;
        case '5': trajectory.target_speed_max_cms = value; break;
        case '6': return Position_Loop_Set_Car_Acceleration_Feedforward_Gain(0U, value);
        case '7': return Position_Loop_Set_Car_Acceleration_Feedforward_Gain(1U, value);
        case '8': trajectory.correction_accel_max_cms2 = value; break;
        default: return 0U;
    }
    return Position_Loop_Set_Trajectory_Config(&trajectory);
}

static uint8 set_position_brake_value(uint8 item, float value)
{
    PositionLoopBrakeConfig_t brake = {0};

    Position_Loop_Get_Brake_Config(&brake);
    switch(item)
    {
        case '1': brake.effective_accel_cms2 = value; break;
        case '2': brake.effective_jerk_cms3 = value; break;
        case '3': brake.control_delay_ms = value; break;
        case '4': brake.safety_buffer_cm = value; break;
        case '5': brake.boost_margin_cm = value; break;
        case '6':
        case '7': return 0U;
        default: return 0U;
    }
    return Position_Loop_Set_Brake_Config(&brake);
}

static uint8 set_hover_angle_compensation_value(uint8 item, float value)
{
    float hover_roll = 0.0f;
    float hover_pitch = 0.0f;

    Flight_Takeoff_Get_Hover_Angle_Compensation(&hover_roll, &hover_pitch);

    switch(item)
    {
        case '1': hover_roll = value; break;
        case '2': hover_pitch = value; break;
        default:  return 0U;
    }

    Flight_Takeoff_Set_Hover_Angle_Compensation(hover_roll, hover_pitch);
    return 1U;
}


/*
 * VOFA 调参：陀螺去旋转增益。
 *   item = 'x'/'X': 设置 K_X（flow_x_rad 对 gyro_y_delta_rad 的去旋转增益）
 *   item = 'y'/'Y': 设置 K_Y（flow_y_rad 对 gyro_x_delta_rad 的去旋转增益）
 * 范围由 PMW_Flow_SetGyroCompGain() 内部夹位，这里负值才表示“不变更”，
 * 所以已 has_value 的正常调参直接传 value 即可。
 */
static uint8 set_pmw_flow_gyro_gain_value(uint8 item, float value)
{
    if(value < 0.0f)
    {
        return 0U;
    }

    switch(item)
    {
        case 'x':
        case 'X':
            PMW_Flow_SetGyroCompGain(value, -1.0f);
            return 1U;
        case 'y':
        case 'Y':
            PMW_Flow_SetGyroCompGain(-1.0f, value);
            return 1U;
        default:
            return 0U;
    }
}

static uint8 set_pmw_flow_fusion_tuning_value(uint8 item, float value)
{
    return PMW_Flow_SetFusionTuning(item, value);
}

static uint8 set_beacon_search_value(uint8 item, float value)
{
    switch(item)
    {
        case '1':
            return BeaconSearch_SetYawRateDps(value);
        case '2':
            return BeaconSearch_SetSweepHalfAngleDeg(value);
        default:
            return 0U;
    }
}

static void send_pid_data_back(uint8 command)
{
    float value_1 = 0.0f;
    float value_2 = 0.0f;
    float value_3 = 0.0f;
    float value_4 = 0.0f;
    float value_5 = 0.0f;

    switch(command)
    {
        case 'R':
            Flight_Control_Get_Angle_PID(AXIS_ROLL, &value_1, &value_2, &value_3);
            break;

        case 'r':
            Flight_Control_Get_Gyro_PID(AXIS_ROLL, &value_1, &value_2, &value_3);
            Flight_Control_Get_Gyro_Limits(AXIS_ROLL, &value_4, &value_5);
            send_four_date(value_1, value_2, value_3, value_4, value_5, 0, 0, 0);
            return;

        case 'P':
            Flight_Control_Get_Angle_PID(AXIS_PITCH, &value_1, &value_2, &value_3);
            break;

        case 'p':
            Flight_Control_Get_Gyro_PID(AXIS_PITCH, &value_1, &value_2, &value_3);
            Flight_Control_Get_Gyro_Limits(AXIS_PITCH, &value_4, &value_5);
            send_four_date(value_1, value_2, value_3, value_4, value_5, 0, 0, 0);
            return;

        case 'Y':
            Flight_Control_Get_Angle_PID(AXIS_YAW, &value_1, &value_2, &value_3);
            break;

        case 'y':
            Flight_Control_Get_Gyro_PID(AXIS_YAW, &value_1, &value_2, &value_3);
            Flight_Control_Get_Gyro_Limits(AXIS_YAW, &value_4, &value_5);
            send_four_date(value_1, value_2, value_3, value_4, value_5, 0, 0, 0);
            return;

        case 'H':
            Flight_Control_Get_Height_PID(&value_1, &value_2, &value_3);
            break;

        case 'h':
            value_1 = HeightVelPID.Kp;
            value_2 = HeightVelPID.Ki;
            value_3 = HeightVelPID.Kd;
            break;

        case 'V':
        {
            float ff_gain = 0.0f;
            float ff_limit_dps = 0.0f;

            value_1 = vel_pid_x.Kp;
            value_2 = vel_pid_x.Ki;
            value_3 = vel_pid_x.Kd;
            Velocity_Loop_Get_Rate_Feedforward_Parameters(VELOCITY_LOOP_AXIS_X,
                                                          &ff_gain,
                                                          &ff_limit_dps);
            send_four_date(value_1, value_2, value_3,
                           vel_pid_x.output_limit,
                           vel_pid_x.integral_limit,
                           ff_gain, ff_limit_dps,
                           Velocity_Loop_Get_D_Filter_Hz(VELOCITY_LOOP_AXIS_X));
            return;
        }

        case 'W':
        {
            float ff_gain = 0.0f;
            float ff_limit_dps = 0.0f;

            value_1 = vel_pid_y.Kp;
            value_2 = vel_pid_y.Ki;
            value_3 = vel_pid_y.Kd;
            Velocity_Loop_Get_Rate_Feedforward_Parameters(VELOCITY_LOOP_AXIS_Y,
                                                          &ff_gain,
                                                          &ff_limit_dps);
            send_four_date(value_1, value_2, value_3,
                           vel_pid_y.output_limit,
                           vel_pid_y.integral_limit,
                           ff_gain, ff_limit_dps,
                           Velocity_Loop_Get_D_Filter_Hz(VELOCITY_LOOP_AXIS_Y));
            return;
        }

        case 'I':
        case 'J':
        {
            PositionLoopTrajectoryConfig_t trajectory = {0};

            Position_Loop_Get_Trajectory_Config(&trajectory);
            send_four_date(trajectory.position_kp,
                           0.0f,
                           0.0f,
                           trajectory.correction_speed_max_cms,
                           0.0f,
                           (float)Position_Loop_Is_Enabled(),
                           0.0f,
                           0.0f);
            return;
        }

        case 'M':
        {
            PositionLoopTrajectoryConfig_t trajectory = {0};
            float accel_gain_x = 0.0f;
            float accel_gain_y = 0.0f;

            Position_Loop_Get_Car_Velocity_Feedforward_Gain(&value_1, &value_2);
            Position_Loop_Get_Car_Acceleration_Feedforward_Gain(&accel_gain_x,
                                                                 &accel_gain_y);
            Position_Loop_Get_Trajectory_Config(&trajectory);
            send_four_date(value_1,
                           value_2,
                           trajectory.position_kp,
                           trajectory.correction_speed_max_cms,
                           trajectory.target_speed_max_cms,
                           accel_gain_x,
                           accel_gain_y,
                           trajectory.correction_accel_max_cms2);
            return;
        }

        case 'N':
        {
            PositionLoopBrakeConfig_t brake = {0};

            Position_Loop_Get_Brake_Config(&brake);
            send_sixteen_date(brake.effective_accel_cms2,
                              brake.effective_jerk_cms3,
                              brake.control_delay_ms,
                              brake.safety_buffer_cm,
                              brake.boost_margin_cm,
                              0.0f,
                              0.0f,
                              0.0f, 0.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            return;
        }

        case 'S':
            value_1 = Flight_Control_Get_Target_Throttle();
            break;

        case 'B':
        {
            float hover_roll = 0.0f;
            float hover_pitch = 0.0f;
            float base_roll = 0.0f;
            float base_pitch = 0.0f;

            Flight_Takeoff_Get_Hover_Angle_Compensation(&hover_roll, &hover_pitch);
            Flight_Takeoff_Get_Captured_Angles(&base_roll, &base_pitch);
            send_four_date(hover_roll, hover_pitch,
                           base_roll, base_pitch,
                           target_angle_roll, target_angle_pitch,
                           current_roll, current_pitch);
            return;
        }

        case 'G':
        {
            float gx_now = 0.0f;
            float gy_now = 0.0f;
            PMW_Flow_GetGyroCompGain(&gx_now, &gy_now);
            send_four_date(gx_now, gy_now, 0, 0, 0, 0, 0, 0);
            return;
        }

        case 'K':
        {
            float settle95_s = 0.0f;
            float correction_limit_cms2 = 0.0f;
            float innovation_lp_tau_s = 0.0f;
            float bias_ki = 0.0f;

            PMW_Flow_GetFusionTuning(&settle95_s,
                                     &correction_limit_cms2,
                                     &innovation_lp_tau_s,
                                     &bias_ki);
            send_four_date(settle95_s,
                           correction_limit_cms2,
                           innovation_lp_tau_s,
                           bias_ki,
                           0, 0, 0, 0);
            return;
        }

        case 'Q':
        {
            float yaw_rate_dps = BeaconSearch_GetYawRateDps();
            float half_angle_deg = BeaconSearch_GetSweepHalfAngleDeg();

            send_four_date(yaw_rate_dps,
                           half_angle_deg,
                           -half_angle_deg,
                           half_angle_deg,
                           BeaconSearch_GetSweepAngleDeg(),
                           BEACON_SEARCH_YAW_RATE_MIN_DPS,
                           BEACON_SEARCH_YAW_RATE_MAX_DPS,
                           BEACON_SEARCH_SWEEP_HALF_ANGLE_MAX_DEG);
            return;
        }

        default:
            return;
    }

    send_four_date(value_1, value_2, value_3, 0,
                   0, 0,
                   0, 0);
}

/*
 * [2026-03-10]
 * 修改目的:
 * 1. `fifo_get_data` 只保存当前正在执行的一条完整命令包。
 * 2. 通过小队列缓存后续命令，避免收到多条完整命令时相互覆盖。
 */
static void enqueue_vofa_packet(const uint8 *packet, uint8 length)
{
    if(packet == NULL || length == 0)
    {
        return;
    }

    if(vofa_packet_queue_count >= VOFA_PACKET_QUEUE_DEPTH)
    {
        vofa_packet_queue_head++;
        if(vofa_packet_queue_head >= VOFA_PACKET_QUEUE_DEPTH)
        {
            vofa_packet_queue_head = 0;
        }
        vofa_packet_queue_count--;
    }

    memset(vofa_packet_queue[vofa_packet_queue_tail], 0, VOFA_PACKET_MAX_LEN);
    memcpy(vofa_packet_queue[vofa_packet_queue_tail], packet, length);

    vofa_packet_queue_tail++;
    if(vofa_packet_queue_tail >= VOFA_PACKET_QUEUE_DEPTH)
    {
        vofa_packet_queue_tail = 0;
    }
    vofa_packet_queue_count++;
}

static uint8 load_next_vofa_packet(void)
{
    if(vofa_packet_ready)
    {
        return 1;
    }

    if(vofa_packet_queue_count == 0)
    {
        return 0;
    }

    memset(fifo_get_data, 0, sizeof(fifo_get_data));
    memcpy(fifo_get_data, vofa_packet_queue[vofa_packet_queue_head], VOFA_PACKET_MAX_LEN);
    memset(vofa_packet_queue[vofa_packet_queue_head], 0, VOFA_PACKET_MAX_LEN);

    vofa_packet_queue_head++;
    if(vofa_packet_queue_head >= VOFA_PACKET_QUEUE_DEPTH)
    {
        vofa_packet_queue_head = 0;
    }
    vofa_packet_queue_count--;
    vofa_packet_ready = 1;
    return 1;
}

static void clear_active_vofa_packet(void)
{
    memset(fifo_get_data, 0, sizeof(fifo_get_data));
    vofa_packet_ready = 0;
}

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 只有在收到终止符 `!` 后，才把当前字节流视为一条完整命令。
 * 2. 自动丢弃超长包和换行噪声，避免缓冲区残留旧命令。
 */
static void collect_vofa_byte(uint8 byte)
{
    if(byte == '\r' || byte == '\n')
    {
        return;
    }

    if(vofa_partial_length >= (VOFA_PACKET_MAX_LEN - 1))
    {
        vofa_partial_length = 0;
    }

    vofa_partial_packet[vofa_partial_length++] = byte;

    if(byte == '!')
    {
        enqueue_vofa_packet(vofa_partial_packet, vofa_partial_length);
        memset(vofa_partial_packet, 0, sizeof(vofa_partial_packet));
        vofa_partial_length = 0;
    }
}

static void send_four_date(float date_1, float date_2, float date_3, float date_4,
                           float date_5, float date_6, float date_7, float date_8)
{
    char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    float byte[8] = {0};

    byte[0] = date_1;
    byte[1] = date_2;
    byte[2] = date_3;
    byte[3] = date_4;
    byte[4] = date_5;
    byte[5] = date_6;
    byte[6] = date_7;
    byte[7] = date_8;

    uart_write_buffer(UART_INDEX, (const uint8 *)byte, 4 * 8);
    uart_write_buffer(UART_INDEX, (const uint8 *)tail, 4);
}

static void send_sixteen_date(float date_1, float date_2, float date_3, float date_4,
                              float date_5, float date_6, float date_7, float date_8,
                              float date_9, float date_10, float date_11, float date_12,
                              float date_13, float date_14, float date_15, float date_16)
{
    char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    float byte[16] = {0};

    byte[0] = date_1;
    byte[1] = date_2;
    byte[2] = date_3;
    byte[3] = date_4;
    byte[4] = date_5;
    byte[5] = date_6;
    byte[6] = date_7;
    byte[7] = date_8;
    byte[8] = date_9;
    byte[9] = date_10;
    byte[10] = date_11;
    byte[11] = date_12;
    byte[12] = date_13;
    byte[13] = date_14;
    byte[14] = date_15;
    byte[15] = date_16;

    uart_write_buffer(UART_INDEX, (const uint8 *)byte, 4 * 16);
    uart_write_buffer(UART_INDEX, (const uint8 *)tail, 4);
}

static void send_rate_loop_tuning_data(uint8 axis)
{
    RateLoopDiag_t diag = {0};
    float target_angle = 0.0f;
    float actual_angle = 0.0f;

    if(!Rate_Loop_Get_Diag(axis, &diag))
    {
        return;
    }

    switch(axis)
    {
        case AXIS_ROLL:
            target_angle = target_angle_roll;
            actual_angle = current_roll;
            break;
        case AXIS_PITCH:
            target_angle = target_angle_pitch;
            actual_angle = current_pitch;
            break;
        case AXIS_YAW:
            target_angle = target_angle_yaw;
            actual_angle = current_yaw;
            break;
        default:
            return;
    }

    send_sixteen_date(diag.target_rate,
                      diag.actual_rate,
                      diag.error,
                      diag.p_output,
                      diag.i_output,
                      diag.d_output,
                      diag.raw_output,
                      diag.output,
                      (float)diag.saturated,
                      target_angle,
                      actual_angle,
                      base_throttle,
                      (float)motor_throttle[0],
                      (float)motor_throttle[1],
                      (float)motor_throttle[2],
                      (float)motor_throttle[3]);
}

static void send_velocity_loop_tuning_data(uint8 axis)
{
    VelocityLoopAxisDiag_t diag = {0};
    AngleLoopRateFeedforwardDiag_t ff_diag = {0};
    float target_angle_deg = 0.0f;
    float actual_angle_deg = 0.0f;
    float target_rate_dps = 0.0f;
    float actual_rate_dps = 0.0f;
    float rate_ff_dps = 0.0f;

    if(!Velocity_Loop_Get_Axis_Diag(axis, &diag))
    {
        return;
    }
    Angle_Loop_Get_Rate_Feedforward_Diag(&ff_diag);

    if(axis == VELOCITY_LOOP_AXIS_X)
    {
        target_angle_deg = target_angle_pitch;
        actual_angle_deg = current_pitch;
        target_rate_dps = target_pitch_rate;
        actual_rate_dps = current_gyro_pitch;
        rate_ff_dps = ff_diag.pitch_rate_ff_dps;
    }
    else if(axis == VELOCITY_LOOP_AXIS_Y)
    {
        target_angle_deg = target_angle_roll;
        actual_angle_deg = current_roll;
        target_rate_dps = target_roll_rate;
        actual_rate_dps = current_gyro_roll;
        rate_ff_dps = ff_diag.roll_rate_ff_dps;
    }
    else
    {
        return;
    }

    send_sixteen_date(diag.target_velocity_cms,
                      diag.actual_velocity_cms,
                      diag.error_velocity_cms,
                      diag.p_output_cms2,
                      diag.i_output_cms2,
                      diag.d_output_cms2,
                      diag.acceleration_ff_cms2,
                      diag.output_cms2,
                      (float)diag.saturated,
                      target_angle_deg,
                      actual_angle_deg,
                      target_rate_dps,
                      actual_rate_dps,
                      rate_ff_dps,
                      (float)diag.active,
                      base_throttle);
}

static void send_height_loop_tuning_data(void)
{
    HeightManagerDiag_t height_diag = {0};
    float target_height_cm = 0.0f;
    float actual_height_cm = 0.0f;
    float height_error_cm = 0.0f;
    float height_p_output = 0.0f;
    float height_i_output = 0.0f;
    float height_d_output = 0.0f;
    float height_total_output = 0.0f;
    float speed_target_cms = 0.0f;
    float actual_speed_cms = 0.0f;
    float speed_error_cms = 0.0f;
    float speed_p_output = 0.0f;
    float speed_i_output = 0.0f;
    float speed_d_output = 0.0f;
    float speed_total_output = 0.0f;
    float speed_final_output = 0.0f;
    uint8 anti_windup_status = 0U;

    HeightManager_Get_Diag(&height_diag);
    target_height_cm = HeightManager_Get_Active_Target();
    actual_height_cm = height_diag.tof_height_filt_cm;
    height_error_cm = target_height_cm - actual_height_cm;
    height_p_output = HeightPID.Kp * height_error_cm;
    height_i_output = HeightPID.Ki * HeightPID.integral;
    height_d_output = HeightPID.Kd * HeightPID.derivative;
    height_total_output = height_p_output + height_i_output + height_d_output;
    speed_target_cms = height_diag.target_velocity_cms;
    actual_speed_cms = height_diag.velocity_feedback_view_cms;
    speed_error_cms = speed_target_cms - actual_speed_cms;
    speed_p_output = HeightVelPID.Kp * speed_error_cms;
    speed_i_output = HeightVelPID.Ki * HeightVelPID.integral;
    speed_d_output = HeightVelPID.Kd * HeightVelPID.derivative;
    speed_total_output = speed_p_output + speed_i_output + speed_d_output;
    speed_final_output = HeightSpeedPID_output;
    anti_windup_status = (uint8)((height_diag.cascade_enabled ? 0x01U : 0U) |
                                 (height_diag.height_integrator_held ? 0x02U : 0U) |
                                 (height_diag.velocity_integrator_held ? 0x04U : 0U) |
                                 ((height_diag.height_pid_limit_flags & MOTOR_MIXER_LIMIT_NEGATIVE) ? 0x08U : 0U) |
                                 ((height_diag.height_pid_limit_flags & MOTOR_MIXER_LIMIT_POSITIVE) ? 0x10U : 0U) |
                                 ((height_diag.velocity_pid_limit_flags & MOTOR_MIXER_LIMIT_NEGATIVE) ? 0x20U : 0U) |
                                 ((height_diag.velocity_pid_limit_flags & MOTOR_MIXER_LIMIT_POSITIVE) ? 0x40U : 0U));

    send_sixteen_date(target_height_cm,
                      actual_height_cm,
                      height_p_output,
                      height_i_output,
                      height_d_output,
                      height_total_output,
                      speed_target_cms,
                      actual_speed_cms,
                      speed_p_output,
                      speed_i_output,
                      speed_d_output,
                      speed_total_output,
                      height_error_cm,
                      speed_error_cms,
                      speed_final_output,
                      (float)anti_windup_status);
}

void Init_uart(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, VOFA_PACKET_MAX_LEN);
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);
    uart_rx_interrupt(UART_INDEX, 1);

    if(CommMux_IsVofaMode())
    {
        uart_write_string(UART_INDEX, "UART Text.");
        uart_write_byte(UART_INDEX, '\r');
        uart_write_byte(UART_INDEX, '\n');
    }
}

void Vofa_ClearRxState(void)
{
    uint8 discard = 0U;

    uart_rx_interrupt(UART_INDEX, 0);

    fifo_clear(&uart_data_fifo);
    memset(uart_get_data, 0, sizeof(uart_get_data));
    memset(fifo_get_data, 0, sizeof(fifo_get_data));

    memset(vofa_partial_packet, 0, sizeof(vofa_partial_packet));
    vofa_partial_length = 0U;

    memset(vofa_packet_queue, 0, sizeof(vofa_packet_queue));
    vofa_packet_queue_head = 0U;
    vofa_packet_queue_tail = 0U;
    vofa_packet_queue_count = 0U;
    vofa_packet_ready = 0U;

    while(uart_query_byte(UART_INDEX, &discard))
    {
    }

    uart_rx_interrupt(UART_INDEX, 1);
}

void Vofa_GetDiag(VofaDiag_t *diag)
{
    if(diag == NULL)
    {
        return;
    }

    diag->comm_mode = (uint8)CommMux_GetMode();
    diag->is_vofa_mode = CommMux_IsVofaMode();
    diag->send_back_flag = send_back_flag;
    diag->send_back_command = send_back_command;
    diag->packet_ready = vofa_packet_ready;
    diag->packet_queue_count = vofa_packet_queue_count;
    diag->loop_last_us = vofa_loop_last_us;
    diag->loop_max_us = vofa_loop_max_us;
    diag->loop_avg_us = vofa_loop_avg_us;
    diag->loop_freq_hz = vofa_loop_freq_hz;
    diag->loop_count = vofa_loop_count;
    diag->loop_overrun_count = vofa_loop_overrun_count;
}

void Send_test(void)
{
    uint8 rx_buffer[VOFA_PACKET_MAX_LEN] = {0};
    uint32 fifo_data_count = 0;
    uint32 read_count = 0;
    uint32 index = 0;

    if(!CommMux_IsVofaMode())
    {
        return;
    }

    fifo_data_count = fifo_used(&uart_data_fifo);
    if(fifo_data_count != 0)
    {
        read_count = fifo_data_count;
        fifo_read_buffer(&uart_data_fifo, rx_buffer, &read_count, FIFO_READ_AND_CLEAN);

        for(index = 0; index < read_count; index++)
        {
            collect_vofa_byte(rx_buffer[index]);
        }
    }

}

void uart_rx_interrupt_handler(void)
{
    if(!CommMux_IsVofaMode())
    {
        uint8 discard = 0U;
        while(uart_query_byte(UART_INDEX, &discard))
        {
        }
        return;
    }

//  get_data = uart_read_byte(UART_INDEX);
    uart_query_byte(UART_INDEX, &get_data);
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);
}

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 解析函数显式返回成功/失败，避免坏包被默认解析成 0。
 * 2. 只有在 `=` 和 `!` 都完整存在时，才允许把数据交给控制层使用。
 */
static uint8 Get_Data(float *value)
{
    uint8 data_start_num = 0;
    uint8 data_end_num = 0;
    uint8 minus_flag = 0;
    uint8 has_point = 0;
    uint8 valid = 0;
    uint8 cursor = 0;
    uint8 has_digit = 0;
    float integer_part = 0.0f;
    float fractional_part = 0.0f;
    float fractional_scale = 0.1f;

    if(value == NULL)
    {
        return 0;
    }

    // 先扫描当前命令包，找到数字区间。
    // 例如 "T2=0.21!" 中，真正的数据部分是 '=' 后、'!' 前的 "0.21"。
    for(cursor = 0; cursor < VOFA_PACKET_MAX_LEN; cursor++)
    {
        if(fifo_get_data[cursor] == '=')
        {
            data_start_num = cursor + 1;
            valid |= 0x01;
        }
        if(fifo_get_data[cursor] == '!')
        {
            data_end_num = cursor;
            valid |= 0x02;
            break;
        }
    }

    if(valid != 0x03 || data_start_num >= data_end_num)
    {
        return 0;
    }

    cursor = data_start_num;
    // 允许数字带一个前导负号，例如 "-0.25"。
    if(fifo_get_data[cursor] == '-')
    {
        minus_flag = 1;
        cursor++;
    }

    /*
     * 下面开始逐字符手动解析数字。
     *
     * 1. 在遇到小数点之前，按“整数部分”处理：
     *    integer_part = integer_part * 10 + 当前数字
     *    例如 "12"：
     *      读到 '1' -> 0 * 10 + 1 = 1
     *      读到 '2' -> 1 * 10 + 2 = 12
     *
     * 2. 遇到小数点之后，按“小数部分”处理：
     *    第一位小数权重是 0.1
     *    第二位小数权重是 0.01
     *    第三位小数权重是 0.001
     *
     * 这里的实现方法是：
     *    - 看到 '.' 时，把 fractional_scale 设为 1.0f
     *    - 每次加入 digit * (fractional_scale * 0.1f)
     *    - 然后再把 fractional_scale 乘 0.1f
     *
     * 例如 "0.21"：
     *    读到小数点后，fractional_scale = 1.0
     *    读到 '2'：2 * (1.0 * 0.1) = 0.2
     *    再把 fractional_scale 变成 0.1
     *    读到 '1'：1 * (0.1 * 0.1) = 0.01
     *    所以小数部分最终是 0.2 + 0.01 = 0.21
     */
    for(; cursor < data_end_num; cursor++)
    {
        uint8 current_char = fifo_get_data[cursor];

        if(current_char == '.')
        {
            // 如果已经进过一次小数部分，又再次遇到 '.'，说明数据格式错误。
            if(has_point)
            {
                return 0;
            }
            // 从这里开始，后续字符按小数部分处理。
            has_point = 1;
            continue;
        }

        if(current_char < '0' || current_char > '9')
        {
            return 0;
        }

        has_digit = 1;
        if(!has_point)
        {
            // 还在整数部分，旧值先左移一位十进制，再加上当前数字。
            integer_part = integer_part * 10.0f + (float)(current_char - '0');
        }
        else
        {
            // 已进入小数部分，当前位按 0.1 / 0.01 / 0.001 ... 这样的权重累加。
            fractional_part += (float)(current_char - '0') * fractional_scale;
            fractional_scale *= 0.1f;
        }
    }

    if(!has_digit)
    {
        return 0;
    }

    *value = integer_part + fractional_part;
    if(minus_flag)
    {
        *value = -(*value);
    }

    return 1;
}

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 每次主循环最多消费一条完整命令，消费后立刻清空当前活动包。
 * 2. 保持原有外部调用方式不变，尽量不影响当前可运行主流程。
 */
void USART_PID_Adjust(void)
{
    float data_get = 0.0f;
    uint8 has_value = 0;
    uint8 command = 0;
    uint8 item = 0;
    uint8 reply_command = 0U;
    uint8 write_success = 0U;

    if(!CommMux_IsVofaMode())
    {
        return;
    }

    if(!load_next_vofa_packet())
    {
        return;
    }

    command = fifo_get_data[0];
    item = fifo_get_data[1];
    has_value = Get_Data(&data_get);

    switch(command)
    {
        case 'R':
            if(has_value)
            {
                write_success = set_angle_pid_value(AXIS_ROLL, item, data_get);
            }
            send_back_command = 'R';
            reply_command = 'R';
            break;

        case 'r':
            if(has_value)
            {
                write_success = set_gyro_pid_value(AXIS_ROLL, item, data_get);
            }
            send_back_command = 'r';
            reply_command = 'r';
            break;

        case 'P':
            if(has_value)
            {
                write_success = set_angle_pid_value(AXIS_PITCH, item, data_get);
            }
            send_back_command = 'P';
            reply_command = 'P';
            break;

        case 'p':
            if(has_value)
            {
                write_success = set_gyro_pid_value(AXIS_PITCH, item, data_get);
            }
            send_back_command = 'p';
            reply_command = 'p';
            break;

        case 'Y':
            if(has_value)
            {
                write_success = set_angle_pid_value(AXIS_YAW, item, data_get);
            }
            send_back_command = 'Y';
            reply_command = 'Y';
            break;

        case 'y':
            if(has_value)
            {
                write_success = set_gyro_pid_value(AXIS_YAW, item, data_get);
            }
            send_back_command = 'y';
            reply_command = 'y';
            break;

        case 'S':
            if(has_value && item == '0')
            {
                Flight_Control_Set_Throttle(data_get);
                write_success = 1U;
            }
            send_back_command = 'S';
            reply_command = 'S';
            break;

        case 'B':
            if(has_value)
            {
                write_success = set_hover_angle_compensation_value(item, data_get);
            }
            send_back_command = 'B';
            reply_command = 'B';
            break;

        case 'V':
            if(has_value)
            {
                write_success = set_vel_pid_value(0U, item, data_get);
            }
            send_back_command = 'V';
            reply_command = 'V';
            break;

        case 'W':
            if(has_value)
            {
                write_success = set_vel_pid_value(1U, item, data_get);
            }
            send_back_command = 'W';
            reply_command = 'W';
            break;

        case 'I':
            if(has_value)
            {
                write_success = set_position_loop_value(0U, item, data_get);
            }
            send_back_command = 'I';
            reply_command = 'I';
            break;

        case 'J':
            if(has_value)
            {
                write_success = set_position_loop_value(1U, item, data_get);
            }
            send_back_command = 'J';
            reply_command = 'J';
            break;

        case 'M':
            if(has_value)
            {
                write_success = set_position_feedforward_value(item, data_get);
            }
            send_back_command = 'M';
            reply_command = 'M';
            break;

        case 'N':
            if(has_value)
            {
                write_success = set_position_brake_value(item, data_get);
            }
            send_back_command = 'N';
            reply_command = 'N';
            break;

        case 'F':
            switch(item)
            {
                case '2':
                    send_back_flag = 2;
                    f2_last_cycle = 0U;
                    f2_frame_sequence = 0U;
                    f2_timing_initialized = 0U;
                    break;
                case '3': send_back_flag = 3; break;
                case '4': send_back_flag = 4; break;
                case '5': send_back_flag = 5; break;
                case '6': send_back_flag = 6; break;
                case '7': send_back_flag = 7; break;
                case '8': send_back_flag = 8; break;
                case '9': send_back_flag = 9; break;
                case '1': send_back_flag = 19; break;
                case 'A':
                case 'a': send_back_flag = 10; break;
                case 'B':
                case 'b': send_back_flag = 12; break;
                case 'C':
                case 'c':
                    send_back_flag = 35;
                    fc_last_cycle = 0U;
                    fc_frame_sequence = 0U;
                    fc_timing_initialized = 0U;
                    break;
                case 'D':
                case 'd': send_back_flag = 13; break;
                case 'E':
                case 'e':
                    send_back_flag = 14;
                    fe_last_cycle = 0U;
                    fe_timing_initialized = 0U;
                    break;
                case 'G':
                case 'g': send_back_flag = 16; break;
                case 'H':
                case 'h':
                    /* FH1!：小车运动；FH2!：位置纠偏速度整形旁路。 */
                    if(fifo_get_data[2] == '1') send_back_flag = 46;
                    else if(fifo_get_data[2] == '2') send_back_flag = 47;
                    break;
                case 'I':
                case 'i': send_back_flag = 18; break;
                case 'J':
                case 'j': send_back_flag = 45; break;
                case 'K':
                case 'k': send_back_flag = 36; break;
                case 'L':
                case 'l': send_back_flag = 31; break;
                case 'M':
                case 'm': send_back_flag = 23; break;
                case 'N':
                case 'n': send_back_flag = 37; break;
                case 'O':
                case 'o': send_back_flag = 22; break;
                case 'P':
                case 'p': send_back_flag = 38; break;
                case 'Q':
                case 'q': send_back_flag = 39; break;
                case 'R':
                case 'r': send_back_flag = 40; break;
                case 'S':
                case 's': send_back_flag = 41; break;
                case 'T':
                case 't': send_back_flag = 42; break;
                case 'U':
                case 'u':
                    /* FU1!/FU!查看惯性系X轴，FU2!查看惯性系Y轴。 */
                    send_back_flag = (fifo_get_data[2] == '2') ? 44 : 29;
                    break;
                case 'V':
                case 'v': send_back_flag = 20; break;
                case 'W':
                case 'w': send_back_flag = 21; break;
                case 'X':
                case 'x': send_back_flag = 43; break;
                case 'Y':
                case 'y': send_back_flag = 33; break;
                case 'Z':
                case 'z': send_back_flag = 34; break;
                default: break;
            }

            break;

        case '0':
            if(item == '0')
            {
                Flight_Control_Disarm();
            }
            break;

        case 'H':
            if(has_value)
            {
                write_success = set_height_pid_value(item, data_get);
            }
            send_back_command = 'H';
            reply_command = 'H';
            break;

        case 'h':
            if(has_value)
            {
                write_success = set_height_vel_pid_value(item, data_get);
            }
            send_back_command = 'h';
            reply_command = 'h';
            break;

        case 'G':
            if(has_value)
            {
                write_success = set_pmw_flow_gyro_gain_value(item, data_get);
            }
            send_back_command = 'G';
            reply_command = 'G';
            break;

        case 'K':
            if(has_value)
            {
                write_success = set_pmw_flow_fusion_tuning_value(item, data_get);
            }
            send_back_command = 'K';
            reply_command = 'K';
            break;

        case 'Q':
            if(has_value)
            {
                write_success = set_beacon_search_value(item, data_get);
            }
            send_back_command = 'Q';
            reply_command = 'Q';
            break;

        default:
            break;
    }

    clear_active_vofa_packet();
    if(reply_command != 0U)
    {
        send_pid_data_back(reply_command);
    }
    if(write_success)
    {
        BoardBuzzer_RequestDoubleBeep();
    }
}

void send_date_back(void)
{
    if(!CommMux_IsVofaMode())
    {
        return;
    }

    switch(send_back_flag)
    {
        case 2:
        {
            PositionLoopDiag_t pos_diag = {0};
            VelocityLoopHorizontalDiag_t horizontal_diag = {0};
            float target_vx_cms = 0.0f;
            float target_vy_cms = 0.0f;
            float frame_interval_ms = 0.0f;
            uint32 now_cycle = 0U;

            Position_Loop_Get_Diag(&pos_diag);
            Velocity_Loop_Get_Horizontal_Diag(&horizontal_diag);
            Velocity_Loop_Get_Target_Velocity(&target_vx_cms, &target_vy_cms);

            vofa_loop_profiler_init();
            if(vofa_loop_profiler_ready && SystemCoreClock > 0U)
            {
                now_cycle = DWT->CYCCNT;
                if(f2_timing_initialized)
                {
                    frame_interval_ms = ((float)(now_cycle - f2_last_cycle) * 1000.0f) /
                                        (float)SystemCoreClock;
                }
                else
                {
                    f2_timing_initialized = 1U;
                }
                f2_last_cycle = now_cycle;
            }

            send_sixteen_date(target_angle_roll,
                              current_roll,
                              target_angle_pitch,
                              current_pitch,
                              target_roll_rate,
                              current_gyro_roll,
                              target_pitch_rate,
                              current_gyro_pitch,
                              target_vx_cms,
                              horizontal_diag.actual_vx_earth_cms,
                              target_vy_cms,
                              horizontal_diag.actual_vy_earth_cms,
                              pos_diag.error_x_cm,
                              pos_diag.error_y_cm,
                              (float)f2_frame_sequence,
                              frame_interval_ms);
            f2_frame_sequence++;
            if(f2_frame_sequence >= 1000000U)
            {
                f2_frame_sequence = 0U;
            }
            break;
        }

        case 3:
            send_four_date(build_vofa_test_channel(), 0, 0, 0, 0, 0, 0, 0);
            break;

        case 4:
        {
            ICM_Health_t imu_health = {0};

            ICM_GetHealth(&imu_health);
            send_sixteen_date((float)icm42688_gyro_x_r,
                              (float)icm42688_gyro_y_r,
                              (float)icm42688_gyro_z_r,
                              icm42688_gyro_x,
                              icm42688_gyro_y,
                              icm42688_gyro_z,
                              (float)icm42688_acc_x_r,
                              (float)icm42688_acc_y_r,
                              (float)icm42688_acc_z_r,
                              icm42688_acc_x,
                              icm42688_acc_y,
                              icm42688_acc_z,
                              acc,
                              acc_norm_filtered,
                              (float)imu_health.fault_mask,
                              (float)imu_health.attitude_valid);
            break;
        }

        case 5:
            send_rate_loop_tuning_data(AXIS_ROLL);
            break;

        case 6:
            send_sixteen_date(target_angle_roll,
                              current_roll,
                              target_roll_rate,
                              current_gyro_roll,
                              roll_output,
                              target_angle_pitch,
                              current_pitch,
                              target_pitch_rate,
                              current_gyro_pitch,
                              pitch_output,
                              target_angle_yaw,
                              current_yaw,
                              target_yaw_rate,
                              current_gyro_yaw,
                              yaw_output,
                              yaw_angle_error_deg);
            break;

        case 7:
            send_rate_loop_tuning_data(AXIS_PITCH);
            break;

        case 8:
        {
            /*
             * F8 - 双轴速度闭环诊断。
             * I0~I7 为 X -> Pitch，I8~I15 为 Y -> Roll；
             * 每轴依次发送目标、反馈、误差、P/I/D、限幅前总和及最终修正角。
             */
            VelocityLoopAxisDiag_t x_diag = {0};
            VelocityLoopAxisDiag_t y_diag = {0};

            (void)Velocity_Loop_Get_Axis_Diag(VELOCITY_LOOP_AXIS_X, &x_diag);
            (void)Velocity_Loop_Get_Axis_Diag(VELOCITY_LOOP_AXIS_Y, &y_diag);

            send_sixteen_date(x_diag.target_velocity_cms,
                              x_diag.actual_velocity_cms,
                              x_diag.error_velocity_cms,
                              x_diag.p_output_cms2,
                              x_diag.i_output_cms2,
                              x_diag.d_output_cms2,
                              x_diag.acceleration_ff_cms2,
                              x_diag.output_cms2,
                              y_diag.target_velocity_cms,
                              y_diag.actual_velocity_cms,
                              y_diag.error_velocity_cms,
                              y_diag.p_output_cms2,
                              y_diag.i_output_cms2,
                              y_diag.d_output_cms2,
                              y_diag.acceleration_ff_cms2,
                              y_diag.output_cms2);
            break;
        }

        case 9:
            send_rate_loop_tuning_data(AXIS_YAW);
            break;

        case 10:
        {
            float ax_level_mg = 0.0f;
            float ay_level_mg = 0.0f;
            float az_level_mg = 0.0f;

            Body_To_RollPitch_Level_Frame(Roll_acc_F, Pitch_acc_F, Yaw_acc_F,
                                          &ax_level_mg, &ay_level_mg, &az_level_mg);
            (void)az_level_mg;

            send_four_date(ax_level_mg,
                           ay_level_mg,
                           acc_level_x_bias_mg,
                           acc_level_y_bias_mg,
                           ax_level_mg - acc_level_x_bias_mg,
                           ay_level_mg - acc_level_y_bias_mg,
                           X_V_F,
                           Y_V_F);
            break;
        }

        case 12:
            send_height_loop_tuning_data();
            break;

        case 14:
        {
            SmallDriverEscStatus esc_status = {0};
            uint8 active_fault_mask = 0U;
            uint8 i;

            if(!vofa_esc_diag_send_due())
            {
                break;
            }

            small_driver_get_esc_status(&esc_status);
            for(i = 0U; i < 4U; i++)
            {
                if(esc_status.active_fault[i] != (uint8)SMALL_DRIVER_ESC_FAULT_NONE)
                {
                    active_fault_mask |= (uint8)(1U << i);
                }
            }

            send_sixteen_date((float)esc_status.valid,
                              (float)esc_status.stale,
                              (float)esc_status.age_ms,
                              (float)esc_status.protocol_version,
                              (float)esc_status.status_frame_count,
                              (float)esc_status.checksum_error_count,
                              (float)esc_status.unknown_frame_count,
                              (float)esc_status.fault_event_count,
                              (float)esc_status.last_event_sequence,
                              (float)esc_status.last_event_code,
                              (float)esc_status.last_event_motor_mask,
                              (float)active_fault_mask,
                              (float)esc_status.active_fault[0],
                              (float)esc_status.active_fault[1],
                              (float)esc_status.active_fault[2],
                              (float)esc_status.active_fault[3]);
            break;
        }

        case 16:
        {
            VelocityLoopRateFeedforwardDiag_t velocity_ff_diag = {0};
            AngleLoopRateFeedforwardDiag_t angle_ff_diag = {0};

            Velocity_Loop_Get_Rate_Feedforward_Diag(&velocity_ff_diag);
            Angle_Loop_Get_Rate_Feedforward_Diag(&angle_ff_diag);
            send_sixteen_date(target_angle_roll,
                              current_roll,
                              velocity_ff_diag.roll_target_rate_raw_dps,
                              velocity_ff_diag.roll_target_rate_filt_dps,
                              angle_ff_diag.roll_pid_rate_dps,
                              angle_ff_diag.roll_rate_ff_dps,
                              angle_ff_diag.roll_rate_final_dps,
                              current_gyro_roll,
                              target_angle_pitch,
                              current_pitch,
                              velocity_ff_diag.pitch_target_rate_raw_dps,
                              velocity_ff_diag.pitch_target_rate_filt_dps,
                              angle_ff_diag.pitch_pid_rate_dps,
                              angle_ff_diag.pitch_rate_ff_dps,
                              angle_ff_diag.pitch_rate_final_dps,
                              current_gyro_pitch);
            break;
        }

        case 35:
        {
            ICM_Health_t imu_health = {0};
            float raw_acc_norm_count;
            float frame_interval_ms = 0.0f;
            uint32 now_cycle = 0U;

            ICM_GetHealth(&imu_health);
            raw_acc_norm_count = sqrtf((float)icm42688_acc_x_r * (float)icm42688_acc_x_r +
                                       (float)icm42688_acc_y_r * (float)icm42688_acc_y_r +
                                       (float)icm42688_acc_z_r * (float)icm42688_acc_z_r);

            vofa_loop_profiler_init();
            if(vofa_loop_profiler_ready && SystemCoreClock > 0U)
            {
                now_cycle = DWT->CYCCNT;
                if(fc_timing_initialized)
                {
                    frame_interval_ms = ((float)(now_cycle - fc_last_cycle) * 1000.0f) /
                                        (float)SystemCoreClock;
                }
                else
                {
                    fc_timing_initialized = 1U;
                }
                fc_last_cycle = now_cycle;
            }

            send_sixteen_date((float)icm42688_acc_x_r,
                              (float)icm42688_acc_y_r,
                              (float)icm42688_acc_z_r,
                              raw_acc_norm_count,
                              icm42688_acc_x,
                              icm42688_acc_y,
                              icm42688_acc_z,
                              acc_norm_filtered,
                              icm42688_gyro_x,
                              icm42688_gyro_y,
                              icm42688_gyro_z,
                              (float)ICM_IsStill(),
                              (float)imu_health.fault_mask,
                              (float)imu_health.sample_sequence,
                              (float)fc_frame_sequence,
                              frame_interval_ms);
            fc_frame_sequence++;
            if(fc_frame_sequence >= 1000000U)
            {
                fc_frame_sequence = 0U;
            }
            break;
        }

        case 13:
        {
            HeightManagerDiag_t height_diag = {0};

            HeightManager_Get_Diag(&height_diag);

            send_sixteen_date(height_diag.tof_raw_cm,
                              height_diag.tof_comp_cm,
                              height_diag.tof_median_cm,
                              height_diag.tof_height_filt_cm,
                              height_diag.tof_predicted_cm,
                              height_diag.tof_innovation_cm,
                              height_diag.tof_gate_cm,
                              height_diag.tof_candidate_cm,
                              (float)height_diag.tof_candidate_count,
                              (float)height_diag.tof_reject_count,
                              (float)height_diag.tof_state,
                              (float)height_diag.tof_raw_age_ms,
                              (float)height_diag.tof_accepted_age_ms,
                              height_diag.tof_velocity_cms,
                              (float)height_diag.tof_sample_sequence,
                              (float)height_diag.tof_valid);
            break;
        }

        case 18:
        {
            PositionLoopDiag_t pos_diag = {0};

            Position_Loop_Get_Diag(&pos_diag);

            send_sixteen_date(pos_diag.target_x_cm,
                              pos_diag.fb_x_cm,
                              pos_diag.error_x_cm,
                              pos_diag.correction_vx_cms,
                              pos_diag.car_ff_filtered_vx_cms,
                              pos_diag.vx_cmd_cms,
                              pos_diag.target_y_cm,
                              pos_diag.fb_y_cm,
                              pos_diag.error_y_cm,
                              pos_diag.correction_vy_cms,
                              pos_diag.car_ff_filtered_vy_cms,
                              pos_diag.vy_cmd_cms,
                              (float)pos_diag.enabled,
                              (float)pos_diag.active,
                              (float)pos_diag.ir_track_valid,
                              (float)pos_diag.velocity_ready);
            break;
        }

        case 19:
        {
            HeightManagerDiag_t height_diag = {0};
            float compensated_disagreement_cm = 0.0f;

            HeightManager_Get_Diag(&height_diag);
            if(height_diag.tof1_valid && height_diag.tof2_valid)
            {
                compensated_disagreement_cm = fabsf(height_diag.tof1_comp_cm -
                                                     height_diag.tof2_comp_cm);
            }

            /* F1 command, internal mode 19: dual ToF input diagnostics. */
            send_sixteen_date(height_diag.tof1_raw_cm,
                              height_diag.tof2_raw_cm,
                              height_diag.tof1_comp_cm,
                              height_diag.tof2_comp_cm,
                              height_diag.tof_selected_comp_cm,
                              height_diag.tof_height_filt_cm,
                              (float)height_diag.tof_source,
                              (float)height_diag.tof1_valid,
                              (float)height_diag.tof2_valid,
                              (float)height_diag.tof1_age_ms,
                              (float)height_diag.tof2_age_ms,
                              compensated_disagreement_cm,
                              (float)height_diag.tof_state,
                              (float)height_diag.tof_valid,
                              (float)height_diag.tof1_invalid_count,
                              (float)height_diag.tof2_invalid_count);
            break;
        }

        case 20:
            send_velocity_loop_tuning_data(VELOCITY_LOOP_AXIS_X);
            break;

        case 21:
            send_velocity_loop_tuning_data(VELOCITY_LOOP_AXIS_Y);
            break;

        case 22:
        {
            PMWFlowState_t flow_state = {0};

            PMW_Flow_GetState(&flow_state);
            send_sixteen_date(flow_state.flow_x_rad,
                              flow_state.flow_comp_x_rad - flow_state.flow_x_rad,
                              flow_state.flow_comp_x_rad,
                              flow_state.flow_y_rad,
                              flow_state.flow_comp_y_rad - flow_state.flow_y_rad,
                              flow_state.flow_comp_y_rad,
                              flow_state.velocity_x_raw_cms,
                              flow_state.velocity_y_raw_cms,
                              flow_state.velocity_x_slew_cms,
                              flow_state.velocity_y_slew_cms,
                              flow_state.velocity_x_lpf_cms,
                              flow_state.velocity_y_lpf_cms,
                              flow_state.velocity_x_output_slew_cms,
                              flow_state.velocity_y_output_slew_cms,
                              flow_state.velocity_x_cms,
                              flow_state.velocity_y_cms);
            break;
        }

        case 23:
        {
            CoreLinkCarMotionDiag_t car_motion = {0};
            PositionLoopDiag_t pos_diag = {0};
            uint8 status_flags;

            core_link_user_get_car_motion_diag(&car_motion);
            Position_Loop_Get_Diag(&pos_diag);
            status_flags = (uint8)(pos_diag.limit_flags & 0x0FU);
            if(!pos_diag.car_ff_valid)
            {
                status_flags |= 0x10U;
            }
            if(car_motion.protocol_error_count > 0U)
            {
                status_flags |= 0x20U;
            }
            if(car_motion.search_requested)
            {
                status_flags |= 0x40U;
            }
            if(pos_diag.limit_flags & POSITION_LOOP_STATUS_BRAKE_LIMITED)
            {
                status_flags |= 0x08U;
            }
            if(pos_diag.limit_flags & POSITION_LOOP_STATUS_TARGET_READY)
            {
                status_flags |= 0x80U;
            }

            send_sixteen_date(car_motion.world_vx_cms,
                              car_motion.world_vy_cms,
                              car_motion.car_yaw_deg,
                              current_yaw,
                              pos_diag.car_ff_raw_vx_cms,
                              pos_diag.car_ff_raw_vy_cms,
                              pos_diag.car_ff_applied_ax_cms2,
                              pos_diag.car_ff_applied_ay_cms2,
                              pos_diag.correction_vx_cms,
                              pos_diag.correction_vy_cms,
                              pos_diag.vx_cmd_cms,
                              pos_diag.vy_cmd_cms,
                              (float)pos_diag.car_ff_valid,
                              (float)car_motion.age_ms,
                              (float)status_flags,
                              (float)car_motion.sequence);
            break;
        }

        case 37:
        {
            PositionLoopDiag_t pos_diag = {0};
            VelocityLoopHorizontalDiag_t vel_diag = {0};

            Position_Loop_Get_Diag(&pos_diag);
            Velocity_Loop_Get_Horizontal_Diag(&vel_diag);
            send_sixteen_date(pos_diag.error_x_cm,
                              pos_diag.error_y_cm,
                              pos_diag.correction_vx_cms,
                              pos_diag.correction_vy_cms,
                              pos_diag.car_ff_filtered_vx_cms,
                              pos_diag.car_ff_filtered_vy_cms,
                              pos_diag.vx_cmd_cms,
                              pos_diag.vy_cmd_cms,
                              pos_diag.target_accel_x_cms2,
                              pos_diag.target_accel_y_cms2,
                              vel_diag.acceleration_correction_x_earth_cms2,
                              vel_diag.acceleration_correction_y_earth_cms2,
                              vel_diag.acceleration_target_x_earth_cms2,
                              vel_diag.acceleration_target_y_earth_cms2,
                              vel_diag.actual_vx_earth_cms,
                              vel_diag.actual_vy_earth_cms);
            break;
        }

        case 38:
        {
            PositionLoopDiag_t pos_diag = {0};

            Position_Loop_Get_Diag(&pos_diag);
            send_sixteen_date(pos_diag.position_error_distance_cm,
                              pos_diag.relative_closing_speed_cms,
                              pos_diag.proportional_speed_cms,
                              pos_diag.sqrt_speed_cms,
                              pos_diag.car_ff_filtered_vx_cms,
                              pos_diag.car_ff_filtered_vy_cms,
                              pos_diag.correction_vx_cms,
                              pos_diag.correction_vy_cms,
                              pos_diag.desired_vx_cms,
                              pos_diag.desired_vy_cms,
                              pos_diag.vx_cmd_cms,
                              pos_diag.vy_cmd_cms,
                              pos_diag.target_accel_x_cms2,
                              pos_diag.target_accel_y_cms2,
                              pos_diag.trajectory_dt_s * 1000.0f,
                              (float)pos_diag.trajectory_reset_reason);
            break;
        }

        case 39:
        {
            PositionLoopDiag_t pos_diag = {0};

            Position_Loop_Get_Diag(&pos_diag);
            send_sixteen_date(pos_diag.error_x_cm,
                              pos_diag.error_y_cm,
                              pos_diag.relative_velocity_x_cms,
                              pos_diag.relative_velocity_y_cms,
                              pos_diag.relative_speed_cms,
                              pos_diag.relative_closing_speed_cms,
                              pos_diag.filtered_closing_speed_cms,
                              pos_diag.brake_allowed_speed_cms,
                              pos_diag.correction_vx_cms,
                              pos_diag.correction_vy_cms,
                              pos_diag.position_error_distance_cm,
                              pos_diag.brake_delay_distance_cm,
                              pos_diag.desired_vx_cms,
                              pos_diag.desired_vy_cms,
                              (float)pos_diag.brake_diag_flags,
                              (pos_diag.limit_flags & POSITION_LOOP_STATUS_BRAKE_LIMITED) ?
                              1.0f : 0.0f);
            break;
        }

        case 40:
        {
            PositionLoopDiag_t pos_diag = {0};
            Position_Loop_Get_Diag(&pos_diag);
            send_sixteen_date(pos_diag.relative_closing_speed_cms,
                              pos_diag.filtered_closing_speed_cms,
                              pos_diag.brake_allowed_speed_cms,
                              (pos_diag.limit_flags & POSITION_LOOP_STATUS_BRAKE_LIMITED) ?
                              1.0f : 0.0f,
                              pos_diag.car_ff_raw_ax_cms2,
                              pos_diag.car_ff_raw_ay_cms2,
                              pos_diag.car_ff_applied_ax_cms2,
                              pos_diag.car_ff_applied_ay_cms2,
                              pos_diag.brake_delay_distance_cm,
                              pos_diag.position_error_distance_cm,
                              pos_diag.target_accel_x_cms2,
                              pos_diag.target_accel_y_cms2,
                              pos_diag.relative_speed_cms,
                              pos_diag.position_correction_speed_cms,
                              (float)pos_diag.brake_diag_flags,
                              (pos_diag.limit_flags & POSITION_LOOP_STATUS_BRAKE_LIMITED) ?
                              1.0f : 0.0f);
            break;
        }

        case 41:
        {
            static uint32 last_sent_build_count = 0U;
            DualCameraFusionSnapshot_t snapshot = {0};
            DualCameraFusionDiag_t fusion_diag = {0};

            DualCameraFusion_GetSnapshot(&snapshot);
            DualCameraFusion_GetDiag(&fusion_diag);
            if(fusion_diag.build_count == last_sent_build_count)
            {
                break;
            }
            last_sent_build_count = fusion_diag.build_count;
            /* FS!: front/rear camera-board vision diagnostics. */
            send_sixteen_date((float)fusion_diag.front_healthy,
                              (float)fusion_diag.rear_healthy,
                              (float)fusion_diag.front_beacon_count,
                              (float)fusion_diag.rear_beacon_count,
                              (float)fusion_diag.front_age_ms,
                              (float)fusion_diag.rear_age_ms,
                              (float)snapshot.front_frame_seq,
                              (float)snapshot.rear_frame_seq,
                              snapshot.front_car_x_cm,
                              snapshot.front_car_y_cm,
                              snapshot.rear_car_x_cm,
                              snapshot.rear_car_y_cm,
                              (float)snapshot.front_car_valid,
                              (float)snapshot.rear_car_valid,
                              (float)fusion_diag.pair_valid,
                              (float)fusion_diag.output_count);
            break;
        }

        case 42:
        {
            MotorMixerLimitDiag_t mixer_diag = {0};

            Motor_Mixer_Get_Limit_Diag(&mixer_diag);
            send_sixteen_date(mixer_diag.raw_motor[0],
                              mixer_diag.raw_motor[1],
                              mixer_diag.raw_motor[2],
                              mixer_diag.raw_motor[3],
                              mixer_diag.allocated_motor[0],
                              mixer_diag.allocated_motor[1],
                              mixer_diag.allocated_motor[2],
                              mixer_diag.allocated_motor[3],
                              mixer_diag.roll_pitch_scale,
                              mixer_diag.yaw_scale,
                              mixer_diag.requested_throttle,
                              mixer_diag.applied_throttle,
                              (float)mixer_diag.roll_limit_flags,
                              (float)mixer_diag.pitch_limit_flags,
                              (float)mixer_diag.yaw_limit_flags,
                              (float)mixer_diag.throttle_limit_flags);
            break;
        }

        case 43:
        {
            AngleLoopSqrtDiag_t sqrt_diag = {0};

            Angle_Loop_Get_Sqrt_Diag(&sqrt_diag);
            send_sixteen_date(sqrt_diag.roll_angle_error_deg,
                              sqrt_diag.roll_linear_p_rate_dps,
                              sqrt_diag.roll_sqrt_rate_dps,
                              sqrt_diag.roll_selected_rate_dps,
                              sqrt_diag.roll_rate_ff_dps,
                              sqrt_diag.roll_rate_total_raw_dps,
                              sqrt_diag.roll_rate_final_dps,
                              current_gyro_roll,
                              sqrt_diag.pitch_angle_error_deg,
                              sqrt_diag.pitch_linear_p_rate_dps,
                              sqrt_diag.pitch_sqrt_rate_dps,
                              sqrt_diag.pitch_selected_rate_dps,
                              sqrt_diag.pitch_rate_ff_dps,
                              sqrt_diag.pitch_rate_total_raw_dps,
                              sqrt_diag.pitch_rate_final_dps,
                              current_gyro_pitch);
            break;
        }

        case 31:
        {
            PMWFlowState_t flow_state = {0};

            PMW_Flow_GetState(&flow_state);
            send_sixteen_date(flow_state.gyro_rate_x_rad_s,
                              flow_state.gyro_rate_y_rad_s,
                              flow_state.gyro_rate_z_rad_s,
                              flow_state.gyro_flow_x_delta_rad,
                              flow_state.gyro_flow_y_delta_rad,
                              flow_state.gyro_flow_z_delta_rad,
                              flow_state.velocity_x_camera_cms,
                              flow_state.velocity_y_camera_cms,
                              flow_state.mount_comp_velocity_x_cms,
                              flow_state.mount_comp_velocity_y_cms,
                              flow_state.velocity_x_center_body_cms,
                              flow_state.velocity_y_center_body_cms,
                              flow_state.velocity_x_raw_cms,
                              flow_state.velocity_y_raw_cms,
                              flow_state.flow_rx_dt_ms,
                              (float)flow_state.flow_sync_valid);
            break;
        }

        case 29:
        {
            PMWFlowState_t pmw_flow_state = {0};

            PMW_Flow_GetState(&pmw_flow_state);

            send_sixteen_date(pmw_flow_state.velocity_x_fused_cms,
                              pmw_flow_state.velocity_x_flow_earth_cms,
                              pmw_flow_state.fusion_acceleration_x_cms2,
                              pmw_flow_state.fusion_imu_delta_x_cms,
                              pmw_flow_state.fusion_prediction_x_cms,
                              pmw_flow_state.fusion_residual_x_cms,
                              pmw_flow_state.fusion_correction_x_cms,
                              pmw_flow_state.fusion_output_x_cms,
                              pmw_flow_state.fusion_alpha_x,
                              pmw_flow_state.fusion_accel_bias_x_cms2,
                              (float)pmw_flow_state.flow_frame_seq,
                              (float)pmw_flow_state.flow_last_accepted_frame_seq,
                              pmw_flow_state.flow_observation_age_s * 1000.0f,
                              (float)pmw_flow_state.flow_last_received_accepted,
                              (float)pmw_flow_state.fusion_mode,
                              (float)pmw_flow_state.fused_velocity_valid);
            break;
        }

        case 44:
        {
            PMWFlowState_t pmw_flow_state = {0};

            PMW_Flow_GetState(&pmw_flow_state);

            send_sixteen_date(pmw_flow_state.velocity_y_fused_cms,
                              pmw_flow_state.velocity_y_flow_earth_cms,
                              pmw_flow_state.fusion_acceleration_y_cms2,
                              pmw_flow_state.fusion_imu_delta_y_cms,
                              pmw_flow_state.fusion_prediction_y_cms,
                              pmw_flow_state.fusion_residual_y_cms,
                              pmw_flow_state.fusion_correction_y_cms,
                              pmw_flow_state.fusion_output_y_cms,
                              pmw_flow_state.fusion_alpha_y,
                              pmw_flow_state.fusion_accel_bias_y_cms2,
                              (float)pmw_flow_state.flow_frame_seq,
                              (float)pmw_flow_state.flow_last_accepted_frame_seq,
                              pmw_flow_state.flow_observation_age_s * 1000.0f,
                              (float)pmw_flow_state.flow_last_received_accepted,
                              (float)pmw_flow_state.fusion_mode,
                              (float)pmw_flow_state.fused_velocity_valid);
            break;
        }

        case 45:
        {
            PositionLoopDiag_t pos_diag = {0};

            Position_Loop_Get_Diag(&pos_diag);
            send_sixteen_date(pos_diag.vision_control_x_cm,
                              pos_diag.vision_control_y_cm,
                              pos_diag.vision_estimate_x_cm,
                              pos_diag.vision_estimate_y_cm,
                              pos_diag.vision_observation_x_cm,
                              pos_diag.vision_observation_y_cm,
                              pos_diag.vision_innovation_cm,
                              pos_diag.vision_gate_cm,
                              pos_diag.vision_realign_error_cm,
                              (float)pos_diag.vision_hold_age_ms,
                              (float)pos_diag.vision_observation_state,
                              (float)pos_diag.vision_reacquire_count,
                              (float)pos_diag.vision_observation_accepted,
                              (float)pos_diag.vision_observation_source,
                              (float)pos_diag.vision_estimator_initialized,
                              (float)pos_diag.active);
            break;
        }

        case 46:
        {
            CoreLinkCarMotionDiag_t car_motion = {0};
            PositionLoopDiag_t pos_diag = {0};

            core_link_user_get_car_motion_diag(&car_motion);
            Position_Loop_Get_Diag(&pos_diag);
            send_sixteen_date(car_motion.raw_world_vx_cms,
                              car_motion.raw_world_vy_cms,
                              car_motion.raw_world_ax_cms2,
                              car_motion.raw_world_ay_cms2,
                              car_motion.world_vx_cms,
                              car_motion.world_vy_cms,
                              car_motion.world_ax_cms2,
                              car_motion.world_ay_cms2,
                              pos_diag.car_ff_raw_vx_cms,
                              pos_diag.car_ff_raw_vy_cms,
                              pos_diag.car_ff_applied_ax_cms2,
                              pos_diag.car_ff_applied_ay_cms2,
                              (float)car_motion.age_ms,
                              (float)car_motion.arrival_interval_ms,
                              (float)car_motion.sequence,
                              (float)car_motion.link_state);
            break;
        }

        case 47:
        {
            PositionLoopDiag_t pos_diag = {0};
            float velocity_error;
            float shaped_acceleration;
            float shaped_jerk;

            Position_Loop_Get_Diag(&pos_diag);
            velocity_error = sqrtf(
                pos_diag.correction_velocity_error_x_cms *
                pos_diag.correction_velocity_error_x_cms +
                pos_diag.correction_velocity_error_y_cms *
                pos_diag.correction_velocity_error_y_cms);
            shaped_acceleration = sqrtf(
                pos_diag.shaped_correction_ax_cms2 *
                pos_diag.shaped_correction_ax_cms2 +
                pos_diag.shaped_correction_ay_cms2 *
                pos_diag.shaped_correction_ay_cms2);
            shaped_jerk = sqrtf(
                pos_diag.correction_shaper_jerk_x_cms3 *
                pos_diag.correction_shaper_jerk_x_cms3 +
                pos_diag.correction_shaper_jerk_y_cms3 *
                pos_diag.correction_shaper_jerk_y_cms3);

            send_sixteen_date(pos_diag.correction_vx_cms,
                              pos_diag.shaped_correction_vx_cms,
                              pos_diag.correction_velocity_error_x_cms,
                              pos_diag.correction_accel_target_x_cms2,
                              pos_diag.shaped_correction_ax_cms2,
                              pos_diag.correction_vy_cms,
                              pos_diag.shaped_correction_vy_cms,
                              pos_diag.correction_velocity_error_y_cms,
                              pos_diag.correction_accel_target_y_cms2,
                              pos_diag.shaped_correction_ay_cms2,
                              pos_diag.correction_shaper_jerk_x_cms3,
                              pos_diag.correction_shaper_jerk_y_cms3,
                              velocity_error,
                              shaped_acceleration,
                              shaped_jerk,
                              (float)pos_diag.correction_shaper_phase);
            break;
        }

        case 36:
        {
            PMWFlowState_t flow_state = {0};

            PMW_Flow_GetState(&flow_state);
            send_sixteen_date(flow_state.fusion_prediction_x_cms,
                              flow_state.velocity_x_flow_earth_cms,
                              flow_state.fusion_residual_x_cms,
                              flow_state.fusion_innovation_evidence_x,
                              flow_state.fusion_alpha_x,
                              flow_state.fusion_bias_ki_x,
                              flow_state.fusion_accel_bias_x_cms2,
                              flow_state.fusion_output_x_cms,
                              flow_state.fusion_prediction_y_cms,
                              flow_state.velocity_y_flow_earth_cms,
                              flow_state.fusion_residual_y_cms,
                              flow_state.fusion_innovation_evidence_y,
                              flow_state.fusion_alpha_y,
                              flow_state.fusion_bias_ki_y,
                              flow_state.fusion_accel_bias_y_cms2,
                              flow_state.fusion_output_y_cms);
            break;
        }

        case 33:
        {
            FlightRealtimeDiag realtime_diag = {0};

            FlightRealtime_GetDiag(&realtime_diag);
            send_sixteen_date(realtime_diag.control_last_us,
                              realtime_diag.control_max_us,
                              realtime_diag.control_interval_min_us,
                              realtime_diag.control_interval_max_us,
                              (float)realtime_diag.control_execution_overrun_count,
                              (float)realtime_diag.control_delayed_entry_count,
                              (float)realtime_diag.control_count,
                              realtime_diag.motor_last_us,
                              realtime_diag.motor_max_us,
                              (float)realtime_diag.motor_execution_overrun_count,
                              (float)realtime_diag.motor_count,
                              vofa_loop_last_us,
                              vofa_loop_max_us,
                              vofa_loop_avg_us,
                              (float)vofa_loop_overrun_count,
                              (float)vofa_loop_count);
            break;
        }

        case 34:
        {
            BatteryStatus battery_status = {0};
            MotorHardwareVoltageCompDiag motor_diag = {0};
            SmallDriverUartDiag uart_diag = {0};

            BatteryMonitor_GetStatus(&battery_status);
            MotorHardware_GetVoltageCompDiag(&motor_diag);
            small_driver_get_diag(&uart_diag);
            send_sixteen_date(battery_status.local_raw_voltage_v,
                              battery_status.local_filtered_voltage_v,
                              battery_status.remote_voltage_v,
                              battery_status.compensation_gain,
                              (float)battery_status.local_valid,
                              (float)battery_status.remote_valid,
                              (float)battery_status.low_warning,
                              motor_diag.compensated_output[0],
                              motor_diag.compensated_output[1],
                              motor_diag.compensated_output[2],
                              motor_diag.compensated_output[3],
                              (float)motor_diag.saturation_mask,
                              (float)motor_diag.invalid_output,
                              (float)uart_diag.overwritten_frame_count,
                              (float)uart_diag.max_pending_age_us,
                              (float)uart_diag.fifo_error_count);
            break;
        }

        default:
            break;
    }
}
