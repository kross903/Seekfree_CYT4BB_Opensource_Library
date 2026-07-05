#include "communication/vofa.h"
#include "communication/comm_mux.h"
#include "common/project_headfile.h"
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

/*
 * 回传模式开关 (FX 命令切换):
 *   F0  光流原始/补偿数据 (case 0)
 *   F1  PID 参数回读 (case 1, 配合 R/r/P/p/Y/y/H/h/V/W 命令)
 *   F2  定位 / 高度观察 (case 2)
 *   F3  VOFA 无线链路测试 (case 3)
 *   F4  陀螺原始/校准数据 (case 4)
 *   F5  加速度原始/校准数据 (case 5)
 *   F6  Roll/Pitch 角度+角速度环监视 (case 6)
 *   F7  ADC + 摇杆历史调试 (case 7)
 *   F8  光流速度闭环综合监视 (case 8): I0~I3 X轴 光流实际/目标/积分项/pitch修正,
 *                                       I4~I7 Y轴 光流实际/目标/积分项/roll 修正
 *   F9  姿态解算总览 16 路诊断 (case 9)
 * FD  高度环监视 (case 13): I0 实际高度(cm), I1 目标高度(cm),
 *                                        I2 高度误差(cm), I3 油门修正,
 *                                        I4 目标垂直速度, I5 融合垂直速度,
 *                                        I6 速度误差, I7 base_throttle
 * FE  光流原始输入诊断 (case 14)
 * FG  红外正方形中心点与丢失保护诊断 (case 16)
 * FH  红外丢失保护状态机诊断 (case 17)
 * FI  红外位置闭环诊断 (case 18)
 * FL  红外位置环 PID 调参诊断 (case 21)
 * FM  X 轴位置-速度-Pitch-角速度链路诊断 (case 22)
 * FN  Y 轴位置-速度-Roll-角速度链路诊断 (case 23)
 * FP  X/Y/高度闭环 16 路总览诊断 (case 24)
 * FQ  小车红外灯/信标红外灯识别效果 16 路诊断 (case 25)
 * FR  0 核主循环耗时诊断 (case 26)
 * FS  高度估计 / 串级 PID 16 路诊断 (case 27)
 * FU  光流融合状态 16 路诊断 (case 29)
 * FV  beacon 下发槽位 16 路诊断 (case 30)
 * FW  当前帧连通域面积排序 16 路诊断 (case 31)
 */
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
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    vofa_loop_profiler_ready = 1U;
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

static void set_angle_pid_value(uint8 axis, uint8 item, float value)
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
        default:  return;
    }

    Flight_Control_Set_Angle_PID(axis, kp, ki, kd);
}

static void set_gyro_pid_value(uint8 axis, uint8 item, float value)
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
            break;

        case '4':
            Flight_Control_Get_Gyro_Limits(axis, &out_limit, &int_limit);
            Flight_Control_Set_Gyro_Limits(axis, out_limit, value);
            break;

        default:
            break;
    }
}

static void set_height_pid_value(uint8 item, float value)
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
        default:  return;
    }

    Flight_Control_Set_Height_PID(kp, ki, kd);
}

static void set_height_vel_pid_value(uint8 item, float value)
{
    switch(item)
    {
        case '1': HeightVelPID.Kp = value; break;
        case '2': HeightVelPID.Ki = value; break;
        case '3': HeightVelPID.Kd = value; break;
        default:  break;
    }
}

/*
 * VOFA 调参: 光流速度闭环 PID。
 *
 * axis = 0 → vel_pid_x (前向速度 vx_cms 反馈, 输出 ax_cmd_cms2)
 * axis = 1 → vel_pid_y (左向速度 vy_cms 反馈, 输出 ay_cmd_cms2)
 *
 * item:
 *   '1' Kp, '2' Ki, '3' Kd, '4' output_limit (cm/s^2), '5' integral_limit
 *
 * 上层在 USART_PID_Adjust 已经按命令字符 'V'/'W' 区分轴，避免与角度环 R/P/Y 混淆。
 */
static void set_vel_pid_value(uint8 axis, uint8 item, float value)
{
    PID_Controller *pid = NULL;

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
        return;
    }

    switch(item)
    {
        case '1': pid->Kp = value; break;
        case '2': pid->Ki = value; break;
        case '3': pid->Kd = value; break;
        case '4': pid->output_limit = value; break;
        case '5': pid->integral_limit = value; break;
        default:  return;
    }
}

static void set_pos_pid_value(uint8 axis, uint8 item, float value)
{
    PID_Controller *pid = NULL;

    if(item == '0')
    {
        Position_Loop_Set_Enabled((value > 0.5f) ? 1U : 0U);
        return;
    }

    if(axis == 0U)
    {
        pid = &pos_pid_x;
    }
    else if(axis == 1U)
    {
        pid = &pos_pid_y;
    }
    else
    {
        return;
    }

    switch(item)
    {
        case '1': pid->Kp = value; break;
        case '2': pid->Ki = value; break;
        case '3': pid->Kd = value; break;
        case '4': pid->output_limit = value; break;
        case '5': pid->integral_limit = value; break;
        default:  return;
    }
}

static void set_hover_angle_compensation_value(uint8 item, float value)
{
    float hover_roll = 0.0f;
    float hover_pitch = 0.0f;

    Flight_Takeoff_Get_Hover_Angle_Compensation(&hover_roll, &hover_pitch);

    switch(item)
    {
        case '1': hover_roll = value; break;
        case '2': hover_pitch = value; break;
        default:  return;
    }

    Flight_Takeoff_Set_Hover_Angle_Compensation(hover_roll, hover_pitch);
}


/*
 * VOFA 调参：陀螺去旋转增益。
 *   item = 'x'/'X': 设置 K_X（flow_x_rad 对 gyro_y_delta_rad 的去旋转增益）
 *   item = 'y'/'Y': 设置 K_Y（flow_y_rad 对 gyro_x_delta_rad 的去旋转增益）
 * 范围由 PMW_Flow_SetGyroCompGain() 内部夹位，这里负值才表示“不变更”，
 * 所以已 has_value 的正常调参直接传 value 即可。
 */
static void set_pmw_flow_gyro_gain_value(uint8 item, float value)
{
    if(value < 0.0f)
    {
        return;
    }

    switch(item)
    {
        case 'x':
        case 'X':
            PMW_Flow_SetGyroCompGain(value, -1.0f);
            break;
        case 'y':
        case 'Y':
            PMW_Flow_SetGyroCompGain(-1.0f, value);
            break;
        default:
            break;
    }
}

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
static void send_pid_data_back(uint8 command)
{
    float value_1 = 0.0f;
    float value_2 = 0.0f;
    float value_3 = 0.0f;

    switch(command)
    {
        case 'R':
            Flight_Control_Get_Angle_PID(AXIS_ROLL, &value_1, &value_2, &value_3);
            break;

        case 'r':
            Flight_Control_Get_Gyro_PID(AXIS_ROLL, &value_1, &value_2, &value_3);
            break;

        case 'P':
            Flight_Control_Get_Angle_PID(AXIS_PITCH, &value_1, &value_2, &value_3);
            break;

        case 'p':
            Flight_Control_Get_Gyro_PID(AXIS_PITCH, &value_1, &value_2, &value_3);
            break;

        case 'Y':
            Flight_Control_Get_Angle_PID(AXIS_YAW, &value_1, &value_2, &value_3);
            break;

        case 'y':
            Flight_Control_Get_Gyro_PID(AXIS_YAW, &value_1, &value_2, &value_3);
            break;

        case 'H':
            Flight_Control_Get_Height_PID(&value_1, &value_2, &value_3);
            break;

        case 'h':
            value_1 = HeightVelPID.Kp;
            value_2 = HeightVelPID.Ki;
            value_3 = HeightVelPID.Kd;
            break;

        case 'V':
            value_1 = vel_pid_x.Kp;
            value_2 = vel_pid_x.Ki;
            value_3 = vel_pid_x.Kd;
            send_four_date(value_1, value_2, value_3,
                           vel_pid_x.output_limit,
                           vel_pid_x.integral_limit,
                           0, 0, 0);
            return;

        case 'W':
            value_1 = vel_pid_y.Kp;
            value_2 = vel_pid_y.Ki;
            value_3 = vel_pid_y.Kd;
            send_four_date(value_1, value_2, value_3,
                           vel_pid_y.output_limit,
                           vel_pid_y.integral_limit,
                           0, 0, 0);
            return;

        case 'I':
            value_1 = pos_pid_x.Kp;
            value_2 = pos_pid_x.Ki;
            value_3 = pos_pid_x.Kd;
            send_four_date(value_1, value_2, value_3,
                           pos_pid_x.output_limit,
                           pos_pid_x.integral_limit,
                           (float)Position_Loop_Is_Enabled(), 0, 0);
            return;

        case 'J':
            value_1 = pos_pid_y.Kp;
            value_2 = pos_pid_y.Ki;
            value_3 = pos_pid_y.Kd;
            send_four_date(value_1, value_2, value_3,
                           pos_pid_y.output_limit,
                           pos_pid_y.integral_limit,
                           (float)Position_Loop_Is_Enabled(), 0, 0);
            return;

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

        case 'T':
            Flight_Control_Get_Target_Angle(&roll, &pitch, &yaw);
            send_four_date(current_roll, current_pitch, current_yaw, icm42688_acc_x,
                icm42688_acc_y, icm42688_acc_z,
                roll, pitch);

            return;

        case 'G':
        {
            float gx_now = 0.0f;
            float gy_now = 0.0f;
            PMW_Flow_GetGyroCompGain(&gx_now, &gy_now);
            send_four_date(gx_now, gy_now, 0, 0, 0, 0, 0, 0);
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
                set_angle_pid_value(AXIS_ROLL, item, data_get);
            }
            send_back_command = 'R';
            break;

        case 'r':
            if(has_value)
            {
                set_gyro_pid_value(AXIS_ROLL, item, data_get);
            }
            send_back_command = 'r';
            break;

        case 'P':
            if(has_value)
            {
                set_angle_pid_value(AXIS_PITCH, item, data_get);
            }
            send_back_command = 'P';
            break;

        case 'p':
            if(has_value)
            {
                set_gyro_pid_value(AXIS_PITCH, item, data_get);
            }
            send_back_command = 'p';
            break;

        case 'Y':
            if(has_value)
            {
                set_angle_pid_value(AXIS_YAW, item, data_get);
            }
            send_back_command = 'Y';
            break;

        case 'y':
            if(has_value)
            {
                set_gyro_pid_value(AXIS_YAW, item, data_get);
            }
            send_back_command = 'y';
            break;

        case 'S':
            if(has_value)
            {
                Flight_Control_Set_Throttle(data_get);
            }
            send_back_command = 'S';
            break;

        case 'B':
            if(has_value)
            {
                set_hover_angle_compensation_value(item, data_get);
            }
            send_back_command = 'B';
            break;

        case 'V':
            if(has_value)
            {
                set_vel_pid_value(0U, item, data_get);
            }
            send_back_command = 'V';
            break;

        case 'W':
            if(has_value)
            {
                set_vel_pid_value(1U, item, data_get);
            }
            send_back_command = 'W';
            break;

        case 'I':
            if(has_value)
            {
                set_pos_pid_value(0U, item, data_get);
            }
            send_back_command = 'I';
            break;

        case 'J':
            if(has_value)
            {
                set_pos_pid_value(1U, item, data_get);
            }
            send_back_command = 'J';
            break;

        case 'F':
            switch(item)
            {
                case '0': send_back_flag = 0; break;
                case '1': send_back_flag = 1; break;
                case '2': send_back_flag = 2; break;
                case '3': send_back_flag = 3; break;
                case '4': send_back_flag = 4; break;
                case '5': send_back_flag = 5; break;
                case '6': send_back_flag = 6; break;
                case '7': send_back_flag = 7; break;
                case '8': send_back_flag = 8; break;
                case '9': send_back_flag = 9; break;
                case 'A':
                case 'a': send_back_flag = 10; break;
                case 'B':
                case 'b': send_back_flag = 11; break;
                case 'C':
                case 'c': send_back_flag = 12; break;
                case 'D':
                case 'd': send_back_flag = 13; break;
                case 'E':
                case 'e': send_back_flag = 14; break;
                case 'F':
                case 'f': send_back_flag = 15; break;
                case 'G':
                case 'g': send_back_flag = 16; break;
                case 'H':
                case 'h': send_back_flag = 17; break;
                case 'I':
                case 'i': send_back_flag = 18; break;
                case 'L':
                case 'l': send_back_flag = 21; break;
                case 'M':
                case 'm': send_back_flag = 22; break;
                case 'N':
                case 'n': send_back_flag = 23; break;
                case 'P':
                case 'p': send_back_flag = 24; break;
                case 'Q':
                case 'q': send_back_flag = 25; break;
                case 'R':
                case 'r': send_back_flag = 26; break;
                case 'S':
                case 's': send_back_flag = 27; break;
                case 'U':
                case 'u': send_back_flag = 29; break;
                case 'V':
                case 'v': send_back_flag = 30; break;
                case 'W':
                case 'w': send_back_flag = 31; break;
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
                set_height_pid_value(item, data_get);
            }
            send_back_command = 'H';
            break;

        case 'h':
            if(has_value)
            {
                set_height_vel_pid_value(item, data_get);
            }
            send_back_command = 'h';
            break;

        case 'G':
            if(has_value)
            {
                set_pmw_flow_gyro_gain_value(item, data_get);
            }
            send_back_command = 'G';
            break;

        default:
            break;
    }

    clear_active_vofa_packet();
}

void send_date_back(void)
{
    uint8 ir_track_valid = 0U;

    if(!CommMux_IsVofaMode())
    {
        return;
    }

    VehicleMarkerLocalization_ServiceLostProtection();
    ir_track_valid = (uint8)((g_vehicle_marker_pose.valid && !g_vehicle_marker_position_lost_flag) ? 1U : 0U);

    switch(send_back_flag)
    {
        case 0:
        {
            PMWFlowState_t pmw_flow_state = {0};
            PMW_Flow_GetState(&pmw_flow_state);

            send_four_date(pmw_flow_state.flow_x_rad,
                           pmw_flow_state.gyro_flow_y_delta_rad,
                           pmw_flow_state.flow_comp_x_rad,
                           pmw_flow_state.velocity_x_raw_cms,
                           pmw_flow_state.flow_y_rad,
                           pmw_flow_state.gyro_flow_x_delta_rad,
                           pmw_flow_state.flow_comp_y_rad,
                           pmw_flow_state.velocity_y_raw_cms);
            break;
        }

        case 1:
            send_pid_data_back(send_back_command);
            break;

        case 2:
            send_four_date(g_vehicle_marker_pose.x, g_height_result.height_cm,
                           g_vehicle_marker_pose.z, tof_height_true_filt * 100.0f,
                           0, current_roll,
                           current_pitch, current_yaw);
            break;

        case 3:
            send_four_date(build_vofa_test_channel(), current_roll,
                           current_pitch, current_yaw,
                           g_height_result.height_cm, (float)dl1b_distance_mm,
                           (float)dl1b_finsh_flag, 0);
            break;

        case 4:
            send_four_date((float)icm42688_gyro_x_r, (float)icm42688_gyro_y_r,
                           (float)icm42688_gyro_z_r, icm42688_gyro_x,
                           icm42688_gyro_y, icm42688_gyro_z,
                           0.0f, 0.0f);
            break;

        case 5:
            send_four_date((float)icm42688_acc_x_r, (float)icm42688_acc_y_r,
                           (float)icm42688_acc_z_r, icm42688_acc_x,
                           icm42688_acc_y, icm42688_acc_z,
                           acc, acc_norm_filtered);
            break;

        case 6:
            send_four_date(current_roll, target_angle_roll,
                           current_gyro_roll, target_roll_rate,
                           current_pitch, target_angle_pitch,
                           current_gyro_pitch, target_pitch_rate);
            break;

        case 7:
            send_four_date((float)adc_x, (float)adc_y,
                           (float)adc_x_filtered, (float)adc_y_filtered,
                           adc_x_nomalize, adc_y_nomalize,
                           target_angle_roll, target_angle_pitch);
            break;

        case 8:
        {
            /*
             * F8 - 速度环监视。
             * 左半 (I0~I3) X 轴: 速度环滤波反馈 / 目标速度 / 积分输出 / 总输出；
             * 右半 (I4~I7) Y 轴: 速度环滤波反馈 / 目标速度 / 积分输出 / 总输出。
             */
            float vx_target_cms = 0.0f;
            float vy_target_cms = 0.0f;
            float pitch_corr_deg = 0.0f;
            float roll_corr_deg = 0.0f;
            float fb_vx_cms = 0.0f;
            float fb_vy_cms = 0.0f;
            float x_i_out = 0.0f;
            float y_i_out = 0.0f;

            Velocity_Loop_Get_Target_Velocity(&vx_target_cms, &vy_target_cms);
            Velocity_Loop_Get_Diag(&pitch_corr_deg, &roll_corr_deg,
                                   &fb_vx_cms, &fb_vy_cms,
                                   &x_i_out, &y_i_out, NULL);

            send_four_date(fb_vx_cms,
                           vx_target_cms,
                           x_i_out,
                           pitch_corr_deg,
                           fb_vy_cms,
                           vy_target_cms,
                           y_i_out,
                           roll_corr_deg);
            break;
        }

        case 9:
            send_sixteen_date(current_roll,
                              current_pitch,
                              current_yaw,
                              Yaw_a_Pi_genrl * Rad2Ang,
                              q0,
                              q1,
                              q2,
                              q3,
                              Roll_g_F,
                              Pitch_g_F,
                              Yaw_g_F,
                              acc_norm_filtered,
                              Roll_acc_F,
                              Pitch_acc_F,
                              Yaw_acc_F,
                              (float)Check_IMU_Data_Quality());
            break;

        case 10:
        {
            float ax_level_mg = 0.0f;
            float ay_level_mg = 0.0f;
            float az_level_mg = 0.0f;

            Body_To_RollPitch_Level_Frame(Roll_acc_F, Pitch_acc_F, Yaw_acc_F,
                                          &ax_level_mg, &ay_level_mg, &az_level_mg);

            send_four_date(current_roll,
                           current_pitch,
                           ax_level_mg,
                           ay_level_mg,
                           az_level_mg,
                           acc_level_y_bias_mg,
                           ax_level_mg - acc_level_x_bias_mg,
                           ay_level_mg - acc_level_y_bias_mg);
            break;
        }

        case 11:
        {
            PMWFlowState_t pmw_flow_state = {0};
            PMW_Flow_GetState(&pmw_flow_state);

            send_four_date(pmw_flow_state.velocity_x_cms,
                           pmw_flow_state.velocity_y_cms,
                           pmw_flow_state.yaw_deg,
                           pmw_flow_state.velocity_x_world_cms,
                           pmw_flow_state.velocity_y_world_cms,
                           pmw_flow_state.position_x_world_cm,
                           pmw_flow_state.position_y_world_cm,
                           (float)pmw_flow_state.fused_velocity_valid);
            break;
        }

        case 12:
        {
            PMWFlowState_t pmw_flow_state = {0};
            PMW_Flow_GetState(&pmw_flow_state);

            send_four_date(pmw_flow_state.position_x_world_cm,
                           pmw_flow_state.position_y_world_cm,
                           pmw_flow_state.flow_position_x_world_cm,
                           pmw_flow_state.flow_position_y_world_cm,
                           pmw_flow_state.flow_delta_x_world_cm,
                           pmw_flow_state.flow_delta_y_world_cm,
                           pmw_flow_state.yaw_deg,
                           (float)pmw_flow_state.flow_position_valid);
            break;
        }

        case 13:
        {
            /* FD: I0 height, I1 target, I2 height error, I3 throttle correction,
             *     I4 target vz, I5 fused vz, I6 velocity error, I7 base throttle.
             */
            HeightManagerDiag_t height_diag;
            float actual_cm;
            float target_cm;
            float error_cm;

            HeightManager_Get_Diag(&height_diag);

            actual_cm  = g_height_result.height_cm;
            target_cm  = HeightManager_Get_Active_Target();
            error_cm   = target_cm - actual_cm;
            if(error_cm > 40.0f)
            {
                error_cm = 40.0f;
            }
            else if(error_cm < -40.0f)
            {
                error_cm = -40.0f;
            }
            send_four_date(actual_cm,
                           target_cm,
                           error_cm,
                           HeightSpeedPID_output,
                           height_diag.target_velocity_cms,
                           g_height_result.velocity_cms,
                           height_diag.velocity_error_cms,
                           base_throttle);
            break;
        }

        case 14:
        {
            PMWFlowState_t pmw_flow_state = {0};
            PMW_Flow_GetState(&pmw_flow_state);

#if FLOW_SENSOR_SELECTED == FLOW_SENSOR_UPFLOW302
            send_four_date((float)FLOW_SENSOR_SELECTED,
                           (float)pmw_flow_state.flow_delta_x_count,
                           (float)pmw_flow_state.flow_delta_y_count,
                           (float)upflow302_receive.upflow302_valid,
                           (float)upflow302_state_flag,
                           (float)upflow302_finsh_flag,
                           (float)upflow302_receive.upflow302_us,
                           pmw_flow_state.dt_s);
#else
            send_four_date((float)FLOW_SENSOR_SELECTED,
                           (float)pmw_flow_state.flow_delta_x_count,
                           (float)(-pmw_flow_state.flow_delta_y_count),
                           pmw_flow_state.valid,
                           pmw_flow_state.dt_s,
                           pmw_flow_state.flow_x_rad,
                           pmw_flow_state.flow_y_rad,
                           pmw_flow_state.tof_los_cm);
#endif
            break;
        }

        case 15:
        {
            float roll_i_out = angle_pid_roll.Ki * angle_pid_roll.integral;
            float pitch_i_out = angle_pid_pitch.Ki * angle_pid_pitch.integral;

            send_four_date(target_angle_roll,
                           current_roll,
                           roll_i_out,
                           target_roll_rate,
                           target_angle_pitch,
                           current_pitch,
                           pitch_i_out,
                           target_pitch_rate);
            break;
        }

        case 16:
            send_four_date(g_vehicle_marker_pose.x,
                           g_vehicle_marker_pose.y,
                           g_vehicle_marker_pose.z,
                           g_vehicle_marker_pose.square_side_cm,
                           (float)ir_track_valid,
                           (float)g_vehicle_marker_position_lost_flag,
                           (float)g_vehicle_marker_position_lost_elapsed_ms,
                           (float)g_vehicle_marker_pose.detected_point_count);
            break;

        case 17:
        {
            VehicleMarkerLossDebug_t ir_loss_debug = {0};
            VehicleMarkerLocalization_GetLossDebug(&ir_loss_debug);

            send_four_date(g_vehicle_marker_pose.x,
                           g_vehicle_marker_pose.y,
                           (float)ir_loss_debug.raw_detect_valid,
                           (float)ir_loss_debug.lost_flag,
                           (float)ir_loss_debug.lost_elapsed_ms,
                           (float)ir_loss_debug.has_trackable_pose,
                           (float)ir_loss_debug.task_5ms_count,
                           (float)ir_loss_debug.timeout_count);
            break;
        }

        case 18:
        {
            PositionLoopDiag_t pos_diag = {0};
            Position_Loop_Get_Diag(&pos_diag);

            send_four_date(pos_diag.fb_x_cm,
                           pos_diag.fb_y_cm,
                           pos_diag.vx_cmd_cms,
                           pos_diag.vy_cmd_cms,
                           (float)pos_diag.enabled,
                           (float)pos_diag.active,
                           (float)pos_diag.ir_track_valid,
                           (float)pos_diag.velocity_ready);
            break;
        }

        case 21:
        {
            PositionLoopDiag_t pos_diag = {0};
            float x_p_out = 0.0f;
            float y_p_out = 0.0f;

            Position_Loop_Get_Diag(&pos_diag);
            x_p_out = pos_pid_x.Kp * pos_diag.error_x_cm;
            y_p_out = pos_pid_y.Kp * pos_diag.error_y_cm;

            send_four_date(pos_diag.error_x_cm,
                           x_p_out,
                           pos_diag.x_i_out,
                           pos_diag.vx_cmd_cms,
                           pos_diag.error_y_cm,
                           y_p_out,
                           pos_diag.y_i_out,
                           pos_diag.vy_cmd_cms);
            break;
        }

        case 22:
        {
            PositionLoopDiag_t pos_diag = {0};
            PMWFlowState_t pmw_flow_state = {0};

            Position_Loop_Get_Diag(&pos_diag);
            PMW_Flow_GetState(&pmw_flow_state);

            send_four_date(pos_diag.fb_x_cm,
                           pos_diag.vx_cmd_cms,
                           pmw_flow_state.velocity_x_fused_cms,
                           target_angle_pitch,
                           current_pitch,
                           target_pitch_rate,
                           current_gyro_pitch,
                           (float)pos_diag.active);
            break;
        }

        case 23:
        {
            PositionLoopDiag_t pos_diag = {0};
            PMWFlowState_t pmw_flow_state = {0};

            Position_Loop_Get_Diag(&pos_diag);
            PMW_Flow_GetState(&pmw_flow_state);

            send_four_date(pos_diag.fb_y_cm,
                           pos_diag.vy_cmd_cms,
                           pmw_flow_state.velocity_y_fused_cms,
                           target_angle_roll,
                           current_roll,
                           target_roll_rate,
                           current_gyro_roll,
                           (float)pos_diag.active);
            break;
        }

        case 24:
        {
            PositionLoopDiag_t pos_diag = {0};
            float fb_vx_cms = 0.0f;
            float fb_vy_cms = 0.0f;

            Position_Loop_Get_Diag(&pos_diag);
            Velocity_Loop_Get_Diag(NULL, NULL,
                                   &fb_vx_cms, &fb_vy_cms,
                                   NULL, NULL, NULL);

            send_sixteen_date(pos_diag.target_x_cm,
                              pos_diag.target_y_cm,
                              pos_diag.vx_cmd_cms,
                              pos_diag.vy_cmd_cms,
                              fb_vx_cms,
                              fb_vy_cms,
                              target_angle_pitch,
                              target_angle_roll,
                              current_pitch,
                              current_roll,
                              target_pitch_rate,
                              target_roll_rate,
                              current_gyro_pitch,
                              current_gyro_roll,
                              HeightManager_Get_Active_Target(),
                              g_height_result.height_cm);
            break;
        }

        case 25:
        {
            VisionRuntimeDiag_t vision_diag = {0};

            VisionRuntime_GetDiag(&vision_diag);

            send_sixteen_date((float)vision_diag.blob_count,
                              (float)vision_diag.ir_split_count,
                              (float)vision_diag.beacon_split_count,
                              (float)vision_diag.ir_blob_count,
                              (float)vision_diag.ir_reject_area_low_count,
                              (float)vision_diag.ir_reject_area_high_count,
                              (float)vision_diag.ir_reject_size_count,
                              (float)vision_diag.ir_reject_aspect_count,
                              (float)vision_diag.ir_reject_insert_count,
                              (float)vision_diag.ir_track_valid,
                              (float)vision_diag.ir_detected_point_count,
                              vision_diag.ir_x_cm,
                              vision_diag.ir_y_cm,
                              vision_diag.ir_square_side_cm,
                              vision_diag.ir_height_cm,
                              (float)vision_diag.beacon_blob_count);
            break;
        }

        case 26:
            send_four_date(vofa_loop_last_us,
                           vofa_loop_max_us,
                           vofa_loop_avg_us,
                           vofa_loop_freq_hz,
                           (float)vofa_loop_count,
                           (float)vofa_loop_overrun_count,
                           VOFA_LOOP_OVERRUN_US,
                           (float)SystemCoreClock);
            break;

        case 27:
        {
            HeightManagerDiag_t height_diag;
            float actual_cm;
            float target_cm;

            HeightManager_Get_Diag(&height_diag);
            actual_cm = g_height_result.height_cm;
            target_cm = HeightManager_Get_Active_Target();

            send_sixteen_date(actual_cm,
                              target_cm,
                              height_diag.tof_median_cm,
                              height_diag.tof_height_filt_cm,
                              height_diag.tof_velocity_cms,
                              height_diag.acc_velocity_cms,
                              height_diag.fused_velocity_cms,
                              height_diag.target_velocity_cms,
                              height_diag.velocity_error_cms,
                              height_diag.velocity_pid_output,
                              height_diag.acc_z_level_mg,
                              height_diag.gravity_bias_mg,
                              height_diag.acc_z_cms2,
                              (float)height_diag.tof_valid,
                              (float)height_diag.acc_used,
                              (float)height_diag.cascade_enabled);
            break;
        }

        case 29:
        {
            PMWFlowState_t pmw_flow_state = {0};
            uint8 tof_source = 0U;
            float fb_vx_cms = 0.0f;
            float fb_vy_cms = 0.0f;

            PMW_Flow_GetState(&pmw_flow_state);
            PMW_Flow_Get_TOF_Status(NULL, NULL, &tof_source);
            Velocity_Loop_Get_Diag(NULL, NULL,
                                   &fb_vx_cms, &fb_vy_cms,
                                   NULL, NULL, NULL);

            send_sixteen_date((float)pmw_flow_state.valid,
                              (float)pmw_flow_state.flow_used,
                              (float)pmw_flow_state.acc_used,
                              (float)pmw_flow_state.fused_velocity_valid,
                              (float)tof_source,
                              pmw_flow_state.range_cm,
                              pmw_flow_state.dt_s,
                              pmw_flow_state.flow_confidence,
                              pmw_flow_state.flow_fusion_alpha,
                              pmw_flow_state.flow_residual_norm_cms,
                              fb_vx_cms,
                              fb_vy_cms,
                              pmw_flow_state.velocity_x_cms,
                              pmw_flow_state.velocity_y_cms,
                              pmw_flow_state.velocity_x_fused_cms,
                              pmw_flow_state.velocity_y_fused_cms);
            break;
        }

        case 30:
        {
            VisionRuntimeDiag_t vision_diag = {0};
            VisionRuntimeBeaconSlot_t beacon_slots[VISION_RUNTIME_BEACON_SLOT_COUNT] = {0};

            VisionRuntime_GetDiag(&vision_diag);
            VisionRuntime_GetBeaconSlots(beacon_slots, VISION_RUNTIME_BEACON_SLOT_COUNT);

            send_sixteen_date((float)vision_diag.beacon_count,
                              (float)beacon_slots[0].valid,
                              beacon_slots[0].x_cm,
                              beacon_slots[0].y_cm,
                              (float)beacon_slots[1].valid,
                              beacon_slots[1].x_cm,
                              beacon_slots[1].y_cm,
                              (float)beacon_slots[2].valid,
                              beacon_slots[2].x_cm,
                              beacon_slots[2].y_cm,
                              (float)beacon_slots[3].valid,
                              beacon_slots[3].x_cm,
                              beacon_slots[3].y_cm,
                              (float)beacon_slots[4].valid,
                              beacon_slots[4].x_cm,
                              beacon_slots[4].y_cm);
            break;
        }

        case 31:
        {
            VisionRuntimeDiag_t vision_diag = {0};

            VisionRuntime_GetDiag(&vision_diag);

            send_sixteen_date((float)vision_diag.blob_area_rank[0],
                              (float)vision_diag.blob_area_rank[1],
                              (float)vision_diag.blob_area_rank[2],
                              (float)vision_diag.blob_area_rank[3],
                              (float)vision_diag.blob_area_rank[4],
                              (float)vision_diag.blob_area_rank[5],
                              (float)vision_diag.blob_area_rank[6],
                              (float)vision_diag.blob_area_rank[7],
                              (float)vision_diag.blob_area_rank[8],
                              (float)vision_diag.blob_area_rank[9],
                              (float)vision_diag.blob_area_rank[10],
                              (float)vision_diag.blob_area_rank[11],
                              (float)vision_diag.blob_area_rank[12],
                              (float)vision_diag.blob_area_rank[13],
                              (float)vision_diag.blob_area_rank[14],
                              (float)vision_diag.blob_area_rank[15]);
            break;
        }

        default:
            break;
    }
}
