#include "common/PIT_Demo.h"
#include "common/battery_monitor.h"
#include "common/project_headfile.h"
#include "control/angle_loop.h"
#include "control/ground_remote_input.h"
#include "control/motor_mixer.h"
#include "control/position_loop.h"
#include "control/rate_loop.h"
#include "control/throttle_runtime.h"
#include "control/velocity_loop.h"
#include "menu/menu_fly_test.h"
#include "menu/menu_motor_test.h"
#include "motor_hardware/motor_hardware.h"
#include "pmw/optical_flow_feed.h"
#include <math.h>

// ==================== Configuration constants ====================

#define THROTTLE_RAMP_RATE             2.0f
#define CONTROL_DT_S                   0.00025f
#define CONTROL_DT_MS                  0.25f

#define ADC_INPUT_LPF_ALPHA            0.20f

#define FLIGHT_REALTIME_DWT_LAR_UNLOCK 0xC5ACCE55UL
#define CONTROL_PERIOD_US              250U
#define MOTOR_PERIOD_US                1000U
#define CONTROL_DELAY_THRESHOLD_US     313U

#define CONTROL_SCHEDULER_TICKS        80U
#define RATE_SLOT_DIVISOR              4U
#define ANGLE_SLOT_DIVISOR             8U
#define ANGLE_SLOT_PHASE               3U
#define VELOCITY_SLOT_DIVISOR          16U
#define VELOCITY_SLOT_PHASE            2U
#define POSITION_SLOT_PHASE            1U

/*
 * 真位置闭环参数。
 *
 * 物理布置：无人机由长 ROPE_LENGTH_M 的绳子系留在地面遥感单元上，期望
 * 始终飞在遥感正上方。adc_x_nomalize / adc_y_nomalize 由遥感单元给出，
 * 已归一化到 [-1, 1]；满量程对应绳子与地面夹角为 ROPE_GROUND_ANGLE_FULL_DEG。
 *
 * 由此推得无人机相对遥感中心的水平位移：
 *   adc 归一化幅值=1 时位移 = L * cos(50°) = ROPE_LENGTH_M * 0.642788 ≈ 0.8678 m
 * 实现里直接按线性折算 position_error = adc_norm * POSITION_ERROR_MAX_M。
 *
 * ATTITUDE_MAX_DEG 是位置环输出（即偏离起飞基准的姿态修正量）的硬限幅，
 * 等价于无人机姿态控制允许的最大倾角。
 */
#define ROPE_LENGTH_M                  1.35f
#define ROPE_GROUND_ANGLE_FULL_DEG     50.0f
#define POSITION_ERROR_MAX_M           (ROPE_LENGTH_M * 0.642788f) /* L * cos(50°) */
#define ATTITUDE_MAX_DEG               3.0f

/*
 * 双数据源校验 + 加权融合参数。
 *
 * 思路：
 *   r_adc 由 adc_x/adc_y 折算得到水平半径；
 *   r_tof 由 sqrt(L² - h²) 反推（h 来自 tof_height_true_filt，单位 m）；
 *   两条独立观测在残差通过 innovation 门限时按方差倒数加权融合，否则记 fault
 *   并退回只信 adc，避免 ToF 失效（出量程、绳子松弛）把控制器带偏。
 *
 * σ 系数是经验起步值，可在试飞时调；权重在编译期算完，热路径上没有除法。
 */
#define FUSION_SIGMA_ADC_M             0.05f
#define FUSION_SIGMA_TOF_M             0.02f
#define FUSION_GATE_M                  0.16f   /* ~3·sqrt(σ_adc² + σ_tof²) */
#define FUSION_R_ADC_EPS_M             0.01f   /* r_adc 太小时不缩放，避免方向放大 */
#define FUSION_TOF_MIN_M               0.10f
#define FUSION_TOF_MAX_M               ROPE_LENGTH_M
#define FUSION_W_ADC                   (1.0f / (FUSION_SIGMA_ADC_M * FUSION_SIGMA_ADC_M))
#define FUSION_W_TOF                   (1.0f / (FUSION_SIGMA_TOF_M * FUSION_SIGMA_TOF_M))

// ==================== Public runtime state ====================

volatile uint8 armed = 0;
volatile uint8 control_mode = 0;

volatile float target_angle_roll = 0.0f;
volatile float target_angle_pitch = 0.0f;
volatile float target_angle_yaw = 0.0f;

/* IMU 安装误差由 icm42688.c 的传感器到机体旋转矩阵统一补偿。 */
float current_roll = 0.0f;
float current_pitch = 0.0f;
float current_yaw = 0.0f;
float current_gyro_roll = 0.0f;
float current_gyro_pitch = 0.0f;
float current_gyro_yaw = 0.0f;

volatile uint16 motor_throttle[4] = {0, 0, 0, 0};
uint16 throttle_buffer[4] = {0, 0, 0, 0};

/* 位置闭环 adc/ToF 双数据校验观测量（VOFA / 监视页可读）。 */
volatile float pos_fusion_r_adc    = 0.0f;
volatile float pos_fusion_r_tof    = 0.0f;
volatile float pos_fusion_r_fused  = 0.0f;
volatile float pos_fusion_residual = 0.0f;
volatile uint8 pos_fusion_fault    = 0U;

static uint8 s_realtime_ready = 0U;
static uint32 s_control_start_cycle = 0U;
static uint32 s_control_last_entry_cycle = 0U;
static uint32 s_control_last_cycles = 0U;
static uint32 s_control_max_cycles = 0U;
static uint32 s_control_interval_min_cycles = 0xFFFFFFFFU;
static uint32 s_control_interval_max_cycles = 0U;
static uint32 s_control_count = 0U;
static uint32 s_control_execution_overrun_count = 0U;
static uint32 s_control_delayed_entry_count = 0U;
static uint32 s_motor_start_cycle = 0U;
static uint32 s_motor_last_cycles = 0U;
static uint32 s_motor_max_cycles = 0U;
static uint32 s_motor_count = 0U;
static uint32 s_motor_execution_overrun_count = 0U;
static uint32 s_rate_slot_count = 0U;
static uint32 s_angle_slot_count = 0U;
static uint32 s_velocity_slot_count = 0U;
static uint32 s_position_slot_count = 0U;
static uint8 s_control_scheduler_tick = 0U;

static void flight_realtime_init(void)
{
    if(s_realtime_ready)
    {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
    {
        return;
    }
    DWT->LAR = FLIGHT_REALTIME_DWT_LAR_UNLOCK;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_realtime_ready = 1U;
}

static uint32 flight_realtime_us_to_cycles(uint32 time_us)
{
    return (uint32)(((uint64)SystemCoreClock * time_us) / 1000000ULL);
}

static float flight_realtime_cycles_to_us(uint32 cycles)
{
    if(!s_realtime_ready || SystemCoreClock == 0U)
    {
        return 0.0f;
    }
    return ((float)cycles * 1000000.0f) / (float)SystemCoreClock;
}

void FlightRealtime_ControlBegin(void)
{
    uint32 now;

    flight_realtime_init();
    if(!s_realtime_ready)
    {
        return;
    }

    now = DWT->CYCCNT;
    if(s_control_last_entry_cycle != 0U)
    {
        uint32 interval = now - s_control_last_entry_cycle;
        if(interval < s_control_interval_min_cycles)
        {
            s_control_interval_min_cycles = interval;
        }
        if(interval > s_control_interval_max_cycles)
        {
            s_control_interval_max_cycles = interval;
        }
        if(interval > flight_realtime_us_to_cycles(CONTROL_DELAY_THRESHOLD_US))
        {
            s_control_delayed_entry_count++;
        }
    }
    s_control_last_entry_cycle = now;
    s_control_start_cycle = now;
}

void FlightRealtime_ControlEnd(void)
{
    uint32 elapsed;

    if(!s_realtime_ready)
    {
        return;
    }
    elapsed = DWT->CYCCNT - s_control_start_cycle;
    s_control_last_cycles = elapsed;
    if(elapsed > s_control_max_cycles)
    {
        s_control_max_cycles = elapsed;
    }
    if(elapsed > flight_realtime_us_to_cycles(CONTROL_PERIOD_US))
    {
        s_control_execution_overrun_count++;
    }
    s_control_count++;
}

void FlightRealtime_MotorBegin(void)
{
    flight_realtime_init();
    if(s_realtime_ready)
    {
        s_motor_start_cycle = DWT->CYCCNT;
    }
}

void FlightRealtime_MotorEnd(void)
{
    uint32 elapsed;

    if(!s_realtime_ready)
    {
        return;
    }
    elapsed = DWT->CYCCNT - s_motor_start_cycle;
    s_motor_last_cycles = elapsed;
    if(elapsed > s_motor_max_cycles)
    {
        s_motor_max_cycles = elapsed;
    }
    if(elapsed > flight_realtime_us_to_cycles(MOTOR_PERIOD_US))
    {
        s_motor_execution_overrun_count++;
    }
    s_motor_count++;
}

void FlightRealtime_GetDiag(FlightRealtimeDiag *diag)
{
    uint32 irq_state;
    uint32 control_last_cycles;
    uint32 control_max_cycles;
    uint32 control_interval_min_cycles;
    uint32 control_interval_max_cycles;
    uint32 motor_last_cycles;
    uint32 motor_max_cycles;

    if(diag == NULL)
    {
        return;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    control_last_cycles = s_control_last_cycles;
    control_max_cycles = s_control_max_cycles;
    control_interval_min_cycles = s_control_interval_min_cycles;
    control_interval_max_cycles = s_control_interval_max_cycles;
    motor_last_cycles = s_motor_last_cycles;
    motor_max_cycles = s_motor_max_cycles;
    diag->control_count = s_control_count;
    diag->control_execution_overrun_count = s_control_execution_overrun_count;
    diag->control_delayed_entry_count = s_control_delayed_entry_count;
    diag->motor_count = s_motor_count;
    diag->motor_execution_overrun_count = s_motor_execution_overrun_count;
    diag->rate_slot_count = s_rate_slot_count;
    diag->angle_slot_count = s_angle_slot_count;
    diag->velocity_slot_count = s_velocity_slot_count;
    diag->position_slot_count = s_position_slot_count;
    if(irq_state == 0U)
    {
        __enable_irq();
    }

    diag->control_last_us = flight_realtime_cycles_to_us(control_last_cycles);
    diag->control_max_us = flight_realtime_cycles_to_us(control_max_cycles);
    diag->control_interval_min_us = (control_interval_min_cycles == 0xFFFFFFFFU)
                                      ? 0.0f
                                      : flight_realtime_cycles_to_us(control_interval_min_cycles);
    diag->control_interval_max_us = flight_realtime_cycles_to_us(control_interval_max_cycles);
    diag->motor_last_us = flight_realtime_cycles_to_us(motor_last_cycles);
    diag->motor_max_us = flight_realtime_cycles_to_us(motor_max_cycles);
}

// ==================== Motor output and motor-test state ====================

// ==================== Motor output helpers ====================

static void clear_motor_throttle_state(void)
{
    motor_throttle[0] = 0U;
    motor_throttle[1] = 0U;
    motor_throttle[2] = 0U;
    motor_throttle[3] = 0U;
}

void PIT_Demo_Safety_Reset_Angle_Rate_Cmd_Lpf(void)
{
    Angle_Loop_Reset_Rate_Cmd_Lpf();
}

void PIT_Demo_Safety_Clear_Motor_Throttle_State(void)
{
    clear_motor_throttle_state();
}

// ==================== Safety and motor mixing helpers ====================

static uint8 check_attitude_safety(void)
{
    return Flight_Safety_Check_Attitude();
}

static uint8 check_fly_test_attitude_safety(void)
{
    return (uint8)(!isfinite(current_roll) || !isfinite(current_pitch) ||
                   (current_roll > FLY_TEST_ATTITUDE_SAFE_DEG) ||
                   (current_roll < -FLY_TEST_ATTITUDE_SAFE_DEG) ||
                   (current_pitch > FLY_TEST_ATTITUDE_SAFE_DEG) ||
                   (current_pitch < -FLY_TEST_ATTITUDE_SAFE_DEG));
}

static float limit_fly_test_attitude_output(float output)
{
    if(output > FLY_TEST_ATTITUDE_OUTPUT_LIMIT)
    {
        output = FLY_TEST_ATTITUDE_OUTPUT_LIMIT;
    }
    else if(output < -FLY_TEST_ATTITUDE_OUTPUT_LIMIT)
    {
        output = -FLY_TEST_ATTITUDE_OUTPUT_LIMIT;
    }

    return output;
}

// ==================== Flight runtime public interface ====================

void Flight_Runtime_Reset_All_Throttle_States(void)
{
    Throttle_Runtime_Reset_Throttle_State();
    MenuMotorTest_SetOutputEnable(0U);
    MenuMotorTest_SetThrottle(0U);
    MenuFlyTest_Stop();
    Angle_Loop_Reset_Rate_Cmd_Lpf();
    Rate_Loop_Reset_Output();
    clear_motor_throttle_state();
}

void Flight_Motor_Test_Start(void)
{
    Flight_Fly_Test_Stop();
    Throttle_Runtime_Reset_Throttle_State();
    MenuMotorTest_Start();
    clear_motor_throttle_state();
}

void Flight_Motor_Test_Stop(void)
{
    MenuMotorTest_Stop();
    Throttle_Runtime_Reset_Throttle_State();
    clear_motor_throttle_state();
}

uint8 Flight_Motor_Test_Is_Active(void)
{
    return MenuMotorTest_IsActive();
}

void Flight_Motor_Test_Set_Selected(uint8 motor_index)
{
    MenuMotorTest_SetSelected(motor_index);
}

uint8 Flight_Motor_Test_Get_Selected(void)
{
    return MenuMotorTest_GetSelected();
}

void Flight_Motor_Test_Set_Throttle(uint16 throttle)
{
    MenuMotorTest_SetThrottle(throttle);
}

uint16 Flight_Motor_Test_Get_Throttle(void)
{
    return MenuMotorTest_GetThrottle();
}

void Flight_Motor_Test_Set_Output_Enable(uint8 enable)
{
    MenuMotorTest_SetOutputEnable(enable);
}

uint8 Flight_Motor_Test_Get_Output_Enable(void)
{
    return MenuMotorTest_GetOutputEnable();
}

uint16 Flight_Motor_Test_Get_Output_Limit(void)
{
    return MenuMotorTest_GetOutputLimit();
}

void Flight_Fly_Test_Start(void)
{
    MenuMotorTest_Stop();
    Flight_Takeoff_AbortAt("FLY TEST START", __FILE__, __LINE__);
    Velocity_Loop_Set_Enabled(0U);
    Position_Loop_Set_Enabled(0U);
    Throttle_Runtime_Reset_Throttle_State();
    Angle_Loop_Reset_Rate_Cmd_Lpf();
    Rate_Loop_Reset_Output();
    PID_Reset(&angle_pid_roll);
    PID_Reset(&angle_pid_pitch);
    PID_Reset(&angle_pid_yaw);
    PID_Reset(&gyro_pid_roll);
    PID_Reset(&gyro_pid_pitch);
    PID_Reset(&gyro_pid_yaw);
    clear_motor_throttle_state();
    MenuFlyTest_Start();
}

void Flight_Fly_Test_Stop(void)
{
    MenuFlyTest_SetOutputEnable(0U);
    MenuFlyTest_SetConfirmState(FLY_TEST_CONFIRM_IDLE);
    MenuFlyTest_Stop();
    Angle_Loop_Reset_Rate_Cmd_Lpf();
    Rate_Loop_Reset_Output();
    PID_Reset(&angle_pid_roll);
    PID_Reset(&angle_pid_pitch);
    PID_Reset(&angle_pid_yaw);
    PID_Reset(&gyro_pid_roll);
    PID_Reset(&gyro_pid_pitch);
    PID_Reset(&gyro_pid_yaw);
    clear_motor_throttle_state();
}

uint8 Flight_Fly_Test_Is_Active(void)
{
    return MenuFlyTest_IsActive();
}

void Flight_Fly_Test_Set_Throttle(uint16 throttle)
{
    MenuFlyTest_SetThrottle(throttle);
}

uint16 Flight_Fly_Test_Get_Throttle(void)
{
    return MenuFlyTest_GetThrottle();
}

void Flight_Fly_Test_Set_Output_Enable(uint8 enable)
{
    if(!enable)
    {
        MenuFlyTest_SetOutputEnable(0U);
        MenuFlyTest_SetConfirmState(FLY_TEST_CONFIRM_IDLE);
        Angle_Loop_Reset_Rate_Cmd_Lpf();
        Rate_Loop_Reset_Output();
        clear_motor_throttle_state();
        return;
    }

    if(emergency_stop_triggered)
    {
        MenuFlyTest_SetOutputEnable(0U);
        MenuFlyTest_SetConfirmState(FLY_TEST_CONFIRM_IDLE);
        clear_motor_throttle_state();
        return;
    }

    if(check_fly_test_attitude_safety())
    {
        MenuFlyTest_SetSafetyLatched(1U);
        MenuFlyTest_SetOutputEnable(0U);
        MenuFlyTest_SetConfirmState(FLY_TEST_CONFIRM_IDLE);
        clear_motor_throttle_state();
        return;
    }

    MenuFlyTest_ClearSafetyLatch();
    MenuFlyTest_CaptureTarget(current_roll, current_pitch, current_yaw);
    Angle_Loop_Reset_Rate_Cmd_Lpf();
    Rate_Loop_Reset_Output();
    PID_Reset(&angle_pid_roll);
    PID_Reset(&angle_pid_pitch);
    PID_Reset(&angle_pid_yaw);
    PID_Reset(&gyro_pid_roll);
    PID_Reset(&gyro_pid_pitch);
    PID_Reset(&gyro_pid_yaw);
    clear_motor_throttle_state();
    MenuFlyTest_SetOutputEnable(1U);
    MenuFlyTest_SetConfirmState(FLY_TEST_CONFIRM_ON);
}

uint8 Flight_Fly_Test_Get_Output_Enable(void)
{
    return MenuFlyTest_GetOutputEnable();
}

void Flight_Fly_Test_Get_Target(float *roll, float *pitch, float *yaw)
{
    MenuFlyTest_GetTarget(roll, pitch, yaw);
}

uint8 Flight_Fly_Test_Get_Safety_Latched(void)
{
    return MenuFlyTest_GetSafetyLatched();
}

void Flight_Fly_Test_Clear_Safety_Latch(void)
{
    MenuFlyTest_ClearSafetyLatch();
}

void Flight_Fly_Test_Set_Confirm_State(uint8 state)
{
    MenuFlyTest_SetConfirmState(state);
}

uint8 Flight_Fly_Test_Get_Confirm_State(void)
{
    return MenuFlyTest_GetConfirmState();
}

// ==================== PIT periodic tasks ====================

void PIT_KeyVoltage_Task_5ms(void)
{
    BoardBuzzer_Task5ms();
    Flight_Safety_Tick_5ms();
    VehicleMarkerLocalization_Task5ms();
    key_scanner();
    key_scan();
    BatteryMonitor_Task_5ms();
}

void PIT_Adc_Task_5ms(void)
{
    GroundRemoteInput_Task_5ms();
}

// ==================== Optical-flow feed task ====================

void PIT_OpticalFlow_Task(void)
{
    OpticalFlowFeed_Task();
}

void PIT_PMW3901_Task_20ms(void)
{
    PIT_OpticalFlow_Task();
}

// ==================== 4 kHz control-loop task ====================

static uint8 control_task_handle_imu_calibration(void)
{
    if(icm_calibration_flag)
    {
        MenuMotorTest_SetOutputEnable(0U);
        MenuFlyTest_SetOutputEnable(0U);
        Angle_Loop_Reset_Rate_Cmd_Lpf();
        Rate_Loop_Reset_Output();
        clear_motor_throttle_state();
        return 1U;
    }

    return 0U;
}

static void control_task_update_sensor_feedback(void)
{
    get_ICM_data();
    ICM_VisionYaw_Update(g_vehicle_marker_pose.heading_deg,
                         (uint8)(g_vehicle_marker_pose.valid &&
                                 !g_vehicle_marker_position_lost_flag &&
                                 g_vehicle_marker_pose.detected_point_count >= 3U),
                         g_vehicle_marker_pose_update_seq);
    Get_Attitude_Angle(&current_roll, &current_pitch, &current_yaw);
    Get_Angular_Rate(&current_gyro_roll, &current_gyro_pitch, &current_gyro_yaw);
    PMW_Flow_AccumulateGyro(icm42688_gyro_x, icm42688_gyro_y, CONTROL_DT_S);
    PMW_Flow_PredictVelocity(X_V_F, Y_V_F, CONTROL_DT_S);
}

static uint8 control_task_handle_attitude_safety(void)
{
    if(check_attitude_safety())
    {
        Angle_Loop_Reset_Rate_Cmd_Lpf();
        return 1U;
    }

    return 0U;
}

static uint8 control_task_handle_motor_test(uint8 rate_due)
{
    if(!MenuMotorTest_IsActive())
    {
        return 0U;
    }

    if(emergency_stop_triggered)
    {
        MenuMotorTest_SetOutputEnable(0U);
        Angle_Loop_Reset_Rate_Cmd_Lpf();
        Rate_Loop_Reset_Output();
        clear_motor_throttle_state();
        return 1U;
    }

    Angle_Loop_Reset_Rate_Cmd_Lpf();
    Rate_Loop_Reset_Output();
    if(rate_due)
    {
        MenuMotorTest_ApplyOutput(motor_throttle);
    }
    return 1U;
}

static uint8 control_task_handle_fly_test(uint8 angle_due, uint8 rate_due)
{
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw = 0.0f;

    if(!MenuFlyTest_IsActive())
    {
        return 0U;
    }

    if(emergency_stop_triggered)
    {
        MenuFlyTest_SetOutputEnable(0U);
        MenuFlyTest_SetConfirmState(FLY_TEST_CONFIRM_IDLE);
        Angle_Loop_Reset_Rate_Cmd_Lpf();
        Rate_Loop_Reset_Output();
        clear_motor_throttle_state();
        return 1U;
    }

    if(!MenuFlyTest_GetOutputEnable())
    {
        if(MenuFlyTest_GetConfirmState() == FLY_TEST_CONFIRM_ON)
        {
            MenuFlyTest_SetConfirmState(FLY_TEST_CONFIRM_IDLE);
        }
        Angle_Loop_Reset_Rate_Cmd_Lpf();
        Rate_Loop_Reset_Output();
        clear_motor_throttle_state();
        return 1U;
    }

    if(check_fly_test_attitude_safety())
    {
        MenuFlyTest_SetSafetyLatched(1U);
        MenuFlyTest_SetOutputEnable(0U);
        MenuFlyTest_SetConfirmState(FLY_TEST_CONFIRM_IDLE);
        Angle_Loop_Reset_Rate_Cmd_Lpf();
        Rate_Loop_Reset_Output();
        clear_motor_throttle_state();
        return 1U;
    }

    if(angle_due)
    {
        MenuFlyTest_GetTarget(&target_roll, &target_pitch, &target_yaw);
        Angle_Loop_Update(target_roll,
                          target_pitch,
                          target_yaw,
                          current_roll,
                          current_pitch,
                          current_yaw);
        target_yaw_rate = 0.0f;
        PID_Reset(&angle_pid_yaw);
    }

    if(rate_due)
    {
        Rate_Loop_Update(target_roll_rate,
                         target_pitch_rate,
                         0.0f,
                         current_gyro_roll,
                         current_gyro_pitch,
                         current_gyro_yaw);
        PID_Reset(&gyro_pid_yaw);

        roll_output = limit_fly_test_attitude_output(roll_output);
        pitch_output = limit_fly_test_attitude_output(pitch_output);
        yaw_output = 0.0f;

        Motor_Mixer_Update_BaseOnly(motor_throttle,
                                    1U,
                                    (float)MenuFlyTest_GetThrottle(),
                                    current_roll,
                                    current_pitch);
    }
    return 1U;
}

static uint8 control_task_handle_disarmed(void)
{
    if(!armed)
    {
        Angle_Loop_Reset_Rate_Cmd_Lpf();
        clear_motor_throttle_state();
        return 1U;
    }

    return 0U;
}

static uint8 control_task_handle_takeoff_base_only(void)
{
    uint16 equal_output;

    if(!Flight_Takeoff_Requires_BaseOnly_Output())
    {
        return 0U;
    }

    /* 零等待、软起转和怠速保持阶段禁止姿态差动与高度 PID 参与。 */
    Angle_Loop_Reset_Rate_Cmd_Lpf();
    Rate_Loop_Reset_Output();
    PID_Reset(&angle_pid_roll);
    PID_Reset(&angle_pid_pitch);
    PID_Reset(&angle_pid_yaw);
    PID_Reset(&gyro_pid_roll);
    PID_Reset(&gyro_pid_pitch);
    PID_Reset(&gyro_pid_yaw);
    if(Velocity_Loop_Is_Enabled())
    {
        Velocity_Loop_Set_Enabled(0U);
    }
    if(Position_Loop_Is_Enabled())
    {
        Position_Loop_Set_Enabled(0U);
    }

    equal_output = Motor_Mixer_Limit_Output(Flight_Takeoff_Get_Commanded_Base_Throttle());
    motor_throttle[0] = equal_output;
    motor_throttle[1] = equal_output;
    motor_throttle[2] = equal_output;
    motor_throttle[3] = equal_output;
    return 1U;
}

static void control_task_update_cruise_throttle(uint8 takeoff_active)
{
    if(!takeoff_active)
    {
        Throttle_Runtime_Ramp(target_throttle, &current_throttle, THROTTLE_RAMP_RATE, CONTROL_DT_MS);
        base_throttle = current_throttle;
    }
}

static uint8 control_task_handle_manual_mode(void)
{
    uint16 manual_output;

    if(control_mode != CONTROL_MODE_MANUAL)
    {
        return 0U;
    }

    if(!isfinite(base_throttle))
    {
        Flight_Runtime_Emergency_StopAt(EMERGENCY_STOP_REASON_INVALID_CONTROL,
                                        "BASE THROTTLE NAN",
                                        __FILE__,
                                        __LINE__);
        return 1U;
    }

    if((base_throttle > 0.0f) && (base_throttle < SAFE_MIN_THROTTLE))
    {
        manual_output = (uint16)SAFE_MIN_THROTTLE;
    }
    else
    {
        manual_output = Motor_Mixer_Limit_Output(base_throttle);
    }
    if(Position_Loop_Is_Enabled())
    {
        Position_Loop_Set_Enabled(0U);
    }
    if(Velocity_Loop_Is_Enabled())
    {
        Velocity_Loop_Set_Enabled(0U);
    }
    Velocity_Loop_Set_Target_Velocity(0.0f, 0.0f);
    Angle_Loop_Reset_Rate_Cmd_Lpf();
    motor_throttle[0] = manual_output;
    motor_throttle[1] = manual_output;
    motor_throttle[2] = manual_output;
    motor_throttle[3] = manual_output;
    return 1U;
}

static void control_task_prepare_angle_mode(uint8 takeoff_active)
{
    if(takeoff_active)
    {
        return;
    }

    if(!Velocity_Loop_Is_Enabled())
    {
        Velocity_Loop_Set_Enabled(1U);
        Velocity_Loop_Reset_Runtime();
        Position_Loop_Set_Enabled(1U);
    }
}

static void control_task_update_position_target(uint8 takeoff_active)
{
    float position_target_vx_cms = 0.0f;
    float position_target_vy_cms = 0.0f;
    PMWFlowState_t flow_snap;

    if(takeoff_active)
    {
        return;
    }

    PMW_Flow_GetState(&flow_snap);
    Position_Loop_Update_Velocity_Target(flow_snap.fused_velocity_valid,
                                         flow_snap.velocity_x_fused_cms,
                                         flow_snap.velocity_y_fused_cms,
                                         &position_target_vx_cms,
                                         &position_target_vy_cms);
    Velocity_Loop_Set_Target_Velocity(position_target_vx_cms, position_target_vy_cms);

    pos_fusion_r_adc    = 0.0f;
    pos_fusion_r_tof    = 0.0f;
    pos_fusion_r_fused  = 0.0f;
    pos_fusion_residual = 0.0f;
    pos_fusion_fault    = 0U;
}

static void control_task_update_velocity_target(uint8 takeoff_active)
{
    float captured_roll = 0.0f;
    float captured_pitch = 0.0f;
    float velocity_target_roll = target_angle_roll;
    float velocity_target_pitch = target_angle_pitch;
    PMWFlowState_t flow_snap;

    if(takeoff_active)
    {
        return;
    }

    Flight_Takeoff_Get_Captured_Angles(&captured_roll, &captured_pitch);
    PMW_Flow_GetState(&flow_snap);
    Velocity_Loop_Update_Attitude_Target(flow_snap.fused_velocity_valid,
                                         flow_snap.velocity_x_fused_cms,
                                         flow_snap.velocity_y_fused_cms,
                                         captured_roll,
                                         captured_pitch,
                                         &velocity_target_roll,
                                         &velocity_target_pitch);
    target_angle_roll = velocity_target_roll;
    target_angle_pitch = velocity_target_pitch;
}

static void control_task_update_angle_loop(void)
{
    Angle_Loop_Update(target_angle_roll,
                      target_angle_pitch,
                      target_angle_yaw,
                      current_roll,
                      current_pitch,
                      current_yaw);
}

static void control_task_update_rate_and_mixer(void)
{
    Rate_Loop_Update(target_roll_rate,
                     target_pitch_rate,
                     target_yaw_rate,
                     current_gyro_roll,
                     current_gyro_pitch,
                     current_gyro_yaw);

    Motor_Mixer_Update(motor_throttle,
                       armed,
                       base_throttle,
                       current_roll,
                       current_pitch);
    if(Motor_Mixer_HasInvalidOutput())
    {
        Flight_Runtime_Emergency_StopAt(EMERGENCY_STOP_REASON_INVALID_CONTROL,
                                        "MIXER OUTPUT NAN",
                                        __FILE__,
                                        __LINE__);
    }
}

static void control_scheduler_begin_tick(uint8 *rate_due,
                                         uint8 *angle_due,
                                         uint8 *velocity_due,
                                         uint8 *position_due)
{
    uint8 tick = s_control_scheduler_tick;

    *rate_due = (uint8)((tick % RATE_SLOT_DIVISOR) == 0U);
    *angle_due = (uint8)((tick % ANGLE_SLOT_DIVISOR) == ANGLE_SLOT_PHASE);
    *velocity_due = (uint8)((tick % VELOCITY_SLOT_DIVISOR) == VELOCITY_SLOT_PHASE);
    *position_due = (uint8)(tick == POSITION_SLOT_PHASE);

    if(*rate_due) s_rate_slot_count++;
    if(*angle_due) s_angle_slot_count++;
    if(*velocity_due) s_velocity_slot_count++;
    if(*position_due) s_position_slot_count++;

    tick++;
    s_control_scheduler_tick = (tick >= CONTROL_SCHEDULER_TICKS) ? 0U : tick;
}

static void control_task_submit_motor_output(void)
{
    uint8 output_enabled = (uint8)(MenuMotorTest_GetOutputEnable() || MenuFlyTest_GetOutputEnable());

    FlightRealtime_MotorBegin();
    MotorHardware_WriteOutput(motor_throttle,
                              armed,
                              output_enabled,
                              MenuMotorTest_IsActive());
    FlightRealtime_MotorEnd();
}

static void control_task_finish_cycle(uint8 rate_due)
{
    if(emergency_stop_triggered)
    {
        clear_motor_throttle_state();
        MotorHardware_ServiceEmergencyStop();
        return;
    }

    if(rate_due)
    {
        control_task_submit_motor_output();
    }
}

void PIT_Control_Task_250us(void)
{
    uint8 takeoff_active = 0U;
    uint8 rate_due;
    uint8 angle_due;
    uint8 velocity_due;
    uint8 position_due;

    MotorHardware_InitOnce();
    control_scheduler_begin_tick(&rate_due, &angle_due, &velocity_due, &position_due);

    if(Flight_Safety_Check_Esc())
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    if(control_task_handle_motor_test(rate_due))
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    if(control_task_handle_imu_calibration())
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    control_task_update_sensor_feedback();

    if(control_task_handle_attitude_safety())
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    if(emergency_stop_triggered)
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    takeoff_active = Flight_Takeoff_Tick_250us();

    if(control_task_handle_fly_test(angle_due, rate_due))
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    if(control_task_handle_disarmed())
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    if(control_task_handle_takeoff_base_only())
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    control_task_update_cruise_throttle(takeoff_active);

    if(control_task_handle_manual_mode())
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    control_task_prepare_angle_mode(takeoff_active);
    if(position_due)
    {
        control_task_update_position_target(takeoff_active);
    }
    if(velocity_due)
    {
        control_task_update_velocity_target(takeoff_active);
    }
    if(angle_due)
    {
        control_task_update_angle_loop();
    }
    if(rate_due)
    {
        control_task_update_rate_and_mixer();
    }
    control_task_finish_cycle(rate_due);
}
