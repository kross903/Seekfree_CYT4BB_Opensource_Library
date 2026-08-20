#include "common/PIT_Demo.h"
#include "common/battery_monitor.h"
#include "common/project_headfile.h"
#include "control/angle_loop.h"
#include "control/ground_remote_input.h"
#include "control/height_manager.h"
#include "control/motor_mixer.h"
#include "control/position_loop.h"
#include "control/rate_loop.h"
#include "control/throttle_runtime.h"
#include "control/velocity_loop.h"
#include "menu/menu_fly_test.h"
#include "menu/menu_motor_test.h"
#include "motor_hardware/motor_hardware.h"
#include "pmw/optical_flow_feed.h"
#include "pmw/pmw_flow.h"
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
#define HORIZONTAL_DT_DEFAULT_S        0.004f
#define HORIZONTAL_DT_MIN_S            0.002f
#define HORIZONTAL_DT_MAX_S            0.008f
#define HORIZONTAL_TIMING_RESET_COUNT  3U

/* 故障保护降落：故障状态下不再运行水平跟踪，只保留可用的姿态/高度链路。 */
#define FAULT_LANDING_ATTITUDE_RATE_DPS  (15.0f)
#define FAULT_LANDING_THROTTLE_RATE      (0.08f) /* 油门单位/ms，即80单位/s */

typedef enum
{
    FAULT_LANDING_IDLE = 0,
    FAULT_LANDING_TOF,
    FAULT_LANDING_NO_TOF
} FaultLandingMode_t;

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
static uint32 s_horizontal_last_cycle = 0U;
static float s_horizontal_dt_s = HORIZONTAL_DT_DEFAULT_S;
static uint8 s_horizontal_timing_initialized = 0U;
static uint8 s_horizontal_timing_valid = 1U;
static uint8 s_horizontal_timing_error_count = 0U;
static uint8 s_horizontal_reset_reason = POSITION_LOOP_RESET_NONE;

static volatile FaultLandingMode_t s_fault_landing_mode = FAULT_LANDING_IDLE;
static volatile float s_fault_landing_yaw_deg = 0.0f;
static volatile float s_fault_landing_throttle = 0.0f;

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

static void control_task_reset_horizontal_timing(uint8 reason)
{
    s_horizontal_last_cycle = 0U;
    s_horizontal_dt_s = HORIZONTAL_DT_DEFAULT_S;
    s_horizontal_timing_initialized = 0U;
    s_horizontal_timing_valid = 1U;
    s_horizontal_timing_error_count = 0U;
    if(reason != s_horizontal_reset_reason)
    {
        Position_Loop_Reset_Trajectory(reason);
        Velocity_Loop_Reset_Runtime();
        s_horizontal_reset_reason = reason;
    }
}

static void control_task_update_horizontal_timing(void)
{
    uint32 now_cycle;
    float dt_s;

    flight_realtime_init();
    if(!s_realtime_ready || (SystemCoreClock == 0U))
    {
        s_horizontal_dt_s = HORIZONTAL_DT_DEFAULT_S;
        s_horizontal_timing_valid = 1U;
        return;
    }

    now_cycle = DWT->CYCCNT;
    if(!s_horizontal_timing_initialized)
    {
        s_horizontal_last_cycle = now_cycle;
        s_horizontal_dt_s = HORIZONTAL_DT_DEFAULT_S;
        s_horizontal_timing_initialized = 1U;
        s_horizontal_timing_valid = 1U;
        s_horizontal_timing_error_count = 0U;
        s_horizontal_reset_reason = POSITION_LOOP_RESET_NONE;
        return;
    }

    dt_s = (float)(now_cycle - s_horizontal_last_cycle) /
           (float)SystemCoreClock;
    s_horizontal_last_cycle = now_cycle;
    s_horizontal_dt_s = dt_s;
    if(isfinite(dt_s) && (dt_s >= HORIZONTAL_DT_MIN_S) &&
       (dt_s <= HORIZONTAL_DT_MAX_S))
    {
        s_horizontal_timing_valid = 1U;
        s_horizontal_timing_error_count = 0U;
        s_horizontal_reset_reason = POSITION_LOOP_RESET_NONE;
        return;
    }

    s_horizontal_timing_valid = 0U;
    if(s_horizontal_timing_error_count < 0xFFU)
    {
        s_horizontal_timing_error_count++;
    }
    if(s_horizontal_timing_error_count == HORIZONTAL_TIMING_RESET_COUNT)
    {
        Position_Loop_Reset_Trajectory(POSITION_LOOP_RESET_TIMING_INVALID);
        Velocity_Loop_Reset_Runtime();
        s_horizontal_reset_reason = POSITION_LOOP_RESET_TIMING_INVALID;
    }
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
    Motor_Mixer_Reset_State();
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

static float fault_landing_move_to_zero(float value, float max_step)
{
    if(value > max_step)
    {
        return value - max_step;
    }
    if(value < -max_step)
    {
        return value + max_step;
    }
    return 0.0f;
}

static void control_task_start_fault_landing(uint8 tof_valid)
{
    uint8 auto_landing_started = 0U;

    if(s_fault_landing_mode != FAULT_LANDING_IDLE)
    {
        return;
    }

    /* 先切断水平任务和小车前馈，避免下一次正常调度重新接管。 */
    Position_Loop_Set_Enabled(0U);
    Velocity_Loop_Set_Enabled(0U);
    Position_Loop_Reset_Car_Velocity_Feedforward();
    Velocity_Loop_Set_Target_State(0.0f, 0.0f, 0.0f, 0.0f);
    control_task_reset_horizontal_timing(POSITION_LOOP_RESET_CONTROL_DISABLED);

    target_angle_roll = isfinite(current_roll) ? current_roll : 0.0f;
    target_angle_pitch = isfinite(current_pitch) ? current_pitch : 0.0f;
    s_fault_landing_yaw_deg = isfinite(current_yaw) ?
                              Angle_WrapDeg180(current_yaw) : 0.0f;
    target_angle_yaw = s_fault_landing_yaw_deg;
    target_yaw_rate = 0.0f;
    Angle_Loop_Reset_Rate_Cmd_Lpf();
    Rate_Loop_Reset_Output();
    HeightSpeedPID_output = 0.0f;
    Flight_Takeoff_AbortAt("FAULT LANDING", __FILE__, __LINE__);

    if(tof_valid)
    {
        auto_landing_started = HeightManager_BeginAutoLanding();
    }
    if(auto_landing_started)
    {
        s_fault_landing_mode = FAULT_LANDING_TOF;
    }
    else
    {
        /* ToF不可用时不伪造高度反馈，只保留姿态环和基础油门斜坡。 */
        s_fault_landing_mode = FAULT_LANDING_NO_TOF;
        s_fault_landing_yaw_deg = target_angle_yaw;
        s_fault_landing_throttle = base_throttle;
        if(current_throttle > s_fault_landing_throttle)
        {
            s_fault_landing_throttle = current_throttle;
        }
        target_throttle = 0.0f;
        current_throttle = Throttle_Runtime_Limit(s_fault_landing_throttle);
        base_throttle = current_throttle;
    }
}

static void control_task_update_fault_landing(uint8 angle_due, uint8 rate_due)
{
    if(!armed)
    {
        s_fault_landing_mode = FAULT_LANDING_IDLE;
        return;
    }

    if((s_fault_landing_mode == FAULT_LANDING_TOF) &&
       !HeightManager_IsTofValid())
    {
        HeightManager_EndAutoLanding();
        s_fault_landing_mode = FAULT_LANDING_NO_TOF;
        s_fault_landing_throttle = base_throttle;
        if(current_throttle > s_fault_landing_throttle)
        {
            s_fault_landing_throttle = current_throttle;
        }
        HeightSpeedPID_output = 0.0f;
    }

    target_angle_roll = fault_landing_move_to_zero(target_angle_roll,
                                                    FAULT_LANDING_ATTITUDE_RATE_DPS * CONTROL_DT_S);
    target_angle_pitch = fault_landing_move_to_zero(target_angle_pitch,
                                                     FAULT_LANDING_ATTITUDE_RATE_DPS * CONTROL_DT_S);
    target_angle_yaw = s_fault_landing_yaw_deg;
    target_yaw_rate = 0.0f;

    if(s_fault_landing_mode == FAULT_LANDING_NO_TOF)
    {
        target_throttle = 0.0f;
        Throttle_Runtime_Ramp(0.0f,
                              &current_throttle,
                              FAULT_LANDING_THROTTLE_RATE,
                              CONTROL_DT_MS);
        base_throttle = current_throttle;
        HeightSpeedPID_output = 0.0f;
    }

    if(angle_due)
    {
        Angle_Loop_Update(target_angle_roll,
                          target_angle_pitch,
                          target_angle_yaw,
                          current_roll,
                          current_pitch,
                          current_yaw);
    }
    if(rate_due)
    {
        Rate_Loop_Update(target_roll_rate,
                         target_pitch_rate,
                         0.0f,
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
                                            "FAULT LAND MIXER NAN",
                                            __FILE__,
                                            __LINE__);
        }
    }

    if((s_fault_landing_mode == FAULT_LANDING_TOF) &&
       HeightManager_AutoLandingTouchdownConfirmed())
    {
        Flight_Control_Disarm();
        s_fault_landing_mode = FAULT_LANDING_IDLE;
    }
    else if((s_fault_landing_mode == FAULT_LANDING_NO_TOF) &&
            (current_throttle <= 0.0f))
    {
        Flight_Control_Disarm();
        s_fault_landing_mode = FAULT_LANDING_IDLE;
    }
}

static uint8 control_task_handle_flow_safety(void)
{
    PMWFlowState_t flow_state = {0};

    if(!armed)
    {
        return 0U;
    }

    PMW_Flow_GetState(&flow_state);
    return flow_state.flow_fault;
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
    s_fault_landing_mode = FAULT_LANDING_IDLE;
    s_fault_landing_yaw_deg = 0.0f;
    s_fault_landing_throttle = 0.0f;
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
    PMW_Flow_AccumulateGyro(icm42688_gyro_x,
                            icm42688_gyro_y,
                            icm42688_gyro_z,
                            current_yaw,
                            CONTROL_DT_S);
    PMW_Flow_PredictVelocity(Horizontal_Accel_X_Cms2,
                             Horizontal_Accel_Y_Cms2,
                             current_yaw,
                             Horizontal_Accel_Valid,
                             CONTROL_DT_S);
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
        control_task_reset_horizontal_timing(POSITION_LOOP_RESET_DISARMED);
        Angle_Loop_Reset_Rate_Cmd_Lpf();
        clear_motor_throttle_state();
        return 1U;
    }

    return 0U;
}

static uint8 control_task_handle_auto_land_disarm(void)
{
    if(!BlindBoxTasks_TakeDisarmRequest())
    {
        return 0U;
    }

    Flight_Control_Disarm();
    clear_motor_throttle_state();
    return 1U;
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
    control_task_reset_horizontal_timing(POSITION_LOOP_RESET_TAKEOFF);

    equal_output = Motor_Mixer_Limit_Output(Flight_Takeoff_Get_Commanded_Base_Throttle());
    motor_throttle[0] = equal_output;
    motor_throttle[1] = equal_output;
    motor_throttle[2] = equal_output;
    motor_throttle[3] = equal_output;
    return 1U;
}

static void control_task_update_cruise_throttle(uint8 takeoff_active)
{
    float landing_throttle;

    if(!takeoff_active)
    {
        Throttle_Runtime_Ramp(target_throttle, &current_throttle, THROTTLE_RAMP_RATE, CONTROL_DT_MS);
        base_throttle = current_throttle;
    }
    if(BlindBoxTasks_GetThrottleOverride(&landing_throttle))
    {
        target_throttle = landing_throttle;
        current_throttle = landing_throttle;
        base_throttle = landing_throttle;
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
    control_task_reset_horizontal_timing(POSITION_LOOP_RESET_MANUAL_MODE);
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
    float position_target_ax_cms2 = 0.0f;
    float position_target_ay_cms2 = 0.0f;
    CoreLinkCarMotionDiag_t car_motion = {0};
    PositionLoopFeedforwardInput_t feedforward = {0};
    PMWFlowState_t flow_snap;

    if(takeoff_active)
    {
        control_task_reset_horizontal_timing(POSITION_LOOP_RESET_TAKEOFF);
        Position_Loop_Reset_Car_Velocity_Feedforward();
        return;
    }

    PMW_Flow_GetState(&flow_snap);
    core_link_user_get_car_motion_diag(&car_motion);
    if(car_motion.valid)
    {
        /* V3已经使用双方对齐后的共享世界系，禁止再次按Yaw旋转。 */
        feedforward.velocity_valid = 1U;
        feedforward.earth_frame = 1U;
        feedforward.vx_cms = car_motion.world_vx_cms;
        feedforward.vy_cms = car_motion.world_vy_cms;
        if(car_motion.acceleration_valid)
        {
            feedforward.acceleration_valid = 1U;
            feedforward.ax_cms2 = car_motion.world_ax_cms2;
            feedforward.ay_cms2 = car_motion.world_ay_cms2;
        }
    }
    Position_Loop_Update_Velocity_Target(flow_snap.fused_velocity_valid,
                                         flow_snap.velocity_x_fused_cms,
                                         flow_snap.velocity_y_fused_cms,
                                         current_yaw,
                                         s_horizontal_dt_s,
                                         s_horizontal_timing_valid,
                                         &feedforward,
                                         &position_target_vx_cms,
                                         &position_target_vy_cms,
                                         &position_target_ax_cms2,
                                         &position_target_ay_cms2);
    Velocity_Loop_Set_Target_State(position_target_vx_cms,
                                   position_target_vy_cms,
                                   position_target_ax_cms2,
                                   position_target_ay_cms2);

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
    if(!s_horizontal_timing_valid)
    {
        return;
    }

    Flight_Takeoff_Get_Captured_Angles(&captured_roll, &captured_pitch);
    PMW_Flow_GetState(&flow_snap);
    Velocity_Loop_Update_Attitude_Target(flow_snap.fused_velocity_valid,
                                         flow_snap.velocity_x_fused_cms,
                                         flow_snap.velocity_y_fused_cms,
                                         current_yaw,
                                         s_horizontal_dt_s,
                                         captured_roll,
                                         captured_pitch,
                                         &velocity_target_roll,
                                         &velocity_target_pitch);
    target_angle_roll = velocity_target_roll;
    target_angle_pitch = velocity_target_pitch;
}

static void control_task_update_angle_loop(void)
{
    BlindBoxTasks_ControlStep(0.002f, current_yaw, &target_angle_yaw);
    BeaconSearch_ControlStep(0.002f, current_yaw, &target_angle_yaw);
    Angle_Loop_Update(target_angle_roll,
                      target_angle_pitch,
                      target_angle_yaw,
                      current_roll,
                      current_pitch,
                      current_yaw);
    target_yaw_rate = BeaconSearch_LimitYawRate(target_yaw_rate);
    target_yaw_rate = BlindBoxTasks_LimitYawRate(target_yaw_rate);
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
    /* 水平位置轨迹与速度控制统一在250 Hz同一节拍更新。 */
    *position_due = *velocity_due;

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
        control_task_reset_horizontal_timing(POSITION_LOOP_RESET_ESTOP);
        Position_Loop_Reset_Car_Velocity_Feedforward();
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
    uint8 fault_landing_request = 0U;
    uint8 rate_due;
    uint8 angle_due;
    uint8 velocity_due;
    uint8 position_due;

    MotorHardware_InitOnce();
    control_scheduler_begin_tick(&rate_due, &angle_due, &velocity_due, &position_due);
    if(position_due)
    {
        core_link_user_motion_tick((uint16)((CONTROL_PERIOD_US *
                                   VELOCITY_SLOT_DIVISOR) / 1000U));
    }

    if(Flight_Safety_Check_Esc())
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    if(Flight_Safety_Check_Battery())
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

    /* 先处理IMU严重故障和姿态数据异常，避免其被光流故障分支掩盖。 */
    if(control_task_handle_attitude_safety())
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    fault_landing_request = control_task_handle_flow_safety();
    if(fault_landing_request ||
       Flight_Safety_Attitude_Limit_Requested() ||
       (s_fault_landing_mode != FAULT_LANDING_IDLE))
    {
        if(s_fault_landing_mode == FAULT_LANDING_IDLE)
        {
            control_task_start_fault_landing(HeightManager_IsTofValid());
            Flight_Safety_Clear_Attitude_Limit_Request();
        }
        control_task_update_fault_landing(angle_due, rate_due);
        control_task_finish_cycle(rate_due);
        return;
    }

    if(emergency_stop_triggered)
    {
        control_task_finish_cycle(rate_due);
        return;
    }

    if(control_task_handle_auto_land_disarm())
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
        control_task_update_horizontal_timing();
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
