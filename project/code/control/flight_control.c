#include "control/flight_control.h"
#include "common/project_headfile.h"

/*********************************************************************************************************************
* 文件名称: flight_control.c
* 功能说明: 飞控状态与控制参数的统一访问实现
*
* 这一层把 VOFA、菜单页、监视界面等慢速模块常用的状态读写收口到一起。
* 高频运行链路仍然在 `PIT_Demo.c` 中，这里主要负责对外提供稳定入口，
* 避免外层模块分散直接操作底层全局变量。
*********************************************************************************************************************/

#define FLIGHT_CONTROL_MIN_THROTTLE  0.0f
// #define FLIGHT_TAKEOFF_DEFAULT_HOVER_ROLL_COMP_DEG   (-2.0f)
// #define FLIGHT_TAKEOFF_DEFAULT_HOVER_PITCH_COMP_DEG  (0.0f)
// #define FLIGHT_TAKEOFF_DEFAULT_HOVER_ROLL_COMP_DEG   (1.2f)
// #define FLIGHT_TAKEOFF_DEFAULT_HOVER_PITCH_COMP_DEG  (-0.5f)
#define FLIGHT_TAKEOFF_DEFAULT_HOVER_ROLL_COMP_DEG   (0.0f)
#define FLIGHT_TAKEOFF_DEFAULT_HOVER_PITCH_COMP_DEG  (0.0f)
extern volatile uint16 motor_throttle[4];

FlightController g_flight_ctrl;

/* 根据轴号返回当前正在使用的角速度内环 PID。 */
static PID_Controller* get_runtime_gyro_pid(uint8 axis)
{
    switch(axis)
    {
        case AXIS_ROLL:  return &gyro_pid_roll;
        case AXIS_PITCH: return &gyro_pid_pitch;
        case AXIS_YAW:   return &gyro_pid_yaw;
        default:         return NULL;
    }
}

/* 根据轴号返回当前正在使用的姿态外环 PID。 */
static PID_Controller* get_runtime_angle_pid(uint8 axis)
{
    switch(axis)
    {
        case AXIS_ROLL:  return &angle_pid_roll;
        case AXIS_PITCH: return &angle_pid_pitch;
        case AXIS_YAW:   return &angle_pid_yaw;
        default:         return NULL;
    }
}

/* 返回运行态实际使用的单个高度环 PID。 */
static PID_Controller* get_runtime_height_pid(void)
{
    return &HeightPID;
}

/*
 * 用运行态全局变量刷新对外快照。
 * 这样可以让 `g_flight_ctrl` 继续作为慢速模块的集中读取入口，
 * 同时不改变底层运行态真正的所有权。
 */
static void sync_runtime_snapshot(void)
{
    g_flight_ctrl.gyro_pid_roll = gyro_pid_roll;
    g_flight_ctrl.gyro_pid_pitch = gyro_pid_pitch;
    g_flight_ctrl.gyro_pid_yaw = gyro_pid_yaw;

    g_flight_ctrl.angle_pid_roll = angle_pid_roll;
    g_flight_ctrl.angle_pid_pitch = angle_pid_pitch;
    g_flight_ctrl.angle_pid_yaw = angle_pid_yaw;
    g_flight_ctrl.height_pid = HeightPID;

    g_flight_ctrl.target_angle_roll = target_angle_roll;
    g_flight_ctrl.target_angle_pitch = target_angle_pitch;
    g_flight_ctrl.target_angle_yaw = target_angle_yaw;

    g_flight_ctrl.armed = armed;
    g_flight_ctrl.control_mode = control_mode;
    g_flight_ctrl.target_throttle = target_throttle;
    g_flight_ctrl.base_throttle = base_throttle;
}

/* 在失锁或故障处理后清空电机油门缓存。 */
static void clear_motor_throttle_buffer(void)
{
    motor_throttle[0] = 0;
    motor_throttle[1] = 0;
    motor_throttle[2] = 0;
    motor_throttle[3] = 0;
}

/*
 * 对目标油门指令做安全限幅。
 * 这里只限制指令路径，最终电机输出仍然会在运行态混控中再次限幅。
 */
static inline float throttle_safe_limit(float throttle)
{
    if(throttle > SAFE_MAX_THROTTLE)
    {
        throttle = SAFE_MAX_THROTTLE;
    }
    else if(throttle < FLIGHT_CONTROL_MIN_THROTTLE)
    {
        throttle = FLIGHT_CONTROL_MIN_THROTTLE;
    }

    return throttle;
}

/* 初始化 Flight_Control 层，并与当前运行态对齐。 */
void Flight_Control_Init(void)
{
    g_flight_ctrl.target_gyro_roll = 0.0f;
    g_flight_ctrl.target_gyro_pitch = 0.0f;
    g_flight_ctrl.target_gyro_yaw = 0.0f;

    Flight_Takeoff_Set_Hover_Angle_Compensation(FLIGHT_TAKEOFF_DEFAULT_HOVER_ROLL_COMP_DEG,
                                                FLIGHT_TAKEOFF_DEFAULT_HOVER_PITCH_COMP_DEG);

    sync_runtime_snapshot();
}

/* 角速度内环 PID 参数读写接口。 */
void Flight_Control_Set_Gyro_PID(uint8 axis, float kp, float ki, float kd)
{
    PID_Controller *pid = get_runtime_gyro_pid(axis);
    if(pid != NULL)
    {
        pid->Kp = kp;
        pid->Ki = ki;
        pid->Kd = kd;
        sync_runtime_snapshot();
    }
}

void Flight_Control_Get_Gyro_PID(uint8 axis, float *kp, float *ki, float *kd)
{
    PID_Controller *pid = get_runtime_gyro_pid(axis);
    if(pid != NULL && kp != NULL && ki != NULL && kd != NULL)
    {
        *kp = pid->Kp;
        *ki = pid->Ki;
        *kd = pid->Kd;
    }
}

void Flight_Control_Set_Gyro_Limits(uint8 axis, float out_limit, float int_limit)
{
    PID_Controller *pid = get_runtime_gyro_pid(axis);
    if(pid != NULL)
    {
        pid->output_limit = out_limit;
        pid->integral_limit = int_limit;
        sync_runtime_snapshot();
    }
}

void Flight_Control_Get_Gyro_Limits(uint8 axis, float *out_limit, float *int_limit)
{
    PID_Controller *pid = get_runtime_gyro_pid(axis);
    if(pid != NULL && out_limit != NULL && int_limit != NULL)
    {
        *out_limit = pid->output_limit;
        *int_limit = pid->integral_limit;
    }
}

/* 姿态外环 PID 参数读写接口。 */
void Flight_Control_Set_Angle_PID(uint8 axis, float kp, float ki, float kd)
{
    PID_Controller *pid = get_runtime_angle_pid(axis);
    if(pid != NULL)
    {
        pid->Kp = kp;
        pid->Ki = ki;
        pid->Kd = kd;
        sync_runtime_snapshot();
    }
}

void Flight_Control_Get_Angle_PID(uint8 axis, float *kp, float *ki, float *kd)
{
    PID_Controller *pid = get_runtime_angle_pid(axis);
    if(pid != NULL && kp != NULL && ki != NULL && kd != NULL)
    {
        *kp = pid->Kp;
        *ki = pid->Ki;
        *kd = pid->Kd;
    }
}

void Flight_Control_Set_Angle_Limits(uint8 axis, float out_limit, float int_limit)
{
    PID_Controller *pid = get_runtime_angle_pid(axis);
    if(pid != NULL)
    {
        pid->output_limit = out_limit;
        pid->integral_limit = int_limit;
        sync_runtime_snapshot();
    }
}

void Flight_Control_Get_Angle_Limits(uint8 axis, float *out_limit, float *int_limit)
{
    PID_Controller *pid = get_runtime_angle_pid(axis);
    if(pid != NULL && out_limit != NULL && int_limit != NULL)
    {
        *out_limit = pid->output_limit;
        *int_limit = pid->integral_limit;
    }
}

/* 高度环 PID 参数读写接口。 */
void Flight_Control_Set_Height_PID(float kp, float ki, float kd)
{
    PID_Controller *pid = get_runtime_height_pid();

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    sync_runtime_snapshot();
}

void Flight_Control_Get_Height_PID(float *kp, float *ki, float *kd)
{
    PID_Controller *pid = get_runtime_height_pid();

    if(kp != NULL)
    {
        *kp = pid->Kp;
    }
    if(ki != NULL)
    {
        *ki = pid->Ki;
    }
    if(kd != NULL)
    {
        *kd = pid->Kd;
    }
}

void Flight_Control_Set_Height_Limits(float out_limit, float int_limit)
{
    PID_Controller *pid = get_runtime_height_pid();

    pid->output_limit = out_limit;
    pid->integral_limit = int_limit;
    sync_runtime_snapshot();
}

void Flight_Control_Get_Height_Limits(float *out_limit, float *int_limit)
{
    PID_Controller *pid = get_runtime_height_pid();

    if(out_limit != NULL)
    {
        *out_limit = pid->output_limit;
    }
    if(int_limit != NULL)
    {
        *int_limit = pid->integral_limit;
    }
}

/*
 * 兼容保留的角速度目标缓存。
 * 当前运行环路不会直接消费这组目标，但为了兼容旧调用点，接口仍然保留。
 */
void Flight_Control_Set_Target_Gyro(float roll, float pitch, float yaw)
{
    g_flight_ctrl.target_gyro_roll = roll;
    g_flight_ctrl.target_gyro_pitch = pitch;
    g_flight_ctrl.target_gyro_yaw = yaw;
}

void Flight_Control_Get_Target_Gyro(float *roll, float *pitch, float *yaw)
{
    if(roll != NULL)
    {
        *roll = g_flight_ctrl.target_gyro_roll;
    }
    if(pitch != NULL)
    {
        *pitch = g_flight_ctrl.target_gyro_pitch;
    }
    if(yaw != NULL)
    {
        *yaw = g_flight_ctrl.target_gyro_yaw;
    }
}

/* 运行态控制分支实际使用的姿态目标。 */
void Flight_Control_Set_Target_Angle(float roll, float pitch, float yaw)
{
    target_angle_roll = roll;
    target_angle_pitch = pitch;
    target_angle_yaw = Angle_WrapDeg180(yaw);

    sync_runtime_snapshot();
}

void Flight_Control_Get_Target_Angle(float *roll, float *pitch, float *yaw)
{
    if(roll != NULL)
    {
        *roll = target_angle_roll;
    }
    if(pitch != NULL)
    {
        *pitch = target_angle_pitch;
    }
    if(yaw != NULL)
    {
        *yaw = target_angle_yaw;
    }
}

/* 传感器安装补偿角，当前用于 IMU roll/pitch 旋转补偿。 */
/*
 * 解锁飞控主链路。
 * 如果当前模式合法则继续保留，并在下一次油门爬坡前清空运行态油门和 PID 历史量。
 */
uint8 Flight_Control_Arm(FlightArmRejectReason *reason)
{
    if(!Flight_Safety_Can_Arm(reason))
    {
        return 0U;
    }

    armed = 1;
    /*
     * [2026-03-10]
     * 修改目的:
     * 1. 解锁时保留用户此前选中的合法模式，而不是强制回到 MANUAL。
     * 2. 在下一次油门爬坡前先清空共享油门状态。
     */
    if(control_mode > CONTROL_MODE_ANGLE)
    {
        control_mode = CONTROL_MODE_MANUAL;
    }
    Flight_Runtime_Reset_All_Throttle_States();

    Yaw_Lock();
    target_angle_yaw = Angle_WrapDeg180(current_yaw);
    gpio_set_level(LED3, GPIO_HIGH);
    Flight_Control_Reset_All_PID();
    sync_runtime_snapshot();
    return 1U;
}

/* 立即失锁，并清空缓存输出状态。 */
void Flight_Control_Disarm(void)
{
    armed = 0;
    Flight_Runtime_Reset_All_Throttle_States();
    /* 若 disarm 发生在缓启动任一阶段，必须同步中断状态机和高度环接管。 */
    Flight_Takeoff_AbortAt("DISARM", __FILE__, __LINE__);
    /* Disarm 同时关掉速度环, 下次起飞会在巡航分支首拍自动重新 enable。 */
    Velocity_Loop_Set_Enabled(0U);
    Position_Loop_Set_Enabled(0U);

    clear_motor_throttle_buffer();
    gpio_set_level(LED3, GPIO_LOW);
    Flight_Control_Reset_All_PID();
    sync_runtime_snapshot();
}

/* 切换控制模式，并清除旧控制器历史量。 */
void Flight_Control_Set_Mode(uint8 mode)
{
    if(mode <= CONTROL_MODE_ANGLE)
    {
        control_mode = mode;
        Flight_Control_Reset_All_PID();
        Flight_Runtime_Sync_Public_Throttle_State();
        sync_runtime_snapshot();
    }
}

uint8 Flight_Control_Get_Mode(void)
{
    sync_runtime_snapshot();
    return control_mode;
}

/* 油门目标相关接口。 */
void Flight_Control_Set_Throttle(float throttle)
{
    float safe_throttle = throttle_safe_limit(throttle);

    Flight_Runtime_Set_Throttle_For_Mode(control_mode, safe_throttle);
    sync_runtime_snapshot();

    if(throttle > SAFE_MAX_THROTTLE)
    {
        printf("[SAFETY] Throttle limited: %.0f -> %.0f\r\n",
               (double)throttle,
               (double)SAFE_MAX_THROTTLE);
    }
}

float Flight_Control_Get_Target_Throttle(void)
{
    sync_runtime_snapshot();
    return throttle_safe_limit(Flight_Runtime_Get_Target_Throttle_For_Mode(control_mode));
}

float Flight_Control_Get_Throttle(void)
{
    sync_runtime_snapshot();
    return throttle_safe_limit(Flight_Runtime_Get_Base_Throttle_For_Mode(control_mode));
}

uint8 Flight_Control_Is_Armed(void)
{
    sync_runtime_snapshot();
    return armed;
}

/* 在切模式、重新解锁或大幅改参后清空 PID 累积历史量。 */
void Flight_Control_Reset_All_PID(void)
{
    Flight_Safety_ResetYawSpinMonitor();
    PID_Reset(&gyro_pid_roll);
    PID_Reset(&gyro_pid_pitch);
    PID_Reset(&gyro_pid_yaw);
    PID_Reset(&angle_pid_roll);
    PID_Reset(&angle_pid_pitch);
    PID_Reset(&angle_pid_yaw);
    PID_Reset(&vel_pid_x);
    PID_Reset(&vel_pid_y);
    Velocity_Loop_Reset_D_Filter_State();
    PID_Reset(&HeightPID);
    PID_Reset(&HeightVelPID);

    sync_runtime_snapshot();
}

void Flight_Control_Reset_Gyro_PID(uint8 axis)
{
    PID_Controller *pid = get_runtime_gyro_pid(axis);
    if(pid != NULL)
    {
        PID_Reset(pid);
        sync_runtime_snapshot();
    }
}

void Flight_Control_Reset_Angle_PID(uint8 axis)
{
    PID_Controller *pid = get_runtime_angle_pid(axis);
    if(pid != NULL)
    {
        PID_Reset(pid);
        sync_runtime_snapshot();
    }
}

void Flight_Control_Reset_Height_PID(void)
{
    PID_Reset(&HeightPID);
    PID_Reset(&HeightVelPID);
    sync_runtime_snapshot();
}

/* 油门安全检查辅助接口。 */
float Flight_Control_Get_Max_Throttle(void)
{
    return SAFE_MAX_THROTTLE;
}

uint8 Flight_Control_Is_Throttle_Safe(float throttle)
{
    return (throttle >= FLIGHT_CONTROL_MIN_THROTTLE && throttle <= SAFE_MAX_THROTTLE);
}

/*********************************************************************************************************************
 * 自动起飞流程实现
 *
 * 控制任务运行频率为 4 kHz。前三个阶段由 PIT_Demo 直接输出四路等值油门；
 * LIFTOFF_RAMP 才启用姿态混控，HEIGHT_BLEND 才允许高度环输出参与混控。
 *********************************************************************************************************************/
#define TAKEOFF_CTRL_TICK_HZ        4000U   /* PIT_Control_Task_250us */
#define TAKEOFF_ARM_DELAY_TICKS     ((uint32)(TAKEOFF_CTRL_TICK_HZ * FLIGHT_TAKEOFF_ARM_DELAY_S))
#define TAKEOFF_MOTOR_SPINUP_TICKS  ((uint32)(TAKEOFF_CTRL_TICK_HZ * FLIGHT_TAKEOFF_MOTOR_SPINUP_S))
#define TAKEOFF_IDLE_HOLD_TICKS     ((uint32)(TAKEOFF_CTRL_TICK_HZ * FLIGHT_TAKEOFF_IDLE_HOLD_S))
#define TAKEOFF_RAMP_TICKS          ((uint32)(TAKEOFF_CTRL_TICK_HZ * FLIGHT_TAKEOFF_RAMP_DURATION_S))
#define TAKEOFF_ATTITUDE_BLEND_TICKS ((uint32)(TAKEOFF_CTRL_TICK_HZ * FLIGHT_TAKEOFF_ATTITUDE_BLEND_S))
#define TAKEOFF_HEIGHT_BLEND_TICKS  ((uint32)(TAKEOFF_CTRL_TICK_HZ * FLIGHT_TAKEOFF_HEIGHT_BLEND_S))
#define TAKEOFF_HEIGHT_OUTPUT_BLEND_TICKS ((uint32)(TAKEOFF_CTRL_TICK_HZ * FLIGHT_TAKEOFF_HEIGHT_OUTPUT_BLEND_S))
#define TAKEOFF_HOVER_ANGLE_COMP_LIMIT_DEG (10.0f)

static volatile flight_takeoff_state_enum g_takeoff_state = FLIGHT_TAKEOFF_IDLE;
static volatile uint32 g_takeoff_tick_in_state = 0U;
static volatile float  g_takeoff_captured_roll = 0.0f;
static volatile float  g_takeoff_captured_pitch = 0.0f;
static volatile float  g_takeoff_initial_roll = 0.0f;
static volatile float  g_takeoff_initial_pitch = 0.0f;
static volatile float  g_takeoff_commanded_base = 0.0f;
static volatile float  g_takeoff_hover_roll_comp_deg = 0.0f;
static volatile float  g_takeoff_hover_pitch_comp_deg = 0.0f;
static volatile uint8  g_takeoff_expected_active = 0U;
static FlightTakeoffDiag g_takeoff_diag = {0U, FLIGHT_TAKEOFF_IDLE, NULL, NULL, 0U};

static void takeoff_diag_clear(void)
{
    g_takeoff_diag.stopped = 0U;
    g_takeoff_diag.state = (uint8)g_takeoff_state;
    g_takeoff_diag.reason = NULL;
    g_takeoff_diag.file = NULL;
    g_takeoff_diag.line = 0U;
}

void Flight_Takeoff_RecordStopAt(const char *reason, const char *file, uint32 line)
{
    g_takeoff_diag.stopped = 1U;
    g_takeoff_diag.state = (uint8)g_takeoff_state;
    g_takeoff_diag.reason = (reason != NULL) ? reason : "UNKNOWN";
    g_takeoff_diag.file = (file != NULL) ? file : "UNKNOWN";
    g_takeoff_diag.line = line;
}

void Flight_Takeoff_GetDiag(FlightTakeoffDiag *diag)
{
    if(diag != NULL)
    {
        uint32 irq_state = __get_PRIMASK();

        __disable_irq();
        *diag = g_takeoff_diag;
        if(irq_state == 0U)
        {
            __enable_irq();
        }
    }
}

static const char *takeoff_arm_reject_text(FlightArmRejectReason reason)
{
    switch(reason)
    {
        case FLIGHT_ARM_REJECT_ESTOP:           return "ESTOP LATCH";
        case FLIGHT_ARM_REJECT_IMU_CALIBRATING: return "IMU CALIBRATING";
        case FLIGHT_ARM_REJECT_INVALID_SENSOR:  return "INVALID SENSOR";
        case FLIGHT_ARM_REJECT_ATTITUDE:        return "ATTITUDE LIMIT";
        case FLIGHT_ARM_REJECT_BATTERY_INVALID: return "BAT INVALID";
        case FLIGHT_ARM_REJECT_BATTERY_LOW:     return "BAT LOW";
        case FLIGHT_ARM_REJECT_MOTOR_NOT_READY:
        {
            const char *esc_fault = Flight_Safety_GetEscFaultText();
            return (esc_fault != NULL) ? esc_fault : "MOTOR NOT READY";
        }
        case FLIGHT_ARM_REJECT_REARM_INHIBIT:   return "ESTOP WAIT 2S";
        case FLIGHT_ARM_REJECT_NONE:
        default:                                return "ARM REJECTED";
    }
}

static float takeoff_clamp_unit(float value)
{
    if(value < 0.0f) return 0.0f;
    if(value > 1.0f) return 1.0f;
    return value;
}

static float takeoff_smoothstep(float progress)
{
    progress = takeoff_clamp_unit(progress);
    return progress * progress * (3.0f - 2.0f * progress);
}

static uint32 takeoff_state_duration_ticks(flight_takeoff_state_enum state)
{
    switch(state)
    {
        case FLIGHT_TAKEOFF_ARM_DELAY:      return TAKEOFF_ARM_DELAY_TICKS;
        case FLIGHT_TAKEOFF_MOTOR_SPINUP:   return TAKEOFF_MOTOR_SPINUP_TICKS;
        case FLIGHT_TAKEOFF_IDLE_HOLD:      return TAKEOFF_IDLE_HOLD_TICKS;
        case FLIGHT_TAKEOFF_LIFTOFF_RAMP:   return TAKEOFF_RAMP_TICKS;
        case FLIGHT_TAKEOFF_HEIGHT_BLEND:   return TAKEOFF_HEIGHT_BLEND_TICKS;
        case FLIGHT_TAKEOFF_IDLE:
        default:                            return 0U;
    }
}

static void takeoff_set_commanded_base(float throttle)
{
    g_takeoff_commanded_base = throttle_safe_limit(throttle);
    target_throttle = g_takeoff_commanded_base;
    current_throttle = g_takeoff_commanded_base;
    base_throttle = g_takeoff_commanded_base;
}

static void takeoff_enter_state(flight_takeoff_state_enum state)
{
    g_takeoff_state = state;
    g_takeoff_tick_in_state = 0U;
}

static float takeoff_limit_hover_angle_comp(float value)
{
    if(value > TAKEOFF_HOVER_ANGLE_COMP_LIMIT_DEG)
    {
        value = TAKEOFF_HOVER_ANGLE_COMP_LIMIT_DEG;
    }
    else if(value < -TAKEOFF_HOVER_ANGLE_COMP_LIMIT_DEG)
    {
        value = -TAKEOFF_HOVER_ANGLE_COMP_LIMIT_DEG;
    }

    return value;
}

static void takeoff_finish_state(void)
{
    g_takeoff_expected_active = 0U;
    takeoff_enter_state(FLIGHT_TAKEOFF_IDLE);
}

static uint8 takeoff_start_armed_flow(void)
{
    FlightArmRejectReason reject_reason = FLIGHT_ARM_REJECT_NONE;

    if(!ICM_Health_IsArmable())
    {
        Flight_Takeoff_RecordStopAt("IMU NOT ARMABLE", __FILE__, __LINE__);
        return 0U;
    }
    if(!Flight_Control_Arm(&reject_reason))
    {
        Flight_Takeoff_RecordStopAt(takeoff_arm_reject_text(reject_reason), __FILE__, __LINE__);
        return 0U;
    }

    g_takeoff_captured_roll = 0.0f;
    g_takeoff_captured_pitch = 0.0f;
    g_takeoff_initial_roll = current_roll;
    g_takeoff_initial_pitch = current_pitch;
    target_angle_yaw = Angle_WrapDeg180(current_yaw);
    takeoff_set_commanded_base(0.0f);

    if(control_mode == CONTROL_MODE_ANGLE)
    {
        HeightManager_PrepareSoftTakeoff();
        Velocity_Loop_Set_Enabled(0U);
        Position_Loop_Set_Enabled(0U);
        g_takeoff_expected_active = 1U;
        takeoff_enter_state(FLIGHT_TAKEOFF_ARM_DELAY);
    }
    else
    {
        takeoff_finish_state();
    }
    return 1U;
}

uint8 Flight_Takeoff_Start(void)
{
    takeoff_diag_clear();

    /* KEY3 只执行当前健康检查和解锁，不再触发额外的静止采集。 */
    if(armed)
    {
        Flight_Takeoff_RecordStopAt("ALREADY ARMED", __FILE__, __LINE__);
        return 0U;
    }
    if(g_takeoff_state != FLIGHT_TAKEOFF_IDLE)
    {
        Flight_Takeoff_RecordStopAt("TAKEOFF STATE BUSY", __FILE__, __LINE__);
        return 0U;
    }
    return takeoff_start_armed_flow();
}

void Flight_Takeoff_Abort(void)
{
    Flight_Takeoff_AbortAt("ABORT API", __FILE__, __LINE__);
}

void Flight_Takeoff_AbortAt(const char *reason, const char *file, uint32 line)
{
    if(g_takeoff_state != FLIGHT_TAKEOFF_IDLE)
    {
        Flight_Takeoff_RecordStopAt(reason, file, line);
    }
    HeightManager_AbortSoftTakeoff();
    g_takeoff_commanded_base = 0.0f;
    takeoff_finish_state();
}

uint8 Flight_Takeoff_Is_Active(void)
{
    return (g_takeoff_state != FLIGHT_TAKEOFF_IDLE) ? 1U : 0U;
}

flight_takeoff_state_enum Flight_Takeoff_Get_State(void)
{
    return g_takeoff_state;
}

float Flight_Takeoff_Get_Progress(void)
{
    uint32 duration_ticks = takeoff_state_duration_ticks(g_takeoff_state);

    if(duration_ticks == 0U)
    {
        return 0.0f;
    }
    return takeoff_clamp_unit((float)g_takeoff_tick_in_state / (float)duration_ticks);
}

float Flight_Takeoff_Get_Commanded_Base_Throttle(void)
{
    return g_takeoff_commanded_base;
}

uint8 Flight_Takeoff_Requires_BaseOnly_Output(void)
{
    return (uint8)(g_takeoff_state == FLIGHT_TAKEOFF_ARM_DELAY ||
                   g_takeoff_state == FLIGHT_TAKEOFF_MOTOR_SPINUP ||
                   g_takeoff_state == FLIGHT_TAKEOFF_IDLE_HOLD);
}

void Flight_Takeoff_Get_Captured_Angles(float *roll, float *pitch)
{
    if(roll != NULL)
    {
        *roll = g_takeoff_captured_roll + g_takeoff_hover_roll_comp_deg;
    }
    if(pitch != NULL)
    {
        *pitch = g_takeoff_captured_pitch + g_takeoff_hover_pitch_comp_deg;
    }
}

void Flight_Takeoff_Set_Hover_Angle_Compensation(float roll_deg, float pitch_deg)
{
    g_takeoff_hover_roll_comp_deg = takeoff_limit_hover_angle_comp(roll_deg);
    g_takeoff_hover_pitch_comp_deg = takeoff_limit_hover_angle_comp(pitch_deg);
}

void Flight_Takeoff_Get_Hover_Angle_Compensation(float *roll_deg, float *pitch_deg)
{
    if(roll_deg != NULL)
    {
        *roll_deg = g_takeoff_hover_roll_comp_deg;
    }
    if(pitch_deg != NULL)
    {
        *pitch_deg = g_takeoff_hover_pitch_comp_deg;
    }
}

uint8 Flight_Takeoff_Tick_250us(void)
{
    uint32 duration_ticks;
    float progress;
    float shaped;

    if(g_takeoff_state == FLIGHT_TAKEOFF_IDLE)
    {
        if(g_takeoff_expected_active)
        {
            Flight_Takeoff_RecordStopAt("STATE CLEARED UNEXPECTED", __FILE__, __LINE__);
            g_takeoff_expected_active = 0U;
        }
        return 0U;
    }
    if(!armed)
    {
        Flight_Takeoff_AbortAt("ARM FLAG CLEARED", __FILE__, __LINE__);
        return 0U;
    }
    duration_ticks = takeoff_state_duration_ticks(g_takeoff_state);
    if(g_takeoff_tick_in_state < duration_ticks)
    {
        g_takeoff_tick_in_state++;
    }
    progress = (duration_ticks > 0U) ?
               ((float)g_takeoff_tick_in_state / (float)duration_ticks) : 1.0f;
    shaped = takeoff_smoothstep(progress);

    switch(g_takeoff_state)
    {
        case FLIGHT_TAKEOFF_ARM_DELAY:
        {
            takeoff_set_commanded_base(0.0f);
            HeightManager_PrepareSoftTakeoff();
            if(g_takeoff_tick_in_state >= TAKEOFF_ARM_DELAY_TICKS)
            {
                takeoff_enter_state(FLIGHT_TAKEOFF_MOTOR_SPINUP);
            }
            return 1U;
        }

        case FLIGHT_TAKEOFF_MOTOR_SPINUP:
        {
            takeoff_set_commanded_base(FLIGHT_TAKEOFF_IDLE_THROTTLE * shaped);
            HeightManager_PrepareSoftTakeoff();
            if(g_takeoff_tick_in_state >= TAKEOFF_MOTOR_SPINUP_TICKS)
            {
                takeoff_set_commanded_base(FLIGHT_TAKEOFF_IDLE_THROTTLE);
                takeoff_enter_state(FLIGHT_TAKEOFF_IDLE_HOLD);
            }
            return 1U;
        }

        case FLIGHT_TAKEOFF_IDLE_HOLD:
        {
            takeoff_set_commanded_base(FLIGHT_TAKEOFF_IDLE_THROTTLE);
            HeightManager_PrepareSoftTakeoff();
            if(g_takeoff_tick_in_state >= TAKEOFF_IDLE_HOLD_TICKS)
            {
                Flight_Control_Reset_All_PID();
                target_angle_roll = g_takeoff_initial_roll;
                target_angle_pitch = g_takeoff_initial_pitch;
                takeoff_enter_state(FLIGHT_TAKEOFF_LIFTOFF_RAMP);
            }
            return 1U;
        }

        case FLIGHT_TAKEOFF_LIFTOFF_RAMP:
        {
            float attitude_progress;
            float attitude_shaped;
            float hover_roll;
            float hover_pitch;

            takeoff_set_commanded_base(FLIGHT_TAKEOFF_IDLE_THROTTLE +
                                       (FLIGHT_TAKEOFF_RAMP_TARGET_THROTTLE -
                                        FLIGHT_TAKEOFF_IDLE_THROTTLE) * shaped);
            HeightManager_PrepareSoftTakeoff();
            Flight_Takeoff_Get_Captured_Angles(&hover_roll, &hover_pitch);
            attitude_progress = (TAKEOFF_ATTITUDE_BLEND_TICKS > 0U) ?
                                ((float)g_takeoff_tick_in_state /
                                 (float)TAKEOFF_ATTITUDE_BLEND_TICKS) : 1.0f;
            attitude_shaped = takeoff_smoothstep(attitude_progress);
            target_angle_roll = g_takeoff_initial_roll +
                                (hover_roll - g_takeoff_initial_roll) * attitude_shaped;
            target_angle_pitch = g_takeoff_initial_pitch +
                                 (hover_pitch - g_takeoff_initial_pitch) * attitude_shaped;

            if(g_takeoff_tick_in_state >= TAKEOFF_RAMP_TICKS)
            {
                takeoff_set_commanded_base(FLIGHT_TAKEOFF_RAMP_TARGET_THROTTLE);
                HeightManager_BeginSoftTakeoff();
                takeoff_enter_state(FLIGHT_TAKEOFF_HEIGHT_BLEND);
            }
            return 1U;
        }

        case FLIGHT_TAKEOFF_HEIGHT_BLEND:
        {
            float output_progress = (TAKEOFF_HEIGHT_OUTPUT_BLEND_TICKS > 0U) ?
                                    ((float)g_takeoff_tick_in_state /
                                     (float)TAKEOFF_HEIGHT_OUTPUT_BLEND_TICKS) : 1.0f;

            takeoff_set_commanded_base(FLIGHT_TAKEOFF_RAMP_TARGET_THROTTLE);
            HeightManager_SetOutputBlend(takeoff_smoothstep(output_progress));
            if(g_takeoff_tick_in_state >= TAKEOFF_HEIGHT_BLEND_TICKS)
            {
                HeightManager_SetOutputBlend(1.0f);
                takeoff_diag_clear();
                takeoff_finish_state();
            }
            return 1U;
        }

        case FLIGHT_TAKEOFF_IDLE:
        default:
            return 0U;
    }
}
