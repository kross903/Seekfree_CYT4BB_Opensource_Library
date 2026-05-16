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
#define FLIGHT_TAKEOFF_DEFAULT_HOVER_ROLL_COMP_DEG   (-2.1f)
#define FLIGHT_TAKEOFF_DEFAULT_HOVER_PITCH_COMP_DEG  (0.65f)
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

    g_flight_ctrl.angle_roll_compensation = angle_roll_compensation;
    g_flight_ctrl.angle_pitch_compensation = angle_pitch_compensation;
    g_flight_ctrl.angle_yaw_compensation = angle_yaw_compensation;

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
    target_angle_yaw = yaw;

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
void Flight_Control_Set_Angle_Compensation(float roll_compensation, float pitch_compensation, float yaw_compensation)
{
    angle_roll_compensation = roll_compensation;
    angle_pitch_compensation = pitch_compensation;
    angle_yaw_compensation = yaw_compensation;

    sync_runtime_snapshot();
}

void Flight_Control_Get_Angle_Compensation(float *roll_compensation, float *pitch_compensation, float *yaw_compensation)
{
    if(roll_compensation != NULL)
    {
        *roll_compensation = angle_roll_compensation;
    }
    if(pitch_compensation != NULL)
    {
        *pitch_compensation = angle_pitch_compensation;
    }
    if(yaw_compensation != NULL)
    {
        *yaw_compensation = angle_yaw_compensation;
    }
}

/*
 * 解锁飞控主链路。
 * 如果当前模式合法则继续保留，并在下一次油门爬坡前清空运行态油门和 PID 历史量。
 */
void Flight_Control_Arm(void)
{
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
    gpio_set_level(LED3, GPIO_HIGH);
    Flight_Control_Reset_All_PID();
    sync_runtime_snapshot();
}

/* 立即失锁，并清空缓存输出状态。 */
void Flight_Control_Disarm(void)
{
    armed = 0;
    Flight_Runtime_Reset_All_Throttle_States();
    /* 若 disarm 发生在自动起飞流程中（采样或爬坡阶段），必须同步中断状态机，
     * 否则采样结束会再次 Arm，出现意外解锁。 */
    Flight_Takeoff_Abort();
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
    PID_Reset(&gyro_pid_roll);
    PID_Reset(&gyro_pid_pitch);
    PID_Reset(&gyro_pid_yaw);
    PID_Reset(&angle_pid_roll);
    PID_Reset(&angle_pid_pitch);
    PID_Reset(&angle_pid_yaw);
    PID_Reset(&vel_pid_x);
    PID_Reset(&vel_pid_y);
    PID_Reset(&pos_pid_x);
    PID_Reset(&pos_pid_y);
    PID_Reset(&HeightPID);

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
 * 控制任务 (PIT_Control_Task_250us) 运行频率 4kHz (250us)，采样阶段每 8 次 tick 采一次，
 * 得到 500Hz 采样率，3s 共 1500 个样本。ramp 阶段共 12000 tick，逐拍线性累加。
 *********************************************************************************************************************/
#define TAKEOFF_CTRL_TICK_HZ        4000U   /* PIT_Control_Task_250us */
#define TAKEOFF_SAMPLE_DIVIDER      (TAKEOFF_CTRL_TICK_HZ / FLIGHT_TAKEOFF_SAMPLE_HZ)
#define TAKEOFF_CAPTURE_SAMPLES     ((uint32)(FLIGHT_TAKEOFF_SAMPLE_HZ * FLIGHT_TAKEOFF_CAPTURE_DURATION_S))
#define TAKEOFF_RAMP_TICKS          ((uint32)(TAKEOFF_CTRL_TICK_HZ * FLIGHT_TAKEOFF_RAMP_DURATION_S))
#define TAKEOFF_HOVER_ANGLE_COMP_LIMIT_DEG (10.0f)

static volatile flight_takeoff_state_enum g_takeoff_state = FLIGHT_TAKEOFF_IDLE;
static volatile uint32 g_takeoff_tick_in_state = 0U;
static volatile uint32 g_takeoff_sample_count = 0U;
static volatile float  g_takeoff_captured_roll = 0.0f;
static volatile float  g_takeoff_captured_pitch = 0.0f;
static volatile float  g_takeoff_hover_roll_comp_deg = 0.0f;
static volatile float  g_takeoff_hover_pitch_comp_deg = 0.0f;

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

static void takeoff_reset_state(void)
{
    g_takeoff_state = FLIGHT_TAKEOFF_IDLE;
    g_takeoff_tick_in_state = 0U;
    g_takeoff_sample_count = 0U;
}

uint8 Flight_Takeoff_Start(void)
{
    /* 仅在角度闭环模式、且流程处于 IDLE 时才接受启动。 */
    if(control_mode != CONTROL_MODE_ANGLE)
    {
        return 0U;
    }
    if(armed)
    {
        return 0U;
    }
    if(g_takeoff_state != FLIGHT_TAKEOFF_IDLE)
    {
        return 0U;
    }

    /* 进入采样阶段前保持当前 disarmed 状态，避免电机输出。 */
    g_takeoff_tick_in_state = 0U;
    g_takeoff_sample_count = 0U;
    g_takeoff_captured_roll = 0.0f;
    g_takeoff_captured_pitch = 0.0f;
    g_takeoff_state = FLIGHT_TAKEOFF_CAPTURE;
    return 1U;
}

void Flight_Takeoff_Abort(void)
{
    takeoff_reset_state();
}

uint8 Flight_Takeoff_Is_Active(void)
{
    return (g_takeoff_state != FLIGHT_TAKEOFF_IDLE) ? 1U : 0U;
}

flight_takeoff_state_enum Flight_Takeoff_Get_State(void)
{
    return g_takeoff_state;
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
    switch(g_takeoff_state)
    {
        case FLIGHT_TAKEOFF_CAPTURE:
        {
            /* Keep the pre-takeoff wait duration, but do not learn a hover attitude from the ground pose. */
            if((g_takeoff_tick_in_state % TAKEOFF_SAMPLE_DIVIDER) == 0U)
            {
                g_takeoff_sample_count++;
            }
            g_takeoff_tick_in_state++;

            if(g_takeoff_sample_count >= TAKEOFF_CAPTURE_SAMPLES)
            {
                /* Hover attitude target is true level, not the startup/ground attitude. */
                g_takeoff_captured_roll  = 0.0f;
                g_takeoff_captured_pitch = 0.0f;
                target_angle_yaw = current_yaw;

                /* 采样完成 -> 自动解锁并进入 ramp 阶段。Arm() 内部会重置所有 PID 与油门状态。 */
                Flight_Runtime_Clear_Emergency_Stop_Latch();
                Flight_Control_Arm();

                g_takeoff_tick_in_state = 0U;
                g_takeoff_state = FLIGHT_TAKEOFF_RAMP;
            }

            /* 采样阶段不接管目标角度/油门：飞机停在地面，由 !armed 分支直接把电机清零。 */
            return 0U;
        }

        case FLIGHT_TAKEOFF_RAMP:
        {
            float progress;
            float ramped;
            float hover_roll;
            float hover_pitch;

            if(!armed)
            {
                /* 外部触发了急停/disarm，ramp 途中被打断：直接退出。 */
                takeoff_reset_state();
                return 0U;
            }

            g_takeoff_tick_in_state++;
            if(g_takeoff_tick_in_state > TAKEOFF_RAMP_TICKS)
            {
                g_takeoff_tick_in_state = TAKEOFF_RAMP_TICKS;
            }

            progress = (float)g_takeoff_tick_in_state / (float)TAKEOFF_RAMP_TICKS;
            ramped = FLIGHT_TAKEOFF_RAMP_TARGET_THROTTLE * progress;

            /* 接管油门：绕过默认 throttle_ramp，直接写 base/current/target，保持三者一致。 */
            target_throttle = ramped;
            current_throttle = ramped;
            base_throttle = ramped;

            /* 接管目标角度：锁定到悬停基准角，覆盖摇杆 PID 的输出。 */
            Flight_Takeoff_Get_Captured_Angles(&hover_roll, &hover_pitch);
            target_angle_roll  = hover_roll;
            target_angle_pitch = hover_pitch;

            if(g_takeoff_tick_in_state >= TAKEOFF_RAMP_TICKS)
            {
                /* ramp 结束：保持飞机 armed，油门停在当前 ramp 目标值，交还控制权给巡航分支。 */
                takeoff_reset_state();
            }
            return 1U;
        }

        case FLIGHT_TAKEOFF_IDLE:
        default:
            return 0U;
    }
}
