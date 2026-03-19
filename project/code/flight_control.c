#include "flight_control.h"
#include "PIT_Demo.h"
#include "zf_common_headfile.h"
#include "ICM.h"

/*********************************************************************************************************************
* 文件名称: flight_control.c
* 功能说明: 飞控状态与控制参数的统一访问实现
*
* 这一层把 VOFA、菜单页、监视界面等慢速模块常用的状态读写收口到一起。
* 高频运行链路仍然在 `PIT_Demo.c` 中，这里主要负责对外提供稳定入口，
* 避免外层模块分散直接操作底层全局变量。
*********************************************************************************************************************/

#define FLIGHT_CONTROL_MIN_THROTTLE  0.0f

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

    g_flight_ctrl.target_angle_roll_trim = target_angle_roll_add;
    g_flight_ctrl.target_angle_pitch_trim = target_angle_pitch_add;
    g_flight_ctrl.target_angle_yaw_trim = target_angle_yaw_add;

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

/* 姿态微调量，通常通过 VOFA 在线调整。 */
void Flight_Control_Set_Target_Angle_Trim(float roll_trim, float pitch_trim, float yaw_trim)
{
    target_angle_roll_add = roll_trim;
    target_angle_pitch_add = pitch_trim;
    target_angle_yaw_add = yaw_trim;

    sync_runtime_snapshot();
}

void Flight_Control_Get_Target_Angle_Trim(float *roll_trim, float *pitch_trim, float *yaw_trim)
{
    if(roll_trim != NULL)
    {
        *roll_trim = target_angle_roll_add;
    }
    if(pitch_trim != NULL)
    {
        *pitch_trim = target_angle_pitch_add;
    }
    if(yaw_trim != NULL)
    {
        *yaw_trim = target_angle_yaw_add;
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
    gpio_set_level(LED1, GPIO_HIGH);
    Flight_Control_Reset_All_PID();
    sync_runtime_snapshot();
}

/* 立即失锁，并清空缓存输出状态。 */
void Flight_Control_Disarm(void)
{
    armed = 0;
    Flight_Runtime_Reset_All_Throttle_States();

    clear_motor_throttle_buffer();
    gpio_set_level(LED1, GPIO_LOW);
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
