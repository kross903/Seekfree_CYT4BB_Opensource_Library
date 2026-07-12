#ifndef FLIGHT_CONTROL_H_
#define FLIGHT_CONTROL_H_

#include "control/pid_controller.h"
#include "common/flight_safety.h"
#include "zf_common_typedef.h"

/*********************************************************************************************************************
* 文件名称: flight_control.h
* 功能说明: 飞控状态与控制参数的对外访问接口
*
* 高频控制环仍然运行在 `PIT_Demo.c` 中，本层主要给 VOFA、菜单页以及
* 其他非中断路径提供统一入口，避免它们直接分散读写底层运行态全局变量。
*********************************************************************************************************************/

/*
 * 控制模式编号保持与现有工程一致，避免破坏已有运行流程以及菜单 / VOFA
 * 命令路径中的模式映射关系。
 *
 * CONTROL_MODE_MANUAL:
 * 直接输出共享油门链路，不进入姿态 PID 分支。
 *
 * CONTROL_MODE_ANGLE:
 * 姿态外环加角速度内环。
 *
 * CONTROL_MODE_CASCADE:
 * 预留给后续扩展，当前运行链路还没有实现第三种控制分支。
 */
#define CONTROL_MODE_MANUAL     0
#define CONTROL_MODE_ANGLE      1
#define CONTROL_MODE_CASCADE    2

/* 为旧代码保留的兼容别名，避免历史调用点失效。 */
#define CONTROL_MODE_GYRO       CONTROL_MODE_MANUAL

#define AXIS_ROLL               0
#define AXIS_PITCH              1
#define AXIS_YAW                2

/*
 * 这里不是飞行状态的唯一真实来源，而是一个便于外层模块读取的快照。
 * 它镜像了最常用的运行态控制参数，方便慢速模块一次性读取一组状态。
 */
typedef struct
{
    PID_Controller gyro_pid_roll;
    PID_Controller gyro_pid_pitch;
    PID_Controller gyro_pid_yaw;

    PID_Controller angle_pid_roll;
    PID_Controller angle_pid_pitch;
    PID_Controller angle_pid_yaw;
    PID_Controller height_pid;

    float target_gyro_roll;
    float target_gyro_pitch;
    float target_gyro_yaw;

    float target_angle_roll;
    float target_angle_pitch;
    float target_angle_yaw;

    uint8 armed;
    uint8 control_mode;
    float target_throttle;
    float base_throttle;
} FlightController;

extern FlightController g_flight_ctrl;

/*
 * 初始化并同步快照，使 Flight_Control 层与当前运行态保持一致。
 */
void Flight_Control_Init(void);

/*
 * 角速度内环 PID 参数接口。
 * 用于读写 Roll / Pitch / Yaw 三轴角速度 PID 的增益和限幅。
 */
void Flight_Control_Set_Gyro_PID(uint8 axis, float kp, float ki, float kd);
void Flight_Control_Get_Gyro_PID(uint8 axis, float *kp, float *ki, float *kd);
void Flight_Control_Set_Gyro_Limits(uint8 axis, float out_limit, float int_limit);
void Flight_Control_Get_Gyro_Limits(uint8 axis, float *out_limit, float *int_limit);

/*
 * 姿态外环 PID 参数接口。
 * 用于读写 Roll / Pitch / Yaw 三轴姿态 PID 的增益和限幅。
 */
void Flight_Control_Set_Angle_PID(uint8 axis, float kp, float ki, float kd);
void Flight_Control_Get_Angle_PID(uint8 axis, float *kp, float *ki, float *kd);
void Flight_Control_Set_Angle_Limits(uint8 axis, float out_limit, float int_limit);
void Flight_Control_Get_Angle_Limits(uint8 axis, float *out_limit, float *int_limit);

/* 高度环 PID 参数接口。 */
void Flight_Control_Set_Height_PID(float kp, float ki, float kd);
void Flight_Control_Get_Height_PID(float *kp, float *ki, float *kd);
void Flight_Control_Set_Height_Limits(float out_limit, float int_limit);
void Flight_Control_Get_Height_Limits(float *out_limit, float *int_limit);

/*
 * 目标指令接口。
 *
 * `Flight_Control_Set_Target_Gyro()` 当前主要作为兼容缓存保留；
 * 实际运行链路仍然使用下面的目标姿态角和姿态微调量。
 */
void Flight_Control_Set_Target_Gyro(float roll, float pitch, float yaw);
void Flight_Control_Get_Target_Gyro(float *roll, float *pitch, float *yaw);

void Flight_Control_Set_Target_Angle(float roll, float pitch, float yaw);
void Flight_Control_Get_Target_Angle(float *roll, float *pitch, float *yaw);

/*
 * 解锁 / 模式 / 油门接口。
 * 慢速模块如果需要切换飞控激活状态、控制模式或油门目标，
 * 建议统一从这里进入。
 */
uint8 Flight_Control_Arm(FlightArmRejectReason *reason);
void Flight_Control_Disarm(void);

void Flight_Control_Set_Mode(uint8 mode);
uint8 Flight_Control_Get_Mode(void);

void Flight_Control_Set_Throttle(float throttle);
float Flight_Control_Get_Target_Throttle(void);
float Flight_Control_Get_Throttle(void);

uint8 Flight_Control_Is_Armed(void);

/*
 * PID 状态复位辅助接口。
 * 用于在切模式、重新解锁或参数大改后清空累计历史量。
 */
void Flight_Control_Reset_All_PID(void);
void Flight_Control_Reset_Gyro_PID(uint8 axis);
void Flight_Control_Reset_Angle_PID(uint8 axis);
void Flight_Control_Reset_Height_PID(void);

/* 安全相关辅助接口。 */
float Flight_Control_Get_Max_Throttle(void);
uint8 Flight_Control_Is_Throttle_Safe(float throttle);

/*
 * 起飞安全检查与分阶段缓启动流程。
 *
 * 角度模式依次执行零输出等待、等值软起转、怠速保持、姿态闭环离地和高度环平滑接管。
 * 手动模式仍只执行安全检查和解锁，不接管油门。
 * Tick_250us: 必须每次 PIT_Control_Task_250us 调用一次；返回 1 表示缓启动流程激活，
 *             此时上层应跳过摇杆→目标角度、以及摇杆油门爬坡对 base_throttle 的覆盖，
 *             让本模块接管 target_angle_roll/pitch/base_throttle。
 * Abort: 任何安全路径（急停、disarm）都会调用，无条件回到 IDLE。
 */
#define FLIGHT_TAKEOFF_ARM_DELAY_S          1.0f
#define FLIGHT_TAKEOFF_MOTOR_SPINUP_S       1.2f
#define FLIGHT_TAKEOFF_IDLE_HOLD_S          0.5f
#define FLIGHT_TAKEOFF_RAMP_DURATION_S      3.0f
#define FLIGHT_TAKEOFF_ATTITUDE_BLEND_S     0.8f
#define FLIGHT_TAKEOFF_HEIGHT_BLEND_S       3.0f
#define FLIGHT_TAKEOFF_HEIGHT_OUTPUT_BLEND_S 1.0f
#define FLIGHT_TAKEOFF_IDLE_THROTTLE        300.0f
#define FLIGHT_TAKEOFF_RAMP_TARGET_THROTTLE 750.0f   // 内部范围 [0,2000]，770≈38.5% 油门；根据悬停实际重量微调

typedef enum
{
    FLIGHT_TAKEOFF_IDLE = 0,
    FLIGHT_TAKEOFF_ARM_DELAY,
    FLIGHT_TAKEOFF_MOTOR_SPINUP,
    FLIGHT_TAKEOFF_IDLE_HOLD,
    FLIGHT_TAKEOFF_LIFTOFF_RAMP,
    FLIGHT_TAKEOFF_HEIGHT_BLEND
} flight_takeoff_state_enum;

typedef struct
{
    uint8 stopped;
    uint8 state;
    const char *reason;
    const char *file;
    uint32 line;
} FlightTakeoffDiag;

uint8                     Flight_Takeoff_Start(void);
void                      Flight_Takeoff_Abort(void);
void                      Flight_Takeoff_AbortAt(const char *reason,
                                                 const char *file,
                                                 uint32 line);
void                      Flight_Takeoff_RecordStopAt(const char *reason,
                                                      const char *file,
                                                      uint32 line);
void                      Flight_Takeoff_GetDiag(FlightTakeoffDiag *diag);
uint8                     Flight_Takeoff_Is_Active(void);
flight_takeoff_state_enum Flight_Takeoff_Get_State(void);
float                     Flight_Takeoff_Get_Progress(void);
float                     Flight_Takeoff_Get_Commanded_Base_Throttle(void);
uint8                     Flight_Takeoff_Requires_BaseOnly_Output(void);
uint8                     Flight_Takeoff_Tick_250us(void);

/*
 * 读取起飞流程使用的悬停基准角。
 * 当前采样基准为 0deg/0deg，返回值会叠加无线调参设置的悬停角补偿。
 */
void                      Flight_Takeoff_Get_Captured_Angles(float *roll, float *pitch);
void                      Flight_Takeoff_Set_Hover_Angle_Compensation(float roll_deg, float pitch_deg);
void                      Flight_Takeoff_Get_Hover_Angle_Compensation(float *roll_deg, float *pitch_deg);

#endif /* FLIGHT_CONTROL_H_ */
