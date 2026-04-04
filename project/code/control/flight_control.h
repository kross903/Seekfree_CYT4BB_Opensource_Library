#ifndef FLIGHT_CONTROL_H_
#define FLIGHT_CONTROL_H_

#include "control/pid_controller.h"
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

    float target_angle_roll_trim;
    float target_angle_pitch_trim;
    float target_angle_yaw_trim;

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

void Flight_Control_Set_Target_Angle_Trim(float roll_trim, float pitch_trim, float yaw_trim);
void Flight_Control_Get_Target_Angle_Trim(float *roll_trim, float *pitch_trim, float *yaw_trim);

/*
 * 解锁 / 模式 / 油门接口。
 * 慢速模块如果需要切换飞控激活状态、控制模式或油门目标，
 * 建议统一从这里进入。
 */
void Flight_Control_Arm(void);
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

#endif /* FLIGHT_CONTROL_H_ */
