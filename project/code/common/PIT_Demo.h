#ifndef _PIT_Demo_h_
#define _PIT_Demo_h_

#include "zf_common_headfile.h"
#include "common/battery_monitor.h"
#include "common/flight_safety.h"
#include "control/angle_loop.h"
#include "control/ground_remote_input.h"
#include "control/motor_mixer.h"
#include "control/pid_controller.h"
#include "control/position_loop.h"
#include "control/rate_loop.h"
#include "control/throttle_runtime.h"
#include "control/velocity_loop.h"
#include "pmw/optical_flow_feed.h"

/* PIT_Demo is the live flight-runtime interface kept stable for ISR, UI, VOFA, and menu callers. */

// ==================== 电机输出协议选择 ====================
// 0 = 标准 PWM（旧电调，200Hz，计数器值 2000~4000）
// 1 = DShot600（新电调，GPIO 模拟时序，值域 0 / 300~2047）
#define MOTOR_OUTPUT_PROTOCOL       1

// ==================== 安全限制参数 ====================
#define MOTOR_OUTPUT_SCALE_MAX      2000U
#define SAFE_MAX_MOTOR_OUTPUT_LIMIT MOTOR_OUTPUT_SCALE_MAX
#define SAFE_MAX_THROTTLE           ((float)SAFE_MAX_MOTOR_OUTPUT_LIMIT)
#define SAFE_MIN_THROTTLE           300.0f
#define MOTOR_TEST_MAX_THROTTLE     450U
#define MOTOR_TEST_STEP             50U

// ==================== Flight runtime state ====================
extern volatile uint8 armed;
extern volatile uint8 control_mode;

extern volatile uint16 motor_throttle[4];
extern uint16 throttle_buffer[4];

// ==================== Attitude feedback ====================
extern float current_roll;
extern float current_pitch;
extern float current_yaw;

extern float current_gyro_roll;
extern float current_gyro_pitch;
extern float current_gyro_yaw;

// ==================== 姿态目标量 ====================
extern volatile float target_angle_roll;
extern volatile float target_angle_pitch;
extern volatile float target_angle_yaw;


/* 位置闭环 adc/ToF 双数据校验观测量。 */
extern volatile float pos_fusion_r_adc;
extern volatile float pos_fusion_r_tof;
extern volatile float pos_fusion_r_fused;
extern volatile float pos_fusion_residual;
extern volatile uint8 pos_fusion_fault;
// ==================== Flight-runtime interface ====================
/*
 * [2026-03-10]
 * Purpose:
 * 1. Keep the existing Flight_Control_* call sites stable while using one shared throttle chain again.
 * 2. The `mode` parameter is retained only for compatibility with existing wrappers.
 */
void Flight_Runtime_Reset_All_Throttle_States(void);

void Flight_Motor_Test_Start(void);
void Flight_Motor_Test_Stop(void);
uint8 Flight_Motor_Test_Is_Active(void);
void Flight_Motor_Test_Set_Selected(uint8 motor_index);
uint8 Flight_Motor_Test_Get_Selected(void);
void Flight_Motor_Test_Set_Throttle(uint16 throttle);
uint16 Flight_Motor_Test_Get_Throttle(void);
void Flight_Motor_Test_Set_Output_Enable(uint8 enable);
uint8 Flight_Motor_Test_Get_Output_Enable(void);
uint16 Flight_Motor_Test_Get_Output_Limit(void);

void Flight_Fly_Test_Start(void);
void Flight_Fly_Test_Stop(void);
uint8 Flight_Fly_Test_Is_Active(void);
void Flight_Fly_Test_Set_Throttle(uint16 throttle);
uint16 Flight_Fly_Test_Get_Throttle(void);
void Flight_Fly_Test_Set_Output_Enable(uint8 enable);
uint8 Flight_Fly_Test_Get_Output_Enable(void);
void Flight_Fly_Test_Get_Target(float *roll, float *pitch, float *yaw);
uint8 Flight_Fly_Test_Get_Safety_Latched(void);
void Flight_Fly_Test_Clear_Safety_Latch(void);
void Flight_Fly_Test_Set_Confirm_State(uint8 state);
uint8 Flight_Fly_Test_Get_Confirm_State(void);

void PIT_KeyVoltage_Task_5ms(void);
void PIT_Adc_Task_5ms(void);
void PIT_OpticalFlow_Task(void);
void PIT_PMW3901_Task_20ms(void);
void PIT_Control_Task_250us(void);
void PIT_Motor_Task_500us(void);

#endif
