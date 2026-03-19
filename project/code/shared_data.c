/*********************************************************************************************************************
* 文件名称          shared_data.c
* 功能说明          共享数据定义
********************************************************************************************************************/

#include "shared_data.h"

// 使用 #pragma section 将共享变量放入共享内存区域

// 电机油门值
volatile uint16 shared_motor_throttle[4] = {0, 0, 0, 0};

// 控制标志
volatile uint8 shared_armed = 0;
volatile uint8 shared_control_mode = 0;
volatile float shared_base_throttle = 0.0f;

// IMU数据
volatile float shared_gyro_x = 0.0f;
volatile float shared_gyro_y = 0.0f;
volatile float shared_gyro_z = 0.0f;

// 目标角速度
volatile float shared_target_gyro_roll = 0.0f;
volatile float shared_target_gyro_pitch = 0.0f;
volatile float shared_target_gyro_yaw = 0.0f;

// 按键状态
volatile uint8 shared_key_state[4] = {1, 1, 1, 1};

// 系统状态
volatile uint32 shared_loop_count = 0;

