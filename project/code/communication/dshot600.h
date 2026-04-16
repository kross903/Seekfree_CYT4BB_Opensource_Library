/*********************************************************************************************************************
* 文件名称          dshot600.h
* 功能说明          DShot600 协议驱动头文件
********************************************************************************************************************/

#ifndef _DSHOT600_H_
#define _DSHOT600_H_

#include "zf_common_headfile.h"

// ==================== 硬件配置 ====================
// #define MOTOR1_PIN              P05_0
// #define MOTOR2_PIN              P05_1
// #define MOTOR3_PIN              P05_2
// #define MOTOR4_PIN              P05_3
#define MOTOR1_PIN             P05_1 
#define MOTOR2_PIN             P05_2
#define MOTOR3_PIN             P05_3
#define MOTOR4_PIN             P05_4

#define LED1                    P20_9
#define LED2                    P20_8
#define LED3                    P21_5
#define LED4                    P21_4

#define KEY1                    P20_6
#define KEY2                    P20_7
#define KEY3                    P11_2
#define KEY4                    P11_3

#define SWITCH1                 P33_11
#define SWITCH2                 P33_12

// ==================== DShot 协议参数 ====================
#define DSHOT_MOTOR_NUM         4
#define DSHOT_MIN_THROTTLE      300
#define DSHOT_MAX_THROTTLE      2047
#define DSHOT_DISARM            0

// DShot 特殊命令
#define DSHOT_CMD_MOTOR_STOP    0
#define DSHOT_CMD_BEEP1         1
#define DSHOT_CMD_BEEP2         2
#define DSHOT_CMD_BEEP3         3
#define DSHOT_CMD_BEEP4         4
#define DSHOT_CMD_BEEP5         5

// ==================== 控制模式 ====================
typedef enum {
    MODE_ALL_SAME = 0,      // 所有电机输出相同油门
    MODE_INDIVIDUAL,        // 单独设置每个电机
    MODE_PAIR,              // 成对设置电机
    MODE_DIAGONAL           // 对角设置电机
} control_mode_t;

// ==================== 电机状态结构体 ====================
typedef struct {
    uint16 throttle[DSHOT_MOTOR_NUM];   // 四路电机油门值
    uint8 armed;                        // 电机解锁状态
    control_mode_t mode;                // 当前电机控制模式
    uint8 speed_level;                  // 速度档位
} motor_state_t;

// ==================== 油门控制参数 ====================
#define THROTTLE_STEP           50      // 普通调节步进
#define THROTTLE_STEP_FAST      100     // 快速调节步进

// ==================== 电调控制接口 ====================
// 初始化 DShot 输出引脚、状态变量和默认油门。
void dshot_init(void);

// 给指定电机发送一帧油门数据。
void dshot_send_throttle(uint8 motor_index, uint16 throttle, uint8 telemetry);

// 同时设置四个电机的输出油门。
void dshot_set_all_throttle(uint16 throttle1, uint16 throttle2, uint16 throttle3, uint16 throttle4);

// 给指定电机发送 DShot 特殊命令。
void dshot_send_command(uint8 motor_index, uint8 command);

// 让全部电机立即停转。
void dshot_motor_stop_all(void);

// 以下函数已废弃，保留仅为兼容性。
void dshot_key_scan(void);          // 已废弃：按键由 CPU1 处理
void dshot_update_motors(void);     // 已废弃：电机由共享变量控制

// 读取当前电机状态快照。
motor_state_t* dshot_get_state(void);

// 更新当前电机解锁状态。
void dshot_set_armed(uint8 armed);

#endif
