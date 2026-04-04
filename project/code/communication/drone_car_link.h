#ifndef _DRONE_CAR_LINK_H_
#define _DRONE_CAR_LINK_H_

#include "zf_common_headfile.h"

/*
 * 无人机 <-> 小车 有线串口通信模块
 *
 * 设计原则：
 * 1. 尽量少改现有 ISR 骨架，复用 UART4 的接收中断入口。
 * 2. 串口接收在中断回调里逐字节拼帧，避免底层 16 字节小缓冲导致丢包。
 * 3. 命令应用放到 10ms 周期任务里，避免把业务逻辑塞进高频控制 ISR。
 * 4. 状态回传放在主循环里发送，避免在中断里做阻塞串口发送。
 *
 * 当前链路使用 UART4：
 * TX -> P14_1
 * RX -> P14_0
 */

/* 无人机侧用于和小车通信的串口号。 */
#define DRONE_CAR_LINK_UART_INDEX           UART_4
/* 无人机侧串口发送引脚。 */
#define DRONE_CAR_LINK_UART_TX_PIN          UART4_TX_P14_1
/* 无人机侧串口接收引脚。 */
#define DRONE_CAR_LINK_UART_RX_PIN          UART4_RX_P14_0
/* 链路波特率，双方必须一致。 */
#define DRONE_CAR_LINK_UART_BAUDRATE        115200U

/* 10ms 服务任务使用的 PIT 通道。 */
#define DRONE_CAR_LINK_SERVICE_PIT          PIT_CH10

/* 命令帧标志位：请求解锁/保持解锁。 */
#define DRONE_CAR_LINK_FLAG_ARMED           0x01U
/* 命令帧标志位：使用外部角度目标而不是本地摇杆 ADC。 */
#define DRONE_CAR_LINK_FLAG_EXTERNAL_ANGLE  0x02U
/* 命令帧标志位：请求急停。 */
#define DRONE_CAR_LINK_FLAG_ESTOP           0x04U

/*
 * 函数简介：初始化有线通信模块。
 * 主要工作：
 * 1. 配置 UART4 为普通串口通信模式。
 * 2. 把 UART4 接收中断回调挂到本模块的接收函数。
 * 3. 打开 UART4 接收中断。
 *
 * 调用时机：
 * 系统初始化阶段调用一次即可。
 */
void DroneCarLink_Init(void);

/*
 * 函数简介：UART4 接收中断回调函数。
 * 主要工作：
 * 1. 从底层串口缓冲中尽快把收到的字节取走。
 * 2. 用状态机查找帧头、长度、校验。
 * 3. 收到完整合法命令帧后，缓存成“待处理命令”。
 *
 * 调用关系：
 * uart4_isr() -> uart_receiver_handler() -> DroneCarLink_UartRxCallback()
 *
 * 备注：
 * 这个函数设计给中断里调用，不建议在普通主循环里直接手动调用。
 */
void DroneCarLink_UartRxCallback(void);

/*
 * 函数简介：10ms 通信服务任务。
 * 主要工作：
 * 1. 取出中断回调缓存好的最新命令。
 * 2. 把小车发来的命令应用到飞控状态：
 *    - 解锁/上锁
 *    - 控制模式切换
 *    - 油门目标更新
 *    - 外部角度目标更新
 * 3. 维护链路超时计数，失联时触发安全处理。
 * 4. 按周期置位状态回传请求。
 *
 * 调用时机：
 * 推荐挂到一个低频 PIT 中断里，例如当前工程里的 10ms 通道。
 */
void DroneCarLink_Service_Task_10ms(void);

/*
 * 函数简介：主循环发送任务。
 * 主要工作：
 * 1. 检查是否到了状态回传时机。
 * 2. 组包当前飞控状态。
 * 3. 通过 UART4 阻塞发送状态帧给小车。
 *
 * 调用时机：
 * 在主循环、监视页面刷新循环、菜单循环等非高频路径中反复调用。
 *
 * 备注：
 * 之所以不放到 PIT 中断里，是因为 uart_write_buffer() 为阻塞发送，
 * 放到中断里会拖长 ISR 执行时间。
 */
void DroneCarLink_MainLoopTask(void);

/*
 * 函数简介：查询当前是否启用了“外部角度目标控制”。
 * 返回值：
 * 1 表示链路有效，且当前命令要求飞控使用串口下发的姿态目标。
 * 0 表示继续使用本地原有控制输入（例如 ADC 摇杆路径）。
 *
 * 使用场景：
 * 在高频控制环里判断 roll/pitch/yaw 目标到底从哪里来。
 */
uint8 DroneCarLink_UsesExternalTargets(void);

/*
 * 函数简介：读取当前有效的外部姿态目标。
 * 参数说明：
 * roll  - 输出外部目标横滚角（单位：度）
 * pitch - 输出外部目标俯仰角（单位：度）
 * yaw   - 输出外部目标航向角（单位：度）
 *
 * 备注：
 * 只有在 DroneCarLink_UsesExternalTargets() 返回 1 时，这组值才会被
 * 控制环作为当前有效目标使用。
 */
void DroneCarLink_Get_Target_Angles(float *roll, float *pitch, float *yaw);

#endif
