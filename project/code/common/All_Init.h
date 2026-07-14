#ifndef _All_Init_h_
#define _All_Init_h_

#include "zf_common_headfile.h"

// 系统启动流程中使用的菜单状态标志。
extern bool Send_state;

// 执行整机初始化流程，按既定顺序完成外设、菜单、飞控和中断启动。
void Device_Init(void);

// 主板蜂鸣器使用 P19_4；成功调参后由 5ms 周期任务非阻塞播放两声。
void BoardBuzzer_RequestDoubleBeep(void);
void BoardBuzzer_Task5ms(void);

// 进入菜单主循环，通常在初始化完成后按需调用。
void menu_open(void);

// 初始化菜单运行前需要的显示与状态。
void menu_open_init(void);

#endif
