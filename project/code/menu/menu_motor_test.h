#ifndef _MENU_MOTOR_TEST_H_
#define _MENU_MOTOR_TEST_H_

#include "zf_common_headfile.h"

void MenuMotorTest_Start(void);
void MenuMotorTest_Stop(void);
uint8 MenuMotorTest_IsActive(void);
void MenuMotorTest_SetSelected(uint8 motor_index);
uint8 MenuMotorTest_GetSelected(void);
void MenuMotorTest_SetThrottle(uint16 throttle);
uint16 MenuMotorTest_GetThrottle(void);
void MenuMotorTest_SetOutputEnable(uint8 enable);
uint8 MenuMotorTest_GetOutputEnable(void);
uint16 MenuMotorTest_GetOutputLimit(void);
void MenuMotorTest_ApplyOutput(volatile uint16 motor_throttle[4]);

#endif
