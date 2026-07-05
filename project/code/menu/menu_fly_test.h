#ifndef _MENU_FLY_TEST_H_
#define _MENU_FLY_TEST_H_

#include "zf_common_headfile.h"

#define FLY_TEST_THROTTLE_DEFAULT        400U
#define FLY_TEST_THROTTLE_MIN            300U
#define FLY_TEST_THROTTLE_MAX            700U
#define FLY_TEST_THROTTLE_STEP           20U
#define FLY_TEST_ATTITUDE_SAFE_DEG       30.0f
#define FLY_TEST_ATTITUDE_OUTPUT_LIMIT   50.0f

typedef enum
{
    FLY_TEST_CONFIRM_IDLE = 0,
    FLY_TEST_CONFIRM_WAIT = 1,
    FLY_TEST_CONFIRM_ON = 2
} FlyTestConfirmState_t;

void MenuFlyTest_Start(void);
void MenuFlyTest_Stop(void);
uint8 MenuFlyTest_IsActive(void);
void MenuFlyTest_SetThrottle(uint16 throttle);
uint16 MenuFlyTest_GetThrottle(void);
void MenuFlyTest_SetOutputEnable(uint8 enable);
uint8 MenuFlyTest_GetOutputEnable(void);
void MenuFlyTest_CaptureTarget(float roll, float pitch, float yaw);
void MenuFlyTest_GetTarget(float *roll, float *pitch, float *yaw);
void MenuFlyTest_SetSafetyLatched(uint8 latched);
uint8 MenuFlyTest_GetSafetyLatched(void);
void MenuFlyTest_ClearSafetyLatch(void);
void MenuFlyTest_SetConfirmState(uint8 state);
uint8 MenuFlyTest_GetConfirmState(void);

#endif
