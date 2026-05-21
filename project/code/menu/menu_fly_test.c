#include "menu/menu_fly_test.h"

static volatile uint8 fly_test_active = 0U;
static volatile uint16 fly_test_throttle = FLY_TEST_THROTTLE_DEFAULT;
static volatile uint8 fly_test_output_enabled = 0U;
static volatile uint8 fly_test_safety_latched = 0U;
static volatile float fly_test_target_roll = 0.0f;
static volatile float fly_test_target_pitch = 0.0f;
static volatile float fly_test_target_yaw = 0.0f;

static uint16 menu_fly_test_limit_throttle(uint16 throttle)
{
    if(throttle < FLY_TEST_THROTTLE_MIN)
    {
        throttle = FLY_TEST_THROTTLE_MIN;
    }
    else if(throttle > FLY_TEST_THROTTLE_MAX)
    {
        throttle = FLY_TEST_THROTTLE_MAX;
    }

    return throttle;
}

void MenuFlyTest_Start(void)
{
    if(!fly_test_active)
    {
        fly_test_throttle = FLY_TEST_THROTTLE_DEFAULT;
        fly_test_output_enabled = 0U;
        fly_test_safety_latched = 0U;
        fly_test_target_roll = 0.0f;
        fly_test_target_pitch = 0.0f;
        fly_test_target_yaw = 0.0f;
    }

    fly_test_active = 1U;
}

void MenuFlyTest_Stop(void)
{
    fly_test_active = 0U;
    fly_test_output_enabled = 0U;
    fly_test_throttle = FLY_TEST_THROTTLE_DEFAULT;
}

uint8 MenuFlyTest_IsActive(void)
{
    return fly_test_active;
}

void MenuFlyTest_SetThrottle(uint16 throttle)
{
    fly_test_throttle = menu_fly_test_limit_throttle(throttle);
}

uint16 MenuFlyTest_GetThrottle(void)
{
    return fly_test_throttle;
}

void MenuFlyTest_SetOutputEnable(uint8 enable)
{
    fly_test_output_enabled = enable ? 1U : 0U;
}

uint8 MenuFlyTest_GetOutputEnable(void)
{
    return fly_test_output_enabled;
}

void MenuFlyTest_CaptureTarget(float roll, float pitch, float yaw)
{
    fly_test_target_roll = roll;
    fly_test_target_pitch = pitch;
    fly_test_target_yaw = yaw;
}

void MenuFlyTest_GetTarget(float *roll, float *pitch, float *yaw)
{
    if(roll != NULL)
    {
        *roll = fly_test_target_roll;
    }
    if(pitch != NULL)
    {
        *pitch = fly_test_target_pitch;
    }
    if(yaw != NULL)
    {
        *yaw = fly_test_target_yaw;
    }
}

void MenuFlyTest_SetSafetyLatched(uint8 latched)
{
    fly_test_safety_latched = latched ? 1U : 0U;
}

uint8 MenuFlyTest_GetSafetyLatched(void)
{
    return fly_test_safety_latched;
}

void MenuFlyTest_ClearSafetyLatch(void)
{
    fly_test_safety_latched = 0U;
}
