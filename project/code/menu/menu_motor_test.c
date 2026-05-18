#include "menu/menu_motor_test.h"
#include "common/PIT_Demo.h"

static volatile uint8 motor_test_active = 0U;
static volatile uint8 motor_test_selected = 0U;
static volatile uint16 motor_test_throttle = 0U;
static volatile uint8 motor_test_output_enabled = 0U;

static void menu_motor_test_clear_output(volatile uint16 motor_throttle[4])
{
    motor_throttle[0] = 0U;
    motor_throttle[1] = 0U;
    motor_throttle[2] = 0U;
    motor_throttle[3] = 0U;
}

void MenuMotorTest_Start(void)
{
    if(!motor_test_active)
    {
        motor_test_selected = 0U;
        motor_test_throttle = 0U;
        motor_test_output_enabled = 0U;
    }

    motor_test_active = 1U;
}

void MenuMotorTest_Stop(void)
{
    motor_test_active = 0U;
    motor_test_output_enabled = 0U;
    motor_test_throttle = 0U;
}

uint8 MenuMotorTest_IsActive(void)
{
    return motor_test_active;
}

void MenuMotorTest_SetSelected(uint8 motor_index)
{
    if(motor_index >= 4U)
    {
        motor_index = 3U;
    }

    motor_test_selected = motor_index;
}

uint8 MenuMotorTest_GetSelected(void)
{
    return motor_test_selected;
}

void MenuMotorTest_SetThrottle(uint16 throttle)
{
    uint16 output_limit = MenuMotorTest_GetOutputLimit();
    uint16 test_limit = MOTOR_TEST_MAX_THROTTLE;

    if(test_limit > output_limit)
    {
        test_limit = output_limit;
    }

    if(throttle > test_limit)
    {
        throttle = test_limit;
    }

    motor_test_throttle = throttle;
}

uint16 MenuMotorTest_GetThrottle(void)
{
    return motor_test_throttle;
}

void MenuMotorTest_SetOutputEnable(uint8 enable)
{
    motor_test_output_enabled = enable ? 1U : 0U;
}

uint8 MenuMotorTest_GetOutputEnable(void)
{
    return motor_test_output_enabled;
}

uint16 MenuMotorTest_GetOutputLimit(void)
{
    if(SAFE_MAX_MOTOR_OUTPUT_LIMIT > MOTOR_OUTPUT_SCALE_MAX)
    {
        return MOTOR_OUTPUT_SCALE_MAX;
    }

    return SAFE_MAX_MOTOR_OUTPUT_LIMIT;
}

void MenuMotorTest_ApplyOutput(volatile uint16 motor_throttle[4])
{
    menu_motor_test_clear_output(motor_throttle);

    if(!motor_test_output_enabled)
    {
        return;
    }

    if(motor_test_selected < 4U)
    {
        motor_throttle[motor_test_selected] = motor_test_throttle;
    }
}
