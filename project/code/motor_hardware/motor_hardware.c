#include "motor_hardware/motor_hardware.h"
#include "common/PIT_Demo.h"
#include "communication/dshot600.h"

#define MOTOR_PWM_FREQ_HZ              200U
#define MOTOR_PWM_CH1                  TCPWM_CH09_P05_0
#define MOTOR_PWM_CH2                  TCPWM_CH10_P05_1
#define MOTOR_PWM_CH3                  TCPWM_CH11_P05_2
#define MOTOR_PWM_CH4                  TCPWM_CH12_P05_3
#define MOTOR_PWM_DUTY_MIN             2000U

#define MOTOR_HW_COMP_M1_SCALE         1.00f
#define MOTOR_HW_COMP_M3_SCALE         1.00f
#define MOTOR_HW_COMP_M4_SCALE         1.00f

static uint8 motor_pwm_initialized = 0U;

static uint16 throttle_to_pwm_duty(uint16 throttle)
{
    if(throttle > MOTOR_OUTPUT_SCALE_MAX)
    {
        throttle = MOTOR_OUTPUT_SCALE_MAX;
    }

    return (uint16)(MOTOR_PWM_DUTY_MIN + throttle);
}

#if MOTOR_OUTPUT_PROTOCOL == 1
static uint16 throttle_to_dshot(uint16 throttle)
{
    if(throttle == 0U)
    {
        return DSHOT_DISARM;
    }
    return (uint16)(DSHOT_MIN_THROTTLE +
                    (uint32)throttle * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)
                    / MOTOR_OUTPUT_SCALE_MAX);
}
#endif

static uint16 apply_motor_hw_compensation(uint8 motor_idx, uint16 throttle, uint8 bypass_compensation)
{
    float scale;
    float scaled;

    if(bypass_compensation)
    {
        return throttle;
    }

    if(motor_idx == 0U)
    {
        scale = MOTOR_HW_COMP_M1_SCALE;
    }
    else if(motor_idx == 2U)
    {
        scale = MOTOR_HW_COMP_M3_SCALE;
    }
    else if(motor_idx == 3U)
    {
        scale = MOTOR_HW_COMP_M4_SCALE;
    }
    else
    {
        return throttle;
    }

    scaled = (float)throttle * scale;
    if(scaled > (float)MOTOR_OUTPUT_SCALE_MAX)
    {
        scaled = (float)MOTOR_OUTPUT_SCALE_MAX;
    }
    return (uint16)scaled;
}

void MotorHardware_InitOnce(void)
{
    if(motor_pwm_initialized)
    {
        return;
    }

#if MOTOR_OUTPUT_PROTOCOL == 0
    pwm_init(MOTOR_PWM_CH1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
    pwm_init(MOTOR_PWM_CH2, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
    pwm_init(MOTOR_PWM_CH3, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
    pwm_init(MOTOR_PWM_CH4, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
#else
    dshot_init();
#endif
    motor_pwm_initialized = 1U;
}

void MotorHardware_WriteOutput(const volatile uint16 motor_throttle[4],
                               uint8 armed,
                               uint8 output_enabled,
                               uint8 bypass_compensation)
{
    if(!armed && !output_enabled)
    {
#if MOTOR_OUTPUT_PROTOCOL == 0
        pwm_set_duty(MOTOR_PWM_CH1, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH2, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH3, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH4, MOTOR_PWM_DUTY_MIN);
#else
        dshot_set_all_throttle(0U, 0U, 0U, 0U);
#endif
        return;
    }

#if MOTOR_OUTPUT_PROTOCOL == 0
    pwm_set_duty(MOTOR_PWM_CH1, throttle_to_pwm_duty(apply_motor_hw_compensation(0U, motor_throttle[0], bypass_compensation)));
    pwm_set_duty(MOTOR_PWM_CH2, throttle_to_pwm_duty(apply_motor_hw_compensation(1U, motor_throttle[1], bypass_compensation)));
    pwm_set_duty(MOTOR_PWM_CH3, throttle_to_pwm_duty(apply_motor_hw_compensation(2U, motor_throttle[2], bypass_compensation)));
    pwm_set_duty(MOTOR_PWM_CH4, throttle_to_pwm_duty(apply_motor_hw_compensation(3U, motor_throttle[3], bypass_compensation)));
#else
    dshot_set_all_throttle(
        throttle_to_dshot(apply_motor_hw_compensation(0U, motor_throttle[0], bypass_compensation)),
        throttle_to_dshot(apply_motor_hw_compensation(1U, motor_throttle[1], bypass_compensation)),
        throttle_to_dshot(apply_motor_hw_compensation(2U, motor_throttle[2], bypass_compensation)),
        throttle_to_dshot(apply_motor_hw_compensation(3U, motor_throttle[3], bypass_compensation))
    );
#endif
}
