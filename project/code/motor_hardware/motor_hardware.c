#include "motor_hardware/motor_hardware.h"
#include "common/PIT_Demo.h"
#include "common/battery_monitor.h"
#include "communication/dshot600.h"
#include "motor_hardware/small_driver_uart_control.h"
#include <math.h>

#define MOTOR_PWM_FREQ_HZ              200U
#define MOTOR_PWM_CH1                  TCPWM_CH09_P05_0
#define MOTOR_PWM_CH2                  TCPWM_CH10_P05_1
#define MOTOR_PWM_CH3                  TCPWM_CH11_P05_2
#define MOTOR_PWM_CH4                  TCPWM_CH12_P05_3
// #define MOTOR_PWM_CH1                  TCPWM_CH12_P05_3
// #define MOTOR_PWM_CH2                  TCPWM_CH11_P05_2
// #define MOTOR_PWM_CH3                  TCPWM_CH10_P05_1
// #define MOTOR_PWM_CH4                  TCPWM_CH09_P05_0
#define MOTOR_PWM_DUTY_MIN             2000U

// 1  2     4   3
// 3  4     2   1


#define MOTOR_HW_COMP_M1_SCALE         1.00f
#define MOTOR_HW_COMP_M3_SCALE         1.00f
#define MOTOR_HW_COMP_M4_SCALE         1.00f

static uint8 motor_pwm_initialized = 0U;
static MotorHardwareVoltageCompDiag s_voltage_comp_diag = {0};

#if MOTOR_OUTPUT_PROTOCOL == 2
static int16 throttle_to_uart_duty(uint16 throttle)
{
    if(throttle > MOTOR_OUTPUT_SCALE_MAX)
    {
        throttle = MOTOR_OUTPUT_SCALE_MAX;
    }

    return (int16)((uint32)throttle * PWM_DUTY_MAX / MOTOR_OUTPUT_SCALE_MAX);
}
#endif

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

static uint16 apply_voltage_compensation(uint8 motor_idx, uint16 throttle, uint8 bypass_compensation)
{
    float gain = bypass_compensation ? 1.0f : BatteryMonitor_GetCompensationGain();
    float compensated = (float)throttle * gain;

    s_voltage_comp_diag.raw_output[motor_idx] = (float)throttle;
    s_voltage_comp_diag.compensation_gain = gain;
    s_voltage_comp_diag.bypassed = bypass_compensation ? 1U : 0U;

    if(!isfinite(compensated) || !isfinite(gain))
    {
        compensated = 0.0f;
        s_voltage_comp_diag.invalid_output = 1U;
    }

    s_voltage_comp_diag.compensated_output[motor_idx] = compensated;
    if(compensated > (float)MOTOR_OUTPUT_SCALE_MAX)
    {
        compensated = (float)MOTOR_OUTPUT_SCALE_MAX;
        s_voltage_comp_diag.saturation_mask |= (uint8)(1U << motor_idx);
    }
    else if(compensated < 0.0f)
    {
        compensated = 0.0f;
        s_voltage_comp_diag.saturation_mask |= (uint8)(1U << motor_idx);
    }

    return (uint16)compensated;
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
#elif MOTOR_OUTPUT_PROTOCOL == 1
    dshot_init();
#else
    small_driver_uart_init();
#endif
    motor_pwm_initialized = 1U;
}

void MotorHardware_WriteOutput(const volatile uint16 motor_throttle[4],
                               uint8 armed,
                               uint8 output_enabled,
                               uint8 bypass_compensation)
{
    uint16 output[4];
    uint8 i;

    s_voltage_comp_diag.saturation_mask = 0U;
    s_voltage_comp_diag.invalid_output = 0U;

    if(!armed && !output_enabled)
    {
#if MOTOR_OUTPUT_PROTOCOL == 0
        pwm_set_duty(MOTOR_PWM_CH1, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH2, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH3, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH4, MOTOR_PWM_DUTY_MIN);
#elif MOTOR_OUTPUT_PROTOCOL == 1
        dshot_set_all_throttle(0U, 0U, 0U, 0U);
#else
        small_driver_set_duty(0, 0, 0, 0);
        small_driver_service_tx();
#endif
        return;
    }

    for(i = 0U; i < 4U; i++)
    {
        output[i] = apply_voltage_compensation(i, motor_throttle[i], bypass_compensation);
        output[i] = apply_motor_hw_compensation(i, output[i], bypass_compensation);
    }

#if MOTOR_OUTPUT_PROTOCOL == 0
    pwm_set_duty(MOTOR_PWM_CH1, throttle_to_pwm_duty(output[0]));
    pwm_set_duty(MOTOR_PWM_CH2, throttle_to_pwm_duty(output[1]));
    pwm_set_duty(MOTOR_PWM_CH3, throttle_to_pwm_duty(output[2]));
    pwm_set_duty(MOTOR_PWM_CH4, throttle_to_pwm_duty(output[3]));
#elif MOTOR_OUTPUT_PROTOCOL == 1
    dshot_set_all_throttle(
        throttle_to_dshot(output[0]),
        throttle_to_dshot(output[1]),
        throttle_to_dshot(output[2]),
        throttle_to_dshot(output[3])
    );
#else
    small_driver_set_duty(
        throttle_to_uart_duty(output[0]),
        throttle_to_uart_duty(output[1]),
        throttle_to_uart_duty(output[2]),
        throttle_to_uart_duty(output[3])
    );
    small_driver_service_tx();
#endif
}

void MotorHardware_GetVoltageCompDiag(MotorHardwareVoltageCompDiag *diag)
{
    if(diag != NULL)
    {
        uint32 irq_state = __get_PRIMASK();

        __disable_irq();
        *diag = s_voltage_comp_diag;
        if(irq_state == 0U)
        {
            __enable_irq();
        }
    }
}

uint8 MotorHardware_IsInitialized(void)
{
    return motor_pwm_initialized;
}
