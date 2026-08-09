#include "control/motor_mixer.h"

#include "common/PIT_Demo.h"
#include "control/ControlMath.h"
#include "control/rate_loop.h"
#include "control/height_manager.h"
#include <math.h>

#define MOTOR_MIXER_TILT_THRUST_COMP_MAX       1.30f
#define MOTOR_MIXER_M0_MASK                    (0x01U)
#define MOTOR_MIXER_M1_MASK                    (0x02U)
#define MOTOR_MIXER_M2_MASK                    (0x04U)
#define MOTOR_MIXER_M3_MASK                    (0x08U)

float motor_output[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint8 s_invalid_output = 0U;
static volatile MotorMixerSaturation_t s_saturation = {0};
static volatile MotorMixerLimitDiag_t s_limit_diag = {0};

static uint16 motor_mixer_output_limit_max(void)
{
    if(SAFE_MAX_MOTOR_OUTPUT_LIMIT > MOTOR_OUTPUT_SCALE_MAX)
    {
        return MOTOR_OUTPUT_SCALE_MAX;
    }

    return SAFE_MAX_MOTOR_OUTPUT_LIMIT;
}

uint16 Motor_Mixer_Limit_Output(float output)
{
    uint16 output_limit = motor_mixer_output_limit_max();

    if(!isfinite(output))
    {
        return 0U;
    }

    if(output > output_limit)
    {
        output = output_limit;
    }
    else if(output < 0.0f)
    {
        output = 0.0f;
    }
    return (uint16)output;
}

static uint16 motor_mixer_flight_limit(float output, uint8 armed_state, float common_throttle)
{
    if((armed_state != 0U) && (common_throttle > 0.0f) && (output < SAFE_MIN_THROTTLE))
    {
        output = SAFE_MIN_THROTTLE;
    }

    return Motor_Mixer_Limit_Output(output);
}

static uint8 motor_mixer_axis_limit_flags(uint8 high_mask,
                                          uint8 low_mask,
                                          uint8 positive_high_mask,
                                          uint8 positive_low_mask)
{
    uint8 flags = 0U;
    uint8 all_motors_mask = (uint8)(MOTOR_MIXER_M0_MASK |
                                    MOTOR_MIXER_M1_MASK |
                                    MOTOR_MIXER_M2_MASK |
                                    MOTOR_MIXER_M3_MASK);
    uint8 negative_high_mask = (uint8)(all_motors_mask & (uint8)~positive_high_mask);
    uint8 negative_low_mask = (uint8)(all_motors_mask & (uint8)~positive_low_mask);

    if((high_mask & positive_high_mask) || (low_mask & positive_low_mask))
    {
        flags |= MOTOR_MIXER_LIMIT_POSITIVE;
    }
    if((high_mask & negative_high_mask) || (low_mask & negative_low_mask))
    {
        flags |= MOTOR_MIXER_LIMIT_NEGATIVE;
    }

    return flags;
}

static void motor_mixer_update_limit_flags(void)
{
    s_limit_diag.roll_limit_flags =
        motor_mixer_axis_limit_flags(s_limit_diag.high_mask,
                                     s_limit_diag.low_mask,
                                     (uint8)(MOTOR_MIXER_M2_MASK | MOTOR_MIXER_M3_MASK),
                                     (uint8)(MOTOR_MIXER_M0_MASK | MOTOR_MIXER_M1_MASK));
    s_limit_diag.pitch_limit_flags =
        motor_mixer_axis_limit_flags(s_limit_diag.high_mask,
                                     s_limit_diag.low_mask,
                                     (uint8)(MOTOR_MIXER_M0_MASK | MOTOR_MIXER_M3_MASK),
                                     (uint8)(MOTOR_MIXER_M1_MASK | MOTOR_MIXER_M2_MASK));
    s_limit_diag.yaw_limit_flags =
        motor_mixer_axis_limit_flags(s_limit_diag.high_mask,
                                     s_limit_diag.low_mask,
                                     (uint8)(MOTOR_MIXER_M1_MASK | MOTOR_MIXER_M3_MASK),
                                     (uint8)(MOTOR_MIXER_M0_MASK | MOTOR_MIXER_M2_MASK));
    if(s_limit_diag.high_mask)
    {
        s_limit_diag.throttle_limit_flags |= MOTOR_MIXER_LIMIT_POSITIVE;
    }
    if(s_limit_diag.low_mask)
    {
        s_limit_diag.throttle_limit_flags |= MOTOR_MIXER_LIMIT_NEGATIVE;
    }
}

static void motor_mixer_update_with_base(volatile uint16 motor_throttle_out[4],
                                         uint8 armed_state,
                                         float thrust_base,
                                         float roll_deg,
                                         float pitch_deg)
{
    uint8 i = 0U;
    uint16 output_limit = motor_mixer_output_limit_max();
    float roll_rad;
    float pitch_rad;
    float tilt_cos;
    float thrust_base_comp;

    s_invalid_output = 0U;
    s_saturation.high = 0U;
    s_saturation.low = 0U;
    s_saturation.roll_limited = 0U;
    s_saturation.pitch_limited = 0U;
    s_limit_diag.high_mask = 0U;
    s_limit_diag.low_mask = 0U;
    s_limit_diag.roll_limit_flags = 0U;
    s_limit_diag.pitch_limit_flags = 0U;
    s_limit_diag.yaw_limit_flags = 0U;
    s_limit_diag.throttle_limit_flags = 0U;
    s_limit_diag.invalid_output = 0U;
    roll_rad = roll_deg * DEG2RAD;
    pitch_rad = pitch_deg * DEG2RAD;
    tilt_cos = cosf(roll_rad) * cosf(pitch_rad);
    if(tilt_cos < (1.0f / MOTOR_MIXER_TILT_THRUST_COMP_MAX))
    {
        tilt_cos = (1.0f / MOTOR_MIXER_TILT_THRUST_COMP_MAX);
    }

    thrust_base_comp = thrust_base / tilt_cos;
    motor_output[3] = thrust_base_comp + roll_output + pitch_output + yaw_output;
    motor_output[2] = thrust_base_comp + roll_output - pitch_output - yaw_output;
    motor_output[1] = thrust_base_comp - roll_output - pitch_output + yaw_output;
    motor_output[0] = thrust_base_comp - roll_output + pitch_output - yaw_output;

    for(i = 0U; i < 4U; i++)
    {
        if(!isfinite(motor_output[i]))
        {
            motor_output[i] = 0.0f;
            s_invalid_output = 1U;
            s_limit_diag.invalid_output = 1U;
        }
        else
        {
            if(motor_output[i] > (float)output_limit)
            {
                s_saturation.high = 1U;
                s_limit_diag.high_mask |= (uint8)(1U << i);
            }
            if((armed_state != 0U) && (thrust_base_comp > 0.0f) &&
               (motor_output[i] < SAFE_MIN_THROTTLE))
            {
                s_saturation.low = 1U;
                s_limit_diag.low_mask |= (uint8)(1U << i);
            }
        }
        s_limit_diag.raw_motor[i] = motor_output[i];
        motor_throttle_out[i] = motor_mixer_flight_limit(motor_output[i], armed_state, thrust_base_comp);
        s_limit_diag.clamped_motor[i] = motor_throttle_out[i];
    }
    motor_mixer_update_limit_flags();
    if(s_saturation.high || s_saturation.low)
    {
        s_saturation.roll_limited = (uint8)(fabsf(roll_output) > 0.01f);
        s_saturation.pitch_limited = (uint8)(fabsf(pitch_output) > 0.01f);
    }
}

void Motor_Mixer_Update(volatile uint16 motor_throttle_out[4],
                        uint8 armed_state,
                        float base_throttle_input,
                        float roll_deg,
                        float pitch_deg)
{
    float thrust_base = HeightSpeedPID_output + base_throttle_input;

    if((armed_state != 0U) && (base_throttle_input > 0.0f) && (thrust_base < SAFE_MIN_THROTTLE))
    {
        thrust_base = SAFE_MIN_THROTTLE;
    }

    motor_mixer_update_with_base(motor_throttle_out,
                                 armed_state,
                                 thrust_base,
                                 roll_deg,
                                 pitch_deg);
}

uint8 Motor_Mixer_HasInvalidOutput(void)
{
    return s_invalid_output;
}

void Motor_Mixer_Get_Saturation(MotorMixerSaturation_t *saturation)
{
    uint32 irq_state;

    if(saturation == NULL)
    {
        return;
    }
    irq_state = __get_PRIMASK();
    __disable_irq();
    *saturation = s_saturation;
    if(irq_state == 0U)
    {
        __enable_irq();
    }
}

void Motor_Mixer_Get_Limit_Diag(MotorMixerLimitDiag_t *diag)
{
    uint32 irq_state;

    if(diag == NULL)
    {
        return;
    }
    irq_state = __get_PRIMASK();
    __disable_irq();
    *diag = s_limit_diag;
    if(irq_state == 0U)
    {
        __enable_irq();
    }
}

void Motor_Mixer_Update_BaseOnly(volatile uint16 motor_throttle_out[4],
                                 uint8 armed_state,
                                 float base_throttle_input,
                                 float roll_deg,
                                 float pitch_deg)
{
    if((armed_state != 0U) && (base_throttle_input > 0.0f) && (base_throttle_input < SAFE_MIN_THROTTLE))
    {
        base_throttle_input = SAFE_MIN_THROTTLE;
    }

    motor_mixer_update_with_base(motor_throttle_out,
                                 armed_state,
                                 base_throttle_input,
                                 roll_deg,
                                 pitch_deg);
}
