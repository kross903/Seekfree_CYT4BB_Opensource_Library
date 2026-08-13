#include "control/motor_mixer.h"

#include "common/PIT_Demo.h"
#include "control/ControlMath.h"
#include "control/rate_loop.h"
#include "control/height_manager.h"
#include <math.h>

#define MOTOR_MIXER_TILT_THRUST_COMP_MAX       1.30f
#define MOTOR_MIXER_AXIS_EPSILON               (0.0001f)

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

static float motor_mixer_clamp_float(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }
    if(value < min_value)
    {
        return min_value;
    }
    return value;
}

static uint8 motor_mixer_direction_flag(float value)
{
    if(value > MOTOR_MIXER_AXIS_EPSILON)
    {
        return MOTOR_MIXER_LIMIT_POSITIVE;
    }
    if(value < -MOTOR_MIXER_AXIS_EPSILON)
    {
        return MOTOR_MIXER_LIMIT_NEGATIVE;
    }
    return 0U;
}

static void motor_mixer_find_min_max(const float values[4],
                                     float *min_value,
                                     float *max_value)
{
    uint8 i;

    *min_value = values[0];
    *max_value = values[0];
    for(i = 1U; i < 4U; i++)
    {
        if(values[i] < *min_value) *min_value = values[i];
        if(values[i] > *max_value) *max_value = values[i];
    }
}

/* 求保持Roll/Pitch不变时，Yaw差动量可以完整装入电机动态范围的最大比例。 */
static float motor_mixer_calculate_yaw_scale(const float roll_pitch[4],
                                             const float yaw[4],
                                             float output_range)
{
    uint8 i;
    uint8 j;
    float scale = 1.0f;

    for(i = 0U; i < 4U; i++)
    {
        for(j = 0U; j < 4U; j++)
        {
            float yaw_difference = yaw[i] - yaw[j];
            if(yaw_difference > MOTOR_MIXER_AXIS_EPSILON)
            {
                float allowed = (output_range - (roll_pitch[i] - roll_pitch[j])) /
                                yaw_difference;
                if(allowed < scale)
                {
                    scale = allowed;
                }
            }
        }
    }

    return motor_mixer_clamp_float(scale, 0.0f, 1.0f);
}

static void motor_mixer_update_with_base(volatile uint16 motor_throttle_out[4],
                                         uint8 armed_state,
                                         float thrust_base,
                                         float roll_deg,
                                         float pitch_deg)
{
    uint8 i = 0U;
    uint16 output_limit = motor_mixer_output_limit_max();
    float output_min;
    float output_max = (float)output_limit;
    float output_range;
    float roll_rad;
    float pitch_rad;
    float tilt_cos;
    float thrust_base_comp;
    float requested_roll_pitch[4];
    float roll_pitch[4];
    float yaw[4];
    float attitude[4];
    float min_attitude;
    float max_attitude;
    float roll_pitch_scale = 1.0f;
    float yaw_scale = 1.0f;
    float throttle_min;
    float throttle_max;
    float applied_throttle;

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
    s_limit_diag.roll_pitch_scale = 1.0f;
    s_limit_diag.yaw_scale = 1.0f;
    s_limit_diag.requested_throttle = 0.0f;
    s_limit_diag.applied_throttle = 0.0f;
    roll_rad = roll_deg * DEG2RAD;
    pitch_rad = pitch_deg * DEG2RAD;
    tilt_cos = cosf(roll_rad) * cosf(pitch_rad);
    if(tilt_cos < (1.0f / MOTOR_MIXER_TILT_THRUST_COMP_MAX))
    {
        tilt_cos = (1.0f / MOTOR_MIXER_TILT_THRUST_COMP_MAX);
    }

    thrust_base_comp = thrust_base / tilt_cos;
    output_min = ((armed_state != 0U) && (thrust_base_comp > 0.0f)) ?
                 (float)SAFE_MIN_THROTTLE : 0.0f;
    if(output_min > output_max)
    {
        output_min = output_max;
    }
    output_range = output_max - output_min;

    if(!isfinite(thrust_base_comp) || !isfinite(roll_output) ||
       !isfinite(pitch_output) || !isfinite(yaw_output))
    {
        s_invalid_output = 1U;
        s_limit_diag.invalid_output = 1U;
        for(i = 0U; i < 4U; i++)
        {
            motor_output[i] = 0.0f;
            s_limit_diag.raw_motor[i] = 0.0f;
            s_limit_diag.allocated_motor[i] = 0.0f;
            s_limit_diag.clamped_motor[i] = 0U;
            motor_throttle_out[i] = 0U;
        }
        return;
    }

    /* X型四轴姿态差动：Roll/Pitch最高优先级，Yaw使用剩余动态范围。 */
    roll_pitch[3] = +roll_output + pitch_output;
    roll_pitch[2] = +roll_output - pitch_output;
    roll_pitch[1] = -roll_output - pitch_output;
    roll_pitch[0] = -roll_output + pitch_output;
    yaw[3] = +yaw_output;
    yaw[2] = -yaw_output;
    yaw[1] = +yaw_output;
    yaw[0] = -yaw_output;
    for(i = 0U; i < 4U; i++)
    {
        requested_roll_pitch[i] = roll_pitch[i];
    }

    motor_mixer_find_min_max(roll_pitch, &min_attitude, &max_attitude);
    if((max_attitude - min_attitude) > output_range)
    {
        roll_pitch_scale = output_range / (max_attitude - min_attitude);
        for(i = 0U; i < 4U; i++)
        {
            roll_pitch[i] *= roll_pitch_scale;
        }
        s_limit_diag.roll_limit_flags = motor_mixer_direction_flag(roll_output);
        s_limit_diag.pitch_limit_flags = motor_mixer_direction_flag(pitch_output);
    }

    yaw_scale = motor_mixer_calculate_yaw_scale(roll_pitch, yaw, output_range);
    if(yaw_scale < 1.0f)
    {
        s_limit_diag.yaw_limit_flags = motor_mixer_direction_flag(yaw_output);
    }
    for(i = 0U; i < 4U; i++)
    {
        attitude[i] = roll_pitch[i] + yaw_scale * yaw[i];
    }
    motor_mixer_find_min_max(attitude, &min_attitude, &max_attitude);

    /* 公共油门只使用姿态差动量剩余的空间，不改变已分配的姿态比例。 */
    throttle_min = output_min - min_attitude;
    throttle_max = output_max - max_attitude;
    applied_throttle = motor_mixer_clamp_float(thrust_base_comp,
                                               throttle_min,
                                               throttle_max);
    if(thrust_base_comp > throttle_max)
    {
        s_limit_diag.throttle_limit_flags |= MOTOR_MIXER_LIMIT_POSITIVE;
    }
    if(thrust_base_comp < throttle_min)
    {
        s_limit_diag.throttle_limit_flags |= MOTOR_MIXER_LIMIT_NEGATIVE;
    }

    s_limit_diag.roll_pitch_scale = roll_pitch_scale;
    s_limit_diag.yaw_scale = yaw_scale;
    s_limit_diag.requested_throttle = thrust_base_comp;
    s_limit_diag.applied_throttle = applied_throttle;

    for(i = 0U; i < 4U; i++)
    {
        float raw_output;

        raw_output = thrust_base_comp + requested_roll_pitch[i] + yaw[i];
        s_limit_diag.raw_motor[i] = raw_output;
        if(raw_output > output_max)
        {
            s_limit_diag.high_mask |= (uint8)(1U << i);
        }
        if(raw_output < output_min)
        {
            s_limit_diag.low_mask |= (uint8)(1U << i);
        }

        motor_output[i] = applied_throttle + attitude[i];
        s_limit_diag.allocated_motor[i] = motor_output[i];
    }

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
        motor_throttle_out[i] = motor_mixer_flight_limit(motor_output[i], armed_state, applied_throttle);
        s_limit_diag.clamped_motor[i] = motor_throttle_out[i];
    }
    s_saturation.high =
        (uint8)(((s_limit_diag.throttle_limit_flags & MOTOR_MIXER_LIMIT_POSITIVE) != 0U) ||
                (s_limit_diag.roll_limit_flags != 0U) ||
                (s_limit_diag.pitch_limit_flags != 0U) ||
                (s_limit_diag.yaw_limit_flags != 0U));
    s_saturation.low =
        (uint8)(((s_limit_diag.throttle_limit_flags & MOTOR_MIXER_LIMIT_NEGATIVE) != 0U) ||
                (s_limit_diag.roll_limit_flags != 0U) ||
                (s_limit_diag.pitch_limit_flags != 0U) ||
                (s_limit_diag.yaw_limit_flags != 0U));
    s_saturation.roll_limited = (uint8)(s_limit_diag.roll_limit_flags != 0U);
    s_saturation.pitch_limited = (uint8)(s_limit_diag.pitch_limit_flags != 0U);
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

void Motor_Mixer_Reset_State(void)
{
    uint8 i;
    MotorMixerSaturation_t empty_saturation = {0};
    MotorMixerLimitDiag_t empty_diag = {0};

    for(i = 0U; i < 4U; i++)
    {
        motor_output[i] = 0.0f;
    }
    empty_diag.roll_pitch_scale = 1.0f;
    empty_diag.yaw_scale = 1.0f;
    s_invalid_output = 0U;
    s_saturation = empty_saturation;
    s_limit_diag = empty_diag;
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
