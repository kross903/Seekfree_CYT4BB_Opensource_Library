#include "control/motor_mixer.h"

#include "common/PIT_Demo.h"
#include "control/ControlMath.h"
#include "control/rate_loop.h"
#include "kalman_filter/height_manager.h"

#define MOTOR_MIXER_TILT_THRUST_COMP_MAX       1.30f

float motor_output[4] = {0.0f, 0.0f, 0.0f, 0.0f};

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
    if((armed_state != 0U) && (common_throttle >= SAFE_MIN_THROTTLE) && (output < SAFE_MIN_THROTTLE))
    {
        output = SAFE_MIN_THROTTLE;
    }

    return Motor_Mixer_Limit_Output(output);
}

void Motor_Mixer_Update(volatile uint16 motor_throttle_out[4],
                        uint8 armed_state,
                        float base_throttle_input,
                        float roll_deg,
                        float pitch_deg)
{
    uint8 i = 0U;
    float roll_rad;
    float pitch_rad;
    float tilt_cos;
    float thrust_base;
    float thrust_base_comp;

    roll_rad = roll_deg * DEG2RAD;
    pitch_rad = pitch_deg * DEG2RAD;
    tilt_cos = cosf(roll_rad) * cosf(pitch_rad);
    if(tilt_cos < (1.0f / MOTOR_MIXER_TILT_THRUST_COMP_MAX))
    {
        tilt_cos = (1.0f / MOTOR_MIXER_TILT_THRUST_COMP_MAX);
    }

    thrust_base = HeightSpeedPID_output + base_throttle_input;
    thrust_base_comp = thrust_base / tilt_cos;

    motor_output[3] = thrust_base_comp + roll_output + pitch_output - yaw_output;
    motor_output[2] = thrust_base_comp + roll_output - pitch_output + yaw_output;
    motor_output[1] = thrust_base_comp - roll_output + pitch_output + yaw_output;
    motor_output[0] = thrust_base_comp - roll_output - pitch_output - yaw_output;

    for(i = 0U; i < 4U; i++)
    {
        motor_throttle_out[i] = motor_mixer_flight_limit(motor_output[i], armed_state, thrust_base_comp);
    }
}
