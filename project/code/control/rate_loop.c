#include "control/rate_loop.h"

#define RATE_LOOP_DT_S                  0.00025f
#define RATE_LOOP_FREQ_HZ               4000.0f
#define RATE_LOOP_DTERM_LPF_HZ          70.0f
#define RATE_LOOP_TWO_PI                6.28318530718f
#define RATE_LOOP_DTERM_LPF_ALPHA       ((RATE_LOOP_TWO_PI * RATE_LOOP_DTERM_LPF_HZ * RATE_LOOP_DT_S) / \
                                        ((RATE_LOOP_TWO_PI * RATE_LOOP_DTERM_LPF_HZ * RATE_LOOP_DT_S) + 1.0f))
#define RATE_LOOP_AXIS_ROLL             0U
#define RATE_LOOP_AXIS_PITCH            1U
#define RATE_LOOP_AXIS_YAW              2U

PID_Controller gyro_pid_roll;
PID_Controller gyro_pid_pitch;
PID_Controller gyro_pid_yaw;

float roll_output = 0.0f;
float pitch_output = 0.0f;
float yaw_output = 0.0f;

typedef struct
{
    float raw_gyro;
    float filtered_gyro;
    float prev_filtered_gyro;
    float raw_derivative;
    float d_output;
    uint8 initialized;
} RateDTermAxisState_t;

static RateDTermAxisState_t s_roll_dterm;
static RateDTermAxisState_t s_pitch_dterm;

static float rate_loop_limit_output(float output, float limit)
{
    if(output > limit)
    {
        output = limit;
    }
    else if(output < -limit)
    {
        output = -limit;
    }

    return output;
}

static void rate_loop_reset_dterm_axis_state(RateDTermAxisState_t *state)
{
    state->raw_gyro = 0.0f;
    state->filtered_gyro = 0.0f;
    state->prev_filtered_gyro = 0.0f;
    state->raw_derivative = 0.0f;
    state->d_output = 0.0f;
    state->initialized = 0U;
}

static float rate_loop_apply_dterm(RateDTermAxisState_t *state,
                                   const PID_Controller *pid,
                                   float current_gyro)
{
    state->raw_gyro = current_gyro;

    if(!state->initialized)
    {
        state->filtered_gyro = current_gyro;
        state->prev_filtered_gyro = current_gyro;
        state->raw_derivative = 0.0f;
        state->d_output = 0.0f;
        state->initialized = 1U;
        return 0.0f;
    }

    state->filtered_gyro += RATE_LOOP_DTERM_LPF_ALPHA * (current_gyro - state->filtered_gyro);
    state->raw_derivative = -(state->filtered_gyro - state->prev_filtered_gyro) * RATE_LOOP_FREQ_HZ;
    state->prev_filtered_gyro = state->filtered_gyro;
    state->d_output = pid->Kd * state->raw_derivative;

    return state->d_output;
}

void Rate_Loop_Reset_Output(void)
{
    roll_output = 0.0f;
    pitch_output = 0.0f;
    yaw_output = 0.0f;
    Rate_Loop_Reset_DTerm();
}

void Rate_Loop_Reset_DTerm(void)
{
    rate_loop_reset_dterm_axis_state(&s_roll_dterm);
    rate_loop_reset_dterm_axis_state(&s_pitch_dterm);
}

void Rate_Loop_Reset_DTerm_Axis(uint8 axis)
{
    switch(axis)
    {
        case RATE_LOOP_AXIS_ROLL:
            rate_loop_reset_dterm_axis_state(&s_roll_dterm);
            break;

        case RATE_LOOP_AXIS_PITCH:
            rate_loop_reset_dterm_axis_state(&s_pitch_dterm);
            break;

        case RATE_LOOP_AXIS_YAW:
        default:
            break;
    }
}

void Rate_Loop_Get_DTerm_Diag(RateLoopDTermDiag_t *diag)
{
    if(diag == NULL)
    {
        return;
    }

    diag->roll_raw_gyro = s_roll_dterm.raw_gyro;
    diag->roll_filtered_gyro = s_roll_dterm.filtered_gyro;
    diag->roll_raw_derivative = s_roll_dterm.raw_derivative;
    diag->roll_d_output = s_roll_dterm.d_output;
    diag->pitch_raw_gyro = s_pitch_dterm.raw_gyro;
    diag->pitch_filtered_gyro = s_pitch_dterm.filtered_gyro;
    diag->pitch_raw_derivative = s_pitch_dterm.raw_derivative;
    diag->pitch_d_output = s_pitch_dterm.d_output;
}

void Rate_Loop_Update(float target_roll_rate,
                      float target_pitch_rate,
                      float target_yaw_rate,
                      float current_gyro_roll,
                      float current_gyro_pitch,
                      float current_gyro_yaw)
{
    float roll_pi = PID_Calculate_PI_Unlimited(&gyro_pid_roll, target_roll_rate, current_gyro_roll, RATE_LOOP_DT_S);
    float pitch_pi = PID_Calculate_PI_Unlimited(&gyro_pid_pitch, target_pitch_rate, current_gyro_pitch, RATE_LOOP_DT_S);
    float yaw_pi = PID_Calculate_PI_Unlimited(&gyro_pid_yaw, target_yaw_rate, current_gyro_yaw, RATE_LOOP_DT_S);
    float roll_d = rate_loop_apply_dterm(&s_roll_dterm, &gyro_pid_roll, current_gyro_roll);
    float pitch_d = rate_loop_apply_dterm(&s_pitch_dterm, &gyro_pid_pitch, current_gyro_pitch);

    roll_output = rate_loop_limit_output(roll_pi + roll_d, gyro_pid_roll.output_limit);
    pitch_output = rate_loop_limit_output(pitch_pi + pitch_d, gyro_pid_pitch.output_limit);
    yaw_output = -rate_loop_limit_output(yaw_pi, gyro_pid_yaw.output_limit);
}
