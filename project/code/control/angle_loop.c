#include "control/angle_loop.h"

#define ANGLE_LOOP_DT_S                 0.00025f
#define ANGLE_RATE_CMD_LPF_ALPHA        0.04f

PID_Controller angle_pid_roll;
PID_Controller angle_pid_pitch;
PID_Controller angle_pid_yaw;

float target_roll_rate = 0.0f;
float target_pitch_rate = 0.0f;
float target_yaw_rate = 0.0f;

static uint8 g_angle_rate_cmd_lpf_init = 0U;
static float g_target_roll_rate_lpf = 0.0f;
static float g_target_pitch_rate_lpf = 0.0f;

void Angle_Loop_Reset_Rate_Cmd_Lpf(void)
{
    g_angle_rate_cmd_lpf_init = 0U;
    g_target_roll_rate_lpf = 0.0f;
    g_target_pitch_rate_lpf = 0.0f;
    target_roll_rate = 0.0f;
    target_pitch_rate = 0.0f;
    target_yaw_rate = 0.0f;
}

static void angle_loop_update_rate_cmd_lpf(float roll_rate_raw, float pitch_rate_raw)
{
    if(!g_angle_rate_cmd_lpf_init)
    {
        g_target_roll_rate_lpf = roll_rate_raw;
        g_target_pitch_rate_lpf = pitch_rate_raw;
        g_angle_rate_cmd_lpf_init = 1U;
    }
    else
    {
        g_target_roll_rate_lpf += ANGLE_RATE_CMD_LPF_ALPHA * (roll_rate_raw - g_target_roll_rate_lpf);
        g_target_pitch_rate_lpf += ANGLE_RATE_CMD_LPF_ALPHA * (pitch_rate_raw - g_target_pitch_rate_lpf);
    }

    target_roll_rate = g_target_roll_rate_lpf;
    target_pitch_rate = g_target_pitch_rate_lpf;
}

void Angle_Loop_Update(float target_roll,
                       float target_pitch,
                       float target_yaw,
                       float current_roll,
                       float current_pitch,
                       float current_yaw)
{
    float target_roll_rate_raw = PID_Calculate(&angle_pid_roll, target_roll, current_roll, ANGLE_LOOP_DT_S);
    float target_pitch_rate_raw = PID_Calculate(&angle_pid_pitch, target_pitch, current_pitch, ANGLE_LOOP_DT_S);

    angle_loop_update_rate_cmd_lpf(target_roll_rate_raw, target_pitch_rate_raw);
    target_yaw_rate = PID_Calculate(&angle_pid_yaw, target_yaw, current_yaw, ANGLE_LOOP_DT_S);
}
