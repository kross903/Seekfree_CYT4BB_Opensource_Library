#include "control/angle_loop.h"
#include <math.h>

#define ANGLE_LOOP_DT_S                 0.002f
#define ANGLE_RATE_CMD_LPF_CUTOFF_HZ    26.0f
#define ANGLE_TWO_PI                    6.28318530718f

PID_Controller angle_pid_roll;
PID_Controller angle_pid_pitch;
PID_Controller angle_pid_yaw;

float target_roll_rate = 0.0f;
float target_pitch_rate = 0.0f;
float target_yaw_rate = 0.0f;
float yaw_angle_error_deg = 0.0f;

static uint8 g_angle_rate_cmd_lpf_init = 0U;
static float g_target_roll_rate_lpf = 0.0f;
static float g_target_pitch_rate_lpf = 0.0f;
static float g_angle_rate_cmd_lpf_alpha = 0.0f;
static volatile float g_roll_rate_ff_dps = 0.0f;
static volatile float g_pitch_rate_ff_dps = 0.0f;
static AngleLoopRateFeedforwardDiag_t g_rate_ff_diag = {0};

static float angle_loop_limit_rate(float value, float limit)
{
    if(!isfinite(value))
    {
        return 0.0f;
    }
    if(!isfinite(limit) || limit <= 0.0f)
    {
        return 0.0f;
    }
    if(value > limit)
    {
        return limit;
    }
    if(value < -limit)
    {
        return -limit;
    }
    return value;
}

void Angle_Loop_InitTiming(void)
{
    g_angle_rate_cmd_lpf_alpha = 1.0f -
                                 expf(-ANGLE_TWO_PI * ANGLE_RATE_CMD_LPF_CUTOFF_HZ * ANGLE_LOOP_DT_S);
}

float Angle_WrapDeg180(float angle_deg)
{
    float wrapped;

    if(!isfinite(angle_deg))
    {
        return angle_deg;
    }
    if(angle_deg >= -180.0f && angle_deg < 180.0f)
    {
        return angle_deg;
    }

    wrapped = fmodf(angle_deg + 180.0f, 360.0f);
    if(wrapped < 0.0f)
    {
        wrapped += 360.0f;
    }
    return wrapped - 180.0f;
}

void Angle_Loop_Set_Rate_Feedforward(float roll_dps, float pitch_dps)
{
    g_roll_rate_ff_dps = isfinite(roll_dps) ? roll_dps : 0.0f;
    g_pitch_rate_ff_dps = isfinite(pitch_dps) ? pitch_dps : 0.0f;
}

void Angle_Loop_Reset_Rate_Feedforward(void)
{
    g_roll_rate_ff_dps = 0.0f;
    g_pitch_rate_ff_dps = 0.0f;
    g_rate_ff_diag.roll_pid_rate_dps = 0.0f;
    g_rate_ff_diag.pitch_pid_rate_dps = 0.0f;
    g_rate_ff_diag.roll_rate_ff_dps = 0.0f;
    g_rate_ff_diag.pitch_rate_ff_dps = 0.0f;
    g_rate_ff_diag.roll_rate_total_raw_dps = 0.0f;
    g_rate_ff_diag.pitch_rate_total_raw_dps = 0.0f;
    g_rate_ff_diag.roll_rate_final_dps = 0.0f;
    g_rate_ff_diag.pitch_rate_final_dps = 0.0f;
}

void Angle_Loop_Get_Rate_Feedforward_Diag(AngleLoopRateFeedforwardDiag_t *diag)
{
    if(diag != NULL)
    {
        *diag = g_rate_ff_diag;
    }
}

void Angle_Loop_Reset_Rate_Cmd_Lpf(void)
{
    g_angle_rate_cmd_lpf_init = 0U;
    g_target_roll_rate_lpf = 0.0f;
    g_target_pitch_rate_lpf = 0.0f;
    target_roll_rate = 0.0f;
    target_pitch_rate = 0.0f;
    target_yaw_rate = 0.0f;
    yaw_angle_error_deg = 0.0f;
    Angle_Loop_Reset_Rate_Feedforward();
}

static void angle_loop_update_rate_cmd_lpf(float roll_rate_raw, float pitch_rate_raw)
{
    if(g_angle_rate_cmd_lpf_alpha <= 0.0f)
    {
        Angle_Loop_InitTiming();
    }

    if(!g_angle_rate_cmd_lpf_init)
    {
        g_target_roll_rate_lpf = roll_rate_raw;
        g_target_pitch_rate_lpf = pitch_rate_raw;
        g_angle_rate_cmd_lpf_init = 1U;
    }
    else
    {
        g_target_roll_rate_lpf += g_angle_rate_cmd_lpf_alpha * (roll_rate_raw - g_target_roll_rate_lpf);
        g_target_pitch_rate_lpf += g_angle_rate_cmd_lpf_alpha * (pitch_rate_raw - g_target_pitch_rate_lpf);
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
    float roll_pid_rate = PID_Calculate(&angle_pid_roll, target_roll, current_roll, ANGLE_LOOP_DT_S);
    float pitch_pid_rate = PID_Calculate(&angle_pid_pitch, target_pitch, current_pitch, ANGLE_LOOP_DT_S);
    float roll_rate_ff = g_roll_rate_ff_dps;
    float pitch_rate_ff = g_pitch_rate_ff_dps;
    float target_roll_rate_raw = angle_loop_limit_rate(roll_pid_rate + roll_rate_ff,
                                                       angle_pid_roll.output_limit);
    float target_pitch_rate_raw = angle_loop_limit_rate(pitch_pid_rate + pitch_rate_ff,
                                                        angle_pid_pitch.output_limit);

    angle_loop_update_rate_cmd_lpf(target_roll_rate_raw, target_pitch_rate_raw);
    g_rate_ff_diag.roll_pid_rate_dps = roll_pid_rate;
    g_rate_ff_diag.pitch_pid_rate_dps = pitch_pid_rate;
    g_rate_ff_diag.roll_rate_ff_dps = roll_rate_ff;
    g_rate_ff_diag.pitch_rate_ff_dps = pitch_rate_ff;
    g_rate_ff_diag.roll_rate_total_raw_dps = target_roll_rate_raw;
    g_rate_ff_diag.pitch_rate_total_raw_dps = target_pitch_rate_raw;
    g_rate_ff_diag.roll_rate_final_dps = target_roll_rate;
    g_rate_ff_diag.pitch_rate_final_dps = target_pitch_rate;
    yaw_angle_error_deg = Angle_WrapDeg180(target_yaw - current_yaw);
    target_yaw_rate = PID_CalculateError(&angle_pid_yaw, yaw_angle_error_deg, ANGLE_LOOP_DT_S);
}
