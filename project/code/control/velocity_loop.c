#include "control/velocity_loop.h"
#include "control/angle_loop.h"
#include "control/ControlMath.h"
#include "zf_common_headfile.h"
#include <math.h>

#define VELOCITY_LOOP_DT_S             0.004f
#define VEL_LOOP_GRAVITY_CMS2          981.0f
#define VEL_LOOP_PITCH_SIGN            (1.0f)
#define VEL_LOOP_ROLL_SIGN             (-1.0f)
#define VEL_LOOP_RATE_FF_LPF_ALPHA      (0.31407784f) /* 15 Hz @ 250 Hz */
#define VEL_LOOP_RATE_FF_GAIN_DEFAULT   (0.0f)
#define VEL_LOOP_RATE_FF_GAIN_MAX       (2.0f)
#define VEL_LOOP_RATE_FF_LIMIT_DEFAULT  (60.0f)
#define VEL_LOOP_RATE_FF_LIMIT_MAX      (350.0f)

PID_Controller vel_pid_x;
PID_Controller vel_pid_y;

static volatile uint8 g_velocity_loop_enabled       = 0U;
static volatile float g_velocity_loop_target_vx_cms = 0.0f;
static volatile float g_velocity_loop_target_vy_cms = 0.0f;
static volatile float g_velocity_loop_pitch_corr_deg = 0.0f;
static volatile float g_velocity_loop_roll_corr_deg  = 0.0f;
static volatile float g_velocity_loop_fb_vx_cms      = 0.0f;
static volatile float g_velocity_loop_fb_vy_cms      = 0.0f;
static volatile float g_velocity_loop_x_i_out        = 0.0f;
static volatile float g_velocity_loop_y_i_out        = 0.0f;
static volatile uint8 g_velocity_loop_active         = 0U;
static volatile float g_velocity_loop_pitch_ff_gain = VEL_LOOP_RATE_FF_GAIN_DEFAULT;
static volatile float g_velocity_loop_roll_ff_gain = VEL_LOOP_RATE_FF_GAIN_DEFAULT;
static volatile float g_velocity_loop_pitch_ff_limit_dps = VEL_LOOP_RATE_FF_LIMIT_DEFAULT;
static volatile float g_velocity_loop_roll_ff_limit_dps = VEL_LOOP_RATE_FF_LIMIT_DEFAULT;
static float g_velocity_loop_prev_pitch_corr_deg = 0.0f;
static float g_velocity_loop_prev_roll_corr_deg = 0.0f;
static uint8 g_velocity_loop_rate_ff_initialized = 0U;
static VelocityLoopRateFeedforwardDiag_t g_velocity_loop_rate_ff_diag = {0};
static volatile VelocityLoopAxisDiag_t g_velocity_loop_axis_diag[2];

static float velocity_loop_limit_float(float value, float min_value, float max_value)
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

static void velocity_loop_reset_rate_feedforward(void)
{
    g_velocity_loop_prev_pitch_corr_deg = 0.0f;
    g_velocity_loop_prev_roll_corr_deg = 0.0f;
    g_velocity_loop_rate_ff_initialized = 0U;
    g_velocity_loop_rate_ff_diag.roll_target_rate_raw_dps = 0.0f;
    g_velocity_loop_rate_ff_diag.pitch_target_rate_raw_dps = 0.0f;
    g_velocity_loop_rate_ff_diag.roll_target_rate_filt_dps = 0.0f;
    g_velocity_loop_rate_ff_diag.pitch_target_rate_filt_dps = 0.0f;
    g_velocity_loop_rate_ff_diag.roll_rate_ff_dps = 0.0f;
    g_velocity_loop_rate_ff_diag.pitch_rate_ff_dps = 0.0f;
    Angle_Loop_Set_Rate_Feedforward(0.0f, 0.0f);
}

static void velocity_loop_update_rate_feedforward(float roll_corr_deg,
                                                  float pitch_corr_deg)
{
    float roll_rate_ff;
    float pitch_rate_ff;

    if(!g_velocity_loop_rate_ff_initialized)
    {
        g_velocity_loop_prev_roll_corr_deg = roll_corr_deg;
        g_velocity_loop_prev_pitch_corr_deg = pitch_corr_deg;
        g_velocity_loop_rate_ff_initialized = 1U;
        Angle_Loop_Set_Rate_Feedforward(0.0f, 0.0f);
        return;
    }

    g_velocity_loop_rate_ff_diag.roll_target_rate_raw_dps =
        (roll_corr_deg - g_velocity_loop_prev_roll_corr_deg) / VELOCITY_LOOP_DT_S;
    g_velocity_loop_rate_ff_diag.pitch_target_rate_raw_dps =
        (pitch_corr_deg - g_velocity_loop_prev_pitch_corr_deg) / VELOCITY_LOOP_DT_S;
    g_velocity_loop_prev_roll_corr_deg = roll_corr_deg;
    g_velocity_loop_prev_pitch_corr_deg = pitch_corr_deg;

    g_velocity_loop_rate_ff_diag.roll_target_rate_filt_dps +=
        VEL_LOOP_RATE_FF_LPF_ALPHA *
        (g_velocity_loop_rate_ff_diag.roll_target_rate_raw_dps -
         g_velocity_loop_rate_ff_diag.roll_target_rate_filt_dps);
    g_velocity_loop_rate_ff_diag.pitch_target_rate_filt_dps +=
        VEL_LOOP_RATE_FF_LPF_ALPHA *
        (g_velocity_loop_rate_ff_diag.pitch_target_rate_raw_dps -
         g_velocity_loop_rate_ff_diag.pitch_target_rate_filt_dps);

    roll_rate_ff = g_velocity_loop_rate_ff_diag.roll_target_rate_filt_dps *
                   g_velocity_loop_roll_ff_gain;
    pitch_rate_ff = g_velocity_loop_rate_ff_diag.pitch_target_rate_filt_dps *
                    g_velocity_loop_pitch_ff_gain;
    roll_rate_ff = velocity_loop_limit_float(roll_rate_ff,
                                             -g_velocity_loop_roll_ff_limit_dps,
                                             g_velocity_loop_roll_ff_limit_dps);
    pitch_rate_ff = velocity_loop_limit_float(pitch_rate_ff,
                                              -g_velocity_loop_pitch_ff_limit_dps,
                                              g_velocity_loop_pitch_ff_limit_dps);
    g_velocity_loop_rate_ff_diag.roll_rate_ff_dps = roll_rate_ff;
    g_velocity_loop_rate_ff_diag.pitch_rate_ff_dps = pitch_rate_ff;
    Angle_Loop_Set_Rate_Feedforward(roll_rate_ff, pitch_rate_ff);
}

static void velocity_loop_reset_diag(void)
{
    VelocityLoopAxisDiag_t empty_axis_diag = {0};

    g_velocity_loop_x_i_out        = 0.0f;
    g_velocity_loop_y_i_out        = 0.0f;
    g_velocity_loop_pitch_corr_deg = 0.0f;
    g_velocity_loop_roll_corr_deg  = 0.0f;
    g_velocity_loop_axis_diag[VELOCITY_LOOP_AXIS_X] = empty_axis_diag;
    g_velocity_loop_axis_diag[VELOCITY_LOOP_AXIS_Y] = empty_axis_diag;
}

static void velocity_loop_update_axis_diag(uint8 axis,
                                           PID_Controller *pid,
                                           float target_velocity_cms,
                                           float actual_velocity_cms,
                                           float controller_output_deg,
                                           float output_sign)
{
    VelocityLoopAxisDiag_t diag = {0};

    diag.target_velocity_cms = target_velocity_cms;
    diag.actual_velocity_cms = actual_velocity_cms;
    diag.error_velocity_cms = target_velocity_cms - actual_velocity_cms;
    diag.p_output_deg = output_sign * pid->Kp * diag.error_velocity_cms;
    diag.i_output_deg = output_sign * pid->Ki * pid->integral;
    diag.d_output_deg = output_sign * pid->Kd * pid->derivative;
    diag.raw_output_deg = diag.p_output_deg + diag.i_output_deg + diag.d_output_deg;
    diag.output_deg = output_sign * controller_output_deg;
    diag.saturated = ((diag.raw_output_deg > pid->output_limit) ||
                      (diag.raw_output_deg < -pid->output_limit)) ? 1U : 0U;
    diag.active = 1U;
    g_velocity_loop_axis_diag[axis] = diag;
}

static void velocity_loop_reset_feedback_diag(void)
{
    g_velocity_loop_fb_vx_cms = 0.0f;
    g_velocity_loop_fb_vy_cms = 0.0f;
}

void Velocity_Loop_Set_Enabled(uint8 enabled)
{
    if(enabled)
    {
        g_velocity_loop_enabled = 1U;
        velocity_loop_reset_rate_feedforward();
    }
    else
    {
        g_velocity_loop_enabled = 0U;
        velocity_loop_reset_diag();
        velocity_loop_reset_feedback_diag();
        g_velocity_loop_active = 0U;
        velocity_loop_reset_rate_feedforward();
    }
}

uint8 Velocity_Loop_Is_Enabled(void)
{
    return g_velocity_loop_enabled;
}

void Velocity_Loop_Set_Target_Velocity(float vx_cms, float vy_cms)
{
    g_velocity_loop_target_vx_cms = vx_cms;
    g_velocity_loop_target_vy_cms = vy_cms;
}

void Velocity_Loop_Get_Target_Velocity(float *vx_cms, float *vy_cms)
{
    if(vx_cms != NULL)
    {
        *vx_cms = g_velocity_loop_target_vx_cms;
    }
    if(vy_cms != NULL)
    {
        *vy_cms = g_velocity_loop_target_vy_cms;
    }
}

void Velocity_Loop_Reset_Runtime(void)
{
    g_velocity_loop_target_vx_cms = 0.0f;
    g_velocity_loop_target_vy_cms = 0.0f;
    PID_Reset(&vel_pid_x);
    PID_Reset(&vel_pid_y);
    velocity_loop_reset_diag();
    velocity_loop_reset_feedback_diag();
    g_velocity_loop_active = 0U;
    velocity_loop_reset_rate_feedforward();
}

uint8 Velocity_Loop_Set_Rate_Feedforward_Parameters(uint8 axis,
                                                    float gain,
                                                    float limit_dps)
{
    if(!isfinite(gain) || !isfinite(limit_dps) ||
       gain < 0.0f || limit_dps < 0.0f)
    {
        return 0U;
    }

    gain = velocity_loop_limit_float(gain, 0.0f, VEL_LOOP_RATE_FF_GAIN_MAX);
    limit_dps = velocity_loop_limit_float(limit_dps, 0.0f, VEL_LOOP_RATE_FF_LIMIT_MAX);
    if(axis == VELOCITY_LOOP_AXIS_X)
    {
        g_velocity_loop_pitch_ff_gain = gain;
        g_velocity_loop_pitch_ff_limit_dps = limit_dps;
    }
    else if(axis == VELOCITY_LOOP_AXIS_Y)
    {
        g_velocity_loop_roll_ff_gain = gain;
        g_velocity_loop_roll_ff_limit_dps = limit_dps;
    }
    else
    {
        return 0U;
    }
    return 1U;
}

void Velocity_Loop_Get_Rate_Feedforward_Parameters(uint8 axis,
                                                   float *gain,
                                                   float *limit_dps)
{
    float axis_gain = 0.0f;
    float axis_limit_dps = 0.0f;

    if(axis == VELOCITY_LOOP_AXIS_X)
    {
        axis_gain = g_velocity_loop_pitch_ff_gain;
        axis_limit_dps = g_velocity_loop_pitch_ff_limit_dps;
    }
    else if(axis == VELOCITY_LOOP_AXIS_Y)
    {
        axis_gain = g_velocity_loop_roll_ff_gain;
        axis_limit_dps = g_velocity_loop_roll_ff_limit_dps;
    }
    if(gain != NULL)
    {
        *gain = axis_gain;
    }
    if(limit_dps != NULL)
    {
        *limit_dps = axis_limit_dps;
    }
}

void Velocity_Loop_Get_Rate_Feedforward_Diag(VelocityLoopRateFeedforwardDiag_t *diag)
{
    if(diag != NULL)
    {
        *diag = g_velocity_loop_rate_ff_diag;
    }
}

uint8 Velocity_Loop_Get_Axis_Diag(uint8 axis, VelocityLoopAxisDiag_t *diag)
{
    uint32 irq_state;

    if((axis > VELOCITY_LOOP_AXIS_Y) || (diag == NULL))
    {
        return 0U;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    *diag = g_velocity_loop_axis_diag[axis];
    if(irq_state == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

void Velocity_Loop_Update_Attitude_Target(uint8 velocity_ready,
                                          float measured_vx_cms,
                                          float measured_vy_cms,
                                          float captured_roll,
                                          float captured_pitch,
                                          float *target_roll,
                                          float *target_pitch)
{
    if((target_roll == NULL) || (target_pitch == NULL))
    {
        return;
    }

    if(!g_velocity_loop_enabled)
    {
        *target_roll = captured_roll;
        *target_pitch = captured_pitch;
        PID_Reset(&vel_pid_x);
        PID_Reset(&vel_pid_y);
        velocity_loop_reset_diag();
        velocity_loop_reset_feedback_diag();
        g_velocity_loop_active = 0U;
        velocity_loop_reset_rate_feedforward();
        return;
    }

    if(velocity_ready)
    {
        float ax_cmd_cms2;
        float ay_cmd_cms2;
        float pitch_corr;
        float roll_corr;
        float fb_vx_cms;
        float fb_vy_cms;

        fb_vx_cms = measured_vx_cms;
        fb_vy_cms = measured_vy_cms;

        ax_cmd_cms2 = PID_Calculate(&vel_pid_x,
                                    g_velocity_loop_target_vx_cms,
                                    fb_vx_cms,
                                    VELOCITY_LOOP_DT_S);
        ay_cmd_cms2 = PID_Calculate(&vel_pid_y,
                                    g_velocity_loop_target_vy_cms,
                                    fb_vy_cms,
                                    VELOCITY_LOOP_DT_S);

        pitch_corr = ax_cmd_cms2;
        roll_corr  = ay_cmd_cms2;

        pitch_corr *= VEL_LOOP_PITCH_SIGN;
        roll_corr  *= VEL_LOOP_ROLL_SIGN;

        velocity_loop_update_axis_diag(VELOCITY_LOOP_AXIS_X,
                                       &vel_pid_x,
                                       g_velocity_loop_target_vx_cms,
                                       fb_vx_cms,
                                       ax_cmd_cms2,
                                       VEL_LOOP_PITCH_SIGN);
        velocity_loop_update_axis_diag(VELOCITY_LOOP_AXIS_Y,
                                       &vel_pid_y,
                                       g_velocity_loop_target_vy_cms,
                                       fb_vy_cms,
                                       ay_cmd_cms2,
                                       VEL_LOOP_ROLL_SIGN);

        velocity_loop_update_rate_feedforward(roll_corr, pitch_corr);

        *target_pitch = captured_pitch + pitch_corr;
        *target_roll  = captured_roll  + roll_corr;

        g_velocity_loop_x_i_out        = vel_pid_x.Ki * vel_pid_x.integral;
        g_velocity_loop_y_i_out        = vel_pid_y.Ki * vel_pid_y.integral;
        g_velocity_loop_pitch_corr_deg = pitch_corr;
        g_velocity_loop_roll_corr_deg  = roll_corr;
        g_velocity_loop_fb_vx_cms      = fb_vx_cms;
        g_velocity_loop_fb_vy_cms      = fb_vy_cms;
        g_velocity_loop_active = 1U;
    }
    else
    {
        *target_roll = captured_roll;
        *target_pitch = captured_pitch;
        PID_Reset(&vel_pid_x);
        PID_Reset(&vel_pid_y);
        velocity_loop_reset_diag();
        velocity_loop_reset_feedback_diag();
        g_velocity_loop_active = 0U;
        velocity_loop_reset_rate_feedforward();
    }
}

void Velocity_Loop_Get_Diag(float *pitch_corr_deg,
                            float *roll_corr_deg,
                            float *fb_vx_cms,
                            float *fb_vy_cms,
                            float *x_i_out,
                            float *y_i_out,
                            uint8 *active)
{
    if(pitch_corr_deg != NULL) *pitch_corr_deg = g_velocity_loop_pitch_corr_deg;
    if(roll_corr_deg  != NULL) *roll_corr_deg  = g_velocity_loop_roll_corr_deg;
    if(fb_vx_cms      != NULL) *fb_vx_cms      = g_velocity_loop_fb_vx_cms;
    if(fb_vy_cms      != NULL) *fb_vy_cms      = g_velocity_loop_fb_vy_cms;
    if(x_i_out        != NULL) *x_i_out        = g_velocity_loop_x_i_out;
    if(y_i_out        != NULL) *y_i_out        = g_velocity_loop_y_i_out;
    if(active         != NULL) *active         = g_velocity_loop_active;
}
