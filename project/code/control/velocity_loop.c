#include "control/velocity_loop.h"
#include "control/ControlMath.h"
#include "zf_common_headfile.h"

#define VELOCITY_LOOP_DT_S             0.00025f
#define VEL_LOOP_GRAVITY_CMS2          981.0f
#define VEL_LOOP_PITCH_SIGN            (+1.0f)
#define VEL_LOOP_ROLL_SIGN             (+1.0f)

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

static void velocity_loop_reset_diag(void)
{
    g_velocity_loop_x_i_out        = 0.0f;
    g_velocity_loop_y_i_out        = 0.0f;
    g_velocity_loop_pitch_corr_deg = 0.0f;
    g_velocity_loop_roll_corr_deg  = 0.0f;
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
    }
    else
    {
        g_velocity_loop_enabled = 0U;
        velocity_loop_reset_diag();
        velocity_loop_reset_feedback_diag();
        g_velocity_loop_active = 0U;
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

        pitch_corr = -(ax_cmd_cms2 / VEL_LOOP_GRAVITY_CMS2) * RAD2DEG;
        roll_corr  = (ay_cmd_cms2 / VEL_LOOP_GRAVITY_CMS2) * RAD2DEG;

        pitch_corr *= VEL_LOOP_PITCH_SIGN;
        roll_corr  *= VEL_LOOP_ROLL_SIGN;

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
