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
#define VEL_LOOP_D_ACCEL_LIMIT_CMS2     (900.0f)
#define VEL_LOOP_TWO_PI_F               (6.28318530718f)

typedef struct
{
    float previous_velocity_cms;
    float accel_history_cms2[3];
    float filtered_accel_cms2;
    volatile float cutoff_hz;
    uint8 history_index;
    uint8 initialized;
} VelocityLoopDState_t;

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
static VelocityLoopDState_t g_velocity_loop_d_state[2];

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

static float velocity_loop_median3(float a, float b, float c)
{
    float temp;

    if(a > b)
    {
        temp = a;
        a = b;
        b = temp;
    }
    if(b > c)
    {
        temp = b;
        b = c;
        c = temp;
    }
    if(a > b)
    {
        b = a;
    }
    return b;
}

static void velocity_loop_reset_d_axis(VelocityLoopDState_t *state)
{
    if(state == NULL)
    {
        return;
    }

    state->previous_velocity_cms = 0.0f;
    state->accel_history_cms2[0] = 0.0f;
    state->accel_history_cms2[1] = 0.0f;
    state->accel_history_cms2[2] = 0.0f;
    state->filtered_accel_cms2 = 0.0f;
    state->history_index = 0U;
    state->initialized = 0U;
}

static void velocity_loop_reset_d_state(void)
{
    velocity_loop_reset_d_axis(&g_velocity_loop_d_state[VELOCITY_LOOP_AXIS_X]);
    velocity_loop_reset_d_axis(&g_velocity_loop_d_state[VELOCITY_LOOP_AXIS_Y]);
}

static float velocity_loop_update_d_derivative(uint8 axis,
                                               PID_Controller *pid,
                                               float actual_velocity_cms,
                                               float dt_s)
{
    VelocityLoopDState_t *state;
    float raw_accel_cms2;
    float median_accel_cms2;
    float limited_accel_cms2;
    float cutoff_hz;
    float filter_alpha;
    float reference_dt_s;

    if((axis > VELOCITY_LOOP_AXIS_Y) || (pid == NULL))
    {
        return 0.0f;
    }

    state = &g_velocity_loop_d_state[axis];
    if(!isfinite(actual_velocity_cms) || !isfinite(dt_s) || !(dt_s > 0.0f))
    {
        velocity_loop_reset_d_axis(state);
        return 0.0f;
    }

    if(!state->initialized)
    {
        state->previous_velocity_cms = actual_velocity_cms;
        state->initialized = 1U;
        return 0.0f;
    }

    raw_accel_cms2 = -(actual_velocity_cms - state->previous_velocity_cms) / dt_s;
    state->previous_velocity_cms = actual_velocity_cms;
    if(!isfinite(raw_accel_cms2))
    {
        velocity_loop_reset_d_axis(state);
        state->previous_velocity_cms = actual_velocity_cms;
        state->initialized = 1U;
        return 0.0f;
    }

    state->accel_history_cms2[state->history_index] = raw_accel_cms2;
    state->history_index = (uint8)((state->history_index + 1U) % 3U);
    median_accel_cms2 = velocity_loop_median3(state->accel_history_cms2[0],
                                              state->accel_history_cms2[1],
                                              state->accel_history_cms2[2]);
    limited_accel_cms2 = velocity_loop_limit_float(median_accel_cms2,
                                                   -VEL_LOOP_D_ACCEL_LIMIT_CMS2,
                                                   VEL_LOOP_D_ACCEL_LIMIT_CMS2);

    cutoff_hz = state->cutoff_hz;
    if(!isfinite(cutoff_hz) ||
       cutoff_hz < VELOCITY_LOOP_D_FILTER_MIN_HZ ||
       cutoff_hz > VELOCITY_LOOP_D_FILTER_MAX_HZ)
    {
        cutoff_hz = VELOCITY_LOOP_D_FILTER_DEFAULT_HZ;
    }
    filter_alpha = 1.0f - expf(-VEL_LOOP_TWO_PI_F * cutoff_hz * dt_s);
    filter_alpha = velocity_loop_limit_float(filter_alpha, 0.0f, 1.0f);
    state->filtered_accel_cms2 += filter_alpha *
                                  (limited_accel_cms2 - state->filtered_accel_cms2);

    reference_dt_s = (isfinite(pid->reference_dt_s) && pid->reference_dt_s > 0.0f)
                       ? pid->reference_dt_s
                       : dt_s;
    return state->filtered_accel_cms2 * reference_dt_s;
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
    velocity_loop_reset_d_state();
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
    velocity_loop_reset_d_state();
    velocity_loop_reset_diag();
    velocity_loop_reset_feedback_diag();
    g_velocity_loop_active = 0U;
    velocity_loop_reset_rate_feedforward();
}

void Velocity_Loop_Reset_D_Filter_State(void)
{
    velocity_loop_reset_d_state();
}

uint8 Velocity_Loop_Set_D_Filter_Hz(uint8 axis, float cutoff_hz)
{
    if((axis > VELOCITY_LOOP_AXIS_Y) ||
       !isfinite(cutoff_hz) ||
       cutoff_hz < VELOCITY_LOOP_D_FILTER_MIN_HZ ||
       cutoff_hz > VELOCITY_LOOP_D_FILTER_MAX_HZ)
    {
        return 0U;
    }

    g_velocity_loop_d_state[axis].cutoff_hz = cutoff_hz;
    return 1U;
}

float Velocity_Loop_Get_D_Filter_Hz(uint8 axis)
{
    if(axis > VELOCITY_LOOP_AXIS_Y)
    {
        return 0.0f;
    }
    return g_velocity_loop_d_state[axis].cutoff_hz;
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
        velocity_loop_reset_d_state();
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
        float x_derivative;
        float y_derivative;

        fb_vx_cms = measured_vx_cms;
        fb_vy_cms = measured_vy_cms;

        x_derivative = velocity_loop_update_d_derivative(VELOCITY_LOOP_AXIS_X,
                                                         &vel_pid_x,
                                                         fb_vx_cms,
                                                         VELOCITY_LOOP_DT_S);
        y_derivative = velocity_loop_update_d_derivative(VELOCITY_LOOP_AXIS_Y,
                                                         &vel_pid_y,
                                                         fb_vy_cms,
                                                         VELOCITY_LOOP_DT_S);
        ax_cmd_cms2 = PID_CalculateErrorWithDerivative(
            &vel_pid_x,
            g_velocity_loop_target_vx_cms - fb_vx_cms,
            x_derivative,
            VELOCITY_LOOP_DT_S);
        ay_cmd_cms2 = PID_CalculateErrorWithDerivative(
            &vel_pid_y,
            g_velocity_loop_target_vy_cms - fb_vy_cms,
            y_derivative,
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
        velocity_loop_reset_d_state();
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
