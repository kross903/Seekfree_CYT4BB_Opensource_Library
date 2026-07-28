#include "control/position_loop.h"
#include "vision/vehicle_marker_localization.h"

#define POSITION_LOOP_TARGET_X_CM      (0.0f)
#define POSITION_LOOP_TARGET_Y_CM      (0.0f)
/* Visual position input now uses X-forward positive, Y-left positive.
 * PID_Calculate() uses error = target - current, so the loop output must flip
 * sign to keep a front/left target commanding positive front/left velocity. */
#define POSITION_LOOP_VX_SIGN          (1.0f)
#define POSITION_LOOP_VY_SIGN          (1.0f)
#define POSITION_LOOP_DEADZONE_CM      (3.0f)
#define POSITION_LOOP_PREDICT_RELEASE_EPS_CM (1.0f)
#define POSITION_LOOP_DT_S             0.020f
#define POSITION_LOOP_CORRECTION_LIMIT_CMS  (70.0f)
#define POSITION_LOOP_TARGET_VEL_LIMIT_CMS  (150.0f)
#define POSITION_LOOP_TARGET_VEL_SLEW_CMS2  (400.0f)
#define POSITION_LOOP_CAR_FF_LPF_HZ          (5.0f)
#define POSITION_LOOP_CAR_FF_GAIN_DEFAULT    (1.0f)
#define POSITION_LOOP_CAR_FF_GAIN_MAX        (1.5f)

PID_Controller pos_pid_x;
PID_Controller pos_pid_y;

static volatile uint8 g_position_loop_enabled        = 0U;
static volatile uint8 g_position_loop_active         = 0U;
static volatile uint8 g_position_loop_ir_track_valid = 0U;
static volatile uint8 g_position_loop_velocity_ready = 0U;
static volatile float g_position_loop_fb_x_cm        = 0.0f;
static volatile float g_position_loop_fb_y_cm        = 0.0f;
static volatile float g_position_loop_error_x_cm     = 0.0f;
static volatile float g_position_loop_error_y_cm     = 0.0f;
static volatile float g_position_loop_vx_cmd_cms     = 0.0f;
static volatile float g_position_loop_vy_cmd_cms     = 0.0f;
static volatile float g_position_loop_correction_vx_cms = 0.0f;
static volatile float g_position_loop_correction_vy_cms = 0.0f;
static volatile float g_position_loop_car_ff_raw_vx_cms = 0.0f;
static volatile float g_position_loop_car_ff_raw_vy_cms = 0.0f;
static volatile float g_position_loop_car_ff_filtered_vx_cms = 0.0f;
static volatile float g_position_loop_car_ff_filtered_vy_cms = 0.0f;
static volatile float g_position_loop_car_ff_gain_x = POSITION_LOOP_CAR_FF_GAIN_DEFAULT;
static volatile float g_position_loop_car_ff_gain_y = POSITION_LOOP_CAR_FF_GAIN_DEFAULT;
static volatile float g_position_loop_x_i_out        = 0.0f;
static volatile float g_position_loop_y_i_out        = 0.0f;
static volatile uint8 g_position_loop_car_ff_valid    = 0U;
static volatile uint8 g_position_loop_limit_flags     = 0U;
static uint8 g_position_loop_car_ff_lpf_initialized = 0U;
static uint8 g_position_loop_last_pose_valid = 0U;
static uint8 g_position_loop_predict_hold_active = 0U;
static float g_position_loop_last_x_cm = 0.0f;
static float g_position_loop_last_y_cm = 0.0f;
static float g_position_loop_hold_start_x_cm = 0.0f;
static float g_position_loop_hold_start_y_cm = 0.0f;
static float g_position_loop_hold_distance_cm = 0.0f;
static float g_position_loop_hold_dx_cm = 0.0f;
static float g_position_loop_hold_dy_cm = 0.0f;
static uint32 g_position_loop_last_pose_seq = 0U;

static uint8 position_loop_limit_vector(float *x, float *y, float limit)
{
    float magnitude;
    float scale;

    if((NULL == x) || (NULL == y) || (limit <= 0.0f))
    {
        return 0U;
    }

    magnitude = sqrtf((*x) * (*x) + (*y) * (*y));
    if((magnitude <= limit) || (magnitude <= 0.0f))
    {
        return 0U;
    }

    scale = limit / magnitude;
    *x *= scale;
    *y *= scale;
    return 1U;
}

static uint8 position_loop_limit_vector_slew(float target_x,
                                             float target_y,
                                             float previous_x,
                                             float previous_y,
                                             float *output_x,
                                             float *output_y)
{
    float delta_x = target_x - previous_x;
    float delta_y = target_y - previous_y;
    float max_step = POSITION_LOOP_TARGET_VEL_SLEW_CMS2 * POSITION_LOOP_DT_S;
    uint8 limited;

    limited = position_loop_limit_vector(&delta_x, &delta_y, max_step);
    *output_x = previous_x + delta_x;
    *output_y = previous_y + delta_y;
    return limited;
}

static void position_loop_update_car_feedforward(
    const PositionLoopFeedforwardInput_t *feedforward)
{
    float target_x = 0.0f;
    float target_y = 0.0f;
    float tau_s = 1.0f / (6.283185307f * POSITION_LOOP_CAR_FF_LPF_HZ);
    float alpha = POSITION_LOOP_DT_S / (tau_s + POSITION_LOOP_DT_S);

    g_position_loop_car_ff_valid = 0U;
    g_position_loop_car_ff_raw_vx_cms = 0.0f;
    g_position_loop_car_ff_raw_vy_cms = 0.0f;
    if((NULL != feedforward) && feedforward->valid &&
       isfinite(feedforward->vx_uav_cms) && isfinite(feedforward->vy_uav_cms))
    {
        g_position_loop_car_ff_valid = 1U;
        g_position_loop_car_ff_raw_vx_cms = feedforward->vx_uav_cms;
        g_position_loop_car_ff_raw_vy_cms = feedforward->vy_uav_cms;
        target_x = feedforward->vx_uav_cms * g_position_loop_car_ff_gain_x;
        target_y = feedforward->vy_uav_cms * g_position_loop_car_ff_gain_y;
    }

    if(!g_position_loop_car_ff_lpf_initialized)
    {
        g_position_loop_car_ff_filtered_vx_cms = target_x;
        g_position_loop_car_ff_filtered_vy_cms = target_y;
        g_position_loop_car_ff_lpf_initialized = 1U;
    }
    else
    {
        g_position_loop_car_ff_filtered_vx_cms +=
            alpha * (target_x - g_position_loop_car_ff_filtered_vx_cms);
        g_position_loop_car_ff_filtered_vy_cms +=
            alpha * (target_y - g_position_loop_car_ff_filtered_vy_cms);
    }
}

static void position_loop_set_output(float vx_cms, float vy_cms, float *target_vx_cms, float *target_vy_cms)
{
    if(target_vx_cms != NULL)
    {
        *target_vx_cms = vx_cms;
    }
    if(target_vy_cms != NULL)
    {
        *target_vy_cms = vy_cms;
    }
}

static void position_loop_reset_runtime(void)
{
    PID_Reset(&pos_pid_x);
    PID_Reset(&pos_pid_y);
    g_position_loop_active = 0U;
    g_position_loop_vx_cmd_cms = 0.0f;
    g_position_loop_vy_cmd_cms = 0.0f;
    g_position_loop_correction_vx_cms = 0.0f;
    g_position_loop_correction_vy_cms = 0.0f;
    g_position_loop_error_x_cm = 0.0f;
    g_position_loop_error_y_cm = 0.0f;
    g_position_loop_x_i_out = 0.0f;
    g_position_loop_y_i_out = 0.0f;
    g_position_loop_limit_flags = 0U;
    Position_Loop_Reset_Car_Velocity_Feedforward();
    g_position_loop_last_pose_valid = 0U;
    g_position_loop_predict_hold_active = 0U;
    g_position_loop_last_x_cm = 0.0f;
    g_position_loop_last_y_cm = 0.0f;
    g_position_loop_hold_start_x_cm = 0.0f;
    g_position_loop_hold_start_y_cm = 0.0f;
    g_position_loop_hold_distance_cm = 0.0f;
    g_position_loop_hold_dx_cm = 0.0f;
    g_position_loop_hold_dy_cm = 0.0f;
    g_position_loop_last_pose_seq = g_vehicle_marker_pose_update_seq;
}

static void position_loop_reset_predict_hold(void)
{
    g_position_loop_predict_hold_active = 0U;
    g_position_loop_hold_start_x_cm = 0.0f;
    g_position_loop_hold_start_y_cm = 0.0f;
    g_position_loop_hold_distance_cm = 0.0f;
    g_position_loop_hold_dx_cm = 0.0f;
    g_position_loop_hold_dy_cm = 0.0f;
}

static float position_loop_apply_deadzone(float error_cm)
{
    if(error_cm >= -POSITION_LOOP_DEADZONE_CM && error_cm <= POSITION_LOOP_DEADZONE_CM)
    {
        return 0.0f;
    }

    return error_cm;
}

static uint8 position_loop_get_raw_ir_track_valid(void)
{
    VehicleMarkerLossDebug_t ir_loss_debug = {0};

    VehicleMarkerLocalization_GetLossDebug(&ir_loss_debug);

    return (uint8)((ir_loss_debug.raw_detect_valid && g_vehicle_marker_pose.valid && !g_vehicle_marker_position_lost_flag) ? 1U : 0U);
}

static uint8 position_loop_start_predict_hold(void)
{
    if(!g_position_loop_last_pose_valid)
    {
        return 0U;
    }

    g_position_loop_hold_start_x_cm = g_position_loop_last_x_cm;
    g_position_loop_hold_start_y_cm = g_position_loop_last_y_cm;
    g_position_loop_hold_distance_cm = sqrtf(g_position_loop_hold_start_x_cm * g_position_loop_hold_start_x_cm +
                                             g_position_loop_hold_start_y_cm * g_position_loop_hold_start_y_cm);

    if(g_position_loop_hold_distance_cm <= (POSITION_LOOP_DEADZONE_CM + POSITION_LOOP_PREDICT_RELEASE_EPS_CM))
    {
        position_loop_reset_predict_hold();
        return 0U;
    }

    g_position_loop_hold_dx_cm = 0.0f;
    g_position_loop_hold_dy_cm = 0.0f;
    g_position_loop_predict_hold_active = 1U;

    return 1U;
}

static uint8 position_loop_update_predict_hold(float measured_vx_cms,
                                               float measured_vy_cms,
                                               uint8 car_velocity_valid,
                                               float car_vx_cms,
                                               float car_vy_cms,
                                               float *predict_x_cm,
                                               float *predict_y_cm)
{
    float displacement_distance_cm;
    float predicted_x_cm;
    float predicted_y_cm;
    float predicted_distance_cm;
    float relative_vx_cms = -measured_vx_cms;
    float relative_vy_cms = -measured_vy_cms;

    if((predict_x_cm == NULL) || (predict_y_cm == NULL))
    {
        return 0U;
    }

    if(!g_position_loop_predict_hold_active)
    {
        if(!position_loop_start_predict_hold())
        {
            return 0U;
        }
    }

    if(car_velocity_valid)
    {
        relative_vx_cms += car_vx_cms;
        relative_vy_cms += car_vy_cms;
    }
    g_position_loop_hold_dx_cm += relative_vx_cms * POSITION_LOOP_DT_S;
    g_position_loop_hold_dy_cm += relative_vy_cms * POSITION_LOOP_DT_S;
    displacement_distance_cm = sqrtf(g_position_loop_hold_dx_cm * g_position_loop_hold_dx_cm +
                                     g_position_loop_hold_dy_cm * g_position_loop_hold_dy_cm);
    predicted_x_cm = g_position_loop_hold_start_x_cm + g_position_loop_hold_dx_cm;
    predicted_y_cm = g_position_loop_hold_start_y_cm + g_position_loop_hold_dy_cm;
    predicted_distance_cm = sqrtf(predicted_x_cm * predicted_x_cm + predicted_y_cm * predicted_y_cm);

    if((displacement_distance_cm >= (g_position_loop_hold_distance_cm - POSITION_LOOP_PREDICT_RELEASE_EPS_CM)) ||
       (predicted_distance_cm <= POSITION_LOOP_DEADZONE_CM))
    {
        g_position_loop_last_pose_valid = 0U;
        position_loop_reset_predict_hold();
        return 0U;
    }

    *predict_x_cm = predicted_x_cm;
    *predict_y_cm = predicted_y_cm;

    return 1U;
}

void Position_Loop_Set_Enabled(uint8 enabled)
{
    if(enabled)
    {
        g_position_loop_enabled = 1U;
        position_loop_reset_runtime();
    }
    else
    {
        g_position_loop_enabled = 0U;
        g_position_loop_ir_track_valid = 0U;
        g_position_loop_velocity_ready = 0U;
        g_position_loop_fb_x_cm = 0.0f;
        g_position_loop_fb_y_cm = 0.0f;
        position_loop_reset_runtime();
    }
}

uint8 Position_Loop_Is_Enabled(void)
{
    return g_position_loop_enabled;
}

uint8 Position_Loop_Set_Car_Velocity_Feedforward_Gain(uint8 axis, float gain)
{
    if(!isfinite(gain) || (gain < 0.0f) || (gain > POSITION_LOOP_CAR_FF_GAIN_MAX))
    {
        return 0U;
    }
    if(0U == axis)
    {
        g_position_loop_car_ff_gain_x = gain;
    }
    else if(1U == axis)
    {
        g_position_loop_car_ff_gain_y = gain;
    }
    else
    {
        return 0U;
    }
    return 1U;
}

void Position_Loop_Get_Car_Velocity_Feedforward_Gain(float *gain_x, float *gain_y)
{
    if(NULL != gain_x)
    {
        *gain_x = g_position_loop_car_ff_gain_x;
    }
    if(NULL != gain_y)
    {
        *gain_y = g_position_loop_car_ff_gain_y;
    }
}

void Position_Loop_Reset_Car_Velocity_Feedforward(void)
{
    g_position_loop_car_ff_raw_vx_cms = 0.0f;
    g_position_loop_car_ff_raw_vy_cms = 0.0f;
    g_position_loop_car_ff_filtered_vx_cms = 0.0f;
    g_position_loop_car_ff_filtered_vy_cms = 0.0f;
    g_position_loop_car_ff_valid = 0U;
    g_position_loop_car_ff_lpf_initialized = 0U;
}

void Position_Loop_Update_Velocity_Target(uint8 velocity_ready,
                                          float measured_vx_cms,
                                          float measured_vy_cms,
                                          const PositionLoopFeedforwardInput_t *feedforward,
                                          float *target_vx_cms,
                                          float *target_vy_cms)
{
    uint8 raw_ir_track_valid = position_loop_get_raw_ir_track_valid();
    uint8 new_visual_pose = (uint8)(raw_ir_track_valid &&
                                    g_vehicle_marker_pose_update_seq != g_position_loop_last_pose_seq);
    uint8 position_input_valid = 0U;
    float position_input_x_cm = 0.0f;
    float position_input_y_cm = 0.0f;
    float pos_error_x_cm;
    float pos_error_y_cm;
    float raw_correction_x;
    float raw_correction_y;
    float correction_vx_cms;
    float correction_vy_cms;
    float desired_vx_cms;
    float desired_vy_cms;
    float limited_vx_cms;
    float limited_vy_cms;
    float previous_integral_x;
    float previous_integral_y;

    if(!g_position_loop_enabled || !velocity_ready)
    {
        g_position_loop_ir_track_valid = 0U;
        g_position_loop_velocity_ready = velocity_ready;
        g_position_loop_fb_x_cm = 0.0f;
        g_position_loop_fb_y_cm = 0.0f;
        g_position_loop_error_x_cm = 0.0f;
        g_position_loop_error_y_cm = 0.0f;
        position_loop_set_output(0.0f, 0.0f, target_vx_cms, target_vy_cms);
        position_loop_reset_runtime();
        return;
    }

    position_loop_update_car_feedforward(feedforward);

    if(new_visual_pose)
    {
        position_input_valid = 1U;
        position_input_x_cm = g_vehicle_marker_pose.x;
        position_input_y_cm = g_vehicle_marker_pose.y;
        g_position_loop_last_pose_seq = g_vehicle_marker_pose_update_seq;
        g_position_loop_last_pose_valid = 1U;
        g_position_loop_last_x_cm = position_input_x_cm;
        g_position_loop_last_y_cm = position_input_y_cm;
        position_loop_reset_predict_hold();
    }
    else
    {
        position_input_valid = position_loop_update_predict_hold(measured_vx_cms,
                                                                 measured_vy_cms,
                                                                 g_position_loop_car_ff_valid,
                                                                 g_position_loop_car_ff_raw_vx_cms,
                                                                 g_position_loop_car_ff_raw_vy_cms,
                                                                 &position_input_x_cm,
                                                                 &position_input_y_cm);
    }

    pos_error_x_cm = position_loop_apply_deadzone(position_input_x_cm);
    pos_error_y_cm = position_loop_apply_deadzone(position_input_y_cm);

    g_position_loop_ir_track_valid = position_input_valid;
    g_position_loop_velocity_ready = velocity_ready;
    g_position_loop_fb_x_cm = position_input_x_cm;
    g_position_loop_fb_y_cm = position_input_y_cm;
    g_position_loop_error_x_cm = pos_error_x_cm;
    g_position_loop_error_y_cm = pos_error_y_cm;

    if(!position_input_valid)
    {
        position_loop_set_output(0.0f, 0.0f, target_vx_cms, target_vy_cms);
        position_loop_reset_runtime();
        return;
    }

    if(pos_error_x_cm == 0.0f)
    {
        PID_Reset(&pos_pid_x);
    }
    if(pos_error_y_cm == 0.0f)
    {
        PID_Reset(&pos_pid_y);
    }

    previous_integral_x = pos_pid_x.integral;
    previous_integral_y = pos_pid_y.integral;
    raw_correction_x = POSITION_LOOP_VX_SIGN *
                       PID_Calculate(&pos_pid_x,
                                     pos_error_x_cm,
                                     POSITION_LOOP_TARGET_X_CM,
                                     POSITION_LOOP_DT_S);
    raw_correction_y = POSITION_LOOP_VY_SIGN *
                       PID_Calculate(&pos_pid_y,
                                     pos_error_y_cm,
                                     POSITION_LOOP_TARGET_Y_CM,
                                     POSITION_LOOP_DT_S);

    g_position_loop_limit_flags = 0U;
    correction_vx_cms = raw_correction_x;
    correction_vy_cms = raw_correction_y;
    if(position_loop_limit_vector(&correction_vx_cms,
                                  &correction_vy_cms,
                                  POSITION_LOOP_CORRECTION_LIMIT_CMS))
    {
        g_position_loop_limit_flags |= POSITION_LOOP_STATUS_CORRECTION_LIMITED;
        pos_pid_x.integral = previous_integral_x;
        pos_pid_y.integral = previous_integral_y;
    }
    g_position_loop_correction_vx_cms = correction_vx_cms;
    g_position_loop_correction_vy_cms = correction_vy_cms;

    desired_vx_cms = g_position_loop_car_ff_filtered_vx_cms + correction_vx_cms;
    desired_vy_cms = g_position_loop_car_ff_filtered_vy_cms + correction_vy_cms;
    if(position_loop_limit_vector(&desired_vx_cms,
                                  &desired_vy_cms,
                                  POSITION_LOOP_TARGET_VEL_LIMIT_CMS))
    {
        g_position_loop_limit_flags |= POSITION_LOOP_STATUS_TARGET_LIMITED;
        pos_pid_x.integral = previous_integral_x;
        pos_pid_y.integral = previous_integral_y;
    }

    if(position_loop_limit_vector_slew(desired_vx_cms,
                                      desired_vy_cms,
                                      g_position_loop_vx_cmd_cms,
                                      g_position_loop_vy_cmd_cms,
                                      &limited_vx_cms,
                                      &limited_vy_cms))
    {
        g_position_loop_limit_flags |= POSITION_LOOP_STATUS_SLEW_LIMITED;
        pos_pid_x.integral = previous_integral_x;
        pos_pid_y.integral = previous_integral_y;
    }
    g_position_loop_vx_cmd_cms = limited_vx_cms;
    g_position_loop_vy_cmd_cms = limited_vy_cms;

    position_loop_set_output(g_position_loop_vx_cmd_cms, g_position_loop_vy_cmd_cms, target_vx_cms, target_vy_cms);
    g_position_loop_x_i_out = pos_pid_x.Ki * pos_pid_x.integral;
    g_position_loop_y_i_out = pos_pid_y.Ki * pos_pid_y.integral;
    g_position_loop_active = 1U;
}

void Position_Loop_Get_Diag(PositionLoopDiag_t *diag)
{
    if(diag == NULL)
    {
        return;
    }

    diag->target_x_cm = POSITION_LOOP_TARGET_X_CM;
    diag->target_y_cm = POSITION_LOOP_TARGET_Y_CM;
    diag->fb_x_cm = g_position_loop_fb_x_cm;
    diag->fb_y_cm = g_position_loop_fb_y_cm;
    diag->error_x_cm = g_position_loop_error_x_cm;
    diag->error_y_cm = g_position_loop_error_y_cm;
    diag->vx_cmd_cms = g_position_loop_vx_cmd_cms;
    diag->vy_cmd_cms = g_position_loop_vy_cmd_cms;
    diag->correction_vx_cms = g_position_loop_correction_vx_cms;
    diag->correction_vy_cms = g_position_loop_correction_vy_cms;
    diag->car_ff_raw_vx_cms = g_position_loop_car_ff_raw_vx_cms;
    diag->car_ff_raw_vy_cms = g_position_loop_car_ff_raw_vy_cms;
    diag->car_ff_filtered_vx_cms = g_position_loop_car_ff_filtered_vx_cms;
    diag->car_ff_filtered_vy_cms = g_position_loop_car_ff_filtered_vy_cms;
    diag->car_ff_gain_x = g_position_loop_car_ff_gain_x;
    diag->car_ff_gain_y = g_position_loop_car_ff_gain_y;
    diag->x_i_out = g_position_loop_x_i_out;
    diag->y_i_out = g_position_loop_y_i_out;
    diag->enabled = g_position_loop_enabled;
    diag->active = g_position_loop_active;
    diag->ir_track_valid = g_position_loop_ir_track_valid;
    diag->velocity_ready = g_position_loop_velocity_ready;
    diag->car_ff_valid = g_position_loop_car_ff_valid;
    diag->limit_flags = g_position_loop_limit_flags;
}
