#include "control/position_loop.h"
#include "vision/vehicle_marker_localization.h"
#include <math.h>

#define POSITION_LOOP_TARGET_X_CM      (0.0f)
#define POSITION_LOOP_TARGET_Y_CM      (0.0f)
/* Visual position input now uses X-forward positive, Y-left positive.
 * PID_Calculate() uses error = target - current, so the loop output must flip
 * sign to keep a front/left target commanding positive front/left velocity. */
#define POSITION_LOOP_VX_SIGN          (1.0f)
#define POSITION_LOOP_VY_SIGN          (1.0f)
#define POSITION_LOOP_DEADZONE_CM      (3.0f)
#define POSITION_LOOP_DT_S             0.020f
#define POSITION_LOOP_PERIOD_MS        (20U)
#define POSITION_LOOP_CORRECTION_LIMIT_CMS  (70.0f)
#define POSITION_LOOP_TARGET_VEL_LIMIT_CMS  (150.0f)
#define POSITION_LOOP_TARGET_VEL_SLEW_CMS2  (400.0f)
#define POSITION_LOOP_CAR_FF_LPF_HZ          (5.0f)
#define POSITION_LOOP_CAR_FF_GAIN_DEFAULT    (0.15f)
#define POSITION_LOOP_CAR_FF_GAIN_MAX        (0.15f)
#define POSITION_LOOP_BRAKE_ACCEL_DEFAULT_CMS2 (100.0f)
#define POSITION_LOOP_BRAKE_DELAY_DEFAULT_S    (0.35f)
#define POSITION_LOOP_BRAKE_REVERSE_DEFAULT_CMS (30.0f)
#define POSITION_LOOP_BRAKE_STOP_RADIUS_DEFAULT_CM (5.0f)
#define POSITION_LOOP_BRAKE_SLEW_ENTRY_CMS2      (80.0f)
#define POSITION_LOOP_BRAKE_SLEW_EXIT_CMS2      (150.0f)
#define POSITION_LOOP_BRAKE_OVERSPEED_ENTRY_CMS  (5.0f)
#define POSITION_LOOP_BRAKE_OVERSPEED_EXIT_CMS   (2.0f)
#define POSITION_LOOP_VISION_HOLD_MS              (150U)
#define POSITION_LOOP_VISION_INNOVATION_GATE_CM   (20.0f)
#define POSITION_LOOP_VISION_REACQUIRE_GATE_CM    (10.0f)
#define POSITION_LOOP_VISION_REACQUIRE_FRAMES     (3U)
#define POSITION_LOOP_BRAKE_ACCEL_MIN_CMS2     (20.0f)
#define POSITION_LOOP_BRAKE_ACCEL_MAX_CMS2     (300.0f)
#define POSITION_LOOP_BRAKE_DELAY_MIN_S         (0.0f)
#define POSITION_LOOP_BRAKE_DELAY_MAX_S         (0.6f)
#define POSITION_LOOP_BRAKE_REVERSE_MIN_CMS     (0.0f)
#define POSITION_LOOP_BRAKE_REVERSE_MAX_CMS    (50.0f)
#define POSITION_LOOP_BRAKE_STOP_RADIUS_MIN_CM  (3.0f)
#define POSITION_LOOP_BRAKE_STOP_RADIUS_MAX_CM (20.0f)
#define POSITION_LOOP_BRAKE_DIRECTION_EPS       (0.01f)

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
static volatile PositionLoopBrakeConfig_t g_position_loop_brake_config = {
    0U,
    POSITION_LOOP_BRAKE_ACCEL_DEFAULT_CMS2,
    POSITION_LOOP_BRAKE_DELAY_DEFAULT_S,
    POSITION_LOOP_BRAKE_REVERSE_DEFAULT_CMS,
    POSITION_LOOP_BRAKE_STOP_RADIUS_DEFAULT_CM
};
static volatile float g_position_loop_brake_position_x_cm = 0.0f;
static volatile float g_position_loop_brake_position_y_cm = 0.0f;
static volatile float g_position_loop_brake_distance_cm = 0.0f;
static volatile float g_position_loop_brake_closing_speed_cms = 0.0f;
static volatile float g_position_loop_brake_physical_distance_cm = 0.0f;
static volatile float g_position_loop_brake_delay_distance_cm = 0.0f;
static volatile float g_position_loop_brake_envelope_speed_cms = 0.0f;
static volatile float g_position_loop_brake_overspeed_cms = 0.0f;
static volatile float g_position_loop_brake_base_vx_cms = 0.0f;
static volatile float g_position_loop_brake_base_vy_cms = 0.0f;
static volatile float g_position_loop_brake_candidate_vx_cms = 0.0f;
static volatile float g_position_loop_brake_candidate_vy_cms = 0.0f;
static volatile float g_position_loop_brake_combined_vx_cms = 0.0f;
static volatile float g_position_loop_brake_combined_vy_cms = 0.0f;
static volatile uint8 g_position_loop_brake_active = 0U;
static volatile uint8 g_position_loop_brake_status_flags = 0U;
static uint8 g_position_loop_car_ff_lpf_initialized = 0U;
static uint8 g_position_loop_brake_velocity_lpf_initialized = 0U;
static uint8 g_position_loop_brake_last_car_velocity_valid = 0U;
static float g_position_loop_brake_rel_vx_lpf_cms = 0.0f;
static float g_position_loop_brake_rel_vy_lpf_cms = 0.0f;
static uint8 g_position_loop_brake_hysteresis_active = 0U;
static float g_position_loop_brake_slew_vx_cms = 0.0f;
static float g_position_loop_brake_slew_vy_cms = 0.0f;
static uint8 g_position_loop_last_pose_valid = 0U;
static uint8 g_position_loop_predict_hold_active = 0U;
static float g_position_loop_last_x_cm = 0.0f;
static float g_position_loop_last_y_cm = 0.0f;
static float g_position_loop_hold_start_x_cm = 0.0f;
static float g_position_loop_hold_start_y_cm = 0.0f;
static float g_position_loop_hold_dx_cm = 0.0f;
static float g_position_loop_hold_dy_cm = 0.0f;
static uint32 g_position_loop_last_pose_seq = 0U;
static uint16 g_position_loop_hold_elapsed_ms = 0U;
static uint8 g_position_loop_observation_state = POSITION_LOOP_OBSERVATION_INVALID;
static uint8 g_position_loop_reacquire_count = 0U;
static float g_position_loop_reacquire_x_cm = 0.0f;
static float g_position_loop_reacquire_y_cm = 0.0f;
static float g_position_loop_vision_innovation_cm = 0.0f;

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

static void position_loop_update_brake_slew(float target_x,
                                             float target_y,
                                             uint8 entering)
{
    float delta_x = target_x - g_position_loop_brake_slew_vx_cms;
    float delta_y = target_y - g_position_loop_brake_slew_vy_cms;
    float slew_rate_cms2 = entering ? POSITION_LOOP_BRAKE_SLEW_ENTRY_CMS2 :
                                      POSITION_LOOP_BRAKE_SLEW_EXIT_CMS2;
    float max_step = slew_rate_cms2 * POSITION_LOOP_DT_S;

    position_loop_limit_vector(&delta_x, &delta_y, max_step);
    g_position_loop_brake_slew_vx_cms += delta_x;
    g_position_loop_brake_slew_vy_cms += delta_y;
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

static void position_loop_reset_brake_runtime(void)
{
    g_position_loop_brake_position_x_cm = 0.0f;
    g_position_loop_brake_position_y_cm = 0.0f;
    g_position_loop_brake_distance_cm = 0.0f;
    g_position_loop_brake_closing_speed_cms = 0.0f;
    g_position_loop_brake_physical_distance_cm = 0.0f;
    g_position_loop_brake_delay_distance_cm = 0.0f;
    g_position_loop_brake_envelope_speed_cms = 0.0f;
    g_position_loop_brake_overspeed_cms = 0.0f;
    g_position_loop_brake_base_vx_cms = 0.0f;
    g_position_loop_brake_base_vy_cms = 0.0f;
    g_position_loop_brake_candidate_vx_cms = 0.0f;
    g_position_loop_brake_candidate_vy_cms = 0.0f;
    g_position_loop_brake_combined_vx_cms = 0.0f;
    g_position_loop_brake_combined_vy_cms = 0.0f;
    g_position_loop_brake_active = 0U;
    g_position_loop_brake_status_flags = 0U;
    g_position_loop_brake_velocity_lpf_initialized = 0U;
    g_position_loop_brake_last_car_velocity_valid = 0U;
    g_position_loop_brake_rel_vx_lpf_cms = 0.0f;
    g_position_loop_brake_rel_vy_lpf_cms = 0.0f;
    g_position_loop_brake_hysteresis_active = 0U;
    g_position_loop_brake_slew_vx_cms = 0.0f;
    g_position_loop_brake_slew_vy_cms = 0.0f;
}

static void position_loop_apply_brake_envelope(float position_x_cm,
                                               float position_y_cm,
                                               float measured_vx_cms,
                                               float measured_vy_cms,
                                               float base_correction_vx_cms,
                                               float base_correction_vy_cms,
                                               float *combined_vx_cms,
                                               float *combined_vy_cms)
{
    PositionLoopBrakeConfig_t config;
    float car_vx_cms = 0.0f;
    float car_vy_cms = 0.0f;
    float relative_vx_cms;
    float relative_vy_cms;
    float relative_speed_cms;
    float distance_cm;
    float direction_x = 0.0f;
    float direction_y = 0.0f;
    float closing_speed_cms = 0.0f;
    float physical_distance_cm = 0.0f;
    float delay_distance_cm = 0.0f;
    float available_distance_cm;
    float envelope_speed_cms = 0.0f;
    float overspeed_cms = 0.0f;
    float brake_speed_cms = 0.0f;
    float candidate_vx_cms = 0.0f;
    float candidate_vy_cms = 0.0f;
    float tau_s = 1.0f / (6.283185307f * POSITION_LOOP_BRAKE_REL_VEL_LPF_HZ);
    float alpha = POSITION_LOOP_DT_S / (tau_s + POSITION_LOOP_DT_S);
    uint8 car_velocity_valid = g_position_loop_car_ff_valid;
    uint8 status_flags = 0U;

    config.enabled = g_position_loop_brake_config.enabled;
    config.brake_accel_cms2 = g_position_loop_brake_config.brake_accel_cms2;
    config.delay_s = g_position_loop_brake_config.delay_s;
    config.reverse_limit_cms = g_position_loop_brake_config.reverse_limit_cms;
    config.stop_radius_cm = g_position_loop_brake_config.stop_radius_cm;

    if(config.enabled)
    {
        status_flags |= POSITION_LOOP_BRAKE_STATUS_ENABLED;
    }
    if(car_velocity_valid)
    {
        car_vx_cms = g_position_loop_car_ff_raw_vx_cms;
        car_vy_cms = g_position_loop_car_ff_raw_vy_cms;
    }
    else
    {
        status_flags |= POSITION_LOOP_BRAKE_STATUS_CAR_VELOCITY_FALLBACK;
    }

    relative_vx_cms = measured_vx_cms - car_vx_cms;
    relative_vy_cms = measured_vy_cms - car_vy_cms;
    if(!g_position_loop_brake_velocity_lpf_initialized ||
       (car_velocity_valid != g_position_loop_brake_last_car_velocity_valid))
    {
        g_position_loop_brake_rel_vx_lpf_cms = relative_vx_cms;
        g_position_loop_brake_rel_vy_lpf_cms = relative_vy_cms;
        g_position_loop_brake_velocity_lpf_initialized = 1U;
    }
    else
    {
        g_position_loop_brake_rel_vx_lpf_cms +=
            alpha * (relative_vx_cms - g_position_loop_brake_rel_vx_lpf_cms);
        g_position_loop_brake_rel_vy_lpf_cms +=
            alpha * (relative_vy_cms - g_position_loop_brake_rel_vy_lpf_cms);
    }
    g_position_loop_brake_last_car_velocity_valid = car_velocity_valid;
    status_flags |= POSITION_LOOP_BRAKE_STATUS_VELOCITY_FILTER_READY;

    distance_cm = sqrtf(position_x_cm * position_x_cm +
                        position_y_cm * position_y_cm);
    relative_speed_cms = sqrtf(
        g_position_loop_brake_rel_vx_lpf_cms * g_position_loop_brake_rel_vx_lpf_cms +
        g_position_loop_brake_rel_vy_lpf_cms * g_position_loop_brake_rel_vy_lpf_cms);

    if(distance_cm > POSITION_LOOP_BRAKE_DIRECTION_EPS)
    {
        direction_x = position_x_cm / distance_cm;
        direction_y = position_y_cm / distance_cm;
        status_flags |= POSITION_LOOP_BRAKE_STATUS_DIRECTION_VALID;
    }
    else if(relative_speed_cms > POSITION_LOOP_BRAKE_DIRECTION_EPS)
    {
        direction_x = g_position_loop_brake_rel_vx_lpf_cms / relative_speed_cms;
        direction_y = g_position_loop_brake_rel_vy_lpf_cms / relative_speed_cms;
        status_flags |= POSITION_LOOP_BRAKE_STATUS_DIRECTION_VALID;
    }

    if(distance_cm <= config.stop_radius_cm)
    {
        status_flags |= POSITION_LOOP_BRAKE_STATUS_INSIDE_STOP_RADIUS;
    }
    if(status_flags & POSITION_LOOP_BRAKE_STATUS_DIRECTION_VALID)
    {
        closing_speed_cms = g_position_loop_brake_rel_vx_lpf_cms * direction_x +
                            g_position_loop_brake_rel_vy_lpf_cms * direction_y;
    }
    if(closing_speed_cms > 0.0f)
    {
        status_flags |= POSITION_LOOP_BRAKE_STATUS_APPROACHING;
        physical_distance_cm = closing_speed_cms * closing_speed_cms /
                               (2.0f * config.brake_accel_cms2);
        delay_distance_cm = closing_speed_cms * config.delay_s;
        available_distance_cm = distance_cm - config.stop_radius_cm -
                                delay_distance_cm;
        if(available_distance_cm < 0.0f)
        {
            available_distance_cm = 0.0f;
        }
        envelope_speed_cms = sqrtf(2.0f * config.brake_accel_cms2 *
                                    available_distance_cm);
        overspeed_cms = closing_speed_cms - envelope_speed_cms;
        if(overspeed_cms < 0.0f)
        {
            overspeed_cms = 0.0f;
        }
        if(config.enabled)
        {
            if(g_position_loop_brake_hysteresis_active)
            {
                if(overspeed_cms < POSITION_LOOP_BRAKE_OVERSPEED_EXIT_CMS)
                {
                    g_position_loop_brake_hysteresis_active = 0U;
                }
            }
            else if(overspeed_cms > POSITION_LOOP_BRAKE_OVERSPEED_ENTRY_CMS)
            {
                g_position_loop_brake_hysteresis_active = 1U;
            }
        }

        if(g_position_loop_brake_hysteresis_active ||
           (!config.enabled &&
            (overspeed_cms > POSITION_LOOP_BRAKE_OVERSPEED_ENTRY_CMS)))
        {
            brake_speed_cms = overspeed_cms;
        }
        if(brake_speed_cms > config.reverse_limit_cms)
        {
            brake_speed_cms = config.reverse_limit_cms;
            status_flags |= POSITION_LOOP_BRAKE_STATUS_REVERSE_LIMITED;
        }
        candidate_vx_cms = -brake_speed_cms * direction_x;
        candidate_vy_cms = -brake_speed_cms * direction_y;
    }
    else
    {
        g_position_loop_brake_hysteresis_active = 0U;
    }

    *combined_vx_cms = base_correction_vx_cms;
    *combined_vy_cms = base_correction_vy_cms;
    g_position_loop_brake_active = 0U;
    if(config.enabled)
    {
        position_loop_update_brake_slew(candidate_vx_cms,
                                        candidate_vy_cms,
                                        g_position_loop_brake_hysteresis_active);
        *combined_vx_cms += g_position_loop_brake_slew_vx_cms;
        *combined_vy_cms += g_position_loop_brake_slew_vy_cms;
        if((g_position_loop_brake_slew_vx_cms *
            g_position_loop_brake_slew_vx_cms +
            g_position_loop_brake_slew_vy_cms *
            g_position_loop_brake_slew_vy_cms) >
           (POSITION_LOOP_BRAKE_DIRECTION_EPS *
            POSITION_LOOP_BRAKE_DIRECTION_EPS))
        {
            g_position_loop_brake_active = 1U;
        }
        candidate_vx_cms = g_position_loop_brake_slew_vx_cms;
        candidate_vy_cms = g_position_loop_brake_slew_vy_cms;
    }
    else
    {
        g_position_loop_brake_hysteresis_active = 0U;
        g_position_loop_brake_slew_vx_cms = 0.0f;
        g_position_loop_brake_slew_vy_cms = 0.0f;
    }

    g_position_loop_brake_position_x_cm = position_x_cm;
    g_position_loop_brake_position_y_cm = position_y_cm;
    g_position_loop_brake_distance_cm = distance_cm;
    g_position_loop_brake_closing_speed_cms = closing_speed_cms;
    g_position_loop_brake_physical_distance_cm = physical_distance_cm;
    g_position_loop_brake_delay_distance_cm = delay_distance_cm;
    g_position_loop_brake_envelope_speed_cms = envelope_speed_cms;
    g_position_loop_brake_overspeed_cms = overspeed_cms;
    g_position_loop_brake_base_vx_cms = base_correction_vx_cms;
    g_position_loop_brake_base_vy_cms = base_correction_vy_cms;
    g_position_loop_brake_candidate_vx_cms = candidate_vx_cms;
    g_position_loop_brake_candidate_vy_cms = candidate_vy_cms;
    g_position_loop_brake_combined_vx_cms = *combined_vx_cms;
    g_position_loop_brake_combined_vy_cms = *combined_vy_cms;
    g_position_loop_brake_status_flags = status_flags;
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
    position_loop_reset_brake_runtime();
    g_position_loop_last_pose_valid = 0U;
    g_position_loop_predict_hold_active = 0U;
    g_position_loop_last_x_cm = 0.0f;
    g_position_loop_last_y_cm = 0.0f;
    g_position_loop_hold_start_x_cm = 0.0f;
    g_position_loop_hold_start_y_cm = 0.0f;
    g_position_loop_hold_dx_cm = 0.0f;
    g_position_loop_hold_dy_cm = 0.0f;
    g_position_loop_last_pose_seq = g_vehicle_marker_pose_update_seq;
    g_position_loop_hold_elapsed_ms = 0U;
    g_position_loop_observation_state = POSITION_LOOP_OBSERVATION_INVALID;
    g_position_loop_reacquire_count = 0U;
    g_position_loop_reacquire_x_cm = 0.0f;
    g_position_loop_reacquire_y_cm = 0.0f;
    g_position_loop_vision_innovation_cm = 0.0f;
}

static void position_loop_reset_predict_hold(void)
{
    g_position_loop_predict_hold_active = 0U;
    g_position_loop_hold_start_x_cm = 0.0f;
    g_position_loop_hold_start_y_cm = 0.0f;
    g_position_loop_hold_dx_cm = 0.0f;
    g_position_loop_hold_dy_cm = 0.0f;
    g_position_loop_hold_elapsed_ms = 0U;
}

static void position_loop_reset_reacquisition(void)
{
    g_position_loop_reacquire_count = 0U;
    g_position_loop_reacquire_x_cm = 0.0f;
    g_position_loop_reacquire_y_cm = 0.0f;
}

static uint8 position_loop_try_accept_visual_pose(float pose_x_cm,
                                                  float pose_y_cm,
                                                  float raw_x_cm,
                                                  float raw_y_cm,
                                                  float *accepted_x_cm,
                                                  float *accepted_y_cm)
{
    float dx_cm;
    float dy_cm;
    float candidate_delta_cm;

    if((accepted_x_cm == NULL) || (accepted_y_cm == NULL) ||
       !isfinite(pose_x_cm) || !isfinite(pose_y_cm) ||
       !isfinite(raw_x_cm) || !isfinite(raw_y_cm))
    {
        return 0U;
    }

    if(g_position_loop_last_pose_valid)
    {
        dx_cm = raw_x_cm - g_position_loop_last_x_cm;
        dy_cm = raw_y_cm - g_position_loop_last_y_cm;
        g_position_loop_vision_innovation_cm = sqrtf(dx_cm * dx_cm +
                                                     dy_cm * dy_cm);
        if(g_position_loop_vision_innovation_cm <=
           POSITION_LOOP_VISION_INNOVATION_GATE_CM)
        {
            *accepted_x_cm = pose_x_cm;
            *accepted_y_cm = pose_y_cm;
            position_loop_reset_reacquisition();
            return 1U;
        }
    }

    if(0U == g_position_loop_reacquire_count)
    {
        g_position_loop_reacquire_x_cm = pose_x_cm;
        g_position_loop_reacquire_y_cm = pose_y_cm;
        g_position_loop_reacquire_count = 1U;
    }
    else
    {
        dx_cm = pose_x_cm - g_position_loop_reacquire_x_cm;
        dy_cm = pose_y_cm - g_position_loop_reacquire_y_cm;
        candidate_delta_cm = sqrtf(dx_cm * dx_cm + dy_cm * dy_cm);
        if(candidate_delta_cm <= POSITION_LOOP_VISION_REACQUIRE_GATE_CM)
        {
            g_position_loop_reacquire_count++;
            g_position_loop_reacquire_x_cm +=
                dx_cm / (float)g_position_loop_reacquire_count;
            g_position_loop_reacquire_y_cm +=
                dy_cm / (float)g_position_loop_reacquire_count;
        }
        else
        {
            g_position_loop_reacquire_x_cm = pose_x_cm;
            g_position_loop_reacquire_y_cm = pose_y_cm;
            g_position_loop_reacquire_count = 1U;
        }
    }

    g_position_loop_observation_state = POSITION_LOOP_OBSERVATION_REACQUIRING;
    if(g_position_loop_reacquire_count >= POSITION_LOOP_VISION_REACQUIRE_FRAMES)
    {
        *accepted_x_cm = g_position_loop_reacquire_x_cm;
        *accepted_y_cm = g_position_loop_reacquire_y_cm;
        position_loop_reset_reacquisition();
        return 1U;
    }

    return 0U;
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
    g_position_loop_hold_dx_cm = 0.0f;
    g_position_loop_hold_dy_cm = 0.0f;
    g_position_loop_hold_elapsed_ms = 0U;
    g_position_loop_predict_hold_active = 1U;
    if(g_position_loop_observation_state != POSITION_LOOP_OBSERVATION_REACQUIRING)
    {
        g_position_loop_observation_state = POSITION_LOOP_OBSERVATION_HOLD;
    }

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
    float predicted_x_cm;
    float predicted_y_cm;
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

    if(((uint32)g_position_loop_hold_elapsed_ms + POSITION_LOOP_PERIOD_MS) >
       POSITION_LOOP_VISION_HOLD_MS)
    {
        g_position_loop_last_pose_valid = 0U;
        position_loop_reset_predict_hold();
        if(0U == g_position_loop_reacquire_count)
        {
            g_position_loop_observation_state = POSITION_LOOP_OBSERVATION_INVALID;
        }
        return 0U;
    }
    g_position_loop_hold_elapsed_ms += POSITION_LOOP_PERIOD_MS;

    if(car_velocity_valid)
    {
        relative_vx_cms += car_vx_cms;
        relative_vy_cms += car_vy_cms;
    }
    g_position_loop_hold_dx_cm += relative_vx_cms * POSITION_LOOP_DT_S;
    g_position_loop_hold_dy_cm += relative_vy_cms * POSITION_LOOP_DT_S;
    predicted_x_cm = g_position_loop_hold_start_x_cm + g_position_loop_hold_dx_cm;
    predicted_y_cm = g_position_loop_hold_start_y_cm + g_position_loop_hold_dy_cm;

    *predict_x_cm = predicted_x_cm;
    *predict_y_cm = predicted_y_cm;
    g_position_loop_last_x_cm = predicted_x_cm;
    g_position_loop_last_y_cm = predicted_y_cm;

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

uint8 Position_Loop_Set_Brake_Config(const PositionLoopBrakeConfig_t *config)
{
    uint32 irq_state;

    if((NULL == config) ||
       (config->enabled > 1U) ||
       !isfinite(config->brake_accel_cms2) ||
       !isfinite(config->delay_s) ||
       !isfinite(config->reverse_limit_cms) ||
       !isfinite(config->stop_radius_cm) ||
       (config->brake_accel_cms2 < POSITION_LOOP_BRAKE_ACCEL_MIN_CMS2) ||
       (config->brake_accel_cms2 > POSITION_LOOP_BRAKE_ACCEL_MAX_CMS2) ||
       (config->delay_s < POSITION_LOOP_BRAKE_DELAY_MIN_S) ||
       (config->delay_s > POSITION_LOOP_BRAKE_DELAY_MAX_S) ||
       (config->reverse_limit_cms < POSITION_LOOP_BRAKE_REVERSE_MIN_CMS) ||
       (config->reverse_limit_cms > POSITION_LOOP_BRAKE_REVERSE_MAX_CMS) ||
       (config->stop_radius_cm < POSITION_LOOP_BRAKE_STOP_RADIUS_MIN_CM) ||
       (config->stop_radius_cm > POSITION_LOOP_BRAKE_STOP_RADIUS_MAX_CM))
    {
        return 0U;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    g_position_loop_brake_config = *config;
    g_position_loop_brake_velocity_lpf_initialized = 0U;
    if(0U == irq_state)
    {
        __enable_irq();
    }
    return 1U;
}

void Position_Loop_Get_Brake_Config(PositionLoopBrakeConfig_t *config)
{
    uint32 irq_state;

    if(NULL == config)
    {
        return;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    *config = g_position_loop_brake_config;
    if(0U == irq_state)
    {
        __enable_irq();
    }
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
    float position_distance_cm;
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
        float visual_x_cm = g_vehicle_marker_pose.x;
        float visual_y_cm = g_vehicle_marker_pose.y;
        float visual_raw_x_cm = g_vehicle_marker_pose.raw_x;
        float visual_raw_y_cm = g_vehicle_marker_pose.raw_y;

        /* 被拒绝的视觉序号也只消费一次，禁止单帧重复满足重捕获计数。 */
        g_position_loop_last_pose_seq = g_vehicle_marker_pose_update_seq;
        position_input_valid = position_loop_try_accept_visual_pose(
            visual_x_cm,
            visual_y_cm,
            visual_raw_x_cm,
            visual_raw_y_cm,
            &position_input_x_cm,
            &position_input_y_cm);
        if(position_input_valid)
        {
            g_position_loop_last_pose_valid = 1U;
            g_position_loop_last_x_cm = position_input_x_cm;
            g_position_loop_last_y_cm = position_input_y_cm;
            g_position_loop_observation_state = POSITION_LOOP_OBSERVATION_LIVE;
            position_loop_reset_predict_hold();
        }
    }

    if(!position_input_valid)
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
    position_distance_cm = sqrtf(position_input_x_cm * position_input_x_cm +
                                 position_input_y_cm * position_input_y_cm);
    if(g_position_loop_brake_config.enabled &&
       (position_distance_cm <= g_position_loop_brake_config.stop_radius_cm))
    {
        pos_error_x_cm = 0.0f;
        pos_error_y_cm = 0.0f;
    }

    g_position_loop_ir_track_valid = position_input_valid;
    g_position_loop_velocity_ready = velocity_ready;
    g_position_loop_fb_x_cm = position_input_x_cm;
    g_position_loop_fb_y_cm = position_input_y_cm;
    g_position_loop_error_x_cm = pos_error_x_cm;
    g_position_loop_error_y_cm = pos_error_y_cm;

    if(!position_input_valid)
    {
        PID_Reset(&pos_pid_x);
        PID_Reset(&pos_pid_y);
        position_loop_reset_brake_runtime();
        g_position_loop_active = 0U;
        g_position_loop_correction_vx_cms = 0.0f;
        g_position_loop_correction_vy_cms = 0.0f;
        g_position_loop_x_i_out = 0.0f;
        g_position_loop_y_i_out = 0.0f;
        g_position_loop_limit_flags = 0U;
        if(position_loop_limit_vector_slew(0.0f,
                                           0.0f,
                                           g_position_loop_vx_cmd_cms,
                                           g_position_loop_vy_cmd_cms,
                                           &limited_vx_cms,
                                           &limited_vy_cms))
        {
            g_position_loop_limit_flags |= POSITION_LOOP_STATUS_SLEW_LIMITED;
        }
        g_position_loop_vx_cmd_cms = limited_vx_cms;
        g_position_loop_vy_cmd_cms = limited_vy_cms;
        position_loop_set_output(g_position_loop_vx_cmd_cms,
                                 g_position_loop_vy_cmd_cms,
                                 target_vx_cms,
                                 target_vy_cms);
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
    position_loop_apply_brake_envelope(position_input_x_cm,
                                       position_input_y_cm,
                                       measured_vx_cms,
                                       measured_vy_cms,
                                       raw_correction_x,
                                       raw_correction_y,
                                       &correction_vx_cms,
                                       &correction_vy_cms);
    if(position_loop_limit_vector(&correction_vx_cms,
                                  &correction_vy_cms,
                                  POSITION_LOOP_CORRECTION_LIMIT_CMS))
    {
        g_position_loop_limit_flags |= POSITION_LOOP_STATUS_CORRECTION_LIMITED;
        g_position_loop_brake_status_flags |=
            POSITION_LOOP_BRAKE_STATUS_CORRECTION_LIMITED;
        pos_pid_x.integral = previous_integral_x;
        pos_pid_y.integral = previous_integral_y;
    }
    g_position_loop_brake_combined_vx_cms = correction_vx_cms;
    g_position_loop_brake_combined_vy_cms = correction_vy_cms;
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
    diag->brake_position_x_cm = g_position_loop_brake_position_x_cm;
    diag->brake_position_y_cm = g_position_loop_brake_position_y_cm;
    diag->brake_distance_cm = g_position_loop_brake_distance_cm;
    diag->brake_closing_speed_cms = g_position_loop_brake_closing_speed_cms;
    diag->brake_physical_distance_cm =
        g_position_loop_brake_physical_distance_cm;
    diag->brake_delay_distance_cm = g_position_loop_brake_delay_distance_cm;
    diag->brake_envelope_speed_cms = g_position_loop_brake_envelope_speed_cms;
    diag->brake_overspeed_cms = g_position_loop_brake_overspeed_cms;
    diag->brake_base_correction_vx_cms = g_position_loop_brake_base_vx_cms;
    diag->brake_base_correction_vy_cms = g_position_loop_brake_base_vy_cms;
    diag->brake_candidate_vx_cms = g_position_loop_brake_candidate_vx_cms;
    diag->brake_candidate_vy_cms = g_position_loop_brake_candidate_vy_cms;
    diag->brake_combined_vx_cms = g_position_loop_brake_combined_vx_cms;
    diag->brake_combined_vy_cms = g_position_loop_brake_combined_vy_cms;
    diag->vision_innovation_cm = g_position_loop_vision_innovation_cm;
    diag->vision_hold_age_ms = g_position_loop_hold_elapsed_ms;
    diag->vision_observation_state = g_position_loop_observation_state;
    diag->vision_reacquire_count = g_position_loop_reacquire_count;
    diag->enabled = g_position_loop_enabled;
    diag->active = g_position_loop_active;
    diag->ir_track_valid = g_position_loop_ir_track_valid;
    diag->velocity_ready = g_position_loop_velocity_ready;
    diag->car_ff_valid = g_position_loop_car_ff_valid;
    diag->limit_flags = g_position_loop_limit_flags;
    diag->brake_active = g_position_loop_brake_active;
    diag->brake_status_flags = g_position_loop_brake_status_flags;
}
