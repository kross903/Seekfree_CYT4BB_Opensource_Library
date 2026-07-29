#include "control/position_loop.h"
#include "vision/vehicle_marker_localization.h"
#include <math.h>

#define POSITION_LOOP_CAR_FF_GAIN_DEFAULT          (0.15f)   // M1/M2：小车X/Y速度前馈增益上电默认值
#define POSITION_LOOP_CAR_FF_GAIN_MAX              (1.2f)    // M1/M2：小车X/Y速度前馈增益允许上限
#define POSITION_LOOP_POSITION_KP_DEFAULT          (0.2f)    // M3：二维位置P增益默认值，单位s^-1
#define POSITION_LOOP_CORRECTION_MAX_DEFAULT_CMS   (70.0f)   // M4：位置修正速度模长上限，单位cm/s
#define POSITION_LOOP_TARGET_MAX_DEFAULT_CMS       (150.0f)  // M5：合成后目标速度模长上限，单位cm/s
#define POSITION_LOOP_ACCEL_MAX_DEFAULT_CMS2       (80.0f)   // M6：轨迹加速度模长上限，单位cm/s^2
#define POSITION_LOOP_JERK_MAX_DEFAULT_CMS3        (200.0f)  // M7：轨迹Jerk模长上限，单位cm/s^3
#define POSITION_LOOP_VISION_HOLD_MS               (150U)    // 视觉断帧后相对位置预测保持时间，单位ms
#define POSITION_LOOP_VISION_INNOVATION_GATE_CM    (20.0f)   // 新视觉位置相对可信轨迹的创新门限，单位cm
#define POSITION_LOOP_VISION_REACQUIRE_GATE_CM     (10.0f)   // 视觉重捕获候选位置的一致性门限，单位cm
#define POSITION_LOOP_VISION_REACQUIRE_FRAMES      (3U)      // 视觉重捕获所需的连续一致候选帧数
#define POSITION_LOOP_EPSILON                      (0.0001f) // 二维距离归一化时的除零保护阈值
#define POSITION_LOOP_YAW_RESET_THRESHOLD_DEG      (30.0f)   // 单个水平控制周期内超过该变化视为Yaw参考系重置

static volatile uint8 g_enabled = 0U;
static volatile PositionLoopTrajectoryConfig_t g_config = {
    POSITION_LOOP_POSITION_KP_DEFAULT,
    POSITION_LOOP_CORRECTION_MAX_DEFAULT_CMS,
    POSITION_LOOP_TARGET_MAX_DEFAULT_CMS,
    POSITION_LOOP_ACCEL_MAX_DEFAULT_CMS2,
    POSITION_LOOP_JERK_MAX_DEFAULT_CMS3
};
static volatile float g_car_ff_gain_x = POSITION_LOOP_CAR_FF_GAIN_DEFAULT;
static volatile float g_car_ff_gain_y = POSITION_LOOP_CAR_FF_GAIN_DEFAULT;
static volatile PositionLoopDiag_t g_diag;

static float g_target_vx_earth_cms = 0.0f;
static float g_target_vy_earth_cms = 0.0f;
static float g_target_ax_earth_cms2 = 0.0f;
static float g_target_ay_earth_cms2 = 0.0f;
static uint8 g_trajectory_initialized = 0U;
static uint8 g_trajectory_reset_reason = POSITION_LOOP_RESET_NONE;
static uint32 g_trajectory_reset_count = 0U;
static float g_last_yaw_deg = 0.0f;
static uint8 g_yaw_initialized = 0U;

static uint8 g_last_pose_valid = 0U;
static uint8 g_predict_hold_active = 0U;
static float g_last_x_earth_cm = 0.0f;
static float g_last_y_earth_cm = 0.0f;
static float g_hold_start_x_earth_cm = 0.0f;
static float g_hold_start_y_earth_cm = 0.0f;
static float g_hold_dx_earth_cm = 0.0f;
static float g_hold_dy_earth_cm = 0.0f;
static uint32 g_last_pose_seq = 0U;
static uint16 g_hold_elapsed_ms = 0U;
static uint8 g_observation_state = POSITION_LOOP_OBSERVATION_INVALID;
static uint8 g_reacquire_count = 0U;
static float g_reacquire_x_earth_cm = 0.0f;
static float g_reacquire_y_earth_cm = 0.0f;
static float g_vision_innovation_cm = 0.0f;

static float position_loop_limit_float(float value, float minimum, float maximum)
{
    if(value > maximum) return maximum;
    if(value < minimum) return minimum;
    return value;
}

static uint8 position_loop_limit_vector(float *x, float *y, float limit)
{
    float magnitude;
    float scale;

    if((x == NULL) || (y == NULL) || !isfinite(*x) || !isfinite(*y) ||
       !isfinite(limit) || !(limit > 0.0f))
    {
        return 0U;
    }
    magnitude = sqrtf((*x) * (*x) + (*y) * (*y));
    if(!(magnitude > limit))
    {
        return 0U;
    }
    scale = limit / magnitude;
    *x *= scale;
    *y *= scale;
    return 1U;
}

static void position_loop_body_to_earth(float body_x,
                                        float body_y,
                                        float yaw_deg,
                                        float *earth_x,
                                        float *earth_y)
{
    float yaw_rad = yaw_deg * 0.01745329252f;
    float cosine = cosf(yaw_rad);
    float sine = sinf(yaw_rad);

    *earth_x = cosine * body_x - sine * body_y;
    *earth_y = sine * body_x + cosine * body_y;
}

static float position_loop_wrap_deg180(float angle_deg)
{
    while(angle_deg > 180.0f) angle_deg -= 360.0f;
    while(angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/* 从零加速度开始并以零加速度停止时，按A/J约束计算完整S曲线停止距离。 */
static float position_loop_stopping_distance(float speed,
                                             float acceleration_limit,
                                             float jerk_limit)
{
    float switch_speed;

    if(!isfinite(speed) || !isfinite(acceleration_limit) ||
       !isfinite(jerk_limit) || !(speed > 0.0f) ||
       !(acceleration_limit > 0.0f) || !(jerk_limit > 0.0f))
    {
        return 0.0f;
    }
    switch_speed = acceleration_limit * acceleration_limit / jerk_limit;
    if(speed <= switch_speed)
    {
        return speed * sqrtf(speed / jerk_limit);
    }
    return speed * speed / (2.0f * acceleration_limit) +
           speed * acceleration_limit / (2.0f * jerk_limit);
}

static float position_loop_envelope_speed(float distance,
                                          float acceleration_limit,
                                          float jerk_limit)
{
    float switch_distance;
    float acceleration_jerk_ratio;

    if(!isfinite(distance) || !isfinite(acceleration_limit) ||
       !isfinite(jerk_limit) || !(distance > 0.0f) ||
       !(acceleration_limit > 0.0f) || !(jerk_limit > 0.0f))
    {
        return 0.0f;
    }
    switch_distance = acceleration_limit * acceleration_limit * acceleration_limit /
                      (jerk_limit * jerk_limit);
    if(distance <= switch_distance)
    {
        return powf(distance * sqrtf(jerk_limit), 0.6666666667f);
    }
    acceleration_jerk_ratio = acceleration_limit * acceleration_limit / jerk_limit;
    return (sqrtf(acceleration_jerk_ratio * acceleration_jerk_ratio +
                  8.0f * acceleration_limit * distance) -
            acceleration_jerk_ratio) * 0.5f;
}

static void position_loop_reset_reacquisition(void)
{
    g_reacquire_count = 0U;
    g_reacquire_x_earth_cm = 0.0f;
    g_reacquire_y_earth_cm = 0.0f;
}

static void position_loop_reset_predict_hold(void)
{
    g_predict_hold_active = 0U;
    g_hold_start_x_earth_cm = 0.0f;
    g_hold_start_y_earth_cm = 0.0f;
    g_hold_dx_earth_cm = 0.0f;
    g_hold_dy_earth_cm = 0.0f;
    g_hold_elapsed_ms = 0U;
}

void Position_Loop_Reset_Car_Velocity_Feedforward(void)
{
    g_diag.car_ff_raw_vx_cms = 0.0f;
    g_diag.car_ff_raw_vy_cms = 0.0f;
    g_diag.car_ff_filtered_vx_cms = 0.0f;
    g_diag.car_ff_filtered_vy_cms = 0.0f;
    g_diag.car_ff_valid = 0U;
}

void Position_Loop_Reset_Trajectory(uint8 reason)
{
    if(reason > POSITION_LOOP_RESET_YAW_RESET)
    {
        reason = POSITION_LOOP_RESET_NONE;
    }
    if(g_trajectory_initialized)
    {
        g_trajectory_reset_count++;
    }
    g_target_vx_earth_cms = 0.0f;
    g_target_vy_earth_cms = 0.0f;
    g_target_ax_earth_cms2 = 0.0f;
    g_target_ay_earth_cms2 = 0.0f;
    g_trajectory_initialized = 0U;
    g_trajectory_reset_reason = reason;
    g_diag.vx_cmd_cms = 0.0f;
    g_diag.vy_cmd_cms = 0.0f;
    g_diag.target_accel_x_cms2 = 0.0f;
    g_diag.target_accel_y_cms2 = 0.0f;
    g_diag.limit_flags &= (uint8)~(POSITION_LOOP_STATUS_TRAJECTORY_READY |
                                  POSITION_LOOP_STATUS_TIMING_VALID);
    g_diag.trajectory_reset_reason = reason;
    g_diag.trajectory_reset_count = g_trajectory_reset_count;
}

static void position_loop_reset_runtime(uint8 reason)
{
    PositionLoopDiag_t empty_diag = {0};

    Position_Loop_Reset_Trajectory(reason);
    g_last_pose_valid = 0U;
    g_last_yaw_deg = 0.0f;
    g_yaw_initialized = 0U;
    position_loop_reset_predict_hold();
    position_loop_reset_reacquisition();
    g_last_x_earth_cm = 0.0f;
    g_last_y_earth_cm = 0.0f;
    g_last_pose_seq = g_vehicle_marker_pose_update_seq;
    g_observation_state = POSITION_LOOP_OBSERVATION_INVALID;
    g_vision_innovation_cm = 0.0f;
    g_diag = empty_diag;
    g_diag.enabled = g_enabled;
    g_diag.car_ff_gain_x = g_car_ff_gain_x;
    g_diag.car_ff_gain_y = g_car_ff_gain_y;
    g_diag.trajectory_reset_reason = g_trajectory_reset_reason;
    g_diag.trajectory_reset_count = g_trajectory_reset_count;
}

static uint8 position_loop_raw_visual_valid(void)
{
    VehicleMarkerLossDebug_t loss = {0};

    VehicleMarkerLocalization_GetLossDebug(&loss);
    return (uint8)(loss.raw_detect_valid && g_vehicle_marker_pose.valid &&
                   !g_vehicle_marker_position_lost_flag);
}

static uint8 position_loop_accept_visual(float pose_x,
                                         float pose_y,
                                         float raw_x,
                                         float raw_y,
                                         float *accepted_x,
                                         float *accepted_y)
{
    float dx;
    float dy;

    if(g_last_pose_valid)
    {
        dx = raw_x - g_last_x_earth_cm;
        dy = raw_y - g_last_y_earth_cm;
        g_vision_innovation_cm = sqrtf(dx * dx + dy * dy);
        if(g_vision_innovation_cm <= POSITION_LOOP_VISION_INNOVATION_GATE_CM)
        {
            *accepted_x = pose_x;
            *accepted_y = pose_y;
            position_loop_reset_reacquisition();
            return 1U;
        }
    }

    if(g_reacquire_count == 0U)
    {
        g_reacquire_x_earth_cm = pose_x;
        g_reacquire_y_earth_cm = pose_y;
        g_reacquire_count = 1U;
    }
    else
    {
        float candidate_delta;
        dx = pose_x - g_reacquire_x_earth_cm;
        dy = pose_y - g_reacquire_y_earth_cm;
        candidate_delta = sqrtf(dx * dx + dy * dy);
        if(candidate_delta <= POSITION_LOOP_VISION_REACQUIRE_GATE_CM)
        {
            g_reacquire_count++;
            g_reacquire_x_earth_cm += dx / (float)g_reacquire_count;
            g_reacquire_y_earth_cm += dy / (float)g_reacquire_count;
        }
        else
        {
            g_reacquire_x_earth_cm = pose_x;
            g_reacquire_y_earth_cm = pose_y;
            g_reacquire_count = 1U;
        }
    }

    g_observation_state = POSITION_LOOP_OBSERVATION_REACQUIRING;
    if(g_reacquire_count >= POSITION_LOOP_VISION_REACQUIRE_FRAMES)
    {
        *accepted_x = g_reacquire_x_earth_cm;
        *accepted_y = g_reacquire_y_earth_cm;
        position_loop_reset_reacquisition();
        return 1U;
    }
    return 0U;
}

static uint8 position_loop_predict_hold(float measured_vx_earth,
                                        float measured_vy_earth,
                                        uint8 car_velocity_valid,
                                        float car_vx_earth,
                                        float car_vy_earth,
                                        float dt_s,
                                        float *position_x,
                                        float *position_y)
{
    uint16 elapsed_ms = (uint16)(dt_s * 1000.0f + 0.5f);

    if(elapsed_ms == 0U) elapsed_ms = 1U;
    if(!g_last_pose_valid)
    {
        return 0U;
    }
    if(!g_predict_hold_active)
    {
        g_hold_start_x_earth_cm = g_last_x_earth_cm;
        g_hold_start_y_earth_cm = g_last_y_earth_cm;
        g_hold_dx_earth_cm = 0.0f;
        g_hold_dy_earth_cm = 0.0f;
        g_hold_elapsed_ms = 0U;
        g_predict_hold_active = 1U;
    }
    if(((uint32)g_hold_elapsed_ms + elapsed_ms) >
       POSITION_LOOP_VISION_HOLD_MS)
    {
        g_last_pose_valid = 0U;
        position_loop_reset_predict_hold();
        if(g_reacquire_count == 0U) g_observation_state = POSITION_LOOP_OBSERVATION_INVALID;
        return 0U;
    }

    g_hold_elapsed_ms += elapsed_ms;
    g_hold_dx_earth_cm += ((car_velocity_valid ? car_vx_earth : 0.0f) -
                           measured_vx_earth) * dt_s;
    g_hold_dy_earth_cm += ((car_velocity_valid ? car_vy_earth : 0.0f) -
                           measured_vy_earth) * dt_s;
    *position_x = g_hold_start_x_earth_cm + g_hold_dx_earth_cm;
    *position_y = g_hold_start_y_earth_cm + g_hold_dy_earth_cm;
    g_last_x_earth_cm = *position_x;
    g_last_y_earth_cm = *position_y;
    if(g_observation_state != POSITION_LOOP_OBSERVATION_REACQUIRING)
    {
        g_observation_state = POSITION_LOOP_OBSERVATION_HOLD;
    }
    return 1U;
}

static float position_loop_correction_speed(float distance,
                                            const PositionLoopTrajectoryConfig_t *config)
{
    float proportional_speed = config->position_kp * distance;
    float envelope_speed = position_loop_envelope_speed(distance,
                                                        config->acceleration_max_cms2,
                                                        config->jerk_max_cms3);
    float speed = (proportional_speed < envelope_speed) ?
                  proportional_speed : envelope_speed;
    return position_loop_limit_float(speed, 0.0f, config->correction_speed_max_cms);
}

static void position_loop_shape_velocity(float desired_vx,
                                         float desired_vy,
                                         const PositionLoopTrajectoryConfig_t *config,
                                         float dt_s,
                                         uint8 *flags)
{
    float delta_vx = desired_vx - g_target_vx_earth_cms;
    float delta_vy = desired_vy - g_target_vy_earth_cms;
    float delta_speed = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
    float desired_ax = 0.0f;
    float desired_ay = 0.0f;
    float old_ax = g_target_ax_earth_cms2;
    float old_ay = g_target_ay_earth_cms2;
    float jerk_x;
    float jerk_y;

    if(delta_speed > POSITION_LOOP_EPSILON)
    {
        float direction_x = delta_vx / delta_speed;
        float direction_y = delta_vy / delta_speed;
        float acceleration_parallel = g_target_ax_earth_cms2 * direction_x +
                                      g_target_ay_earth_cms2 * direction_y;
        float delta_v_to_zero_accel = 0.0f;

        if(acceleration_parallel > 0.0f)
        {
            delta_v_to_zero_accel = acceleration_parallel * acceleration_parallel /
                                    (2.0f * config->jerk_max_cms3);
        }
        /* 剩余速度差不足以让当前加速度按Jerk回零时，立即开始撤销加速度。 */
        if(delta_speed > delta_v_to_zero_accel)
        {
            desired_ax = config->acceleration_max_cms2 * direction_x;
            desired_ay = config->acceleration_max_cms2 * direction_y;
        }
    }

    jerk_x = (desired_ax - g_target_ax_earth_cms2) / dt_s;
    jerk_y = (desired_ay - g_target_ay_earth_cms2) / dt_s;
    if(position_loop_limit_vector(&jerk_x, &jerk_y, config->jerk_max_cms3))
    {
        *flags |= POSITION_LOOP_STATUS_JERK_LIMITED;
    }
    g_target_ax_earth_cms2 += jerk_x * dt_s;
    g_target_ay_earth_cms2 += jerk_y * dt_s;
    if(position_loop_limit_vector(&g_target_ax_earth_cms2,
                                  &g_target_ay_earth_cms2,
                                  config->acceleration_max_cms2))
    {
        *flags |= POSITION_LOOP_STATUS_ACCEL_LIMITED;
    }

    g_target_vx_earth_cms += 0.5f * (old_ax + g_target_ax_earth_cms2) * dt_s;
    g_target_vy_earth_cms += 0.5f * (old_ay + g_target_ay_earth_cms2) * dt_s;

    if((delta_speed <= config->jerk_max_cms3 * dt_s * dt_s) &&
       (sqrtf(g_target_ax_earth_cms2 * g_target_ax_earth_cms2 +
              g_target_ay_earth_cms2 * g_target_ay_earth_cms2) <=
        config->jerk_max_cms3 * dt_s))
    {
        g_target_vx_earth_cms = desired_vx;
        g_target_vy_earth_cms = desired_vy;
        g_target_ax_earth_cms2 = 0.0f;
        g_target_ay_earth_cms2 = 0.0f;
    }
    if(position_loop_limit_vector(&g_target_vx_earth_cms,
                                  &g_target_vy_earth_cms,
                                  config->target_speed_max_cms))
    {
        float inverse_speed = 1.0f / config->target_speed_max_cms;
        float direction_x = g_target_vx_earth_cms * inverse_speed;
        float direction_y = g_target_vy_earth_cms * inverse_speed;
        float outward_accel = g_target_ax_earth_cms2 * direction_x +
                              g_target_ay_earth_cms2 * direction_y;
        if(outward_accel > 0.0f)
        {
            g_target_ax_earth_cms2 -= outward_accel * direction_x;
            g_target_ay_earth_cms2 -= outward_accel * direction_y;
        }
        *flags |= POSITION_LOOP_STATUS_TARGET_LIMITED;
    }
}

void Position_Loop_Set_Enabled(uint8 enabled)
{
    g_enabled = enabled ? 1U : 0U;
    position_loop_reset_runtime(g_enabled ? POSITION_LOOP_RESET_FIRST_ENABLE :
                                            POSITION_LOOP_RESET_CONTROL_DISABLED);
}

uint8 Position_Loop_Is_Enabled(void)
{
    return g_enabled;
}

uint8 Position_Loop_Set_Car_Velocity_Feedforward_Gain(uint8 axis, float gain)
{
    if(!isfinite(gain) || (gain < 0.0f) || (gain > POSITION_LOOP_CAR_FF_GAIN_MAX))
    {
        return 0U;
    }
    if(axis == 0U) g_car_ff_gain_x = gain;
    else if(axis == 1U) g_car_ff_gain_y = gain;
    else return 0U;
    return 1U;
}

void Position_Loop_Get_Car_Velocity_Feedforward_Gain(float *gain_x, float *gain_y)
{
    if(gain_x != NULL) *gain_x = g_car_ff_gain_x;
    if(gain_y != NULL) *gain_y = g_car_ff_gain_y;
}

uint8 Position_Loop_Set_Trajectory_Config(const PositionLoopTrajectoryConfig_t *config)
{
    uint32 irq_state;

    if((config == NULL) || !isfinite(config->position_kp) ||
       !isfinite(config->correction_speed_max_cms) ||
       !isfinite(config->target_speed_max_cms) ||
       !isfinite(config->acceleration_max_cms2) ||
       !isfinite(config->jerk_max_cms3) ||
       (config->position_kp < 0.05f) || (config->position_kp > 3.0f) ||
       (config->correction_speed_max_cms < 10.0f) ||
       (config->correction_speed_max_cms > 200.0f) ||
       (config->target_speed_max_cms < 20.0f) ||
       (config->target_speed_max_cms > 300.0f) ||
       (config->acceleration_max_cms2 < 10.0f) ||
       (config->acceleration_max_cms2 > 400.0f) ||
       (config->jerk_max_cms3 < 20.0f) ||
       (config->jerk_max_cms3 > 1000.0f))
    {
        return 0U;
    }
    irq_state = __get_PRIMASK();
    __disable_irq();
    g_config = *config;
    if(irq_state == 0U) __enable_irq();
    return 1U;
}

void Position_Loop_Get_Trajectory_Config(PositionLoopTrajectoryConfig_t *config)
{
    uint32 irq_state;

    if(config == NULL) return;
    irq_state = __get_PRIMASK();
    __disable_irq();
    *config = g_config;
    if(irq_state == 0U) __enable_irq();
}

void Position_Loop_Update_Velocity_Target(uint8 velocity_ready,
                                          float measured_vx_body_cms,
                                          float measured_vy_body_cms,
                                          float yaw_deg,
                                          float dt_s,
                                          uint8 timing_valid,
                                          const PositionLoopFeedforwardInput_t *feedforward,
                                          float *target_vx_earth_cms,
                                          float *target_vy_earth_cms,
                                          float *target_ax_earth_cms2,
                                          float *target_ay_earth_cms2)
{
    PositionLoopTrajectoryConfig_t config;
    float measured_vx_earth = 0.0f;
    float measured_vy_earth = 0.0f;
    float car_vx_earth = 0.0f;
    float car_vy_earth = 0.0f;
    float car_actual_vx_earth = 0.0f;
    float car_actual_vy_earth = 0.0f;
    float position_x = 0.0f;
    float position_y = 0.0f;
    float desired_vx = 0.0f;
    float desired_vy = 0.0f;
    float correction_vx = 0.0f;
    float correction_vy = 0.0f;
    float distance = 0.0f;
    float closing_speed = 0.0f;
    float stopping_distance = 0.0f;
    float envelope_speed = 0.0f;
    uint8 position_valid = 0U;
    uint8 car_velocity_valid = 0U;
    uint8 flags = 0U;
    uint8 raw_visual_valid;
    uint8 new_visual_pose;

    if((target_vx_earth_cms == NULL) || (target_vy_earth_cms == NULL) ||
       (target_ax_earth_cms2 == NULL) || (target_ay_earth_cms2 == NULL)) return;
    *target_vx_earth_cms = 0.0f;
    *target_vy_earth_cms = 0.0f;
    *target_ax_earth_cms2 = 0.0f;
    *target_ay_earth_cms2 = 0.0f;

    if(!g_enabled)
    {
        position_loop_reset_runtime(POSITION_LOOP_RESET_CONTROL_DISABLED);
        return;
    }
    if(!velocity_ready || !isfinite(yaw_deg) ||
       !isfinite(measured_vx_body_cms) || !isfinite(measured_vy_body_cms))
    {
        Position_Loop_Reset_Trajectory(POSITION_LOOP_RESET_VELOCITY_INVALID);
        g_diag.velocity_ready = velocity_ready;
        g_diag.timing_valid = timing_valid;
        g_diag.trajectory_dt_s = dt_s;
        g_diag.limit_flags &= (uint8)~POSITION_LOOP_STATUS_TIMING_VALID;
        return;
    }
    if(!timing_valid || !isfinite(dt_s) || (dt_s < 0.002f) || (dt_s > 0.008f))
    {
        *target_vx_earth_cms = g_target_vx_earth_cms;
        *target_vy_earth_cms = g_target_vy_earth_cms;
        *target_ax_earth_cms2 = g_target_ax_earth_cms2;
        *target_ay_earth_cms2 = g_target_ay_earth_cms2;
        g_diag.velocity_ready = velocity_ready;
        g_diag.timing_valid = 0U;
        g_diag.trajectory_dt_s = dt_s;
        g_diag.limit_flags &= (uint8)~POSITION_LOOP_STATUS_TIMING_VALID;
        return;
    }
    flags |= POSITION_LOOP_STATUS_TIMING_VALID;

    if(!g_yaw_initialized)
    {
        g_last_yaw_deg = yaw_deg;
        g_yaw_initialized = 1U;
    }
    else
    {
        float yaw_delta = position_loop_wrap_deg180(yaw_deg - g_last_yaw_deg);

        g_last_yaw_deg = yaw_deg;
        if(fabsf(yaw_delta) > POSITION_LOOP_YAW_RESET_THRESHOLD_DEG)
        {
            position_loop_reset_runtime(POSITION_LOOP_RESET_YAW_RESET);
            g_last_yaw_deg = yaw_deg;
            g_yaw_initialized = 1U;
        }
    }

    config = g_config;
    position_loop_body_to_earth(measured_vx_body_cms, measured_vy_body_cms,
                                yaw_deg, &measured_vx_earth, &measured_vy_earth);
    if((feedforward != NULL) && feedforward->valid &&
       isfinite(feedforward->vx_uav_cms) && isfinite(feedforward->vy_uav_cms))
    {
        float gained_body_x;
        float gained_body_y;

        position_loop_body_to_earth(feedforward->vx_uav_cms,
                                    feedforward->vy_uav_cms,
                                    yaw_deg,
                                    &car_actual_vx_earth,
                                    &car_actual_vy_earth);
        gained_body_x = feedforward->vx_uav_cms * g_car_ff_gain_x;
        gained_body_y = feedforward->vy_uav_cms * g_car_ff_gain_y;
        position_loop_body_to_earth(gained_body_x, gained_body_y, yaw_deg,
                                    &car_vx_earth, &car_vy_earth);
        car_velocity_valid = 1U;
        flags |= POSITION_LOOP_STATUS_CAR_FF_VALID;
    }

    raw_visual_valid = position_loop_raw_visual_valid();
    new_visual_pose = (uint8)(raw_visual_valid &&
                              (g_vehicle_marker_pose_update_seq != g_last_pose_seq));
    if(new_visual_pose)
    {
        float pose_x_earth;
        float pose_y_earth;
        float raw_x_earth;
        float raw_y_earth;
        g_last_pose_seq = g_vehicle_marker_pose_update_seq;
        position_loop_body_to_earth(g_vehicle_marker_pose.x,
                                    g_vehicle_marker_pose.y,
                                    yaw_deg, &pose_x_earth, &pose_y_earth);
        position_loop_body_to_earth(g_vehicle_marker_pose.raw_x,
                                    g_vehicle_marker_pose.raw_y,
                                    yaw_deg, &raw_x_earth, &raw_y_earth);
        position_valid = position_loop_accept_visual(pose_x_earth, pose_y_earth,
                                                     raw_x_earth, raw_y_earth,
                                                     &position_x, &position_y);
        if(position_valid)
        {
            g_last_pose_valid = 1U;
            g_last_x_earth_cm = position_x;
            g_last_y_earth_cm = position_y;
            g_observation_state = POSITION_LOOP_OBSERVATION_LIVE;
            position_loop_reset_predict_hold();
        }
    }
    if(!position_valid)
    {
        position_valid = position_loop_predict_hold(measured_vx_earth,
                                                    measured_vy_earth,
                                                    car_velocity_valid,
                                                    car_actual_vx_earth,
                                                    car_actual_vy_earth,
                                                    dt_s,
                                                    &position_x,
                                                    &position_y);
    }

    if(position_valid)
    {
        distance = sqrtf(position_x * position_x + position_y * position_y);
        if(distance > POSITION_LOOP_EPSILON)
        {
            float correction_speed = position_loop_correction_speed(distance, &config);
            float direction_x = position_x / distance;
            float direction_y = position_y / distance;

            correction_vx = correction_speed * position_x / distance;
            correction_vy = correction_speed * position_y / distance;
            closing_speed = (measured_vx_earth -
                             (car_velocity_valid ? car_actual_vx_earth : 0.0f)) *
                            direction_x +
                            (measured_vy_earth -
                             (car_velocity_valid ? car_actual_vy_earth : 0.0f)) *
                            direction_y;
            stopping_distance = position_loop_stopping_distance(
                (closing_speed > 0.0f) ? closing_speed : 0.0f,
                config.acceleration_max_cms2,
                config.jerk_max_cms3);
            envelope_speed = position_loop_envelope_speed(distance,
                                                          config.acceleration_max_cms2,
                                                          config.jerk_max_cms3);
            if(correction_speed >= (config.correction_speed_max_cms - 0.001f))
            {
                flags |= POSITION_LOOP_STATUS_CORRECTION_LIMITED;
            }
        }
        desired_vx = car_vx_earth + correction_vx;
        desired_vy = car_vy_earth + correction_vy;
        if(position_loop_limit_vector(&desired_vx, &desired_vy,
                                      config.target_speed_max_cms))
        {
            flags |= POSITION_LOOP_STATUS_TARGET_LIMITED;
        }
    }

    if(!g_trajectory_initialized)
    {
        g_target_vx_earth_cms = measured_vx_earth;
        g_target_vy_earth_cms = measured_vy_earth;
        g_target_ax_earth_cms2 = 0.0f;
        g_target_ay_earth_cms2 = 0.0f;
        g_trajectory_initialized = 1U;
        if(g_trajectory_reset_reason == POSITION_LOOP_RESET_VELOCITY_INVALID)
        {
            g_trajectory_reset_reason = POSITION_LOOP_RESET_VELOCITY_RECOVERED;
        }
    }
    position_loop_shape_velocity(desired_vx, desired_vy, &config, dt_s, &flags);
    flags |= POSITION_LOOP_STATUS_TRAJECTORY_READY;

    *target_vx_earth_cms = g_target_vx_earth_cms;
    *target_vy_earth_cms = g_target_vy_earth_cms;
    *target_ax_earth_cms2 = g_target_ax_earth_cms2;
    *target_ay_earth_cms2 = g_target_ay_earth_cms2;
    g_diag.target_x_cm = 0.0f;
    g_diag.target_y_cm = 0.0f;
    g_diag.fb_x_cm = position_x;
    g_diag.fb_y_cm = position_y;
    g_diag.error_x_cm = position_x;
    g_diag.error_y_cm = position_y;
    g_diag.vx_cmd_cms = g_target_vx_earth_cms;
    g_diag.vy_cmd_cms = g_target_vy_earth_cms;
    g_diag.correction_vx_cms = correction_vx;
    g_diag.correction_vy_cms = correction_vy;
    g_diag.car_ff_raw_vx_cms = car_vx_earth;
    g_diag.car_ff_raw_vy_cms = car_vy_earth;
    g_diag.car_ff_filtered_vx_cms = car_vx_earth;
    g_diag.car_ff_filtered_vy_cms = car_vy_earth;
    g_diag.car_ff_gain_x = g_car_ff_gain_x;
    g_diag.car_ff_gain_y = g_car_ff_gain_y;
    g_diag.desired_vx_cms = desired_vx;
    g_diag.desired_vy_cms = desired_vy;
    g_diag.target_accel_x_cms2 = g_target_ax_earth_cms2;
    g_diag.target_accel_y_cms2 = g_target_ay_earth_cms2;
    g_diag.position_error_distance_cm = distance;
    g_diag.position_correction_speed_cms = sqrtf(correction_vx * correction_vx +
                                                   correction_vy * correction_vy);
    g_diag.relative_closing_speed_cms = closing_speed;
    g_diag.stopping_distance_cm = stopping_distance;
    g_diag.envelope_speed_cms = envelope_speed;
    g_diag.trajectory_dt_s = dt_s;
    g_diag.vision_innovation_cm = g_vision_innovation_cm;
    g_diag.vision_hold_age_ms = g_hold_elapsed_ms;
    g_diag.vision_observation_state = g_observation_state;
    g_diag.vision_reacquire_count = g_reacquire_count;
    g_diag.car_ff_valid = car_velocity_valid;
    g_diag.limit_flags = flags;
    g_diag.enabled = g_enabled;
    g_diag.active = position_valid;
    g_diag.ir_track_valid = position_valid;
    g_diag.velocity_ready = velocity_ready;
    g_diag.timing_valid = timing_valid;
    g_diag.trajectory_reset_reason = g_trajectory_reset_reason;
    g_diag.trajectory_reset_count = g_trajectory_reset_count;
}

void Position_Loop_Get_Diag(PositionLoopDiag_t *diag)
{
    uint32 irq_state;

    if(diag == NULL) return;
    irq_state = __get_PRIMASK();
    __disable_irq();
    *diag = g_diag;
    if(irq_state == 0U) __enable_irq();
}
