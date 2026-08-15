#include "control/position_loop.h"
#include "vision/vehicle_marker_localization.h"
#include <math.h>
//P分支由简单P控制切换到Sqrt分支的 切换距离 = M8 / M3²
#define POSITION_LOOP_CAR_FF_GAIN_DEFAULT          (0.0f)   // M1/M2：小车X/Y速度前馈增益上电默认值
#define POSITION_LOOP_CAR_FF_GAIN_MAX              (1.2f)    // M1/M2：小车X/Y速度前馈增益允许上限
#define POSITION_LOOP_POSITION_KP_DEFAULT          (0.0f)    // M3：二维位置P增益默认值，单位s^-1
#define POSITION_LOOP_CORRECTION_MAX_DEFAULT_CMS   (100.0f)   // M4：位置修正速度模长上限，单位cm/s
#define POSITION_LOOP_TARGET_MAX_DEFAULT_CMS       (150.0f)   // M5：合成后目标速度模长上限，单位cm/s
#define POSITION_LOOP_ACCEL_MAX_DEFAULT_CMS2       (80.0f)   // M6：轨迹加速度模长上限，单位cm/s^2
#define POSITION_LOOP_JERK_MAX_DEFAULT_CMS3        (150.0f)  // M7：轨迹Jerk模长上限，单位cm/s^3
#define POSITION_LOOP_CORRECTION_ACCEL_DEFAULT_CMS2 (35.0f)  // M8：P/Sqrt位置纠偏包络加速度，单位cm/s^2
//P分支由简单P控制切换到Sqrt分支的 切换距离 = M8 / M3² 
//v_corr = sqrt(2×M8×(e-0.5×M8/M3²))
#define POSITION_LOOP_VISION_HOLD_MS               (200U)   // 视觉断帧后相对位置预测保持时间，单位ms
#define POSITION_LOOP_EPSILON                      (0.0001f) // 二维距离归一化时的除零保护阈值
#define POSITION_LOOP_YAW_RESET_THRESHOLD_DEG      (30.0f)   // 单个水平控制周期内超过该变化视为Yaw参考系重置
#define POSITION_LOOP_EFFECTIVE_ACCEL_DEFAULT_CMS2  (170.0f)  // N1：单一停车模型有效加速度，单位cm/s^2
#define POSITION_LOOP_EFFECTIVE_JERK_DEFAULT_CMS3   (350.0f) // N2：单一停车模型有效Jerk，单位cm/s^3
#define POSITION_LOOP_CONTROL_DELAY_DEFAULT_MS      (120.0f) // N3：控制系统等效延迟，单位ms
#define POSITION_LOOP_SAFETY_BUFFER_DEFAULT_CM      (5.0f)  // N4：停车安全余量，单位cm
#define POSITION_LOOP_BOOST_MARGIN_DEFAULT_CM       (15.0f)  // N5：增强制动连续介入区宽度，单位cm
#define POSITION_LOOP_BRAKE_ACCEL_DEFAULT_CMS2      (230.0f) // N6：增强制动整形加速度上限，单位cm/s^2
#define POSITION_LOOP_BRAKE_JERK_DEFAULT_CMS3       (420.0f) // N7：增强制动整形Jerk上限，单位cm/s^3
#define POSITION_LOOP_CLOSING_SPEED_LPF_HZ          (3.0f)    // 径向接近速度调度低通截止频率，单位Hz
#define POSITION_LOOP_TWO_PI                        (6.28318530718f) // 一阶低通系数计算使用的2π常量
static volatile uint8 g_enabled = 0U;
static volatile PositionLoopTrajectoryConfig_t g_config = {
    POSITION_LOOP_POSITION_KP_DEFAULT,
    POSITION_LOOP_CORRECTION_ACCEL_DEFAULT_CMS2,
    POSITION_LOOP_CORRECTION_MAX_DEFAULT_CMS,
    POSITION_LOOP_TARGET_MAX_DEFAULT_CMS,
    POSITION_LOOP_ACCEL_MAX_DEFAULT_CMS2,
    POSITION_LOOP_JERK_MAX_DEFAULT_CMS3
};
static volatile PositionLoopBrakeConfig_t g_brake_config = {
    POSITION_LOOP_EFFECTIVE_ACCEL_DEFAULT_CMS2,
    POSITION_LOOP_EFFECTIVE_JERK_DEFAULT_CMS3,
    POSITION_LOOP_CONTROL_DELAY_DEFAULT_MS,
    POSITION_LOOP_SAFETY_BUFFER_DEFAULT_CM,
    POSITION_LOOP_BOOST_MARGIN_DEFAULT_CM,
    POSITION_LOOP_BRAKE_ACCEL_DEFAULT_CMS2,
    POSITION_LOOP_BRAKE_JERK_DEFAULT_CMS3
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
static float g_filtered_closing_speed_cms = 0.0f;
static uint8 g_closing_speed_initialized = 0U;
static float g_brake_schedule_ratio = 0.0f;

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
static float g_vision_innovation_cm = 0.0f;
static volatile uint8 g_frozen_hold_active = 0U;

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

static void position_loop_reset_brake_supervisor(void)
{
    g_filtered_closing_speed_cms = 0.0f;
    g_closing_speed_initialized = 0U;
    g_brake_schedule_ratio = 0.0f;
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

static void position_loop_update_brake_supervisor(
    float closing_speed,
    float distance,
    float dt_s,
    const PositionLoopTrajectoryConfig_t *trajectory,
    const PositionLoopBrakeConfig_t *brake,
    float *schedule_ratio,
    float *acceleration_limit,
    float *jerk_limit,
    float *stopping_distance,
    float *delay_distance,
    float *margin)
{
    float alpha;
    float positive_closing;
    float target_ratio = 0.0f;
    float brake_acceleration;
    float brake_jerk;

    if((trajectory == NULL) || (brake == NULL) || (schedule_ratio == NULL) ||
       (acceleration_limit == NULL) || (jerk_limit == NULL) ||
       (stopping_distance == NULL) || (delay_distance == NULL) ||
       (margin == NULL))
    {
        return;
    }

    if(!isfinite(closing_speed) || !isfinite(distance) ||
       !isfinite(dt_s) || !(dt_s > 0.0f))
    {
        position_loop_reset_brake_supervisor();
        *schedule_ratio = 0.0f;
        *acceleration_limit = trajectory->acceleration_max_cms2;
        *jerk_limit = trajectory->jerk_max_cms3;
        *stopping_distance = 0.0f;
        *delay_distance = 0.0f;
        *margin = 0.0f;
        return;
    }

    if(!g_closing_speed_initialized)
    {
        g_filtered_closing_speed_cms = closing_speed;
        g_closing_speed_initialized = 1U;
    }
    else
    {
        alpha = 1.0f - expf(-POSITION_LOOP_TWO_PI *
                            POSITION_LOOP_CLOSING_SPEED_LPF_HZ * dt_s);
        alpha = position_loop_limit_float(alpha, 0.0f, 1.0f);
        g_filtered_closing_speed_cms +=
            alpha * (closing_speed - g_filtered_closing_speed_cms);
    }

    positive_closing = (g_filtered_closing_speed_cms > 0.0f) ?
                       g_filtered_closing_speed_cms : 0.0f;
    *stopping_distance = position_loop_stopping_distance(
        positive_closing,
        brake->effective_accel_cms2,
        brake->effective_jerk_cms3);
    *delay_distance = positive_closing * brake->control_delay_ms * 0.001f;
    *margin = distance - *stopping_distance -
              *delay_distance - brake->safety_buffer_cm;

    if((positive_closing > POSITION_LOOP_EPSILON) &&
       (brake->boost_margin_cm > POSITION_LOOP_EPSILON))
    {
        target_ratio = (brake->boost_margin_cm - *margin) /
                       brake->boost_margin_cm;
        target_ratio = position_loop_limit_float(target_ratio, 0.0f, 1.0f);
        target_ratio = target_ratio * target_ratio * (3.0f - 2.0f * target_ratio);
    }

    alpha = 1.0f - expf(-POSITION_LOOP_TWO_PI *
                        POSITION_LOOP_CLOSING_SPEED_LPF_HZ * dt_s);
    alpha = position_loop_limit_float(alpha, 0.0f, 1.0f);
    g_brake_schedule_ratio += alpha * (target_ratio - g_brake_schedule_ratio);

    brake_acceleration = (brake->brake_accel_max_cms2 >
                          trajectory->acceleration_max_cms2) ?
                         brake->brake_accel_max_cms2 :
                         trajectory->acceleration_max_cms2;
    brake_jerk = (brake->brake_jerk_max_cms3 > trajectory->jerk_max_cms3) ?
                 brake->brake_jerk_max_cms3 : trajectory->jerk_max_cms3;
    *schedule_ratio = g_brake_schedule_ratio;
    *acceleration_limit = trajectory->acceleration_max_cms2 +
                          g_brake_schedule_ratio *
                          (brake_acceleration - trajectory->acceleration_max_cms2);
    *jerk_limit = trajectory->jerk_max_cms3 +
                  g_brake_schedule_ratio *
                  (brake_jerk - trajectory->jerk_max_cms3);
}

static void position_loop_reset_reacquisition(void)
{
    g_reacquire_count = 0U;
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
    position_loop_reset_brake_supervisor();
    g_diag.vx_cmd_cms = 0.0f;
    g_diag.vy_cmd_cms = 0.0f;
    g_diag.target_accel_x_cms2 = 0.0f;
    g_diag.target_accel_y_cms2 = 0.0f;
    g_diag.relative_closing_speed_cms = 0.0f;
    g_diag.filtered_closing_speed_cms = 0.0f;
    g_diag.brake_schedule_ratio = 0.0f;
    g_diag.brake_accel_limit_cms2 = 0.0f;
    g_diag.brake_jerk_limit_cms3 = 0.0f;
    g_diag.brake_schedule_active = 0U;
    g_diag.relative_velocity_x_cms = 0.0f;
    g_diag.relative_velocity_y_cms = 0.0f;
    g_diag.relative_speed_cms = 0.0f;
    g_diag.proportional_speed_cms = 0.0f;
    g_diag.sqrt_speed_cms = 0.0f;
    g_diag.stopping_distance_cm = 0.0f;
    g_diag.brake_delay_distance_cm = 0.0f;
    g_diag.brake_margin_cm = 0.0f;
    g_diag.brake_diag_flags = 0U;
    g_diag.sqrt_controller_active = 0U;
    g_diag.limit_flags &= (uint8)~(POSITION_LOOP_STATUS_TRAJECTORY_READY |
                                  POSITION_LOOP_STATUS_TIMING_VALID |
                                  POSITION_LOOP_STATUS_BRAKE_MONITORING);
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
    g_frozen_hold_active = 0U;
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
    }
    else
    {
        g_vision_innovation_cm = 0.0f;
    }
    *accepted_x = pose_x;
    *accepted_y = pose_y;
    position_loop_reset_reacquisition();
    return 1U;
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

static uint8 position_loop_update_frozen_hold(float measured_vx_earth,
                                               float measured_vy_earth,
                                               float dt_s,
                                               float *position_x,
                                               float *position_y)
{
    if(!g_frozen_hold_active || !g_last_pose_valid ||
       !isfinite(measured_vx_earth) || !isfinite(measured_vy_earth) ||
       !isfinite(dt_s) || !(dt_s > 0.0f) ||
       (position_x == NULL) || (position_y == NULL))
    {
        return 0U;
    }

    g_last_x_earth_cm -= measured_vx_earth * dt_s;
    g_last_y_earth_cm -= measured_vy_earth * dt_s;
    *position_x = g_last_x_earth_cm;
    *position_y = g_last_y_earth_cm;
    g_hold_elapsed_ms = 0U;
    g_observation_state = POSITION_LOOP_OBSERVATION_HOLD;
    return 1U;
}

static float position_loop_correction_speed(float distance,
                                            const PositionLoopTrajectoryConfig_t *config,
                                            float *proportional_speed,
                                            float *sqrt_speed,
                                            uint8 *sqrt_active)
{
    float linear_distance;
    float speed;

    if((config == NULL) || (proportional_speed == NULL) ||
       (sqrt_speed == NULL) || (sqrt_active == NULL) ||
       !isfinite(distance) || !(distance > 0.0f))
    {
        return 0.0f;
    }

    *proportional_speed = config->position_kp * distance;
    linear_distance = config->correction_accel_max_cms2 /
                      (config->position_kp * config->position_kp);
    if(distance > linear_distance)
    {
        speed = sqrtf(2.0f * config->correction_accel_max_cms2 *
                      (distance - 0.5f * linear_distance));
        *sqrt_active = 1U;
    }
    else
    {
        speed = *proportional_speed;
        *sqrt_active = 0U;
    }
    *sqrt_speed = speed;
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
    float old_acceleration_magnitude;
    float acceleration_clamp;
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
    old_acceleration_magnitude = sqrtf(old_ax * old_ax + old_ay * old_ay);
    acceleration_clamp = config->acceleration_max_cms2;
    if(old_acceleration_magnitude > acceleration_clamp)
    {
        /* 动态制动退出时保留当前包络，让加速度按Jerk逐步退回基础上限。 */
        acceleration_clamp = old_acceleration_magnitude;
    }
    if(position_loop_limit_vector(&g_target_ax_earth_cms2,
                                  &g_target_ay_earth_cms2,
                                  acceleration_clamp))
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

uint8 Position_Loop_Begin_Frozen_Hold(void)
{
    uint8 accepted = 0U;
    uint32 irq_state = __get_PRIMASK();

    __disable_irq();
    if(g_enabled && g_last_pose_valid &&
       ((g_observation_state == POSITION_LOOP_OBSERVATION_LIVE) ||
        (g_observation_state == POSITION_LOOP_OBSERVATION_HOLD)))
    {
        g_frozen_hold_active = 1U;
        position_loop_reset_predict_hold();
        position_loop_reset_reacquisition();
        g_observation_state = POSITION_LOOP_OBSERVATION_HOLD;
        accepted = 1U;
    }
    if(irq_state == 0U)
    {
        __enable_irq();
    }
    return accepted;
}

void Position_Loop_End_Frozen_Hold(void)
{
    uint32 irq_state = __get_PRIMASK();

    __disable_irq();
    if(g_frozen_hold_active)
    {
        g_frozen_hold_active = 0U;
        g_last_pose_valid = 0U;
        g_observation_state = POSITION_LOOP_OBSERVATION_INVALID;
        position_loop_reset_predict_hold();
        position_loop_reset_reacquisition();
    }
    if(irq_state == 0U)
    {
        __enable_irq();
    }
}

uint8 Position_Loop_Is_Frozen_Hold_Active(void)
{
    return g_frozen_hold_active ? 1U : 0U;
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
       !isfinite(config->correction_accel_max_cms2) ||
       !isfinite(config->correction_speed_max_cms) ||
       !isfinite(config->target_speed_max_cms) ||
       !isfinite(config->acceleration_max_cms2) ||
       !isfinite(config->jerk_max_cms3) ||
       (config->position_kp < 0.05f) || (config->position_kp > 3.0f) ||
       (config->correction_accel_max_cms2 < 5.0f) ||
       (config->correction_accel_max_cms2 > 400.0f) ||
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

uint8 Position_Loop_Set_Brake_Config(const PositionLoopBrakeConfig_t *config)
{
    uint32 irq_state;

    if((config == NULL) ||
       !isfinite(config->effective_accel_cms2) ||
       !isfinite(config->effective_jerk_cms3) ||
       !isfinite(config->control_delay_ms) ||
       !isfinite(config->safety_buffer_cm) ||
       !isfinite(config->boost_margin_cm) ||
       !isfinite(config->brake_accel_max_cms2) ||
       !isfinite(config->brake_jerk_max_cms3) ||
       (config->effective_accel_cms2 < 10.0f) ||
       (config->effective_accel_cms2 > 400.0f) ||
       (config->effective_jerk_cms3 < 20.0f) ||
       (config->effective_jerk_cms3 > 1000.0f) ||
       (config->control_delay_ms < 0.0f) || (config->control_delay_ms > 500.0f) ||
       (config->safety_buffer_cm < 0.0f) || (config->safety_buffer_cm > 100.0f) ||
       (config->boost_margin_cm < 1.0f) || (config->boost_margin_cm > 100.0f) ||
       (config->brake_accel_max_cms2 < 10.0f) ||
       (config->brake_accel_max_cms2 > 400.0f) ||
       (config->brake_jerk_max_cms3 < 20.0f) ||
       (config->brake_jerk_max_cms3 > 1000.0f))
    {
        return 0U;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    g_brake_config = *config;
    if(irq_state == 0U) __enable_irq();
    return 1U;
}

void Position_Loop_Get_Brake_Config(PositionLoopBrakeConfig_t *config)
{
    uint32 irq_state;

    if(config == NULL) return;
    irq_state = __get_PRIMASK();
    __disable_irq();
    *config = g_brake_config;
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
    PositionLoopTrajectoryConfig_t shape_config;
    PositionLoopBrakeConfig_t brake_config;
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
    float relative_velocity_x = 0.0f;
    float relative_velocity_y = 0.0f;
    float relative_speed = 0.0f;
    float closing_speed = 0.0f;
    float target_direction_x = 0.0f;
    float target_direction_y = 0.0f;
    float brake_schedule_ratio = 0.0f;
    float brake_accel_limit = 0.0f;
    float brake_jerk_limit = 0.0f;
    float stopping_distance = 0.0f;
    float brake_delay_distance = 0.0f;
    float brake_margin = 0.0f;
    float envelope_speed = 0.0f;
    float proportional_speed = 0.0f;
    uint8 position_valid = 0U;
    uint8 car_velocity_valid = 0U;
    uint8 flags = 0U;
    uint8 brake_diag_flags = 0U;
    uint8 brake_schedule_active = 0U;
    uint8 sqrt_controller_active = 0U;
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
        g_diag.brake_schedule_active = 0U;
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
    brake_config = g_brake_config;
    shape_config = config;
    brake_accel_limit = config.acceleration_max_cms2;
    brake_jerk_limit = config.jerk_max_cms3;
    position_loop_body_to_earth(measured_vx_body_cms, measured_vy_body_cms,
                                yaw_deg, &measured_vx_earth, &measured_vy_earth);
    if(!g_frozen_hold_active && (feedforward != NULL) && feedforward->valid &&
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

    raw_visual_valid = (uint8)(!g_frozen_hold_active &&
                               position_loop_raw_visual_valid());
    new_visual_pose = (uint8)(raw_visual_valid &&
                              (g_vehicle_marker_pose_update_seq != g_last_pose_seq));
    if(g_frozen_hold_active)
    {
        position_valid = position_loop_update_frozen_hold(measured_vx_earth,
                                                          measured_vy_earth,
                                                          dt_s,
                                                          &position_x,
                                                          &position_y);
    }
    else if(new_visual_pose)
    {
        float pose_x_body;
        float pose_y_body;
        float raw_x_body;
        float raw_y_body;
        float pose_x_earth;
        float pose_y_earth;
        float raw_x_earth;
        float raw_y_earth;
        g_last_pose_seq = g_vehicle_marker_pose_update_seq;
        if(VehicleMarkerLocalization_GetFollowPoint(&g_vehicle_marker_pose,
                                                    &pose_x_body, &pose_y_body,
                                                    &raw_x_body, &raw_y_body))
        {
            position_loop_body_to_earth(pose_x_body, pose_y_body,
                                        yaw_deg, &pose_x_earth, &pose_y_earth);
            position_loop_body_to_earth(raw_x_body, raw_y_body,
                                        yaw_deg, &raw_x_earth, &raw_y_earth);
            position_valid = position_loop_accept_visual(
                pose_x_earth, pose_y_earth,
                raw_x_earth, raw_y_earth,
                &position_x, &position_y);
        }
        if(position_valid)
        {
            g_last_pose_valid = 1U;
            g_last_x_earth_cm = position_x;
            g_last_y_earth_cm = position_y;
            g_observation_state = POSITION_LOOP_OBSERVATION_LIVE;
            position_loop_reset_predict_hold();
        }
    }
    if(!position_valid && !g_frozen_hold_active)
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
        /* 真实位置误差只进入P/Sqrt纠偏；停止距离仅用于制动余量监督。 */
        distance = sqrtf(position_x * position_x + position_y * position_y);
        relative_velocity_x = measured_vx_earth -
                              (car_velocity_valid ? car_actual_vx_earth : 0.0f);
        relative_velocity_y = measured_vy_earth -
                              (car_velocity_valid ? car_actual_vy_earth : 0.0f);
        relative_speed = sqrtf(relative_velocity_x * relative_velocity_x +
                               relative_velocity_y * relative_velocity_y);
        if(car_velocity_valid)
        {
            brake_diag_flags |= POSITION_LOOP_BRAKE_DIAG_CAR_VELOCITY_VALID;
        }
        else
        {
            brake_diag_flags |= POSITION_LOOP_BRAKE_DIAG_STATIC_FALLBACK;
        }
        if(distance > POSITION_LOOP_EPSILON)
        {
            float correction_speed;

            target_direction_x = position_x / distance;
            target_direction_y = position_y / distance;
            closing_speed = relative_velocity_x * target_direction_x +
                            relative_velocity_y * target_direction_y;
            position_loop_update_brake_supervisor(
                closing_speed,
                distance,
                dt_s,
                &config,
                &brake_config,
                &brake_schedule_ratio,
                &brake_accel_limit,
                &brake_jerk_limit,
                &stopping_distance,
                &brake_delay_distance,
                &brake_margin);

            if(g_filtered_closing_speed_cms > POSITION_LOOP_EPSILON)
            {
                brake_diag_flags |= POSITION_LOOP_BRAKE_DIAG_CLOSING_ACTIVE;
                flags |= POSITION_LOOP_STATUS_BRAKE_MONITORING;
            }
            if(brake_margin <= 0.0f)
            {
                brake_diag_flags |= POSITION_LOOP_BRAKE_DIAG_MARGIN_LOW;
            }

            correction_speed = position_loop_correction_speed(
                distance,
                &config,
                &proportional_speed,
                &envelope_speed,
                &sqrt_controller_active);

            correction_vx = correction_speed * target_direction_x;
            correction_vy = correction_speed * target_direction_y;
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

    if(!position_valid)
    {
        position_loop_reset_brake_supervisor();
        brake_schedule_ratio = 0.0f;
        brake_accel_limit = config.acceleration_max_cms2;
        brake_jerk_limit = config.jerk_max_cms3;
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
    if(position_valid && (distance > POSITION_LOOP_EPSILON))
    {
        float delta_vx = desired_vx - g_target_vx_earth_cms;
        float delta_vy = desired_vy - g_target_vy_earth_cms;
        float direction_dot = delta_vx * relative_velocity_x +
                              delta_vy * relative_velocity_y;
        float reverse_dot = desired_vx * measured_vx_earth +
                            desired_vy * measured_vy_earth;
        float measured_speed_sq = measured_vx_earth * measured_vx_earth +
                                  measured_vy_earth * measured_vy_earth;
        float desired_speed_sq = desired_vx * desired_vx + desired_vy * desired_vy;
        uint8 reverse_active = (uint8)((reverse_dot < 0.0f) &&
            (measured_speed_sq > POSITION_LOOP_EPSILON) &&
            (desired_speed_sq > POSITION_LOOP_EPSILON));
        uint8 direction_braking = (uint8)(direction_dot < 0.0f);

        if(direction_braking)
        {
            brake_diag_flags |= POSITION_LOOP_BRAKE_DIAG_DIRECTION_ACTIVE;
        }
        if(reverse_active)
        {
            brake_diag_flags |= POSITION_LOOP_BRAKE_DIAG_REVERSE_ACTIVE;
            shape_config.acceleration_max_cms2 =
                (brake_config.brake_accel_max_cms2 > config.acceleration_max_cms2) ?
                brake_config.brake_accel_max_cms2 : config.acceleration_max_cms2;
            shape_config.jerk_max_cms3 =
                (brake_config.brake_jerk_max_cms3 > config.jerk_max_cms3) ?
                brake_config.brake_jerk_max_cms3 : config.jerk_max_cms3;
            brake_schedule_ratio = 1.0f;
            brake_accel_limit = shape_config.acceleration_max_cms2;
            brake_jerk_limit = shape_config.jerk_max_cms3;
        }
        else if(direction_braking &&
                (brake_schedule_ratio > POSITION_LOOP_EPSILON))
        {
            shape_config.acceleration_max_cms2 = brake_accel_limit;
            shape_config.jerk_max_cms3 = brake_jerk_limit;
        }
        brake_schedule_active = (uint8)((reverse_active || direction_braking) &&
            ((shape_config.acceleration_max_cms2 > config.acceleration_max_cms2) ||
             (shape_config.jerk_max_cms3 > config.jerk_max_cms3)));
    }
    position_loop_shape_velocity(desired_vx, desired_vy, &shape_config, dt_s, &flags);
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
    g_diag.proportional_speed_cms = proportional_speed;
    g_diag.sqrt_speed_cms = envelope_speed;
    g_diag.relative_closing_speed_cms = closing_speed;
    g_diag.filtered_closing_speed_cms = g_filtered_closing_speed_cms;
    g_diag.brake_schedule_ratio = brake_schedule_ratio;
    g_diag.brake_accel_limit_cms2 = brake_accel_limit;
    g_diag.brake_jerk_limit_cms3 = brake_jerk_limit;
    g_diag.brake_schedule_active = brake_schedule_active;
    g_diag.relative_velocity_x_cms = relative_velocity_x;
    g_diag.relative_velocity_y_cms = relative_velocity_y;
    g_diag.relative_speed_cms = relative_speed;
    g_diag.stopping_distance_cm = stopping_distance;
    g_diag.brake_delay_distance_cm = brake_delay_distance;
    g_diag.brake_margin_cm = brake_margin;
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
    g_diag.brake_diag_flags = brake_diag_flags;
    g_diag.sqrt_controller_active = sqrt_controller_active;
    g_diag.frozen_hold_active = g_frozen_hold_active;
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
