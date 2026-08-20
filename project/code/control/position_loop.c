#include "control/position_loop.h"
#include "communication/camera_board_link.h"
#include "vision/vehicle_marker_localization.h"
#include "vision/dual_camera_fusion.h"
#include <math.h>
//P分支由简单P控制切换到Sqrt分支的 切换距离 = M8 / M3²
#define POSITION_LOOP_CAR_VELOCITY_FF_GAIN_DEFAULT (0.6f)   // M1/M2：小车X/Y速度前馈增益上电默认值
#define POSITION_LOOP_CAR_VELOCITY_FF_GAIN_MAX     (1.0f)   // M1/M2：小车X/Y速度前馈增益允许上限
#define POSITION_LOOP_CAR_ACCEL_FF_GAIN_DEFAULT    (1.0f)   // M6/M7：小车X/Y加速度前馈增益上电默认值
#define POSITION_LOOP_CAR_ACCEL_FF_GAIN_MAX        (1.0f)   // M6/M7：小车X/Y加速度前馈增益允许上限
#define POSITION_LOOP_POSITION_KP_DEFAULT          (0.22f)    // M3：二维位置P增益默认值，单位s^-1
#define POSITION_LOOP_CORRECTION_MAX_DEFAULT_CMS   (100.0f)   // M4：位置修正速度模长上限，单位cm/s
#define POSITION_LOOP_TARGET_MAX_DEFAULT_CMS       (300.0f)   // M5：合成后目标速度模长上限，单位cm/s
#define POSITION_LOOP_CORRECTION_ACCEL_DEFAULT_CMS2 (35.0f)  // M8：P/Sqrt位置纠偏包络加速度，单位cm/s^2
//P分支由简单P控制切换到Sqrt分支的 切换距离 = M8 / M3² 
//v_corr = sqrt(2×M8×(e-0.5×M8/M3²))
#define POSITION_LOOP_VISION_HOLD_MS               (200U)   // 视觉断帧后相对位置预测保持时间，单位ms
#define POSITION_LOOP_VISION_MAX_AGE_MS             (100U)   // 新视觉观测允许参与更新的最大帧龄
#define POSITION_LOOP_VISION_REACQUIRE_SPAN_MS      (150U)   // 三帧重捕获允许的最大时间跨度
#define POSITION_LOOP_VISION_REACQUIRE_FRAMES       (3U)     // 重捕获所需连续一致观测数
#define POSITION_LOOP_VISION_LIVE_GATE_BASE_CM      (20.0f)  // 正常跟踪创新门基础值
#define POSITION_LOOP_VISION_LIVE_GATE_MAX_CM       (40.0f)  // 正常跟踪创新门上限
#define POSITION_LOOP_VISION_REACQUIRE_GATE_BASE_CM (10.0f)  // 重捕获帧间一致性基础门限
#define POSITION_LOOP_VISION_REACQUIRE_GATE_MAX_CM  (20.0f)  // 重捕获帧间一致性门限上限
#define POSITION_LOOP_VISION_VELOCITY_SIGMA_CMS     (50.0f)  // 相对速度预测不确定度
#define POSITION_LOOP_VISION_ACCEL_SIGMA_CMS2       (300.0f) // 未建模相对加速度不确定度
#define POSITION_LOOP_VISION_POSITION_VARIANCE_CM2  (25.0f)  // 视觉位置标准差5cm对应方差
#define POSITION_LOOP_VISION_PROCESS_REFERENCE_S    (0.020f) // 50Hz视觉周期，用于连续过程噪声离散化
#define POSITION_LOOP_EPSILON                      (0.0001f) // 二维距离归一化时的除零保护阈值
#define POSITION_LOOP_YAW_RESET_THRESHOLD_DEG      (60.0f)   // 单个水平控制周期内超过该变化视为Yaw参考系重置
#define POSITION_LOOP_EFFECTIVE_ACCEL_DEFAULT_CMS2  (200.0f)  // N1：A/J安全速度模型的有效加速度，单位cm/s^2
#define POSITION_LOOP_EFFECTIVE_JERK_DEFAULT_CMS3   (500.0f) // N2：A/J安全速度模型的有效Jerk，单位cm/s^3
#define POSITION_LOOP_CONTROL_DELAY_DEFAULT_MS      (0.0f)   // N3：控制系统等效延迟，单位ms
#define POSITION_LOOP_SAFETY_BUFFER_DEFAULT_CM      (0.0f)   // N4：安全余量，单位cm
#define POSITION_LOOP_BOOST_MARGIN_DEFAULT_CM       (0.0f)   // N5：速度包络提前收紧宽度，单位cm
#define POSITION_LOOP_CLOSING_SPEED_LPF_HZ          (3.0f)    // 径向接近速度诊断低通截止频率，单位Hz
#define POSITION_LOOP_TWO_PI                        (6.28318530718f) // 一阶低通系数计算使用的2π常量
#define POSITION_LOOP_VISION_SOURCE_FRONT           (0U)
#define POSITION_LOOP_VISION_SOURCE_REAR            (1U)
static volatile uint8 g_enabled = 0U;
static volatile PositionLoopTrajectoryConfig_t g_config = {
    POSITION_LOOP_POSITION_KP_DEFAULT,
    POSITION_LOOP_CORRECTION_ACCEL_DEFAULT_CMS2,
    POSITION_LOOP_CORRECTION_MAX_DEFAULT_CMS,
    POSITION_LOOP_TARGET_MAX_DEFAULT_CMS
};
static volatile PositionLoopBrakeConfig_t g_brake_config = {
    POSITION_LOOP_EFFECTIVE_ACCEL_DEFAULT_CMS2,
    POSITION_LOOP_EFFECTIVE_JERK_DEFAULT_CMS3,
    POSITION_LOOP_CONTROL_DELAY_DEFAULT_MS,
    POSITION_LOOP_SAFETY_BUFFER_DEFAULT_CM,
    POSITION_LOOP_BOOST_MARGIN_DEFAULT_CM
};
static volatile float g_car_velocity_ff_gain_x = POSITION_LOOP_CAR_VELOCITY_FF_GAIN_DEFAULT;
static volatile float g_car_velocity_ff_gain_y = POSITION_LOOP_CAR_VELOCITY_FF_GAIN_DEFAULT;
static volatile float g_car_accel_ff_gain_x = POSITION_LOOP_CAR_ACCEL_FF_GAIN_DEFAULT;
static volatile float g_car_accel_ff_gain_y = POSITION_LOOP_CAR_ACCEL_FF_GAIN_DEFAULT;
static volatile PositionLoopDiag_t g_diag;

static float g_target_vx_earth_cms = 0.0f;
static float g_target_vy_earth_cms = 0.0f;
static float g_target_ax_earth_cms2 = 0.0f;
static float g_target_ay_earth_cms2 = 0.0f;
static uint8 g_target_initialized = 0U;
static uint8 g_trajectory_reset_reason = POSITION_LOOP_RESET_NONE;
static uint32 g_trajectory_reset_count = 0U;
static float g_last_yaw_deg = 0.0f;
static uint8 g_yaw_initialized = 0U;
static float g_filtered_closing_speed_cms = 0.0f;
static uint8 g_closing_speed_initialized = 0U;

static uint8 g_last_pose_valid = 0U;
static float g_last_x_earth_cm = 0.0f;
static float g_last_y_earth_cm = 0.0f;
static float g_position_covariance_x_cm2 = POSITION_LOOP_VISION_POSITION_VARIANCE_CM2;
static float g_position_covariance_y_cm2 = POSITION_LOOP_VISION_POSITION_VARIANCE_CM2;
static uint8 g_position_estimator_initialized = 0U;
static uint32 g_last_pose_seq = 0U;
static uint32 g_last_rear_pose_seq = 0U;
static uint8 g_vision_source = POSITION_LOOP_VISION_SOURCE_FRONT;
static uint16 g_hold_elapsed_ms = 0U;
static uint8 g_observation_state = POSITION_LOOP_OBSERVATION_INVALID;
static uint8 g_reacquire_count = 0U;
static uint8 g_reacquire_source = POSITION_LOOP_VISION_SOURCE_FRONT;
static float g_reacquire_x_earth_cm = 0.0f;
static float g_reacquire_y_earth_cm = 0.0f;
static uint32 g_reacquire_first_time_us = 0U;
static uint32 g_reacquire_last_time_us = 0U;
static float g_vision_innovation_cm = 0.0f;
static float g_vision_gate_cm = 0.0f;
static float g_vision_observation_x_earth_cm = 0.0f;
static float g_vision_observation_y_earth_cm = 0.0f;
static uint16 g_vision_observation_age_ms = 0U;
static uint8 g_vision_observation_accepted = 0U;
static uint8 g_vision_observation_source = POSITION_LOOP_VISION_SOURCE_FRONT;
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
}

/* 按A/J约束将剩余有效距离反解为允许的最大相对接近速度。 */
static float position_loop_allowed_speed_from_distance(float distance,
                                                        float acceleration_limit,
                                                        float jerk_limit)
{
    float switch_speed;
    float switch_distance;
    float radicand;

    if(!isfinite(distance) || !isfinite(acceleration_limit) ||
       !isfinite(jerk_limit) || !(distance > 0.0f) ||
       !(acceleration_limit > 0.0f) || !(jerk_limit > 0.0f))
    {
        return 0.0f;
    }

    switch_speed = acceleration_limit * acceleration_limit / jerk_limit;
    switch_distance = acceleration_limit * acceleration_limit *
                      acceleration_limit / (jerk_limit * jerk_limit);
    if(distance <= switch_distance)
    {
        return powf(distance * distance * jerk_limit, 1.0f / 3.0f);
    }

    radicand = acceleration_limit * acceleration_limit *
               acceleration_limit * acceleration_limit /
               (4.0f * jerk_limit * jerk_limit) +
               2.0f * acceleration_limit * distance;
    if(!(radicand > 0.0f))
    {
        return switch_speed;
    }
    return sqrtf(radicand) -
           acceleration_limit * acceleration_limit / (2.0f * jerk_limit);
}

static void position_loop_update_brake_supervisor(
    float closing_speed,
    float distance,
    float dt_s,
    const PositionLoopBrakeConfig_t *brake,
    float *delay_distance,
    float *allowed_speed)
{
    float alpha;
    float positive_closing;

    if((brake == NULL) || (delay_distance == NULL) ||
       (allowed_speed == NULL))
    {
        return;
    }

    if(!isfinite(closing_speed) || !isfinite(distance) ||
       !isfinite(dt_s) || !(dt_s > 0.0f))
    {
        position_loop_reset_brake_supervisor();
        *delay_distance = 0.0f;
        *allowed_speed = 0.0f;
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
    *delay_distance = positive_closing * brake->control_delay_ms * 0.001f;
    /* N1/N2只建立安全速度边界，不再生成独立的停车距离或余量诊断量。 */
    *allowed_speed = position_loop_allowed_speed_from_distance(
        distance - *delay_distance - brake->safety_buffer_cm -
        brake->boost_margin_cm,
        brake->effective_accel_cms2,
        brake->effective_jerk_cms3);
}

static void position_loop_reset_reacquisition(void)
{
    g_reacquire_count = 0U;
    g_reacquire_x_earth_cm = 0.0f;
    g_reacquire_y_earth_cm = 0.0f;
    g_reacquire_first_time_us = 0U;
    g_reacquire_last_time_us = 0U;
}

static void position_loop_reset_predict_hold(void)
{
    g_hold_elapsed_ms = 0U;
}

void Position_Loop_Reset_Car_Velocity_Feedforward(void)
{
    position_loop_reset_brake_supervisor();
    g_diag.car_ff_raw_vx_cms = 0.0f;
    g_diag.car_ff_raw_vy_cms = 0.0f;
    g_diag.car_ff_filtered_vx_cms = 0.0f;
    g_diag.car_ff_filtered_vy_cms = 0.0f;
    g_diag.car_ff_raw_ax_cms2 = 0.0f;
    g_diag.car_ff_raw_ay_cms2 = 0.0f;
    g_diag.car_ff_applied_ax_cms2 = 0.0f;
    g_diag.car_ff_applied_ay_cms2 = 0.0f;
    g_diag.car_ff_valid = 0U;
    g_diag.car_accel_ff_valid = 0U;
}

void Position_Loop_Reset_Trajectory(uint8 reason)
{
    if(reason > POSITION_LOOP_RESET_YAW_RESET)
    {
        reason = POSITION_LOOP_RESET_NONE;
    }
    if(g_target_initialized)
    {
        g_trajectory_reset_count++;
    }
    g_target_vx_earth_cms = 0.0f;
    g_target_vy_earth_cms = 0.0f;
    g_target_ax_earth_cms2 = 0.0f;
    g_target_ay_earth_cms2 = 0.0f;
    g_target_initialized = 0U;
    g_trajectory_reset_reason = reason;
    position_loop_reset_brake_supervisor();
    g_diag.vx_cmd_cms = 0.0f;
    g_diag.vy_cmd_cms = 0.0f;
    g_diag.target_accel_x_cms2 = 0.0f;
    g_diag.target_accel_y_cms2 = 0.0f;
    g_diag.relative_closing_speed_cms = 0.0f;
    g_diag.filtered_closing_speed_cms = 0.0f;
    g_diag.relative_velocity_x_cms = 0.0f;
    g_diag.relative_velocity_y_cms = 0.0f;
    g_diag.relative_speed_cms = 0.0f;
    g_diag.proportional_speed_cms = 0.0f;
    g_diag.sqrt_speed_cms = 0.0f;
    g_diag.brake_delay_distance_cm = 0.0f;
    g_diag.brake_allowed_speed_cms = 0.0f;
    g_diag.brake_diag_flags = 0U;
    g_diag.sqrt_controller_active = 0U;
    g_diag.limit_flags &= (uint8)~(POSITION_LOOP_STATUS_TARGET_READY |
                                  POSITION_LOOP_STATUS_TIMING_VALID |
                                  POSITION_LOOP_STATUS_BRAKE_LIMITED);
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
    g_position_covariance_x_cm2 = POSITION_LOOP_VISION_POSITION_VARIANCE_CM2;
    g_position_covariance_y_cm2 = POSITION_LOOP_VISION_POSITION_VARIANCE_CM2;
    g_position_estimator_initialized = 0U;
    /* 使用与观测快照一致的前/后摄帧序号，不混用全局视觉姿态序号。 */
    g_last_pose_seq = 0U;
    g_last_rear_pose_seq = 0U;
    g_vision_source = POSITION_LOOP_VISION_SOURCE_FRONT;
    g_observation_state = POSITION_LOOP_OBSERVATION_INVALID;
    g_vision_innovation_cm = 0.0f;
    g_vision_gate_cm = 0.0f;
    g_vision_observation_x_earth_cm = 0.0f;
    g_vision_observation_y_earth_cm = 0.0f;
    g_vision_observation_age_ms = 0U;
    g_vision_observation_accepted = 0U;
    g_vision_observation_source = POSITION_LOOP_VISION_SOURCE_FRONT;
    g_frozen_hold_active = 0U;
    g_diag = empty_diag;
    g_diag.enabled = g_enabled;
    g_diag.car_velocity_ff_gain_x = g_car_velocity_ff_gain_x;
    g_diag.car_velocity_ff_gain_y = g_car_velocity_ff_gain_y;
    g_diag.car_accel_ff_gain_x = g_car_accel_ff_gain_x;
    g_diag.car_accel_ff_gain_y = g_car_accel_ff_gain_y;
    g_diag.trajectory_reset_reason = g_trajectory_reset_reason;
    g_diag.trajectory_reset_count = g_trajectory_reset_count;
}

static uint8 position_loop_get_front_frame(CrossCoreCameraFrontFrame_t *frame)
{
    if((frame == NULL) ||
       !CrossCoreCamera_ControlCoreGetFrontFrame(frame) ||
       (frame->frame_seq == 0U) ||
       !frame->yaw_valid ||
       !frame->car_valid ||
       !frame->car_raw_valid ||
       !isfinite(frame->yaw_deg) ||
       !frame->car_pose.valid ||
       (frame->car_pose.detected_point_count < 3U))
    {
        return 0U;
    }

    return 1U;
}

static uint8 position_loop_get_rear_follow_point(float *x_body,
                                                 float *y_body,
                                                 float *capture_yaw_deg,
                                                 uint32 *capture_time_us,
                                                 uint32 *frame_seq)
{
    DualCameraFusionSnapshot_t snapshot = {0};

    if((x_body == NULL) || (y_body == NULL) ||
       (capture_yaw_deg == NULL) || (capture_time_us == NULL) ||
       (frame_seq == NULL) ||
       !DualCameraFusion_GetSnapshot(&snapshot) ||
       !snapshot.rear_car_valid ||
       !snapshot.rear_car_raw_valid ||
       !snapshot.rear_yaw_valid ||
       (snapshot.rear_frame_seq == 0U) ||
       (snapshot.rear_capture_time_us == 0U) ||
       !isfinite(snapshot.rear_observation_x_cm) ||
       !isfinite(snapshot.rear_observation_y_cm) ||
       !isfinite(snapshot.rear_yaw_deg))
    {
        return 0U;
    }

    *x_body = snapshot.rear_observation_x_cm;
    *y_body = snapshot.rear_observation_y_cm;
    *capture_yaw_deg = snapshot.rear_yaw_deg;
    *capture_time_us = snapshot.rear_capture_time_us;
    *frame_seq = snapshot.rear_frame_seq;
    return 1U;
}

static float position_loop_visual_gate(float base_cm,
                                       float maximum_cm,
                                       float elapsed_s)
{
    float gate;

    gate = base_cm + POSITION_LOOP_VISION_VELOCITY_SIGMA_CMS * elapsed_s +
           0.5f * POSITION_LOOP_VISION_ACCEL_SIGMA_CMS2 * elapsed_s * elapsed_s;
    return position_loop_limit_float(gate, base_cm, maximum_cm);
}

static void position_loop_predict_estimator(float relative_vx_earth,
                                            float relative_vy_earth,
                                            float dt_s)
{
    uint16 elapsed_ms = (uint16)(dt_s * 1000.0f + 0.5f);
    float process_variance;

    if(!g_position_estimator_initialized)
    {
        return;
    }
    g_last_x_earth_cm += relative_vx_earth * dt_s;
    g_last_y_earth_cm += relative_vy_earth * dt_s;
    process_variance = POSITION_LOOP_VISION_VELOCITY_SIGMA_CMS *
                       POSITION_LOOP_VISION_VELOCITY_SIGMA_CMS *
                       POSITION_LOOP_VISION_PROCESS_REFERENCE_S * dt_s;
    g_position_covariance_x_cm2 += process_variance;
    g_position_covariance_y_cm2 += process_variance;

    if(!g_last_pose_valid)
    {
        return;
    }
    if(elapsed_ms == 0U) elapsed_ms = 1U;
    if(((uint32)g_hold_elapsed_ms + elapsed_ms) > POSITION_LOOP_VISION_HOLD_MS)
    {
        g_last_pose_valid = 0U;
        g_hold_elapsed_ms = POSITION_LOOP_VISION_HOLD_MS;
        g_observation_state = (g_reacquire_count > 0U) ?
            POSITION_LOOP_OBSERVATION_REACQUIRING :
            POSITION_LOOP_OBSERVATION_INVALID;
        return;
    }

    g_hold_elapsed_ms += elapsed_ms;
    if(g_observation_state != POSITION_LOOP_OBSERVATION_REACQUIRING)
    {
        g_observation_state = POSITION_LOOP_OBSERVATION_HOLD;
    }
}

static uint8 position_loop_update_visual_estimator(float observation_x_earth,
                                                    float observation_y_earth,
                                                    uint32 capture_time_us,
                                                    uint32 now_us,
                                                    float relative_vx_earth,
                                                    float relative_vy_earth,
                                                    uint8 source)
{
    uint32 age_us;
    uint32 candidate_span_us;
    float age_s;
    float dx;
    float dy;
    float gain_x;
    float gain_y;

    g_vision_observation_accepted = 0U;
    g_vision_observation_source = source;
    if((capture_time_us == 0U) ||
       !isfinite(observation_x_earth) || !isfinite(observation_y_earth))
    {
        return g_last_pose_valid;
    }
    age_us = (uint32)(now_us - capture_time_us);
    if(age_us > (uint32)POSITION_LOOP_VISION_MAX_AGE_MS * 1000U)
    {
        g_vision_observation_age_ms = (age_us / 1000U > 0xFFFFU) ?
                                      0xFFFFU : (uint16)(age_us / 1000U);
        return g_last_pose_valid;
    }

    age_s = (float)age_us * 0.000001f;
    g_vision_observation_age_ms = (uint16)(age_us / 1000U);
    observation_x_earth += relative_vx_earth * age_s;
    observation_y_earth += relative_vy_earth * age_s;
    g_vision_observation_x_earth_cm = observation_x_earth;
    g_vision_observation_y_earth_cm = observation_y_earth;

    dx = observation_x_earth - g_last_x_earth_cm;
    dy = observation_y_earth - g_last_y_earth_cm;
    g_vision_innovation_cm = g_position_estimator_initialized ?
        sqrtf(dx * dx + dy * dy) : 0.0f;
    g_vision_gate_cm = position_loop_visual_gate(
        POSITION_LOOP_VISION_LIVE_GATE_BASE_CM,
        POSITION_LOOP_VISION_LIVE_GATE_MAX_CM,
        age_s);
    if(g_position_estimator_initialized && g_last_pose_valid &&
       (g_vision_innovation_cm <= g_vision_gate_cm))
    {
        gain_x = g_position_covariance_x_cm2 /
                 (g_position_covariance_x_cm2 +
                  POSITION_LOOP_VISION_POSITION_VARIANCE_CM2);
        gain_y = g_position_covariance_y_cm2 /
                 (g_position_covariance_y_cm2 +
                  POSITION_LOOP_VISION_POSITION_VARIANCE_CM2);
        g_last_x_earth_cm += gain_x * dx;
        g_last_y_earth_cm += gain_y * dy;
        g_position_covariance_x_cm2 *= 1.0f - gain_x;
        g_position_covariance_y_cm2 *= 1.0f - gain_y;
        g_vision_observation_accepted = 1U;
        g_vision_source = source;
        g_observation_state = POSITION_LOOP_OBSERVATION_LIVE;
        position_loop_reset_predict_hold();
        position_loop_reset_reacquisition();
        return 1U;
    }

    candidate_span_us = (g_reacquire_count == 0U) ? 0U :
                        (uint32)(now_us - g_reacquire_first_time_us);
    if((g_reacquire_count == 0U) || (g_reacquire_source != source) ||
       (candidate_span_us >
        (uint32)POSITION_LOOP_VISION_REACQUIRE_SPAN_MS * 1000U))
    {
        g_reacquire_count = 1U;
        g_reacquire_source = source;
        g_reacquire_first_time_us = now_us;
        g_vision_gate_cm = POSITION_LOOP_VISION_REACQUIRE_GATE_BASE_CM;
    }
    else
    {
        float candidate_dt_s = (float)(uint32)(now_us - g_reacquire_last_time_us) *
                               0.000001f;
        float span_s = (float)candidate_span_us * 0.000001f;
        float predicted_x = g_reacquire_x_earth_cm +
                            relative_vx_earth * candidate_dt_s;
        float predicted_y = g_reacquire_y_earth_cm +
                            relative_vy_earth * candidate_dt_s;
        float candidate_dx = observation_x_earth - predicted_x;
        float candidate_dy = observation_y_earth - predicted_y;
        float candidate_residual = sqrtf(candidate_dx * candidate_dx +
                                         candidate_dy * candidate_dy);

        g_vision_gate_cm = position_loop_visual_gate(
            POSITION_LOOP_VISION_REACQUIRE_GATE_BASE_CM,
            POSITION_LOOP_VISION_REACQUIRE_GATE_MAX_CM,
            span_s);
        if(candidate_residual <= g_vision_gate_cm)
        {
            if(g_reacquire_count < 0xFFU) g_reacquire_count++;
        }
        else
        {
            g_reacquire_count = 1U;
            g_reacquire_first_time_us = now_us;
            g_vision_gate_cm = POSITION_LOOP_VISION_REACQUIRE_GATE_BASE_CM;
        }
    }
    g_reacquire_x_earth_cm = observation_x_earth;
    g_reacquire_y_earth_cm = observation_y_earth;
    g_reacquire_last_time_us = now_us;
    g_observation_state = POSITION_LOOP_OBSERVATION_REACQUIRING;

    if(g_reacquire_count >= POSITION_LOOP_VISION_REACQUIRE_FRAMES)
    {
        g_last_x_earth_cm = observation_x_earth;
        g_last_y_earth_cm = observation_y_earth;
        g_position_covariance_x_cm2 = POSITION_LOOP_VISION_POSITION_VARIANCE_CM2;
        g_position_covariance_y_cm2 = POSITION_LOOP_VISION_POSITION_VARIANCE_CM2;
        g_position_estimator_initialized = 1U;
        g_last_pose_valid = 1U;
        g_vision_observation_accepted = 1U;
        g_vision_source = source;
        g_observation_state = POSITION_LOOP_OBSERVATION_LIVE;
        position_loop_reset_predict_hold();
        position_loop_reset_reacquisition();
    }
    return g_last_pose_valid;
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
    if(!isfinite(gain) || (gain < 0.0f) ||
       (gain > POSITION_LOOP_CAR_VELOCITY_FF_GAIN_MAX))
    {
        return 0U;
    }
    if(axis == 0U) g_car_velocity_ff_gain_x = gain;
    else if(axis == 1U) g_car_velocity_ff_gain_y = gain;
    else return 0U;
    return 1U;
}

void Position_Loop_Get_Car_Velocity_Feedforward_Gain(float *gain_x, float *gain_y)
{
    if(gain_x != NULL) *gain_x = g_car_velocity_ff_gain_x;
    if(gain_y != NULL) *gain_y = g_car_velocity_ff_gain_y;
}

uint8 Position_Loop_Set_Car_Acceleration_Feedforward_Gain(uint8 axis, float gain)
{
    if(!isfinite(gain) || (gain < 0.0f) ||
       (gain > POSITION_LOOP_CAR_ACCEL_FF_GAIN_MAX))
    {
        return 0U;
    }
    if(axis == 0U) g_car_accel_ff_gain_x = gain;
    else if(axis == 1U) g_car_accel_ff_gain_y = gain;
    else return 0U;
    return 1U;
}

void Position_Loop_Get_Car_Acceleration_Feedforward_Gain(float *gain_x, float *gain_y)
{
    if(gain_x != NULL) *gain_x = g_car_accel_ff_gain_x;
    if(gain_y != NULL) *gain_y = g_car_accel_ff_gain_y;
}

uint8 Position_Loop_Set_Trajectory_Config(const PositionLoopTrajectoryConfig_t *config)
{
    uint32 irq_state;

    if((config == NULL) || !isfinite(config->position_kp) ||
       !isfinite(config->correction_accel_max_cms2) ||
       !isfinite(config->correction_speed_max_cms) ||
       !isfinite(config->target_speed_max_cms) ||
       (config->position_kp < 0.05f) || (config->position_kp > 3.0f) ||
       (config->correction_accel_max_cms2 < 5.0f) ||
       (config->correction_accel_max_cms2 > 400.0f) ||
       (config->correction_speed_max_cms < 10.0f) ||
       (config->correction_speed_max_cms > 200.0f) ||
       (config->target_speed_max_cms < 20.0f) ||
       (config->target_speed_max_cms > 300.0f))
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
       (config->effective_accel_cms2 < 10.0f) ||
       (config->effective_accel_cms2 > 400.0f) ||
       (config->effective_jerk_cms3 < 20.0f) ||
       (config->effective_jerk_cms3 > 1000.0f) ||
       (config->control_delay_ms < 0.0f) || (config->control_delay_ms > 500.0f) ||
       (config->safety_buffer_cm < 0.0f) || (config->safety_buffer_cm > 100.0f) ||
       (config->boost_margin_cm < 0.0f) || (config->boost_margin_cm > 100.0f))
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
                                          float measured_vx_earth_cms,
                                          float measured_vy_earth_cms,
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
    PositionLoopBrakeConfig_t brake_config;
    float car_vx_earth = 0.0f;
    float car_vy_earth = 0.0f;
    float car_actual_vx_earth = 0.0f;
    float car_actual_vy_earth = 0.0f;
    float car_actual_ax_earth = 0.0f;
    float car_actual_ay_earth = 0.0f;
    float car_ax_earth = 0.0f;
    float car_ay_earth = 0.0f;
    float relative_target_vx_earth = 0.0f;
    float relative_target_vy_earth = 0.0f;
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
    float brake_delay_distance = 0.0f;
    float brake_allowed_speed = 0.0f;
    float envelope_speed = 0.0f;
    float proportional_speed = 0.0f;
    uint8 position_valid = 0U;
    uint8 car_velocity_valid = 0U;
    uint8 car_acceleration_valid = 0U;
    uint8 flags = 0U;
    uint8 brake_diag_flags = 0U;
    uint8 sqrt_controller_active = 0U;
    uint8 new_visual_pose;
    uint8 rear_frame_valid = 0U;
    uint8 new_rear_pose;
    CrossCoreCameraFrontFrame_t front_frame = {0};
    uint8 front_frame_valid = 0U;
    uint32 now_us = 0U;

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
       !isfinite(measured_vx_earth_cms) || !isfinite(measured_vy_earth_cms))
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
        position_loop_reset_brake_supervisor();
        g_target_ax_earth_cms2 = 0.0f;
        g_target_ay_earth_cms2 = 0.0f;
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
    brake_config = g_brake_config;
    if(!g_frozen_hold_active && (feedforward != NULL) &&
       feedforward->velocity_valid &&
       isfinite(feedforward->vx_cms) && isfinite(feedforward->vy_cms))
    {
        if(feedforward->earth_frame)
        {
            car_actual_vx_earth = feedforward->vx_cms;
            car_actual_vy_earth = feedforward->vy_cms;
            car_vx_earth = feedforward->vx_cms * g_car_velocity_ff_gain_x;
            car_vy_earth = feedforward->vy_cms * g_car_velocity_ff_gain_y;
        }
        else
        {
            float gained_body_x = feedforward->vx_cms *
                                  g_car_velocity_ff_gain_x;
            float gained_body_y = feedforward->vy_cms *
                                  g_car_velocity_ff_gain_y;

            position_loop_body_to_earth(feedforward->vx_cms,
                                        feedforward->vy_cms,
                                        yaw_deg,
                                        &car_actual_vx_earth,
                                        &car_actual_vy_earth);
            position_loop_body_to_earth(gained_body_x, gained_body_y, yaw_deg,
                                        &car_vx_earth, &car_vy_earth);
        }
        car_velocity_valid = 1U;
        flags |= POSITION_LOOP_STATUS_CAR_FF_VALID;
    }
    if(!g_frozen_hold_active && (feedforward != NULL) &&
       feedforward->acceleration_valid &&
       isfinite(feedforward->ax_cms2) &&
       isfinite(feedforward->ay_cms2))
    {
        if(feedforward->earth_frame)
        {
            car_actual_ax_earth = feedforward->ax_cms2;
            car_actual_ay_earth = feedforward->ay_cms2;
            car_ax_earth = feedforward->ax_cms2 * g_car_accel_ff_gain_x;
            car_ay_earth = feedforward->ay_cms2 * g_car_accel_ff_gain_y;
        }
        else
        {
            float gained_body_x = feedforward->ax_cms2 *
                                  g_car_accel_ff_gain_x;
            float gained_body_y = feedforward->ay_cms2 *
                                  g_car_accel_ff_gain_y;

            position_loop_body_to_earth(feedforward->ax_cms2,
                                        feedforward->ay_cms2,
                                        yaw_deg,
                                        &car_actual_ax_earth,
                                        &car_actual_ay_earth);
            position_loop_body_to_earth(gained_body_x, gained_body_y, yaw_deg,
                                        &car_ax_earth, &car_ay_earth);
        }
        car_acceleration_valid = 1U;
    }

    relative_target_vx_earth = (car_velocity_valid ? car_actual_vx_earth : 0.0f) -
                               measured_vx_earth_cms;
    relative_target_vy_earth = (car_velocity_valid ? car_actual_vy_earth : 0.0f) -
                               measured_vy_earth_cms;
    now_us = CameraBoardLink_TimeNowUs();
    if(g_frozen_hold_active)
    {
        position_valid = position_loop_update_frozen_hold(measured_vx_earth_cms,
                                                          measured_vy_earth_cms,
                                                          dt_s,
                                                          &position_x,
                                                          &position_y);
    }
    else
    {
        float rear_x_body;
        float rear_y_body;
        float rear_capture_yaw_deg = 0.0f;
        uint32 rear_capture_time_us = 0U;
        uint32 rear_frame_seq = 0U;
        uint8 process_front = 0U;
        uint8 process_rear = 0U;

        position_loop_predict_estimator(relative_target_vx_earth,
                                        relative_target_vy_earth,
                                        dt_s);
        front_frame_valid = position_loop_get_front_frame(&front_frame);
        new_visual_pose = (uint8)(front_frame_valid &&
                                  (front_frame.frame_seq != g_last_pose_seq));
        rear_frame_valid = position_loop_get_rear_follow_point(
            &rear_x_body, &rear_y_body,
            &rear_capture_yaw_deg, &rear_capture_time_us,
            &rear_frame_seq);
        new_rear_pose = (uint8)(rear_frame_valid &&
                                (rear_frame_seq != g_last_rear_pose_seq));

        if(g_vision_source == POSITION_LOOP_VISION_SOURCE_FRONT)
        {
            process_front = new_visual_pose;
            process_rear = (uint8)(!process_front && new_rear_pose);
        }
        else
        {
            process_rear = new_rear_pose;
            process_front = (uint8)(!process_rear && new_visual_pose);
        }

        if(process_front)
        {
            float follow_x_body;
            float follow_y_body;
            float observation_x_earth;
            float observation_y_earth;

            g_last_pose_seq = front_frame.frame_seq;
            if(VehicleMarkerLocalization_GetFollowPoint(
                   &front_frame.car_pose,
                   &follow_x_body, &follow_y_body,
                   NULL, NULL))
            {
                /* 位置与Yaw必须来自同一采集帧，再进入惯性系估计器。 */
                position_loop_body_to_earth(
                    follow_x_body, follow_y_body, front_frame.yaw_deg,
                    &observation_x_earth, &observation_y_earth);
                (void)position_loop_update_visual_estimator(
                    observation_x_earth, observation_y_earth,
                    front_frame.capture_time_us, now_us,
                    relative_target_vx_earth, relative_target_vy_earth,
                    POSITION_LOOP_VISION_SOURCE_FRONT);
            }
        }
        else if(process_rear)
        {
            float observation_x_earth;
            float observation_y_earth;

            g_last_rear_pose_seq = rear_frame_seq;
            position_loop_body_to_earth(
                rear_x_body, rear_y_body, rear_capture_yaw_deg,
                &observation_x_earth, &observation_y_earth);
            (void)position_loop_update_visual_estimator(
                observation_x_earth, observation_y_earth,
                rear_capture_time_us, now_us,
                relative_target_vx_earth, relative_target_vy_earth,
                POSITION_LOOP_VISION_SOURCE_REAR);
        }

        position_valid = g_last_pose_valid;
        if(g_position_estimator_initialized)
        {
            position_x = g_last_x_earth_cm;
            position_y = g_last_y_earth_cm;
        }
    }

    if(position_valid)
    {
        /* 真实位置误差进入P/Sqrt；停车包络只约束朝向小车的目标速度。 */
        distance = sqrtf(position_x * position_x + position_y * position_y);
        relative_velocity_x = measured_vx_earth_cms -
                              (car_velocity_valid ? car_actual_vx_earth : 0.0f);
        relative_velocity_y = measured_vy_earth_cms -
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
                &brake_config,
                &brake_delay_distance,
                &brake_allowed_speed);

            if(g_filtered_closing_speed_cms > POSITION_LOOP_EPSILON)
            {
                brake_diag_flags |= POSITION_LOOP_BRAKE_DIAG_CLOSING_ACTIVE;
            }
            correction_speed = position_loop_correction_speed(
                distance,
                &config,
                &proportional_speed,
                &envelope_speed,
                &sqrt_controller_active);
            /* A/J只限制位置纠偏径向速度，小车速度前馈保持完整叠加。 */
            if(correction_speed > brake_allowed_speed)
            {
                correction_speed = brake_allowed_speed;
                flags |= POSITION_LOOP_STATUS_BRAKE_LIMITED;
            }
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
        brake_allowed_speed = 0.0f;
    }

    g_target_vx_earth_cms = position_valid ? desired_vx : 0.0f;
    g_target_vy_earth_cms = position_valid ? desired_vy : 0.0f;
    g_target_ax_earth_cms2 = (position_valid && car_acceleration_valid) ?
                             car_ax_earth : 0.0f;
    g_target_ay_earth_cms2 = (position_valid && car_acceleration_valid) ?
                             car_ay_earth : 0.0f;
    if(position_valid && car_acceleration_valid)
    {
        flags |= POSITION_LOOP_STATUS_ACCEL_FF_VALID;
    }
    g_target_initialized = 1U;
    if(g_trajectory_reset_reason == POSITION_LOOP_RESET_VELOCITY_INVALID)
    {
        g_trajectory_reset_reason = POSITION_LOOP_RESET_VELOCITY_RECOVERED;
    }
    flags |= POSITION_LOOP_STATUS_TARGET_READY;

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
    g_diag.car_ff_raw_ax_cms2 = car_actual_ax_earth;
    g_diag.car_ff_raw_ay_cms2 = car_actual_ay_earth;
    g_diag.car_ff_applied_ax_cms2 = g_target_ax_earth_cms2;
    g_diag.car_ff_applied_ay_cms2 = g_target_ay_earth_cms2;
    g_diag.car_velocity_ff_gain_x = g_car_velocity_ff_gain_x;
    g_diag.car_velocity_ff_gain_y = g_car_velocity_ff_gain_y;
    g_diag.car_accel_ff_gain_x = g_car_accel_ff_gain_x;
    g_diag.car_accel_ff_gain_y = g_car_accel_ff_gain_y;
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
    g_diag.relative_velocity_x_cms = relative_velocity_x;
    g_diag.relative_velocity_y_cms = relative_velocity_y;
    g_diag.relative_speed_cms = relative_speed;
    g_diag.brake_delay_distance_cm = brake_delay_distance;
    g_diag.brake_allowed_speed_cms = brake_allowed_speed;
    g_diag.trajectory_dt_s = dt_s;
    g_diag.vision_innovation_cm = g_vision_innovation_cm;
    g_diag.vision_gate_cm = g_vision_gate_cm;
    g_diag.vision_observation_x_cm = g_vision_observation_x_earth_cm;
    g_diag.vision_observation_y_cm = g_vision_observation_y_earth_cm;
    g_diag.vision_covariance_x_cm2 = g_position_covariance_x_cm2;
    g_diag.vision_covariance_y_cm2 = g_position_covariance_y_cm2;
    g_diag.vision_hold_age_ms = g_hold_elapsed_ms;
    g_diag.vision_observation_age_ms = g_vision_observation_age_ms;
    g_diag.vision_observation_state = g_observation_state;
    g_diag.vision_reacquire_count = g_reacquire_count;
    g_diag.vision_observation_accepted = g_vision_observation_accepted;
    g_diag.vision_observation_source = g_vision_observation_source;
    g_diag.vision_estimator_initialized = g_position_estimator_initialized;
    g_diag.car_ff_valid = car_velocity_valid;
    g_diag.car_accel_ff_valid = (uint8)(position_valid &&
                                        car_acceleration_valid);
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
