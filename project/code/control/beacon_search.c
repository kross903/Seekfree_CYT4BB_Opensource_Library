#include "common/project_headfile.h"

#include <math.h>
#include <string.h>

#define BEACON_SEARCH_LOSS_DELAY_MS             (300U)   /* 视觉丢失后等待进入自搜索的延时 */
#define BEACON_SEARCH_VERIFY_TIMEOUT_MS         (300U)   /* 搜索中看到信标后，连续确认的最长等待时间 */
#define BEACON_SEARCH_CONFIRM_FRAMES              (3U)   /* 判定可信信标所需的连续确认帧数 */
#define BEACON_SEARCH_MIN_CONFIDENCE              (96U)  /* 信标可信判断的最低识别置信度 */
#define BEACON_SEARCH_MIN_SIGMA_CM                 (5U)  /* 信标位置sigma下限，过滤异常过小估计 */
#define BEACON_SEARCH_MAX_SIGMA_CM                (15U)  /* 信标位置sigma上限，过滤定位发散目标 */
#define BEACON_SEARCH_ENDPOINT_HOLD_MS           (200U)  /* 到达搜索端点后的短暂停留时间 */
#define BEACON_SEARCH_ENDPOINT_TOLERANCE_DEG       (2.0f)/* 判定到达目标Yaw端点的角度容差 */
#define BEACON_SEARCH_PREDICT_HOLD_MS           (1000U)  /* 预测方向到位后的等待时间 */
#define BEACON_SEARCH_MAX_YAW_STEP_DEG           (10.0f) /* 单周期Yaw跳变过滤阈值，避免IMU突变误计 */
#define BEACON_SEARCH_DEG_TO_RAD                 (0.01745329252f)

typedef enum
{
    BEACON_SEARCH_CONTROL_NONE = 0,
    BEACON_SEARCH_CONTROL_SPIN,
    BEACON_SEARCH_CONTROL_HOLD
} BeaconSearchControl_t;

typedef enum
{
    BEACON_SEARCH_SWEEP_ALIGN_PREDICTED = 0,
    BEACON_SEARCH_SWEEP_HOLD_PREDICTED,
    BEACON_SEARCH_SWEEP_TO_FIRST,
    BEACON_SEARCH_SWEEP_HOLD_FIRST,
    BEACON_SEARCH_SWEEP_TO_SECOND,
    BEACON_SEARCH_SWEEP_HOLD_SECOND
} BeaconSearchSweepPhase_t;

static volatile BeaconSearchState_t s_state = BEACON_SEARCH_DISABLED;
static volatile BeaconSearchControl_t s_control = BEACON_SEARCH_CONTROL_NONE;
static volatile uint8 s_search_generation = 0U;
static volatile float s_current_yaw_deg = 0.0f;
static volatile float s_target_yaw_deg = 0.0f;
static volatile float s_turn_progress_deg = 0.0f;
static volatile float s_yaw_rate_dps = BEACON_SEARCH_YAW_RATE_DEFAULT_DPS;
static volatile float s_sweep_half_angle_deg =
    BEACON_SEARCH_SWEEP_HALF_ANGLE_DEFAULT_DEG;
static volatile BeaconSearchSweepPhase_t s_sweep_phase =
    BEACON_SEARCH_SWEEP_TO_FIRST;
static float s_endpoint_hold_elapsed_s = 0.0f;
static float s_predicted_hold_elapsed_s = 0.0f;
static volatile BeaconSearchDirectionMode_t s_direction_mode =
    BEACON_SEARCH_DIRECTION_DEFAULT;
static volatile uint8 s_direction_map_count = 0U;
static volatile uint8 s_direction_predicted_valid = 0U;
static volatile int8 s_direction_first_sign = 1;
static volatile float s_search_origin_yaw_deg = 0.0f;
static volatile float s_first_endpoint_yaw_deg = 90.0f;
static volatile float s_second_endpoint_yaw_deg = -90.0f;
static volatile float s_predicted_target_yaw_deg = 0.0f;
static volatile uint8 s_map_fallback_valid = 0U;
static int8 s_last_map_tie_first_sign = -1;

static uint8 s_was_armed = 0U;
static uint8 s_eligible = 0U;
static uint8 s_ever_seen = 0U;
static uint8 s_candidate_track_id = 0U;
static uint8 s_candidate_confirm_count = 0U;
static uint8 s_verify_from_failed = 0U;
static uint8 s_process_count_valid = 0U;
static uint32 s_last_process_count = 0U;
static uint16 s_loss_age_ms = 0U;
static uint16 s_search_elapsed_ms = 0U;
static uint16 s_verify_elapsed_ms = 0U;
static uint16 s_car_link_age_ms = 60000U;
static uint8 s_position_window[BEACON_SEARCH_POSITION_WINDOW_FRAMES] = {0};
static uint8 s_position_window_head = 0U;
static uint8 s_position_window_count = 0U;
static uint8 s_position_qualified_count = 0U;
static uint8 s_stability_process_count_valid = 0U;
static uint8 s_start_gate_ready = 0U;
static uint8 s_position_sample_valid = 0U;
static uint8 s_fused_velocity_valid = 0U;
static uint32 s_stability_last_process_count = 0U;
static uint16 s_speed_stable_ms = 0U;
static uint16 s_vision_frame_age_ms = 60000U;
static float s_position_distance_cm = 0.0f;
static float s_horizontal_speed_cms = 0.0f;
static BeaconSearchFailure_t s_failure_reason = BEACON_SEARCH_FAILURE_NONE;

static uint16 beacon_search_add_ms(uint16 value, uint16 elapsed_ms)
{
    if(value <= (uint16)(60000U - elapsed_ms))
    {
        return (uint16)(value + elapsed_ms);
    }
    return 60000U;
}
static float beacon_search_wrap_deg(float angle_deg)
{
    while(angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while(angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/* 相机地面视场长轴沿机体系Y，+Y与-Y是同一条无向长轴。
 * 将任意目标方向折叠为[-90, 90]内的最短Yaw旋转量。 */
static uint8 beacon_search_get_long_axis_delta(float x, float y,
                                                float *delta_deg)
{
    float bearing_deg;
    float delta;

    if((NULL == delta_deg) || !isfinite(x) || !isfinite(y) ||
       ((x * x + y * y) < 1.0f))
    {
        return 0U;
    }

    bearing_deg = atan2f(y, x) / BEACON_SEARCH_DEG_TO_RAD;
    delta = beacon_search_wrap_deg(bearing_deg - 90.0f);
    if(delta > 90.0f)
    {
        delta -= 180.0f;
    }
    else if(delta < -90.0f)
    {
        delta += 180.0f;
    }
    *delta_deg = delta;
    return 1U;
}

/* 所有有效地图信标等权。按最短对准角的正负分组，数量多的一侧先扫；
 * 数量相同时选择组内累计转角较小的一侧，仍相同则固定正转优先。 */
static int8 beacon_search_choose_map_first_sign(
    const BeaconSearchDirectionInput_t *input)
{
    uint8 positive_count = 0U;
    uint8 negative_count = 0U;
    float positive_cost = 0.0f;
    float negative_cost = 0.0f;
    uint8 i;

    if((NULL == input) || !input->map_valid)
    {
        return 1;
    }

    for(i = 0U; (i < input->map_count) &&
                    (i < CORE_LINK_BEACON_MAP_COUNT); ++i)
    {
        float delta_deg;

        if((0U == input->beacon[i].id) ||
           (0U == input->beacon[i].state) ||
           !beacon_search_get_long_axis_delta(
               (float)input->beacon[i].x_mm,
               (float)input->beacon[i].y_mm, &delta_deg))
        {
            continue;
        }

        if(delta_deg > 0.0f)
        {
            positive_count++;
            positive_cost += delta_deg;
        }
        else if(delta_deg < 0.0f)
        {
            negative_count++;
            negative_cost -= delta_deg;
        }
        else
        {
            /* 已经落在长轴方向的信标对两个旋转方向等权。 */
            positive_count++;
            negative_count++;
        }
    }

    if(positive_count > negative_count)
    {
        return 1;
    }
    if(negative_count > positive_count)
    {
        return -1;
    }
    if(negative_cost < positive_cost)
    {
        return -1;
    }
    if(positive_cost < negative_cost)
    {
        return 1;
    }

    s_last_map_tie_first_sign =
        (s_last_map_tie_first_sign >= 0) ? -1 : 1;
    return s_last_map_tie_first_sign;
}

static uint8 beacon_search_get_predicted_delta(
    const BeaconSearchDirectionInput_t *input,
    float current_yaw_deg, float *delta_deg)
{
    uint8 i;

    if((NULL == input) || (NULL == delta_deg) ||
       !input->map_valid || !input->predicted_valid ||
       (0U == input->predicted_id) || !isfinite(current_yaw_deg))
    {
        return 0U;
    }

    /* 首选同一0x13地图快照中的预测ID位置。 */
    for(i = 0U; (i < input->map_count) &&
                    (i < CORE_LINK_BEACON_MAP_COUNT); ++i)
    {
        if((input->beacon[i].id == input->predicted_id) &&
           (0U != input->beacon[i].state) &&
           beacon_search_get_long_axis_delta(
               (float)input->beacon[i].x_mm,
               (float)input->beacon[i].y_mm, delta_deg))
        {
            return 1U;
        }
    }

    /* 预测ID未进入5个地图槽位时，使用0x15车体系预测向量兜底。
     * 若0x13同时给出了小车位置，再补偿无人机到小车的水平偏移。 */
    {
        float yaw_delta_rad = beacon_search_wrap_deg(
            (float)input->car_absolute_yaw_ddeg * 0.1f -
            current_yaw_deg) * BEACON_SEARCH_DEG_TO_RAD;
        float cos_yaw = cosf(yaw_delta_rad);
        float sin_yaw = sinf(yaw_delta_rad);
        float forward_mm = (float)input->predicted_beacon_forward_mm;
        float left_mm = (float)input->predicted_beacon_left_mm;
        float x_uav = cos_yaw * forward_mm - sin_yaw * left_mm;
        float y_uav = sin_yaw * forward_mm + cos_yaw * left_mm;

        if(input->car_position_valid)
        {
            x_uav += (float)input->car_x_mm;
            y_uav += (float)input->car_y_mm;
        }
        return beacon_search_get_long_axis_delta(x_uav, y_uav, delta_deg);
    }
}

static uint8 beacon_search_yaw_at_endpoint(float current_yaw_deg,
                                           float endpoint_yaw_deg)
{
    return (uint8)(fabsf(beacon_search_wrap_deg(endpoint_yaw_deg -
                                                current_yaw_deg)) <=
                   BEACON_SEARCH_ENDPOINT_TOLERANCE_DEG);
}

static uint8 beacon_search_step_target(float dt_s,
                                       float endpoint_yaw_deg,
                                       volatile float *target_yaw_deg)
{
    float error_deg;
    float max_step_deg;

    if((NULL == target_yaw_deg) || !isfinite(dt_s) || (dt_s <= 0.0f))
    {
        return 0U;
    }

    error_deg = beacon_search_wrap_deg(endpoint_yaw_deg - *target_yaw_deg);
    max_step_deg = BeaconSearch_GetYawRateDps() * dt_s;
    if(fabsf(error_deg) <= max_step_deg)
    {
        *target_yaw_deg = endpoint_yaw_deg;
        return 1U;
    }

    *target_yaw_deg = beacon_search_wrap_deg(
        *target_yaw_deg + ((error_deg > 0.0f) ? max_step_deg : -max_step_deg));
    return 0U;
}

static uint8 beacon_search_hold_endpoint(float dt_s,
                                         float current_yaw_deg,
                                         float endpoint_yaw_deg,
                                         volatile float *target_yaw_deg)
{
    float hold_s = (float)BEACON_SEARCH_ENDPOINT_HOLD_MS * 0.001f;

    *target_yaw_deg = endpoint_yaw_deg;
    if(!beacon_search_yaw_at_endpoint(current_yaw_deg, endpoint_yaw_deg))
    {
        s_endpoint_hold_elapsed_s = 0.0f;
        return 0U;
    }

    s_endpoint_hold_elapsed_s += dt_s;
    if(s_endpoint_hold_elapsed_s >= hold_s)
    {
        s_endpoint_hold_elapsed_s = 0.0f;
        return 1U;
    }
    return 0U;
}

static void beacon_search_clear_candidate(void)
{
    s_candidate_track_id = 0U;
    s_candidate_confirm_count = 0U;
    s_verify_elapsed_ms = 0U;
    s_verify_from_failed = 0U;
}

static void beacon_search_clear_position_window(void)
{
    memset(s_position_window, 0, sizeof(s_position_window));
    s_position_window_head = 0U;
    s_position_window_count = 0U;
    s_position_qualified_count = 0U;
}

static void beacon_search_reset_stability(void)
{
    beacon_search_clear_position_window();
    s_stability_process_count_valid = 0U;
    s_start_gate_ready = 0U;
    s_position_sample_valid = 0U;
    s_fused_velocity_valid = 0U;
    s_stability_last_process_count = 0U;
    s_speed_stable_ms = 0U;
    s_vision_frame_age_ms = 60000U;
    s_position_distance_cm = 0.0f;
    s_horizontal_speed_cms = 0.0f;
}

static void beacon_search_push_position_result(uint8 qualified)
{
    if(s_position_window_count >= BEACON_SEARCH_POSITION_WINDOW_FRAMES)
    {
        s_position_qualified_count = (uint8)(
            s_position_qualified_count - s_position_window[s_position_window_head]);
    }
    else
    {
        s_position_window_count++;
    }

    s_position_window[s_position_window_head] = qualified ? 1U : 0U;
    s_position_qualified_count = (uint8)(s_position_qualified_count +
                                         s_position_window[s_position_window_head]);
    s_position_window_head++;
    if(s_position_window_head >= BEACON_SEARCH_POSITION_WINDOW_FRAMES)
    {
        s_position_window_head = 0U;
    }
}

static void beacon_search_update_stability(
    uint16 elapsed_ms,
    const BeaconSearchStabilityInput_t *input)
{
    uint8 new_vision_frame = 0U;
    uint8 position_qualified = 0U;
    uint8 velocity_stable = 0U;

    if(NULL == input)
    {
        beacon_search_reset_stability();
        return;
    }

    if(input->vision_process_count != 0U)
    {
        if(!s_stability_process_count_valid)
        {
            s_stability_process_count_valid = 1U;
            s_stability_last_process_count = input->vision_process_count;
            s_vision_frame_age_ms = 0U;
            new_vision_frame = 1U;
        }
        else if(input->vision_process_count != s_stability_last_process_count)
        {
            uint32 frame_delta = input->vision_process_count -
                                 s_stability_last_process_count;

            if((input->vision_process_count < s_stability_last_process_count) ||
               (frame_delta > 1U))
            {
                beacon_search_clear_position_window();
            }
            s_stability_last_process_count = input->vision_process_count;
            s_vision_frame_age_ms = 0U;
            new_vision_frame = 1U;
        }
        else
        {
            uint16 previous_age_ms = s_vision_frame_age_ms;

            s_vision_frame_age_ms = beacon_search_add_ms(s_vision_frame_age_ms,
                                                          elapsed_ms);
            if((previous_age_ms <= BEACON_SEARCH_VISION_FRAME_MAX_AGE_MS) &&
               (s_vision_frame_age_ms > BEACON_SEARCH_VISION_FRAME_MAX_AGE_MS))
            {
                beacon_search_clear_position_window();
                s_position_sample_valid = 0U;
            }
        }
    }
    else
    {
        beacon_search_clear_position_window();
        s_vision_frame_age_ms = 60000U;
        s_position_sample_valid = 0U;
    }

    if(new_vision_frame)
    {
        if(isfinite(input->position_error_x_cm) &&
           isfinite(input->position_error_y_cm))
        {
            s_position_distance_cm = sqrtf(
                input->position_error_x_cm * input->position_error_x_cm +
                input->position_error_y_cm * input->position_error_y_cm);
        }
        else
        {
            s_position_distance_cm = 0.0f;
        }

        s_position_sample_valid = (uint8)(input->position_valid &&
                                           isfinite(s_position_distance_cm));
        position_qualified = (uint8)(s_position_sample_valid &&
            (s_position_distance_cm <= BEACON_SEARCH_POSITION_GATE_CM));
        beacon_search_push_position_result(position_qualified);
    }

    if(input->fused_velocity_valid &&
       isfinite(input->fused_velocity_x_cms) &&
       isfinite(input->fused_velocity_y_cms))
    {
        s_horizontal_speed_cms = sqrtf(
            input->fused_velocity_x_cms * input->fused_velocity_x_cms +
            input->fused_velocity_y_cms * input->fused_velocity_y_cms);
        velocity_stable = (uint8)(isfinite(s_horizontal_speed_cms) &&
            (s_horizontal_speed_cms <= BEACON_SEARCH_SPEED_GATE_CMS));
        s_fused_velocity_valid = (uint8)isfinite(s_horizontal_speed_cms);
    }
    else
    {
        s_horizontal_speed_cms = 0.0f;
        s_fused_velocity_valid = 0U;
    }

    if(velocity_stable)
    {
        s_speed_stable_ms = beacon_search_add_ms(s_speed_stable_ms, elapsed_ms);
    }
    else
    {
        s_speed_stable_ms = 0U;
    }

    /* 自搜索启动只保留无人机水平速度稳定门控；位置窗口和视觉帧新鲜度
     * 不再阻塞“小车无目标后”的搜索启动。 */
    s_start_gate_ready = (uint8)(
        s_speed_stable_ms >= BEACON_SEARCH_SPEED_STABLE_MS);
}

static void beacon_search_reset_session(void)
{
    s_state = BEACON_SEARCH_DISABLED;
    s_control = BEACON_SEARCH_CONTROL_NONE;
    s_eligible = 0U;
    s_ever_seen = 0U;
    s_loss_age_ms = 0U;
    s_search_elapsed_ms = 0U;
    s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
    s_process_count_valid = 0U;
    s_last_process_count = 0U;
    s_direction_mode = BEACON_SEARCH_DIRECTION_DEFAULT;
    s_direction_map_count = 0U;
    s_direction_predicted_valid = 0U;
    s_direction_first_sign = 1;
    s_search_origin_yaw_deg = 0.0f;
    s_first_endpoint_yaw_deg = 90.0f;
    s_second_endpoint_yaw_deg = -90.0f;
    s_predicted_target_yaw_deg = 0.0f;
    s_map_fallback_valid = 0U;
    s_predicted_hold_elapsed_s = 0.0f;
    beacon_search_clear_candidate();
    beacon_search_reset_stability();
}

static void beacon_search_begin(
    float current_yaw_deg,
    const BeaconSearchDirectionInput_t *direction_input)
{
    float predicted_delta_deg = 0.0f;
    float half_angle_deg = BeaconSearch_GetSweepHalfAngleDeg();

    if(!isfinite(current_yaw_deg))
    {
        current_yaw_deg = 0.0f;
    }

    s_state = BEACON_SEARCH_SPINNING;
    s_control = BEACON_SEARCH_CONTROL_SPIN;
    s_search_elapsed_ms = 0U;
    s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
    s_search_generation++;
    s_search_origin_yaw_deg = beacon_search_wrap_deg(current_yaw_deg);
    s_direction_mode = BEACON_SEARCH_DIRECTION_DEFAULT;
    s_direction_map_count = 0U;
    s_direction_predicted_valid = 0U;
    s_direction_first_sign = 1;
    s_map_fallback_valid = 0U;
    s_predicted_hold_elapsed_s = 0.0f;

    if((NULL != direction_input) && direction_input->map_valid)
    {
        s_direction_map_count = direction_input->map_count;
        s_map_fallback_valid = 1U;
        s_direction_first_sign = beacon_search_choose_map_first_sign(
            direction_input);
        s_direction_mode = BEACON_SEARCH_DIRECTION_MAP;
    }

    if((NULL != direction_input) && direction_input->map_valid &&
       direction_input->predicted_valid &&
       beacon_search_get_predicted_delta(direction_input,
                                         current_yaw_deg,
                                         &predicted_delta_deg))
    {
        s_direction_mode = BEACON_SEARCH_DIRECTION_PREDICTED;
        s_direction_predicted_valid = 1U;
        s_predicted_target_yaw_deg = beacon_search_wrap_deg(
            s_search_origin_yaw_deg + predicted_delta_deg);
    }

    s_first_endpoint_yaw_deg = beacon_search_wrap_deg(
        s_search_origin_yaw_deg +
        ((s_direction_first_sign >= 0) ? half_angle_deg : -half_angle_deg));
    s_second_endpoint_yaw_deg = beacon_search_wrap_deg(
        s_search_origin_yaw_deg -
        ((s_direction_first_sign >= 0) ? half_angle_deg : -half_angle_deg));
    s_sweep_phase = (BEACON_SEARCH_DIRECTION_PREDICTED == s_direction_mode) ?
                    BEACON_SEARCH_SWEEP_ALIGN_PREDICTED :
                    BEACON_SEARCH_SWEEP_TO_FIRST;
    beacon_search_clear_candidate();
}

static void beacon_search_fail(BeaconSearchFailure_t reason)
{
    s_state = BEACON_SEARCH_FAILED;
    s_control = BEACON_SEARCH_CONTROL_HOLD;
    s_failure_reason = reason;
    beacon_search_clear_candidate();
}

/* 这里只处理正常信标观测，起飞后360度预扫由盲盒任务模块管理。 */
static uint8 beacon_search_observation_is_trusted(
    const BeaconObservation *observation)
{
    if(NULL == observation)
    {
        return 0U;
    }

    return (uint8)((observation->track_id != 0U) &&
                   (observation->flags & CORE_LINK_BEACON_OBSERVATION_FLAG_LIT) &&
                   (observation->detection_confidence >=
                    BEACON_SEARCH_MIN_CONFIDENCE) &&
                   (observation->position_sigma_cm >=
                    BEACON_SEARCH_MIN_SIGMA_CM) &&
                   (observation->position_sigma_cm <=
                    BEACON_SEARCH_MAX_SIGMA_CM));
}

static uint8 beacon_search_select_trusted_track(
    const core_link_beacon_observation_packet_struct *packet)
{
    uint8 first_track_id = 0U;
    uint8 i;

    if(NULL == packet)
    {
        return 0U;
    }

    for(i = 0U; i < packet->count; ++i)
    {
        if(!beacon_search_observation_is_trusted(&packet->observation[i]))
        {
            continue;
        }
        if(0U == first_track_id)
        {
            first_track_id = packet->observation[i].track_id;
        }
        if(packet->observation[i].track_id == s_candidate_track_id)
        {
            return s_candidate_track_id;
        }
    }

    return first_track_id;
}

static uint8 beacon_search_confirm_candidate(uint8 track_id)
{
    if(0U == track_id)
    {
        s_candidate_track_id = 0U;
        s_candidate_confirm_count = 0U;
        return 0U;
    }

    if(track_id == s_candidate_track_id)
    {
        if(s_candidate_confirm_count < 255U)
        {
            s_candidate_confirm_count++;
        }
    }
    else
    {
        s_candidate_track_id = track_id;
        s_candidate_confirm_count = 1U;
    }

    return (s_candidate_confirm_count >= BEACON_SEARCH_CONFIRM_FRAMES) ? 1U : 0U;
}

void BeaconSearch_Init(void)
{
    s_was_armed = 0U;
    s_car_link_age_ms = 60000U;
    s_current_yaw_deg = 0.0f;
    s_target_yaw_deg = 0.0f;
    s_turn_progress_deg = 0.0f;
    s_sweep_phase = BEACON_SEARCH_SWEEP_TO_FIRST;
    s_endpoint_hold_elapsed_s = 0.0f;
    s_last_map_tie_first_sign = -1;
    s_search_generation = 0U;
    beacon_search_reset_session();
}

void BeaconSearch_Tick(uint16 elapsed_ms,
                       uint8 armed_state,
                       uint8 angle_mode,
                       uint8 flight_ready,
                       uint8 velocity_loop_enabled,
                       uint8 car_link_valid,
                       uint16 car_link_age_ms,
                       uint8 car_motion_valid,
                       uint8 car_target_valid,
                       float current_yaw_deg,
                       const BeaconSearchDirectionInput_t *direction_input,
                       const BeaconSearchStabilityInput_t *stability_input)
{
    uint8 eligible;

    s_car_link_age_ms = car_link_age_ms;
    if(!armed_state)
    {
        s_was_armed = 0U;
        beacon_search_reset_session();
        return;
    }

    if(!s_was_armed)
    {
        beacon_search_reset_session();
        s_was_armed = 1U;
    }

    eligible = (uint8)(angle_mode && flight_ready &&
                       !BlindBoxTasks_IsBeaconSearchPaused() &&
                       velocity_loop_enabled &&
                       (NULL != stability_input) &&
                       stability_input->position_loop_enabled);
    s_eligible = eligible;
    if(!eligible)
    {
        s_state = BEACON_SEARCH_DISABLED;
        s_control = BEACON_SEARCH_CONTROL_NONE;
        s_loss_age_ms = 0U;
        s_search_elapsed_ms = 0U;
        s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
        beacon_search_clear_candidate();
        beacon_search_reset_stability();
        return;
    }

    beacon_search_update_stability(elapsed_ms, stability_input);

    /* 只有小车运动状态有效且 TARGET_VALID=0，才累计“无目标”；
     * 链路或运动状态失效时不把旧标志当作“无目标”。 */
    if(car_link_valid && car_motion_valid && !car_target_valid)
    {
        s_loss_age_ms = beacon_search_add_ms(s_loss_age_ms, elapsed_ms);
    }
    else
    {
        s_loss_age_ms = 0U;
    }

    if(BEACON_SEARCH_DISABLED == s_state)
    {
        s_state = s_ever_seen ? BEACON_SEARCH_TRACKING :
                                BEACON_SEARCH_WAIT_FIRST;
    }

    if((BEACON_SEARCH_SPINNING == s_state) ||
       (BEACON_SEARCH_VERIFYING == s_state))
    {
        if(!car_link_valid)
        {
            beacon_search_fail(BEACON_SEARCH_FAILURE_CAR_LINK);
            return;
        }
    }

    if((BEACON_SEARCH_SPINNING == s_state) ||
       ((BEACON_SEARCH_VERIFYING == s_state) && !s_verify_from_failed))
    {
        s_search_elapsed_ms = beacon_search_add_ms(s_search_elapsed_ms,
                                                   elapsed_ms);
    }

    if(BEACON_SEARCH_VERIFYING == s_state)
    {
        s_verify_elapsed_ms = beacon_search_add_ms(s_verify_elapsed_ms,
                                                   elapsed_ms);
        if(s_verify_elapsed_ms >= BEACON_SEARCH_VERIFY_TIMEOUT_MS)
        {
            if(s_verify_from_failed)
            {
                beacon_search_fail(s_failure_reason);
            }
            else
            {
                s_state = BEACON_SEARCH_SPINNING;
                s_control = BEACON_SEARCH_CONTROL_SPIN;
                beacon_search_clear_candidate();
            }
        }
        return;
    }

    if((BEACON_SEARCH_WAIT_FIRST == s_state) ||
       (BEACON_SEARCH_TRACKING == s_state) ||
       (BEACON_SEARCH_LOSS_WAIT == s_state))
    {
        s_state = BEACON_SEARCH_LOSS_WAIT;
        if(s_loss_age_ms >= BEACON_SEARCH_LOSS_DELAY_MS)
        {
            if(s_start_gate_ready)
            {
                beacon_search_begin(current_yaw_deg, direction_input);
            }
        }
    }
}

void BeaconSearch_OnObservationFrame(
    const core_link_beacon_observation_packet_struct *packet,
    uint32 vision_process_count)
{
    uint8 trusted_track_id;

    if((NULL == packet) || !s_eligible ||
       BlindBoxTasks_IsBeaconSearchPaused())
    {
        return;
    }
    if(s_process_count_valid &&
       (vision_process_count == s_last_process_count))
    {
        return;
    }
    s_process_count_valid = 1U;
    s_last_process_count = vision_process_count;
    trusted_track_id = beacon_search_select_trusted_track(packet);

    if(BEACON_SEARCH_WAIT_FIRST == s_state)
    {
        if(beacon_search_confirm_candidate(trusted_track_id))
        {
            s_ever_seen = 1U;
            s_state = BEACON_SEARCH_TRACKING;
            beacon_search_clear_candidate();
        }
        return;
    }

    if((BEACON_SEARCH_TRACKING == s_state) ||
       (BEACON_SEARCH_LOSS_WAIT == s_state))
    {
        if(0U != trusted_track_id)
        {
            s_state = BEACON_SEARCH_TRACKING;
            beacon_search_clear_candidate();
        }
        return;
    }

    if(BEACON_SEARCH_SPINNING == s_state)
    {
        if(0U != trusted_track_id)
        {
            s_state = BEACON_SEARCH_VERIFYING;
            s_control = BEACON_SEARCH_CONTROL_HOLD;
            s_verify_from_failed = 0U;
            s_verify_elapsed_ms = 0U;
            (void)beacon_search_confirm_candidate(trusted_track_id);
        }
        return;
    }

    if(BEACON_SEARCH_FAILED == s_state)
    {
        if(0U != trusted_track_id)
        {
            s_state = BEACON_SEARCH_VERIFYING;
            s_control = BEACON_SEARCH_CONTROL_HOLD;
            s_verify_from_failed = 1U;
            s_verify_elapsed_ms = 0U;
            (void)beacon_search_confirm_candidate(trusted_track_id);
        }
        return;
    }

    if(BEACON_SEARCH_VERIFYING == s_state)
    {
        if(beacon_search_confirm_candidate(trusted_track_id))
        {
            s_ever_seen = 1U;
            s_state = BEACON_SEARCH_TRACKING;
            s_control = BEACON_SEARCH_CONTROL_NONE;
            s_search_elapsed_ms = 0U;
            s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
            beacon_search_clear_candidate();
        }
    }
}

void BeaconSearch_ControlStep(float dt_s,
                              float current_yaw_deg,
                              volatile float *target_yaw_deg)
{
    static BeaconSearchControl_t last_control = BEACON_SEARCH_CONTROL_NONE;
    static uint8 last_search_generation = 0U;
    static float previous_yaw_deg = 0.0f;
    BeaconSearchControl_t control = s_control;

    s_current_yaw_deg = current_yaw_deg;
    if(NULL == target_yaw_deg)
    {
        return;
    }

    if(control != last_control)
    {
        if((BEACON_SEARCH_CONTROL_SPIN == control) ||
           (BEACON_SEARCH_CONTROL_HOLD == control))
        {
            *target_yaw_deg = current_yaw_deg;
            previous_yaw_deg = current_yaw_deg;
            if(BEACON_SEARCH_CONTROL_SPIN == control)
            {
                s_endpoint_hold_elapsed_s = 0.0f;
            }
        }
        last_control = control;
    }

    if(BEACON_SEARCH_CONTROL_SPIN == control)
    {
        float yaw_delta_deg;
        uint8 target_at_endpoint;

        if(last_search_generation != s_search_generation)
        {
            last_search_generation = s_search_generation;
            s_turn_progress_deg = 0.0f;
            s_endpoint_hold_elapsed_s = 0.0f;
            s_predicted_hold_elapsed_s = 0.0f;
            previous_yaw_deg = current_yaw_deg;
            *target_yaw_deg = current_yaw_deg;
        }

        yaw_delta_deg = beacon_search_wrap_deg(current_yaw_deg -
                                                previous_yaw_deg);
        previous_yaw_deg = current_yaw_deg;
        if((fabsf(yaw_delta_deg) <= BEACON_SEARCH_MAX_YAW_STEP_DEG) &&
           ((BEACON_SEARCH_SWEEP_TO_FIRST == s_sweep_phase) ||
            (BEACON_SEARCH_SWEEP_TO_SECOND == s_sweep_phase)))
        {
            s_turn_progress_deg += fabsf(yaw_delta_deg);
        }

        switch(s_sweep_phase)
        {
        case BEACON_SEARCH_SWEEP_ALIGN_PREDICTED:
            target_at_endpoint = beacon_search_step_target(
                dt_s, s_predicted_target_yaw_deg,
                target_yaw_deg);
            if(target_at_endpoint &&
               beacon_search_yaw_at_endpoint(current_yaw_deg,
                                             s_predicted_target_yaw_deg))
            {
                s_sweep_phase = BEACON_SEARCH_SWEEP_HOLD_PREDICTED;
                s_predicted_hold_elapsed_s = 0.0f;
            }
            break;

        case BEACON_SEARCH_SWEEP_HOLD_PREDICTED:
            *target_yaw_deg = s_predicted_target_yaw_deg;
            if(!beacon_search_yaw_at_endpoint(current_yaw_deg,
                                              s_predicted_target_yaw_deg))
            {
                s_predicted_hold_elapsed_s = 0.0f;
                s_sweep_phase = BEACON_SEARCH_SWEEP_ALIGN_PREDICTED;
                break;
            }
            s_predicted_hold_elapsed_s += dt_s;
            if(s_predicted_hold_elapsed_s >=
               ((float)BEACON_SEARCH_PREDICT_HOLD_MS * 0.001f))
            {
                s_predicted_hold_elapsed_s = 0.0f;
                s_direction_mode = s_map_fallback_valid ?
                    BEACON_SEARCH_DIRECTION_MAP :
                    BEACON_SEARCH_DIRECTION_DEFAULT;
                s_sweep_phase = BEACON_SEARCH_SWEEP_TO_FIRST;
            }
            break;

        case BEACON_SEARCH_SWEEP_TO_FIRST:
            target_at_endpoint = beacon_search_step_target(
                dt_s, s_first_endpoint_yaw_deg,
                target_yaw_deg);
            if(target_at_endpoint &&
               beacon_search_yaw_at_endpoint(current_yaw_deg,
                                             s_first_endpoint_yaw_deg))
            {
                s_sweep_phase = BEACON_SEARCH_SWEEP_HOLD_FIRST;
                s_endpoint_hold_elapsed_s = 0.0f;
            }
            break;

        case BEACON_SEARCH_SWEEP_HOLD_FIRST:
            if(beacon_search_hold_endpoint(dt_s, current_yaw_deg,
                                           s_first_endpoint_yaw_deg,
                                           target_yaw_deg))
            {
                s_sweep_phase = BEACON_SEARCH_SWEEP_TO_SECOND;
            }
            break;

        case BEACON_SEARCH_SWEEP_TO_SECOND:
            target_at_endpoint = beacon_search_step_target(
                dt_s, s_second_endpoint_yaw_deg,
                target_yaw_deg);
            if(target_at_endpoint &&
               beacon_search_yaw_at_endpoint(current_yaw_deg,
                                             s_second_endpoint_yaw_deg))
            {
                s_sweep_phase = BEACON_SEARCH_SWEEP_HOLD_SECOND;
                s_endpoint_hold_elapsed_s = 0.0f;
            }
            break;

        case BEACON_SEARCH_SWEEP_HOLD_SECOND:
            if(beacon_search_hold_endpoint(dt_s, current_yaw_deg,
                                           s_second_endpoint_yaw_deg,
                                           target_yaw_deg))
            {
                /* 不设搜索轮数上限：+/-90度端点持续往返，直到可信目标出现。 */
                s_sweep_phase = BEACON_SEARCH_SWEEP_TO_FIRST;
            }
            break;

        default:
            s_sweep_phase = (BEACON_SEARCH_DIRECTION_PREDICTED ==
                             s_direction_mode) ?
                            BEACON_SEARCH_SWEEP_ALIGN_PREDICTED :
                            BEACON_SEARCH_SWEEP_TO_FIRST;
            break;
        }
    }

    s_target_yaw_deg = *target_yaw_deg;
}

uint8 BeaconSearch_SetYawRateDps(float yaw_rate_dps)
{
    uint32 irq_state;

    if(!isfinite(yaw_rate_dps) ||
       yaw_rate_dps < BEACON_SEARCH_YAW_RATE_MIN_DPS ||
       yaw_rate_dps > BEACON_SEARCH_YAW_RATE_MAX_DPS)
    {
        return 0U;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    s_yaw_rate_dps = yaw_rate_dps;
    if(0U == irq_state)
    {
        __enable_irq();
    }
    return 1U;
}

float BeaconSearch_GetYawRateDps(void)
{
    float yaw_rate_dps = s_yaw_rate_dps;

    if(!isfinite(yaw_rate_dps) ||
       yaw_rate_dps < BEACON_SEARCH_YAW_RATE_MIN_DPS ||
       yaw_rate_dps > BEACON_SEARCH_YAW_RATE_MAX_DPS)
    {
        return BEACON_SEARCH_YAW_RATE_DEFAULT_DPS;
    }
    return yaw_rate_dps;
}

uint8 BeaconSearch_SetSweepHalfAngleDeg(float half_angle_deg)
{
    uint32 irq_state;

    if(!isfinite(half_angle_deg) ||
       half_angle_deg < BEACON_SEARCH_SWEEP_HALF_ANGLE_MIN_DEG ||
       half_angle_deg > BEACON_SEARCH_SWEEP_HALF_ANGLE_MAX_DEG)
    {
        return 0U;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    s_sweep_half_angle_deg = half_angle_deg;
    if(0U == irq_state)
    {
        __enable_irq();
    }
    return 1U;
}

float BeaconSearch_GetSweepHalfAngleDeg(void)
{
    float half_angle_deg = s_sweep_half_angle_deg;

    if(!isfinite(half_angle_deg) ||
       half_angle_deg < BEACON_SEARCH_SWEEP_HALF_ANGLE_MIN_DEG ||
       half_angle_deg > BEACON_SEARCH_SWEEP_HALF_ANGLE_MAX_DEG)
    {
        return BEACON_SEARCH_SWEEP_HALF_ANGLE_DEFAULT_DEG;
    }
    return half_angle_deg;
}

float BeaconSearch_GetSweepAngleDeg(void)
{
    return 2.0f * BeaconSearch_GetSweepHalfAngleDeg();
}

float BeaconSearch_LimitYawRate(float yaw_rate_dps)
{
    float search_yaw_rate_dps = BeaconSearch_GetYawRateDps();

    if(!isfinite(yaw_rate_dps))
    {
        return 0.0f;
    }
    if(BEACON_SEARCH_CONTROL_NONE == s_control)
    {
        return yaw_rate_dps;
    }
    if(yaw_rate_dps > search_yaw_rate_dps)
    {
        return search_yaw_rate_dps;
    }
    if(yaw_rate_dps < -search_yaw_rate_dps)
    {
        return -search_yaw_rate_dps;
    }
    return yaw_rate_dps;
}

uint8 BeaconSearch_IsPausedByBlindBoxTask(void)
{
    return BlindBoxTasks_IsBeaconSearchPaused();
}

uint8 BeaconSearch_GetFrameFlags(void)
{
    uint8 frame_flags = BlindBoxTasks_GetFrameFlags();

    if((BEACON_SEARCH_FAILED == s_state) ||
       ((BEACON_SEARCH_VERIFYING == s_state) && s_verify_from_failed))
    {
        frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_SEARCH_FAILED;
    }
    if((BEACON_SEARCH_SPINNING == s_state) ||
       (BEACON_SEARCH_VERIFYING == s_state))
    {
        frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_SEARCH_ACTIVE;
    }
    return frame_flags;
}

void BeaconSearch_GetDiag(BeaconSearchDiag_t *diag)
{
    uint32 irq_state;

    if(NULL == diag)
    {
        return;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    memset(diag, 0, sizeof(*diag));
    diag->state = (uint8)s_state;
    diag->eligible = s_eligible;
    diag->ever_seen = s_ever_seen;
    diag->candidate_track_id = s_candidate_track_id;
    diag->candidate_confirm_count = s_candidate_confirm_count;
    diag->failure_reason = (uint8)s_failure_reason;
    diag->loss_age_ms = s_loss_age_ms;
    diag->search_elapsed_ms = s_search_elapsed_ms;
    diag->car_link_age_ms = s_car_link_age_ms;
    diag->current_yaw_deg = s_current_yaw_deg;
    diag->target_yaw_deg = s_target_yaw_deg;
    diag->turn_progress_deg = s_turn_progress_deg;
    diag->sweep_angle_deg = BeaconSearch_GetSweepAngleDeg();
    diag->yaw_rate_dps = BeaconSearch_GetYawRateDps();
    diag->sweep_half_angle_deg = BeaconSearch_GetSweepHalfAngleDeg();
    diag->position_distance_cm = s_position_distance_cm;
    diag->horizontal_speed_cms = s_horizontal_speed_cms;
    diag->speed_stable_ms = s_speed_stable_ms;
    diag->vision_frame_age_ms = s_vision_frame_age_ms;
    diag->position_window_count = s_position_window_count;
    diag->position_qualified_count = s_position_qualified_count;
    diag->position_sample_valid = s_position_sample_valid;
    diag->fused_velocity_valid = s_fused_velocity_valid;
    diag->start_gate_ready = s_start_gate_ready;
    diag->direction_mode = (uint8)s_direction_mode;
    diag->direction_map_count = s_direction_map_count;
    diag->direction_predicted_valid = s_direction_predicted_valid;
    diag->direction_first_sign = s_direction_first_sign;
    diag->search_origin_yaw_deg = s_search_origin_yaw_deg;
    diag->predicted_target_yaw_deg = s_predicted_target_yaw_deg;
    if(s_predicted_hold_elapsed_s <= 0.0f)
    {
        diag->predicted_hold_ms = 0U;
    }
    else if(s_predicted_hold_elapsed_s >= 65.535f)
    {
        diag->predicted_hold_ms = 65535U;
    }
    else
    {
        diag->predicted_hold_ms = (uint16)(s_predicted_hold_elapsed_s * 1000.0f);
    }
    if(0U == irq_state)
    {
        __enable_irq();
    }
}
