#include "common/project_headfile.h"

#include <math.h>
#include <string.h>

#define BEACON_SEARCH_LOSS_DELAY_MS             (500U)   /* 视觉丢失后等待进入自搜索的延时 */
#define BEACON_SEARCH_STARTUP_ALIGN_DEG           (90.0f)/* 缓启动结束后相机长轴对准赛道的转角 */
#define BEACON_SEARCH_STARTUP_HOLD_MS            (8000U) /* 首次对准后等待信标的最长时间 */
#define BEACON_SEARCH_POST_SWEEP_HOLD_MS         (3000U) /* 完成三轮摆扫后再次搜索前的等待时间 */
#define BEACON_SEARCH_ENDPOINT_HOLD_MS           (200U)  /* 到达搜索端点后的短暂停留时间 */
#define BEACON_SEARCH_ENDPOINT_TOLERANCE_DEG       (2.0f)/* 判定到达目标Yaw端点的角度容差 */
#define BEACON_SEARCH_MAX_YAW_STEP_DEG           (10.0f) /* 单周期Yaw跳变过滤阈值，避免IMU突变误计 */
#define BEACON_SEARCH_MAX_SWEEP_ROUNDS             (3U)  /* 每组完成三轮正反向360度搜索 */

typedef enum
{
    BEACON_SEARCH_CONTROL_NONE = 0,
    BEACON_SEARCH_CONTROL_SPIN,
    BEACON_SEARCH_CONTROL_HOLD
} BeaconSearchControl_t;

static volatile BeaconSearchState_t s_state = BEACON_SEARCH_DISABLED;
static volatile BeaconSearchControl_t s_control = BEACON_SEARCH_CONTROL_NONE;
static volatile uint8 s_search_generation = 0U;
static volatile float s_current_yaw_deg = 0.0f;
static volatile float s_target_yaw_deg = 0.0f;
static volatile float s_turn_progress_deg = 0.0f;
static volatile float s_yaw_rate_dps = BEACON_SEARCH_YAW_RATE_DEFAULT_DPS;
static volatile float s_sweep_direction_angle_deg =
    BEACON_SEARCH_SWEEP_DIRECTION_DEFAULT_DEG;
static volatile BeaconSearchPhase_t s_sweep_phase =
    BEACON_SEARCH_PHASE_STARTUP_ALIGN;
static float s_endpoint_hold_elapsed_s = 0.0f;
static volatile float s_search_origin_yaw_deg = 0.0f;
static volatile float s_first_endpoint_yaw_deg = 90.0f;
static volatile float s_second_endpoint_yaw_deg = -90.0f;
static volatile uint8 s_sweep_round_count = 0U;
static uint8 s_startup_alignment_started = 0U;
static volatile uint8 s_startup_alignment_complete = 0U;
static volatile uint8 s_startup_sequence_complete = 0U;
static volatile uint16 s_phase_hold_elapsed_ms = 0U;

static uint8 s_was_armed = 0U;
static uint8 s_eligible = 0U;
static uint8 s_ever_seen = 0U;
static uint8 s_candidate_track_id = 0U;
static uint8 s_candidate_confirm_count = 0U;
static uint8 s_process_count_valid = 0U;
static uint32 s_last_process_count = 0U;
static uint16 s_loss_age_ms = 0U;
static uint16 s_search_elapsed_ms = 0U;
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
     * 不再阻塞自动导航许可后的搜索启动。 */
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
    s_search_origin_yaw_deg = 0.0f;
    s_first_endpoint_yaw_deg = 90.0f;
    s_second_endpoint_yaw_deg = -90.0f;
    s_sweep_phase = BEACON_SEARCH_PHASE_STARTUP_ALIGN;
    s_sweep_round_count = 0U;
    s_startup_alignment_started = 0U;
    s_startup_alignment_complete = 0U;
    s_startup_sequence_complete = 0U;
    s_phase_hold_elapsed_ms = 0U;
    beacon_search_clear_candidate();
    beacon_search_reset_stability();
}

static void beacon_search_begin_startup_alignment(float current_yaw_deg)
{
    if(!isfinite(current_yaw_deg))
    {
        current_yaw_deg = 0.0f;
    }

    s_state = BEACON_SEARCH_SPINNING;
    s_control = BEACON_SEARCH_CONTROL_SPIN;
    s_search_elapsed_ms = 0U;
    s_loss_age_ms = 0U;
    s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
    s_search_generation++;
    s_search_origin_yaw_deg = beacon_search_wrap_deg(current_yaw_deg);
    s_first_endpoint_yaw_deg = beacon_search_wrap_deg(
        s_search_origin_yaw_deg + BEACON_SEARCH_STARTUP_ALIGN_DEG);
    s_startup_alignment_started = 1U;
    s_startup_alignment_complete = 0U;
    s_sweep_phase = BEACON_SEARCH_PHASE_STARTUP_ALIGN;
    s_sweep_round_count = 0U;
    s_phase_hold_elapsed_ms = 0U;
    beacon_search_clear_candidate();
}

static void beacon_search_resume_startup_sequence(void)
{
    s_state = BEACON_SEARCH_SPINNING;
    s_control = BEACON_SEARCH_CONTROL_SPIN;
    s_search_generation++;
    s_sweep_phase = s_startup_alignment_complete ?
                    BEACON_SEARCH_PHASE_STARTUP_HOLD :
                    BEACON_SEARCH_PHASE_STARTUP_ALIGN;
    s_phase_hold_elapsed_ms = 0U;
}

static void beacon_search_begin_sweep(float current_yaw_deg)
{
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

    /* 完整一周后的绝对Yaw与起点相同，不能再用归一化绝对端点规划。
     * ControlStep按旋转方向累计实际Yaw增量，端点变量只在转满后保存保持角。 */
    s_first_endpoint_yaw_deg = s_search_origin_yaw_deg;
    s_second_endpoint_yaw_deg = s_search_origin_yaw_deg;
    s_sweep_phase = BEACON_SEARCH_PHASE_SWEEP_TO_FIRST;
    s_sweep_round_count = 0U;
    s_turn_progress_deg = 0.0f;
    s_phase_hold_elapsed_ms = 0U;
    beacon_search_clear_candidate();
}

static void beacon_search_restart_sweep_cycle(void)
{
    s_state = BEACON_SEARCH_SPINNING;
    s_control = BEACON_SEARCH_CONTROL_SPIN;
    s_search_generation++;
    s_sweep_phase = BEACON_SEARCH_PHASE_SWEEP_TO_FIRST;
    s_sweep_round_count = 0U;
    s_turn_progress_deg = 0.0f;
    s_phase_hold_elapsed_ms = 0U;
    beacon_search_clear_candidate();
}

static void beacon_search_track_observation(void)
{
    s_ever_seen = 1U;
    s_state = BEACON_SEARCH_TRACKING;
    s_control = BEACON_SEARCH_CONTROL_NONE;
    s_loss_age_ms = 0U;
    s_search_elapsed_ms = 0U;
    s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
    s_sweep_round_count = 0U;
    s_phase_hold_elapsed_ms = 0U;
}

/* packet由视觉融合层生成；进入packet的目标已经通过视觉算法有效性判定。
 * 搜索层只做数据完整性检查，不再重复置信度、sigma或连续帧门控。 */
static uint8 beacon_search_select_valid_track(
    const core_link_beacon_observation_packet_struct *packet)
{
    uint8 i;

    if((NULL == packet) ||
       (packet->count > CORE_LINK_BEACON_OBSERVATION_MAX_COUNT))
    {
        return 0U;
    }

    for(i = 0U; i < packet->count; ++i)
    {
        if((packet->observation[i].track_id != 0U) &&
           (packet->observation[i].flags &
            CORE_LINK_BEACON_OBSERVATION_FLAG_LIT))
        {
            return packet->observation[i].track_id;
        }
    }

    return 0U;
}

void BeaconSearch_Init(void)
{
    s_was_armed = 0U;
    s_car_link_age_ms = 60000U;
    s_current_yaw_deg = 0.0f;
    s_target_yaw_deg = 0.0f;
    s_turn_progress_deg = 0.0f;
    s_sweep_phase = BEACON_SEARCH_PHASE_STARTUP_ALIGN;
    s_endpoint_hold_elapsed_s = 0.0f;
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
                       uint8 car_search_requested,
                       float current_yaw_deg,
                       const BeaconSearchStabilityInput_t *stability_input)
{
    uint8 eligible;

    s_car_link_age_ms = car_link_age_ms;
    if(isfinite(current_yaw_deg))
    {
        s_current_yaw_deg = current_yaw_deg;
    }
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
                       car_link_valid && car_search_requested &&
                       (NULL != stability_input) &&
                       stability_input->position_loop_enabled);
    s_eligible = eligible;
    if(!eligible)
    {
        s_state = BEACON_SEARCH_DISABLED;
        s_control = BEACON_SEARCH_CONTROL_NONE;
        s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
        s_loss_age_ms = 0U;
        s_search_elapsed_ms = 0U;
        s_phase_hold_elapsed_ms = 0U;
        beacon_search_clear_candidate();
        beacon_search_reset_stability();
        return;
    }

    beacon_search_update_stability(elapsed_ms, stability_input);

    /* 每次上锁仅执行一次赛道方向对准。对准转动必须完整完成；只有进入
     * 8秒保持阶段后，新视觉帧中的有效信标才允许打断等待。 */
    if(!s_startup_sequence_complete)
    {
        if((BEACON_SEARCH_DISABLED == s_state) ||
           (BEACON_SEARCH_SPINNING != s_state) ||
           ((BEACON_SEARCH_PHASE_STARTUP_ALIGN != s_sweep_phase) &&
            (BEACON_SEARCH_PHASE_STARTUP_HOLD != s_sweep_phase)))
        {
            if(s_startup_alignment_started)
            {
                beacon_search_resume_startup_sequence();
            }
            else
            {
                beacon_search_begin_startup_alignment(current_yaw_deg);
            }
        }

        s_search_elapsed_ms = beacon_search_add_ms(s_search_elapsed_ms,
                                                   elapsed_ms);
        if(BEACON_SEARCH_PHASE_STARTUP_HOLD == s_sweep_phase)
        {
            s_phase_hold_elapsed_ms = beacon_search_add_ms(
                s_phase_hold_elapsed_ms, elapsed_ms);
            if(s_phase_hold_elapsed_ms >= BEACON_SEARCH_STARTUP_HOLD_MS)
            {
                s_startup_sequence_complete = 1U;
                s_state = BEACON_SEARCH_LOSS_WAIT;
                s_control = BEACON_SEARCH_CONTROL_NONE;
                s_loss_age_ms = 0U;
                s_search_elapsed_ms = 0U;
                s_phase_hold_elapsed_ms = 0U;
            }
        }
        return;
    }

    if(BEACON_SEARCH_SPINNING == s_state)
    {
        s_search_elapsed_ms = beacon_search_add_ms(s_search_elapsed_ms,
                                                   elapsed_ms);
        if(BEACON_SEARCH_PHASE_POST_SWEEP_HOLD == s_sweep_phase)
        {
            s_phase_hold_elapsed_ms = beacon_search_add_ms(
                s_phase_hold_elapsed_ms, elapsed_ms);
            if(s_phase_hold_elapsed_ms >= BEACON_SEARCH_POST_SWEEP_HOLD_MS)
            {
                /* 不附加无信标等待延时，沿用当前搜索中心直接开始下一组三轮。 */
                beacon_search_restart_sweep_cycle();
            }
        }
        return;
    }

    if(BEACON_SEARCH_DISABLED == s_state)
    {
        s_state = s_ever_seen ? BEACON_SEARCH_TRACKING :
                                BEACON_SEARCH_WAIT_FIRST;
    }

    /* 仅在小车自动任务明确请求时累计；遥控器模式和链路失效不会触发搜索。 */
    s_loss_age_ms = beacon_search_add_ms(s_loss_age_ms, elapsed_ms);
    if((BEACON_SEARCH_WAIT_FIRST == s_state) ||
       (BEACON_SEARCH_TRACKING == s_state) ||
       (BEACON_SEARCH_LOSS_WAIT == s_state) ||
       (BEACON_SEARCH_FAILED == s_state))
    {
        s_state = BEACON_SEARCH_LOSS_WAIT;
        if(s_loss_age_ms >= BEACON_SEARCH_LOSS_DELAY_MS)
        {
            if(s_start_gate_ready)
            {
                beacon_search_begin_sweep(current_yaw_deg);
            }
        }
    }
}

void BeaconSearch_OnObservationFrame(
    const core_link_beacon_observation_packet_struct *packet,
    uint32 vision_process_count)
{
    uint8 valid_track_id;

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
    valid_track_id = beacon_search_select_valid_track(packet);
    if(0U == valid_track_id)
    {
        return;
    }

    /* +90度对准动作不允许被信标打断，但该帧已经被序号去重记录。
     * 到位后的8秒保持、三轮正反向360度搜索及3秒保持均允许单帧打断。 */
    if((BEACON_SEARCH_SPINNING == s_state) &&
       (BEACON_SEARCH_PHASE_STARTUP_ALIGN == s_sweep_phase))
    {
        return;
    }

    if((BEACON_SEARCH_SPINNING == s_state) &&
       (BEACON_SEARCH_PHASE_STARTUP_HOLD == s_sweep_phase))
    {
        s_startup_sequence_complete = 1U;
    }

    if((BEACON_SEARCH_WAIT_FIRST == s_state) ||
       (BEACON_SEARCH_TRACKING == s_state) ||
       (BEACON_SEARCH_LOSS_WAIT == s_state) ||
       (BEACON_SEARCH_SPINNING == s_state) ||
       (BEACON_SEARCH_VERIFYING == s_state) ||
       (BEACON_SEARCH_FAILED == s_state))
    {
        beacon_search_track_observation();
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
        if((BEACON_SEARCH_CONTROL_NONE == control) ||
           (BEACON_SEARCH_CONTROL_SPIN == control) ||
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
        float direction_angle_deg =
            BeaconSearch_GetSweepDirectionAngleDeg();
        uint8 target_at_endpoint;

        if(last_search_generation != s_search_generation)
        {
            last_search_generation = s_search_generation;
            s_turn_progress_deg = 0.0f;
            s_endpoint_hold_elapsed_s = 0.0f;
            previous_yaw_deg = current_yaw_deg;
            *target_yaw_deg = current_yaw_deg;
        }

        yaw_delta_deg = beacon_search_wrap_deg(current_yaw_deg -
                                                previous_yaw_deg);
        previous_yaw_deg = current_yaw_deg;
        if((fabsf(yaw_delta_deg) <= BEACON_SEARCH_MAX_YAW_STEP_DEG) &&
           (BEACON_SEARCH_PHASE_STARTUP_ALIGN == s_sweep_phase))
        {
            s_turn_progress_deg += fabsf(yaw_delta_deg);
        }
        else if((fabsf(yaw_delta_deg) <= BEACON_SEARCH_MAX_YAW_STEP_DEG) &&
                (BEACON_SEARCH_PHASE_SWEEP_TO_FIRST == s_sweep_phase))
        {
            s_turn_progress_deg += yaw_delta_deg;
            if(s_turn_progress_deg < 0.0f)
            {
                s_turn_progress_deg = 0.0f;
            }
        }
        else if((fabsf(yaw_delta_deg) <= BEACON_SEARCH_MAX_YAW_STEP_DEG) &&
                (BEACON_SEARCH_PHASE_SWEEP_TO_SECOND == s_sweep_phase))
        {
            s_turn_progress_deg -= yaw_delta_deg;
            if(s_turn_progress_deg < 0.0f)
            {
                s_turn_progress_deg = 0.0f;
            }
        }

        switch(s_sweep_phase)
        {
        case BEACON_SEARCH_PHASE_STARTUP_ALIGN:
            target_at_endpoint = beacon_search_step_target(
                dt_s, s_first_endpoint_yaw_deg,
                target_yaw_deg);
            if(target_at_endpoint &&
               beacon_search_yaw_at_endpoint(current_yaw_deg,
                                             s_first_endpoint_yaw_deg))
            {
                s_startup_alignment_complete = 1U;
                s_sweep_phase = BEACON_SEARCH_PHASE_STARTUP_HOLD;
                s_phase_hold_elapsed_ms = 0U;
                s_endpoint_hold_elapsed_s = 0.0f;
            }
            break;

        case BEACON_SEARCH_PHASE_STARTUP_HOLD:
            *target_yaw_deg = s_first_endpoint_yaw_deg;
            break;

        case BEACON_SEARCH_PHASE_SWEEP_TO_FIRST:
            if(s_turn_progress_deg >= direction_angle_deg)
            {
                s_turn_progress_deg = direction_angle_deg;
                s_first_endpoint_yaw_deg = beacon_search_wrap_deg(
                    current_yaw_deg);
                *target_yaw_deg = s_first_endpoint_yaw_deg;
                s_sweep_phase = BEACON_SEARCH_PHASE_SWEEP_HOLD_FIRST;
                s_endpoint_hold_elapsed_s = 0.0f;
            }
            else if(isfinite(dt_s) && (dt_s > 0.0f))
            {
                *target_yaw_deg = beacon_search_wrap_deg(
                    *target_yaw_deg + BeaconSearch_GetYawRateDps() * dt_s);
            }
            break;

        case BEACON_SEARCH_PHASE_SWEEP_HOLD_FIRST:
            if(beacon_search_hold_endpoint(dt_s, current_yaw_deg,
                                           s_first_endpoint_yaw_deg,
                                           target_yaw_deg))
            {
                s_sweep_phase = BEACON_SEARCH_PHASE_SWEEP_TO_SECOND;
                s_turn_progress_deg = 0.0f;
            }
            break;

        case BEACON_SEARCH_PHASE_SWEEP_TO_SECOND:
            if(s_turn_progress_deg >= direction_angle_deg)
            {
                s_turn_progress_deg = direction_angle_deg;
                s_second_endpoint_yaw_deg = beacon_search_wrap_deg(
                    current_yaw_deg);
                *target_yaw_deg = s_second_endpoint_yaw_deg;
                s_sweep_phase = BEACON_SEARCH_PHASE_SWEEP_HOLD_SECOND;
                s_endpoint_hold_elapsed_s = 0.0f;
            }
            else if(isfinite(dt_s) && (dt_s > 0.0f))
            {
                *target_yaw_deg = beacon_search_wrap_deg(
                    *target_yaw_deg - BeaconSearch_GetYawRateDps() * dt_s);
            }
            break;

        case BEACON_SEARCH_PHASE_SWEEP_HOLD_SECOND:
            if(beacon_search_hold_endpoint(dt_s, current_yaw_deg,
                                           s_second_endpoint_yaw_deg,
                                           target_yaw_deg))
            {
                s_sweep_round_count++;
                if(s_sweep_round_count >= BEACON_SEARCH_MAX_SWEEP_ROUNDS)
                {
                    /* 三轮完成后直接保持，不再附加额外360度旋转。 */
                    s_phase_hold_elapsed_ms = 0U;
                    s_sweep_phase = BEACON_SEARCH_PHASE_POST_SWEEP_HOLD;
                }
                else
                {
                    s_turn_progress_deg = 0.0f;
                    s_sweep_phase = BEACON_SEARCH_PHASE_SWEEP_TO_FIRST;
                }
            }
            break;

        case BEACON_SEARCH_PHASE_POST_SWEEP_HOLD:
            /* 保持第三轮结束时的目标Yaw，等待Tick重启下一组三轮。 */
            break;

        default:
            s_sweep_phase = BEACON_SEARCH_PHASE_SWEEP_TO_FIRST;
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

uint8 BeaconSearch_SetSweepDirectionAngleDeg(float direction_angle_deg)
{
    uint32 irq_state;

    if(!isfinite(direction_angle_deg) ||
       direction_angle_deg < BEACON_SEARCH_SWEEP_DIRECTION_MIN_DEG ||
       direction_angle_deg > BEACON_SEARCH_SWEEP_DIRECTION_MAX_DEG)
    {
        return 0U;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    s_sweep_direction_angle_deg = direction_angle_deg;
    if(0U == irq_state)
    {
        __enable_irq();
    }
    return 1U;
}

float BeaconSearch_GetSweepDirectionAngleDeg(void)
{
    float direction_angle_deg = s_sweep_direction_angle_deg;

    if(!isfinite(direction_angle_deg) ||
       direction_angle_deg < BEACON_SEARCH_SWEEP_DIRECTION_MIN_DEG ||
       direction_angle_deg > BEACON_SEARCH_SWEEP_DIRECTION_MAX_DEG)
    {
        return BEACON_SEARCH_SWEEP_DIRECTION_DEFAULT_DEG;
    }
    return direction_angle_deg;
}

float BeaconSearch_GetSweepAngleDeg(void)
{
    return 2.0f * BeaconSearch_GetSweepDirectionAngleDeg();
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

    if(BEACON_SEARCH_FAILED == s_state)
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
    diag->sweep_direction_angle_deg =
        BeaconSearch_GetSweepDirectionAngleDeg();
    diag->position_distance_cm = s_position_distance_cm;
    diag->horizontal_speed_cms = s_horizontal_speed_cms;
    diag->speed_stable_ms = s_speed_stable_ms;
    diag->vision_frame_age_ms = s_vision_frame_age_ms;
    diag->position_window_count = s_position_window_count;
    diag->position_qualified_count = s_position_qualified_count;
    diag->position_sample_valid = s_position_sample_valid;
    diag->fused_velocity_valid = s_fused_velocity_valid;
    diag->start_gate_ready = s_start_gate_ready;
    diag->sweep_round_count = s_sweep_round_count;
    diag->terminal_latched = 0U;
    diag->sweep_phase = (uint8)s_sweep_phase;
    diag->startup_alignment_complete = s_startup_alignment_complete;
    diag->startup_sequence_complete = s_startup_sequence_complete;
    diag->phase_hold_elapsed_ms = s_phase_hold_elapsed_ms;
    diag->search_origin_yaw_deg = s_search_origin_yaw_deg;
    if(0U == irq_state)
    {
        __enable_irq();
    }
}
