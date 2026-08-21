#include "common/project_headfile.h"

#define CORE_LINK_USER_KEY_QUEUE_SIZE       (8U)
#define CORE_LINK_USER_KEY_ACTIVE_THRESHOLD (0.5f)
#define CORE_LINK_USER_SCREEN_SWITCH_THRESHOLD (0.5f)
#define CORE_LINK_USER_BEACON_OBSERVATION_PERIOD_MS (50U)
#define CORE_LINK_USER_BEACON_TIMEOUT_MS    (200U)
#define CORE_LINK_USER_BEACON_TRACK_GATE_MIN_CM (30.0f)
#define CORE_LINK_USER_BEACON_TRACK_MAX_MISS_FRAMES (5U)
#define CORE_LINK_USER_BEACON_CAR_EXCLUSION_RADIUS_CM (35.0f)
#define CORE_LINK_USER_MOTION_STALE_MS      (80U)
#define CORE_LINK_USER_MOTION_INVALID_MS    (130U)
#define CORE_LINK_USER_PLAN_VEL_SLEW_ACCEL_CMS2 (200.0f)
#define CORE_LINK_USER_PLAN_ACCEL_LPF_HZ    (10.0f)
#define CORE_LINK_USER_TWO_PI               (6.28318530718f)
#define CORE_LINK_USER_MOTION_OUTPUT_EPSILON (0.01f)
#define CORE_LINK_USER_MOTION_SPEED_MAX_CMS (400.0f)
#define CORE_LINK_USER_MOTION_ACCEL_MAX_CMS2 (500.0f)

typedef struct
{
    float x_cm;
    float y_cm;
    uint8 track_id;
    uint8 miss_count;
    uint8 valid;
} core_link_user_beacon_track_struct;

static volatile uint8 s_core_link_user_key_queue[CORE_LINK_USER_KEY_QUEUE_SIZE] = {0};
static volatile uint8 s_core_link_user_key_queue_head = 0;
static volatile uint8 s_core_link_user_key_queue_tail = 0;
static volatile uint8 s_core_link_user_monitor_exit_request = 0;
static volatile uint8 s_core_link_user_screen_mirror_enabled = 0;
static volatile uint8 s_core_link_user_screen_mirror_enabled_changed = 0;
static volatile uint8 s_core_link_user_key_flag_remote = 0;
static char s_core_link_user_text_receive[CORE_LINK_PAYLOAD_MAX_LEN + 1] = "none";
static uint16 s_core_link_user_beacon_tx_elapsed_ms = 0U;
static uint16 s_core_link_user_car_trajectory_age_ms = 60000U;
static uint16 s_core_link_user_car_trajectory_protocol_error_count = 0U;
static uint8 s_core_link_user_car_trajectory_valid = 0U;
static core_link_car_trajectory_packet_struct
    s_core_link_user_car_trajectory = {0};
static volatile uint16 s_core_link_user_car_trajectory_write_seq = 0U;
static uint8 s_core_link_user_car_sequence_valid = 0U;
static uint16 s_core_link_user_car_last_sequence = 0U;
static uint32 s_core_link_user_car_last_sample_time_ms = 0U;
static uint16 s_core_link_user_car_arrival_interval_ms = 0U;
static float s_core_link_user_car_smooth_vx_cms = 0.0f;
static float s_core_link_user_car_smooth_vy_cms = 0.0f;
static float s_core_link_user_car_lpf_ax_cms2 = 0.0f;
static float s_core_link_user_car_lpf_ay_cms2 = 0.0f;
static volatile uint32 s_core_link_user_time_ms = 0U;
static core_link_user_beacon_track_struct
    s_core_link_user_beacon_tracks[CORE_LINK_BEACON_TRACK_MAX_COUNT] = {0};
static uint16 s_core_link_user_beacon_sequence = 0U;
static uint8 s_core_link_user_next_track_id = 1U;
static uint8 s_core_link_user_tracker_reset_pending = 1U;
static uint32 s_core_link_user_live_fusion_reset_seen = 0U;

static uint8 core_link_user_queue_next_index (uint8 index)
{
    index ++;
    if(index >= CORE_LINK_USER_KEY_QUEUE_SIZE)
    {
        index = 0;
    }

    return index;
}

static uint8 core_link_user_beacon_overlaps_car(float beacon_x_cm,
                                                 float beacon_y_cm,
                                                 uint8 car_position_valid,
                                                 float car_x_cm,
                                                 float car_y_cm)
{
    float dx;
    float dy;

    if(!car_position_valid)
    {
        return 0U;
    }

    dx = beacon_x_cm - car_x_cm;
    dy = beacon_y_cm - car_y_cm;
    return (uint8)((dx * dx + dy * dy) <=
                   (CORE_LINK_USER_BEACON_CAR_EXCLUSION_RADIUS_CM *
                    CORE_LINK_USER_BEACON_CAR_EXCLUSION_RADIUS_CM));
}

static int16 core_link_user_read_i16_le(const uint8 *data)
{
    return (int16)((uint16)data[0] | ((uint16)data[1] << 8));
}

static uint16 core_link_user_read_u16_le(const uint8 *data)
{
    return (uint16)data[0] | ((uint16)data[1] << 8);
}

static uint32 core_link_user_read_u32_le(const uint8 *data)
{
    return (uint32)data[0] |
           ((uint32)data[1] << 8) |
           ((uint32)data[2] << 16) |
           ((uint32)data[3] << 24);
}

static void core_link_user_slew_vector(float target_x,
                                       float target_y,
                                       float max_step,
                                       float *state_x,
                                       float *state_y)
{
    float delta_x;
    float delta_y;
    float delta_length;

    if((NULL == state_x) || (NULL == state_y) || !(max_step > 0.0f))
    {
        return;
    }
    delta_x = target_x - *state_x;
    delta_y = target_y - *state_y;
    delta_length = sqrtf(delta_x * delta_x + delta_y * delta_y);
    if(!isfinite(delta_length))
    {
        return;
    }
    if(delta_length <= max_step)
    {
        *state_x = target_x;
        *state_y = target_y;
    }
    else if(delta_length > 0.0f)
    {
        float scale = max_step / delta_length;

        *state_x += delta_x * scale;
        *state_y += delta_y * scale;
    }
}

static uint16 core_link_user_motion_age_ms(uint32 receive_time_ms,
                                            uint32 now_ms)
{
    uint32 age_ms = (uint32)(now_ms - receive_time_ms);

    return (uint16)((age_ms > 65535U) ? 65535U : age_ms);
}

static void core_link_user_write_i16_le(uint8 *data, int16 value)
{
    if(NULL == data)
    {
        return;
    }

    data[0] = (uint8)((uint16)value & 0xFFU);
    data[1] = (uint8)(((uint16)value >> 8) & 0xFFU);
}

static void core_link_user_write_u16_le(uint8 *data, uint16 value)
{
    if(NULL == data)
    {
        return;
    }

    data[0] = (uint8)(value & 0xFFU);
    data[1] = (uint8)((value >> 8) & 0xFFU);
}

static float core_link_user_wrap_angle(float angle_deg)
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

static uint8 core_link_user_get_car_trajectory_snapshot(
    core_link_car_trajectory_packet_struct *snapshot)
{
    uint8 attempt;

    if(NULL == snapshot)
    {
        return 0U;
    }
    for(attempt = 0U; attempt < 4U; ++attempt)
    {
        uint16 seq_begin = s_core_link_user_car_trajectory_write_seq;
        uint16 seq_end;

        if(seq_begin & 1U)
        {
            continue;
        }
        memcpy(snapshot, &s_core_link_user_car_trajectory, sizeof(*snapshot));
        seq_end = s_core_link_user_car_trajectory_write_seq;
        if((seq_begin == seq_end) && !(seq_end & 1U))
        {
            return 1U;
        }
    }
    return 0U;
}

static float core_link_user_distance_squared_cm(float x1_cm, float y1_cm, float x2_cm, float y2_cm)
{
    float dx = x1_cm - x2_cm;
    float dy = y1_cm - y2_cm;

    return dx * dx + dy * dy;
}

static int16 core_link_user_cm_to_mm_i16(float cm)
{
    float mm = cm * 10.0f;

    if(mm > 32767.0f)
    {
        mm = 32767.0f;
    }
    else if(mm < -32768.0f)
    {
        mm = -32768.0f;
    }

    return (int16)((mm >= 0.0f) ? (mm + 0.5f) : (mm - 0.5f));
}

static int16 core_link_user_deg_to_ddeg_i16(float deg)
{
    float ddeg = core_link_user_wrap_angle(deg) * 10.0f;

    if(ddeg > 32767.0f)
    {
        ddeg = 32767.0f;
    }
    else if(ddeg < -32768.0f)
    {
        ddeg = -32768.0f;
    }

    return (int16)((ddeg >= 0.0f) ? (ddeg + 0.5f) : (ddeg - 0.5f));
}

static uint8 core_link_user_get_uplink_car_absolute_yaw_deg(float *yaw_deg)
{
    core_link_car_trajectory_packet_struct trajectory;

    if(NULL == yaw_deg)
    {
        return 0U;
    }

    if(s_core_link_user_car_trajectory_valid &&
       (s_core_link_user_car_trajectory_age_ms <=
        CORE_LINK_USER_MOTION_INVALID_MS) &&
       core_link_user_get_car_trajectory_snapshot(&trajectory) &&
       (trajectory.flags & CORE_LINK_CAR_TRAJECTORY_FLAG_YAW_VALID))
    {
        *yaw_deg = core_link_user_wrap_angle(
            ((float)trajectory.car_absolute_yaw_ddeg) * 0.1f);
        return 1U;
    }
    return 0U;
}

static uint8 core_link_user_track_id_in_use(uint8 track_id)
{
    uint8 i;

    for(i = 0U; i < CORE_LINK_BEACON_TRACK_MAX_COUNT; ++i)
    {
        if(s_core_link_user_beacon_tracks[i].valid &&
           (s_core_link_user_beacon_tracks[i].track_id == track_id))
        {
            return 1U;
        }
    }
    return 0U;
}

static uint8 core_link_user_allocate_track_id(void)
{
    uint16 attempts;

    for(attempts = 0U; attempts < 255U; ++attempts)
    {
        uint8 track_id = s_core_link_user_next_track_id;

        s_core_link_user_next_track_id++;
        if(0U == s_core_link_user_next_track_id)
        {
            s_core_link_user_next_track_id = 1U;
        }
        if((0U != track_id) && !core_link_user_track_id_in_use(track_id))
        {
            return track_id;
        }
    }
    return 0U;
}

static uint8 core_link_user_allocate_track_slot(
    const uint8 track_used[CORE_LINK_BEACON_TRACK_MAX_COUNT])
{
    uint8 best = 0xFFU;
    uint8 i;

    for(i = 0U; i < CORE_LINK_BEACON_TRACK_MAX_COUNT; ++i)
    {
        if(track_used[i])
        {
            continue;
        }
        if(!s_core_link_user_beacon_tracks[i].valid)
        {
            return i;
        }
        if((0xFFU == best) ||
           (s_core_link_user_beacon_tracks[i].miss_count >
            s_core_link_user_beacon_tracks[best].miss_count))
        {
            best = i;
        }
    }
    return best;
}

static void core_link_user_update_beacon_tracks(
    const VisionRuntimeBeaconSlot_t slots[VISION_RUNTIME_BEACON_SLOT_COUNT],
    uint8 slot_track_id[VISION_RUNTIME_BEACON_SLOT_COUNT])
{
    uint8 slot_used[VISION_RUNTIME_BEACON_SLOT_COUNT] = {0};
    uint8 track_used[CORE_LINK_BEACON_TRACK_MAX_COUNT] = {0};
    uint8 i;

    memset(slot_track_id, 0, VISION_RUNTIME_BEACON_SLOT_COUNT);
    for(;;)
    {
        uint8 best_slot = 0xFFU;
        uint8 best_track = 0xFFU;
        float best_distance_sq = 0.0f;
        uint8 track;

        for(i = 0U; i < VISION_RUNTIME_BEACON_SLOT_COUNT; ++i)
        {
            float gate_cm;

            if(slot_used[i] || !slots[i].valid)
            {
                continue;
            }
            gate_cm = 3.0f * (float)slots[i].position_sigma_cm;
            if(gate_cm < CORE_LINK_USER_BEACON_TRACK_GATE_MIN_CM)
            {
                gate_cm = CORE_LINK_USER_BEACON_TRACK_GATE_MIN_CM;
            }
            for(track = 0U; track < CORE_LINK_BEACON_TRACK_MAX_COUNT; ++track)
            {
                float distance_sq;

                if(track_used[track] || !s_core_link_user_beacon_tracks[track].valid)
                {
                    continue;
                }
                distance_sq = core_link_user_distance_squared_cm(
                    slots[i].x_cm, slots[i].y_cm,
                    s_core_link_user_beacon_tracks[track].x_cm,
                    s_core_link_user_beacon_tracks[track].y_cm);
                if((distance_sq <= gate_cm * gate_cm) &&
                   ((0xFFU == best_track) || (distance_sq < best_distance_sq)))
                {
                    best_slot = i;
                    best_track = track;
                    best_distance_sq = distance_sq;
                }
            }
        }

        if(0xFFU == best_track)
        {
            break;
        }
        slot_used[best_slot] = 1U;
        track_used[best_track] = 1U;
        slot_track_id[best_slot] = s_core_link_user_beacon_tracks[best_track].track_id;
        s_core_link_user_beacon_tracks[best_track].x_cm = slots[best_slot].x_cm;
        s_core_link_user_beacon_tracks[best_track].y_cm = slots[best_slot].y_cm;
        s_core_link_user_beacon_tracks[best_track].miss_count = 0U;
    }

    for(i = 0U; i < CORE_LINK_BEACON_TRACK_MAX_COUNT; ++i)
    {
        if(s_core_link_user_beacon_tracks[i].valid && !track_used[i])
        {
            s_core_link_user_beacon_tracks[i].miss_count++;
            if(s_core_link_user_beacon_tracks[i].miss_count >=
               CORE_LINK_USER_BEACON_TRACK_MAX_MISS_FRAMES)
            {
                memset(&s_core_link_user_beacon_tracks[i], 0,
                       sizeof(s_core_link_user_beacon_tracks[i]));
            }
        }
    }

    for(i = 0U; i < VISION_RUNTIME_BEACON_SLOT_COUNT; ++i)
    {
        uint8 track;
        uint8 track_id;

        if(!slots[i].valid || slot_used[i])
        {
            continue;
        }
        track = core_link_user_allocate_track_slot(track_used);
        if(0xFFU == track)
        {
            continue;
        }
        track_id = core_link_user_allocate_track_id();
        if(0U == track_id)
        {
            continue;
        }
        memset(&s_core_link_user_beacon_tracks[track], 0,
               sizeof(s_core_link_user_beacon_tracks[track]));
        s_core_link_user_beacon_tracks[track].valid = 1U;
        s_core_link_user_beacon_tracks[track].track_id = track_id;
        s_core_link_user_beacon_tracks[track].x_cm = slots[i].x_cm;
        s_core_link_user_beacon_tracks[track].y_cm = slots[i].y_cm;
        track_used[track] = 1U;
        slot_track_id[i] = track_id;
    }
}

static void core_link_user_send_beacon_observations(
    const core_link_beacon_observation_packet_struct *packet)
{
    uint8 payload[CORE_LINK_BEACON_OBSERVATION_PAYLOAD_MAX_LEN] = {0};
    uint8 payload_len;
    uint8 i;

    if((NULL == packet) ||
       (packet->count > CORE_LINK_BEACON_OBSERVATION_MAX_COUNT))
    {
        return;
    }

    core_link_user_write_u16_le(&payload[0], packet->sequence);
    payload[2] = packet->count;
    payload[3] = packet->frame_flags;
    payload[4] = packet->car_visible;
    payload[5] = packet->reserved;
    core_link_user_write_i16_le(&payload[6], packet->car_x_mm);
    core_link_user_write_i16_le(&payload[8], packet->car_y_mm);
    core_link_user_write_i16_le(&payload[10], packet->uav_yaw_ddeg);
    core_link_user_write_i16_le(&payload[12], packet->car_yaw_rel_uav_ddeg);
    for(i = 0U; i < packet->count; ++i)
    {
        uint8 offset = (uint8)(CORE_LINK_BEACON_OBSERVATION_HEADER_LEN +
                               i * CORE_LINK_BEACON_OBSERVATION_WIRE_LEN);

        payload[offset] = packet->observation[i].track_id;
        core_link_user_write_i16_le(&payload[offset + 1U], packet->observation[i].x_mm);
        core_link_user_write_i16_le(&payload[offset + 3U], packet->observation[i].y_mm);
        payload[offset + 5U] = packet->observation[i].detection_confidence;
        payload[offset + 6U] = packet->observation[i].position_sigma_cm;
        payload[offset + 7U] = packet->observation[i].flags;
    }

    payload_len = (uint8)(CORE_LINK_BEACON_OBSERVATION_HEADER_LEN +
                          packet->count * CORE_LINK_BEACON_OBSERVATION_WIRE_LEN);
    core_link_send_frame(CORE_LINK_PROTOCOL_BEACON_OBSERVATIONS,
                         payload, payload_len);
}

static void core_link_user_build_beacon_observations(
    core_link_beacon_observation_packet_struct *packet)
{
#if (VISION_FUSION_OUTPUT_MODE == VISION_FUSION_OUTPUT_LIVE)
    DualCameraFusionSnapshot_t fusion_snapshot = {0};
#else
    VehicleMarkerLossDebug_t loss_debug = {0};
    VisionRuntimeBeaconSlot_t slots[VISION_RUNTIME_BEACON_SLOT_COUNT] = {0};
    uint8 slot_track_id[VISION_RUNTIME_BEACON_SLOT_COUNT] = {0};
#endif
    float car_absolute_yaw_deg = 0.0f;
    float car_yaw_rel_uav_deg = 0.0f;
#if (VISION_FUSION_OUTPUT_MODE == VISION_FUSION_OUTPUT_SHADOW)
    float uav_yaw_deg = 0.0f;
    float follow_x_cm = 0.0f;
    float follow_y_cm = 0.0f;
    uint8 uav_yaw_valid;
    uint8 car_visible;
    uint8 follow_valid;
#endif
    uint8 i;

    if(NULL == packet)
    {
        return;
    }

    memset(packet, 0, sizeof(*packet));
    packet->sequence = ++s_core_link_user_beacon_sequence;
#if (VISION_FUSION_OUTPUT_MODE == VISION_FUSION_OUTPUT_LIVE)
    DualCameraFusion_GetSnapshot(&fusion_snapshot);
    if(fusion_snapshot.tracker_reset_count !=
       s_core_link_user_live_fusion_reset_seen)
    {
        packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_TRACKER_RESET;
        s_core_link_user_live_fusion_reset_seen =
            fusion_snapshot.tracker_reset_count;
        s_core_link_user_tracker_reset_pending = 0U;
    }
    if(fusion_snapshot.valid)
    {
        packet->car_visible = fusion_snapshot.car_valid;
        packet->car_x_mm = core_link_user_cm_to_mm_i16(fusion_snapshot.car_x_cm);
        packet->car_y_mm = core_link_user_cm_to_mm_i16(fusion_snapshot.car_y_cm);
        if(fusion_snapshot.car_valid)
        {
            packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_CAR_POSITION_VALID;
        }
        if(fusion_snapshot.car_raw_valid)
        {
            packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_CAR_RAW_DETECTED;
        }
        if(fusion_snapshot.yaw_valid)
        {
            packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_UAV_YAW_VALID;
            packet->uav_yaw_ddeg = core_link_user_deg_to_ddeg_i16(
                fusion_snapshot.yaw_deg);
        }
        if(fusion_snapshot.car_valid && fusion_snapshot.yaw_valid &&
           core_link_user_get_uplink_car_absolute_yaw_deg(&car_absolute_yaw_deg))
        {
            car_yaw_rel_uav_deg = core_link_user_wrap_angle(
                car_absolute_yaw_deg - fusion_snapshot.yaw_deg);
            packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_CAR_YAW_VALID;
            packet->car_yaw_rel_uav_ddeg =
                core_link_user_deg_to_ddeg_i16(car_yaw_rel_uav_deg);
        }
        if(!Flight_Takeoff_Is_Active() &&
           !BeaconSearch_IsPausedByBlindBoxTask())
        {
            for(i = 0U; (i < fusion_snapshot.target_count) &&
                        (packet->count < CORE_LINK_BEACON_OBSERVATION_MAX_COUNT);
                ++i)
            {
                BeaconObservation *observation =
                    &packet->observation[packet->count];
                uint8 sigma_cm = fusion_snapshot.target[i].position_sigma_cm;

                if(!fusion_snapshot.target[i].valid ||
                   (fusion_snapshot.target[i].track_id == 0U)) continue;
                if(core_link_user_beacon_overlaps_car(
                       fusion_snapshot.target[i].x_cm,
                       fusion_snapshot.target[i].y_cm,
                       fusion_snapshot.car_valid,
                       fusion_snapshot.car_x_cm,
                       fusion_snapshot.car_y_cm))
                {
                    continue;
                }
                if(sigma_cm < CORE_LINK_BEACON_POSITION_SIGMA_MIN_CM)
                    sigma_cm = CORE_LINK_BEACON_POSITION_SIGMA_MIN_CM;
                else if(sigma_cm > CORE_LINK_BEACON_POSITION_SIGMA_MAX_CM)
                    sigma_cm = CORE_LINK_BEACON_POSITION_SIGMA_MAX_CM;
                observation->track_id = fusion_snapshot.target[i].track_id;
                observation->x_mm = core_link_user_cm_to_mm_i16(
                    fusion_snapshot.target[i].x_cm);
                observation->y_mm = core_link_user_cm_to_mm_i16(
                    fusion_snapshot.target[i].y_cm);
                observation->detection_confidence =
                    fusion_snapshot.target[i].detection_confidence;
                observation->position_sigma_cm = sigma_cm;
                observation->flags = CORE_LINK_BEACON_OBSERVATION_FLAG_LIT;
                packet->count++;
            }
        }
        if(s_core_link_user_tracker_reset_pending)
        {
            packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_TRACKER_RESET;
            s_core_link_user_tracker_reset_pending = 0U;
        }
        BeaconSearch_OnObservationFrame(packet,
                                        (uint32)fusion_snapshot.reference_frame_seq);
        packet->frame_flags |= BeaconSearch_GetFrameFlags();
        return;
    }
    if(s_core_link_user_tracker_reset_pending)
    {
        packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_TRACKER_RESET;
        s_core_link_user_tracker_reset_pending = 0U;
    }
    BeaconSearch_OnObservationFrame(packet, 0U);
    packet->frame_flags |= BeaconSearch_GetFrameFlags();
    return;
#else
    VisionRuntime_GetBeaconSlots(slots, VISION_RUNTIME_BEACON_SLOT_COUNT);
    if(Flight_Takeoff_Is_Active() ||
       BeaconSearch_IsPausedByBlindBoxTask())
    {
        memset(slots, 0, sizeof(slots));
    }
    core_link_user_update_beacon_tracks(slots, slot_track_id);
    VehicleMarkerLocalization_GetLossDebug(&loss_debug);
    follow_valid = VehicleMarkerLocalization_GetFollowPoint(
        &g_vehicle_marker_pose, &follow_x_cm, &follow_y_cm, NULL, NULL);

    car_visible = (uint8)(g_vehicle_marker_pose.valid &&
                          !g_vehicle_marker_position_lost_flag &&
                          follow_valid);
    uav_yaw_valid = CrossCoreCamera_ControlCoreGetVisionYaw(&uav_yaw_deg);
    if(uav_yaw_valid)
    {
        packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_UAV_YAW_VALID;
        packet->uav_yaw_ddeg = core_link_user_deg_to_ddeg_i16(uav_yaw_deg);
    }
    if(car_visible)
    {
        packet->car_visible = 1U;
        packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_CAR_POSITION_VALID;
        if(loss_debug.raw_detect_valid)
        {
            packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_CAR_RAW_DETECTED;
        }
        if(follow_valid)
        {
            packet->car_x_mm = core_link_user_cm_to_mm_i16(follow_x_cm);
            packet->car_y_mm = core_link_user_cm_to_mm_i16(follow_y_cm);
        }
        if(uav_yaw_valid &&
           core_link_user_get_uplink_car_absolute_yaw_deg(&car_absolute_yaw_deg))
        {
            /* Use IMU yaw on both sides. CrossCoreCamera returns the UAV IMU
             * yaw sampled with this vision frame; it is not a visual heading.
             * Do not replace this calculation with
             * g_vehicle_marker_pose.heading_deg: marker-angle noise is
             * unsuitable for navigation heading. Both IMUs require the same
             * sign and aligned yaw zero. */
            car_yaw_rel_uav_deg = core_link_user_wrap_angle(
                car_absolute_yaw_deg - uav_yaw_deg);
            packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_CAR_YAW_VALID;
            packet->car_yaw_rel_uav_ddeg =
                core_link_user_deg_to_ddeg_i16(car_yaw_rel_uav_deg);
        }
    }
    if(s_core_link_user_tracker_reset_pending)
    {
        packet->frame_flags |= CORE_LINK_BEACON_FRAME_FLAG_TRACKER_RESET;
        s_core_link_user_tracker_reset_pending = 0U;
    }

    for(i = 0U; (i < VISION_RUNTIME_BEACON_SLOT_COUNT) &&
                (packet->count < CORE_LINK_BEACON_OBSERVATION_MAX_COUNT); ++i)
    {
        BeaconObservation *observation;
        uint8 sigma_cm;

        if(!slots[i].valid || (0U == slot_track_id[i]))
        {
            continue;
        }
        if(core_link_user_beacon_overlaps_car(slots[i].x_cm,
                                              slots[i].y_cm,
                                              car_visible,
                                              follow_x_cm,
                                              follow_y_cm))
        {
            continue;
        }
        observation = &packet->observation[packet->count];
        sigma_cm = slots[i].position_sigma_cm;
        if(sigma_cm < CORE_LINK_BEACON_POSITION_SIGMA_MIN_CM)
        {
            sigma_cm = CORE_LINK_BEACON_POSITION_SIGMA_MIN_CM;
        }
        else if(sigma_cm > CORE_LINK_BEACON_POSITION_SIGMA_MAX_CM)
        {
            sigma_cm = CORE_LINK_BEACON_POSITION_SIGMA_MAX_CM;
        }
        observation->track_id = slot_track_id[i];
        observation->x_mm = core_link_user_cm_to_mm_i16(slots[i].x_cm);
        observation->y_mm = core_link_user_cm_to_mm_i16(slots[i].y_cm);
        observation->detection_confidence = slots[i].detection_confidence;
        observation->position_sigma_cm = sigma_cm;
        observation->flags = CORE_LINK_BEACON_OBSERVATION_FLAG_LIT;
        packet->count++;
    }

    BeaconSearch_OnObservationFrame(packet, loss_debug.process_count);
    packet->frame_flags |= BeaconSearch_GetFrameFlags();
#endif
}

static void core_link_user_send_beacon_observations_if_due(void)
{
    core_link_beacon_observation_packet_struct packet;

    if(s_core_link_user_beacon_tx_elapsed_ms < CORE_LINK_USER_BEACON_OBSERVATION_PERIOD_MS)
    {
        return;
    }

    s_core_link_user_beacon_tx_elapsed_ms -= CORE_LINK_USER_BEACON_OBSERVATION_PERIOD_MS;
#if (VISION_FUSION_OUTPUT_MODE == VISION_FUSION_OUTPUT_SHADOW) || \
    (VISION_FUSION_OUTPUT_MODE == VISION_FUSION_OUTPUT_LIVE)
    DualCameraFusion_BuildSnapshot(CameraBoardLink_TimeNowUs());
#endif
    core_link_user_build_beacon_observations(&packet);
    core_link_user_send_beacon_observations(&packet);
}

static void core_link_user_push_key_event (uint8 key)
{
    uint8 next_tail = 0;

    if(0 == key)
    {
        return;
    }

    next_tail = core_link_user_queue_next_index(s_core_link_user_key_queue_tail);
    if(next_tail == s_core_link_user_key_queue_head)
    {
        return;
    }

    s_core_link_user_key_queue[s_core_link_user_key_queue_tail] = key;
    s_core_link_user_key_queue_tail = next_tail;
}

static uint8 core_link_user_pop_key_event (uint8 *key)
{
    if((NULL == key) || (s_core_link_user_key_queue_head == s_core_link_user_key_queue_tail))
    {
        return 0;
    }

    *key = s_core_link_user_key_queue[s_core_link_user_key_queue_head];
    s_core_link_user_key_queue_head = core_link_user_queue_next_index(s_core_link_user_key_queue_head);

    return 1;
}

static void core_link_user_clear_remote_events(void)
{
    s_core_link_user_key_queue_head = 0;
    s_core_link_user_key_queue_tail = 0;
    s_core_link_user_monitor_exit_request = 0;
    memset((void *)s_core_link_user_key_queue, 0, sizeof(s_core_link_user_key_queue));
}

void core_link_user_init (void)
{
    BeaconSearch_Init();
    BlindBoxTasks_Init();
    core_link_user_clear_remote_events();
    s_core_link_user_key_flag_remote = 0;
    s_core_link_user_screen_mirror_enabled = 0;
    s_core_link_user_screen_mirror_enabled_changed = 0;
    s_core_link_user_beacon_tx_elapsed_ms = 0U;
    s_core_link_user_car_trajectory_age_ms = 60000U;
    s_core_link_user_car_trajectory_protocol_error_count = 0U;
    s_core_link_user_car_trajectory_valid = 0U;
    s_core_link_user_car_trajectory_write_seq = 0U;
    s_core_link_user_car_sequence_valid = 0U;
    s_core_link_user_car_last_sequence = 0U;
    s_core_link_user_car_last_sample_time_ms = 0U;
    s_core_link_user_car_arrival_interval_ms = 0U;
    s_core_link_user_car_smooth_vx_cms = 0.0f;
    s_core_link_user_car_smooth_vy_cms = 0.0f;
    s_core_link_user_car_lpf_ax_cms2 = 0.0f;
    s_core_link_user_car_lpf_ay_cms2 = 0.0f;
    s_core_link_user_time_ms = 0U;
    s_core_link_user_beacon_sequence = 0U;
    s_core_link_user_next_track_id = 1U;
    s_core_link_user_tracker_reset_pending = 1U;
    s_core_link_user_live_fusion_reset_seen = 0U;
    DualCameraFusion_Init();
    memset(s_core_link_user_beacon_tracks, 0,
           sizeof(s_core_link_user_beacon_tracks));
    memset((void *)&s_core_link_user_car_trajectory, 0,
           sizeof(s_core_link_user_car_trajectory));
    memset(s_core_link_user_text_receive, 0, sizeof(s_core_link_user_text_receive));
}

void core_link_user_task (void)
{
    uint8 key = 0;

    if(0 != key_flag)
    {
        return;
    }

    s_core_link_user_key_flag_remote = 0;
    if(core_link_user_pop_key_event(&key))
    {
        key_flag = key;
        s_core_link_user_key_flag_remote = 1;
    }
}

void core_link_user_poll (void)
{
    core_link_task();
    core_link_user_task();
}

void core_link_user_tick (uint16 elapsed_ms)
{
    BeaconSearchStabilityInput_t search_stability = {0};
    PMWFlowState_t flow_state = {0};
    VehicleMarkerLossDebug_t marker_loss = {0};
    core_link_car_trajectory_packet_struct trajectory_snapshot = {0};
    float follow_x_cm = 0.0f;
    float follow_y_cm = 0.0f;
    uint8 follow_valid;
    uint8 car_link_valid = 0U;
    uint8 car_search_requested = 0U;
    uint16 car_link_age_ms = 60000U;

    if(s_core_link_user_beacon_tx_elapsed_ms <= (uint16)(60000U - elapsed_ms))
    {
        s_core_link_user_beacon_tx_elapsed_ms += elapsed_ms;
    }
    else
    {
        s_core_link_user_beacon_tx_elapsed_ms = 60000U;
    }

    car_link_valid = (uint8)(s_core_link_user_car_trajectory_valid &&
        (s_core_link_user_car_trajectory_age_ms <=
         CORE_LINK_USER_MOTION_STALE_MS) &&
        core_link_user_get_car_trajectory_snapshot(&trajectory_snapshot));
    if(car_link_valid)
    {
        car_link_age_ms = s_core_link_user_car_trajectory_age_ms;
        car_search_requested = (uint8)(
            (trajectory_snapshot.flags &
             CORE_LINK_CAR_TRAJECTORY_FLAG_VELOCITY_VALID) &&
            (trajectory_snapshot.flags &
             CORE_LINK_CAR_TRAJECTORY_FLAG_SEARCH_REQUEST));
    }

    PMW_Flow_GetState(&flow_state);
    VehicleMarkerLocalization_GetLossDebug(&marker_loss);
    follow_valid = VehicleMarkerLocalization_GetFollowPoint(
        &g_vehicle_marker_pose,
        &follow_x_cm, &follow_y_cm,
        NULL, NULL);
    search_stability.vision_process_count = marker_loss.process_count;
    search_stability.position_loop_enabled = Position_Loop_Is_Enabled();
    search_stability.position_valid = (uint8)(marker_loss.raw_detect_valid &&
                                               marker_loss.pose_valid &&
                                               !marker_loss.lost_flag &&
                                               follow_valid);
    search_stability.fused_velocity_valid = flow_state.fused_velocity_valid;
    search_stability.position_error_x_cm = follow_x_cm;
    search_stability.position_error_y_cm = follow_y_cm;
    search_stability.fused_velocity_x_cms = flow_state.velocity_x_fused_cms;
    search_stability.fused_velocity_y_cms = flow_state.velocity_y_fused_cms;

    BeaconSearch_Tick(elapsed_ms,
                      armed,
                      (uint8)(control_mode == CONTROL_MODE_ANGLE),
                      (uint8)!Flight_Takeoff_Is_Active(),
                      Velocity_Loop_Is_Enabled(),
                      car_link_valid,
                      car_link_age_ms,
                      car_search_requested,
                      current_yaw,
                      &search_stability);
    BlindBoxTasks_Tick(elapsed_ms);
}

void core_link_user_motion_tick(uint16 elapsed_ms)
{
    core_link_car_trajectory_packet_struct trajectory = {0};
    float target_vx_cms = 0.0f;
    float target_vy_cms = 0.0f;
    float target_ax_cms2 = 0.0f;
    float target_ay_cms2 = 0.0f;
    float dt_s;
    float lpf_term;
    float lpf_alpha;

    s_core_link_user_time_ms += elapsed_ms;
    if(s_core_link_user_car_trajectory_age_ms <=
       (uint16)(60000U - elapsed_ms))
    {
        s_core_link_user_car_trajectory_age_ms += elapsed_ms;
    }
    else
    {
        s_core_link_user_car_trajectory_age_ms = 60000U;
    }
    if(s_core_link_user_car_trajectory_valid &&
       (s_core_link_user_car_trajectory_age_ms <=
        CORE_LINK_USER_MOTION_STALE_MS) &&
       core_link_user_get_car_trajectory_snapshot(&trajectory))
    {
        if(trajectory.flags & CORE_LINK_CAR_TRAJECTORY_FLAG_VELOCITY_VALID)
        {
            target_vx_cms = (float)trajectory.planned_world_vx_dcms * 0.1f;
            target_vy_cms = (float)trajectory.planned_world_vy_dcms * 0.1f;
        }
        if(trajectory.flags & CORE_LINK_CAR_TRAJECTORY_FLAG_ACCEL_VALID)
        {
            target_ax_cms2 = (float)trajectory.planned_world_ax_cms2;
            target_ay_cms2 = (float)trajectory.planned_world_ay_cms2;
        }
    }

    dt_s = (float)elapsed_ms * 0.001f;
    core_link_user_slew_vector(target_vx_cms,
                               target_vy_cms,
                               CORE_LINK_USER_PLAN_VEL_SLEW_ACCEL_CMS2 * dt_s,
                               &s_core_link_user_car_smooth_vx_cms,
                               &s_core_link_user_car_smooth_vy_cms);
    lpf_term = CORE_LINK_USER_TWO_PI * CORE_LINK_USER_PLAN_ACCEL_LPF_HZ *
               dt_s;
    lpf_alpha = lpf_term / (1.0f + lpf_term);
    s_core_link_user_car_lpf_ax_cms2 += lpf_alpha *
        (target_ax_cms2 - s_core_link_user_car_lpf_ax_cms2);
    s_core_link_user_car_lpf_ay_cms2 += lpf_alpha *
        (target_ay_cms2 - s_core_link_user_car_lpf_ay_cms2);
}

void core_link_user_get_car_motion_diag(CoreLinkCarMotionDiag_t *diag)
{
    core_link_car_trajectory_packet_struct trajectory = {0};
    uint8 velocity_source_valid;
    uint8 acceleration_source_valid;
    uint8 output_active;

    if(NULL == diag)
    {
        return;
    }

    memset(diag, 0, sizeof(*diag));
    diag->protocol_error_count =
        s_core_link_user_car_trajectory_protocol_error_count;
    if(!s_core_link_user_car_trajectory_valid ||
       !core_link_user_get_car_trajectory_snapshot(&trajectory))
    {
        diag->age_ms = 65535U;
        return;
    }

    diag->age_ms = s_core_link_user_car_trajectory_age_ms;
    diag->arrival_interval_ms = s_core_link_user_car_arrival_interval_ms;
    diag->flags = trajectory.flags;
    diag->sequence = trajectory.sequence;
    diag->sample_time_ms = trajectory.sample_time_ms;
    diag->raw_world_vx_cms =
        (float)trajectory.planned_world_vx_dcms * 0.1f;
    diag->raw_world_vy_cms =
        (float)trajectory.planned_world_vy_dcms * 0.1f;
    diag->raw_world_ax_cms2 = (float)trajectory.planned_world_ax_cms2;
    diag->raw_world_ay_cms2 = (float)trajectory.planned_world_ay_cms2;
    diag->world_vx_cms = s_core_link_user_car_smooth_vx_cms;
    diag->world_vy_cms = s_core_link_user_car_smooth_vy_cms;
    diag->world_ax_cms2 = s_core_link_user_car_lpf_ax_cms2;
    diag->world_ay_cms2 = s_core_link_user_car_lpf_ay_cms2;
    diag->car_yaw_deg = core_link_user_wrap_angle(
        (float)trajectory.car_absolute_yaw_ddeg * 0.1f);

    if(diag->age_ms <= CORE_LINK_USER_MOTION_STALE_MS)
    {
        diag->link_state = CORE_LINK_CAR_MOTION_STATE_FRESH;
    }
    else if(diag->age_ms <= CORE_LINK_USER_MOTION_INVALID_MS)
    {
        diag->link_state = CORE_LINK_CAR_MOTION_STATE_STALE;
    }
    else
    {
        diag->link_state = CORE_LINK_CAR_MOTION_STATE_INVALID;
    }
    velocity_source_valid = (uint8)(
        (diag->link_state == CORE_LINK_CAR_MOTION_STATE_FRESH) &&
        (trajectory.flags & CORE_LINK_CAR_TRAJECTORY_FLAG_VELOCITY_VALID));
    acceleration_source_valid = (uint8)(
        (diag->link_state == CORE_LINK_CAR_MOTION_STATE_FRESH) &&
        (trajectory.flags & CORE_LINK_CAR_TRAJECTORY_FLAG_ACCEL_VALID));
    output_active = (uint8)(
        (fabsf(diag->world_vx_cms) > CORE_LINK_USER_MOTION_OUTPUT_EPSILON) ||
        (fabsf(diag->world_vy_cms) > CORE_LINK_USER_MOTION_OUTPUT_EPSILON) ||
        (fabsf(diag->world_ax_cms2) > CORE_LINK_USER_MOTION_OUTPUT_EPSILON) ||
        (fabsf(diag->world_ay_cms2) > CORE_LINK_USER_MOTION_OUTPUT_EPSILON));
    diag->valid = (uint8)(velocity_source_valid || output_active);
    diag->acceleration_valid = (uint8)(diag->valid &&
        (acceleration_source_valid ||
         (fabsf(diag->world_ax_cms2) >
          CORE_LINK_USER_MOTION_OUTPUT_EPSILON) ||
         (fabsf(diag->world_ay_cms2) >
          CORE_LINK_USER_MOTION_OUTPUT_EPSILON)));
    diag->yaw_valid = (uint8)(
        (diag->link_state != CORE_LINK_CAR_MOTION_STATE_INVALID) &&
        (trajectory.flags & CORE_LINK_CAR_TRAJECTORY_FLAG_YAW_VALID));
    diag->search_requested = (uint8)(
        (diag->link_state != CORE_LINK_CAR_MOTION_STATE_INVALID) &&
        (trajectory.flags & CORE_LINK_CAR_TRAJECTORY_FLAG_SEARCH_REQUEST));
}

void core_link_user_service_beacon_link(void)
{
    DualCameraFusion_ServiceInputs();
    core_link_user_send_beacon_observations_if_due();
}

uint8 core_link_user_is_screen_mirror_enabled (void)
{
    return s_core_link_user_screen_mirror_enabled ? 1U : 0U;
}

uint8 core_link_user_consume_screen_mirror_enabled_changed (void)
{
    if(!s_core_link_user_screen_mirror_enabled_changed)
    {
        return 0U;
    }

    s_core_link_user_screen_mirror_enabled_changed = 0;
    return 1U;
}

uint8 core_link_user_consume_monitor_exit_request (void)
{
    if(!s_core_link_user_monitor_exit_request)
    {
        return 0;
    }

    s_core_link_user_monitor_exit_request = 0;
    return 1;
}

void core_link_user_send_drone_status_from_snapshot (const MonitorRuntimeSnapshot *snapshot)
{
    core_link_drone_status_packet_struct status = {0};

    if(NULL == snapshot)
    {
        return;
    }

    status.armed = snapshot->armed;
    status.control_mode = snapshot->control_mode;
    status.low_voltage = snapshot->low_voltage;
    status.beacon_count = (uint8)((snapshot->beacon_count < 0) ? 0 : snapshot->beacon_count);
    // status.voltage_mv = (uint16)(snapshot->voltage * 1000.0f);
    status.voltage_mv = (uint16)(snapshot->voltage * 1000.0f);
    status.target_throttle = (int16)snapshot->target_throttle;
    status.base_throttle = (int16)snapshot->base_throttle;
    status.roll_ddeg = (int16)(snapshot->roll * 10.0f);
    status.pitch_ddeg = (int16)(snapshot->pitch * 10.0f);
    status.yaw_ddeg = (int16)(snapshot->yaw * 10.0f);
    status.tof_height_dcm = (int16)(snapshot->tof_height_cm * 10.0f);
    status.fused_height_dcm = (int16)(snapshot->fused_height_cm * 10.0f);

    core_link_send_drone_status(&status);
}

void ips200_mirror_send_packet (const uint8 *payload, uint8 length)
{
    if(!s_core_link_user_screen_mirror_enabled)
    {
        return;
    }

    core_link_send_frame(CORE_LINK_PROTOCOL_SCREEN_CMD, payload, length);
}

void core_link_on_text_received (const uint8 *text, uint8 length)
{
    uint8 copy_length = 0;

    if(NULL == text)
    {
        return;
    }

    copy_length = (length < CORE_LINK_PAYLOAD_MAX_LEN) ? length : CORE_LINK_PAYLOAD_MAX_LEN;
    memcpy(s_core_link_user_text_receive, text, copy_length);
    s_core_link_user_text_receive[copy_length] = 0;
}

void core_link_on_param_received (const core_link_param_packet_struct *param)
{
    int32 adc_value = 0;
    uint8 next_screen_mirror_enabled = 0;

    if(NULL == param)
    {
        return;
    }

    if(('S' == param->command) && ('1' == param->item))
    {
        next_screen_mirror_enabled = (param->value >= CORE_LINK_USER_SCREEN_SWITCH_THRESHOLD) ? 1U : 0U;
        if(s_core_link_user_screen_mirror_enabled != next_screen_mirror_enabled)
        {
            s_core_link_user_screen_mirror_enabled = next_screen_mirror_enabled;
            s_core_link_user_screen_mirror_enabled_changed = 1U;
        }
        return;
    }

    if('B' == param->command && 'V' == param->item)
    {
        /* 小车发来的电池电压，单位 V。小车与飞机共用同一块电池，直接用于监视页显示。 */
        if(param->value >= 10.0f && param->value <= 18.0f)
        {
            BatteryMonitor_SetRemoteVoltage(param->value);
        }
        return;
    }

    if('A' == param->command)
    {
        adc_value = (int32)(param->value + ((param->value >= 0.0f) ? 0.5f : -0.5f));

        switch(param->item)
        {
            case '1':
                adc_x = (int)adc_value;
                break;

            case '2':
                adc_y = (int)adc_value;
                break;

            default:
                break;
        }

        return;
    }

    if(('K' != param->command) || (CORE_LINK_USER_KEY_ACTIVE_THRESHOLD > param->value))
    {
        return;
    }

    switch(param->item)
    {
        case '1':
            core_link_user_push_key_event(Button_1);
            break;

        case '2':
            core_link_user_push_key_event(Button_2);
            break;

        case '3':
            core_link_user_push_key_event(Button_3);
            break;

        case '4':
            core_link_user_push_key_event(Button_4);
            break;

        case 'M':
            if(menu_is_monitor_view())
            {
                s_core_link_user_monitor_exit_request = 1;
            }
            else
            {
                core_link_user_push_key_event(Button_4);
            }
            break;

        default:
            break;
    }
}

static void core_link_user_parse_car_trajectory(const uint8 *payload,
                                                 uint8 length)
{
    core_link_car_trajectory_packet_struct parsed;
    uint8 allowed_flags =
        CORE_LINK_CAR_TRAJECTORY_FLAG_VELOCITY_VALID |
        CORE_LINK_CAR_TRAJECTORY_FLAG_ACCEL_VALID |
        CORE_LINK_CAR_TRAJECTORY_FLAG_SEARCH_REQUEST |
        CORE_LINK_CAR_TRAJECTORY_FLAG_YAW_VALID;
    uint32 now_ms;
    float velocity_x;
    float velocity_y;
    float acceleration_x;
    float acceleration_y;

    if((NULL == payload) ||
       (CORE_LINK_CAR_TRAJECTORY_PAYLOAD_LEN != length) ||
       (CORE_LINK_CAR_TRAJECTORY_VERSION != payload[0]) ||
       (payload[1] & (uint8)~allowed_flags) ||
       ((payload[1] & CORE_LINK_CAR_TRAJECTORY_FLAG_ACCEL_VALID) &&
        !(payload[1] & CORE_LINK_CAR_TRAJECTORY_FLAG_VELOCITY_VALID)))
    {
        if(s_core_link_user_car_trajectory_protocol_error_count < 65535U)
        {
            s_core_link_user_car_trajectory_protocol_error_count++;
        }
        return;
    }

    memset(&parsed, 0, sizeof(parsed));
    parsed.version = payload[0];
    parsed.flags = payload[1];
    parsed.sequence = core_link_user_read_u16_le(&payload[2]);
    parsed.sample_time_ms = core_link_user_read_u32_le(&payload[4]);
    parsed.car_absolute_yaw_ddeg = core_link_user_read_i16_le(&payload[8]);
    parsed.planned_world_vx_dcms = core_link_user_read_i16_le(&payload[10]);
    parsed.planned_world_vy_dcms = core_link_user_read_i16_le(&payload[12]);
    parsed.planned_world_ax_cms2 = core_link_user_read_i16_le(&payload[14]);
    parsed.planned_world_ay_cms2 = core_link_user_read_i16_le(&payload[16]);

    velocity_x = (float)parsed.planned_world_vx_dcms * 0.1f;
    velocity_y = (float)parsed.planned_world_vy_dcms * 0.1f;
    acceleration_x = (float)parsed.planned_world_ax_cms2;
    acceleration_y = (float)parsed.planned_world_ay_cms2;
    if(((parsed.flags & CORE_LINK_CAR_TRAJECTORY_FLAG_YAW_VALID) &&
        ((parsed.car_absolute_yaw_ddeg < -1800) ||
         (parsed.car_absolute_yaw_ddeg > 1800))) ||
       (sqrtf(velocity_x * velocity_x + velocity_y * velocity_y) >
        CORE_LINK_USER_MOTION_SPEED_MAX_CMS) ||
       (sqrtf(acceleration_x * acceleration_x +
              acceleration_y * acceleration_y) >
        CORE_LINK_USER_MOTION_ACCEL_MAX_CMS2))
    {
        if(s_core_link_user_car_trajectory_protocol_error_count < 65535U)
        {
            s_core_link_user_car_trajectory_protocol_error_count++;
        }
        return;
    }

    if(s_core_link_user_car_sequence_valid &&
       (s_core_link_user_car_trajectory_age_ms <=
        CORE_LINK_USER_MOTION_INVALID_MS))
    {
        int16 sequence_delta = (int16)(parsed.sequence -
                                       s_core_link_user_car_last_sequence);
        int32 sample_delta_ms = (int32)(parsed.sample_time_ms -
                                        s_core_link_user_car_last_sample_time_ms);

        /* UART重发的完全重复包不刷新年龄，也不计为协议错误。 */
        if((sequence_delta == 0) && (sample_delta_ms == 0))
        {
            return;
        }
        if((sequence_delta <= 0) || (sample_delta_ms <= 0))
        {
            if(s_core_link_user_car_trajectory_protocol_error_count < 65535U)
            {
                s_core_link_user_car_trajectory_protocol_error_count++;
            }
            return;
        }
    }

    now_ms = s_core_link_user_time_ms;
    if(s_core_link_user_car_trajectory_valid)
    {
        s_core_link_user_car_arrival_interval_ms =
            core_link_user_motion_age_ms(
                s_core_link_user_car_trajectory.receive_time_ms, now_ms);
    }
    parsed.receive_time_ms = now_ms;

    s_core_link_user_car_trajectory_write_seq++;
    s_core_link_user_car_trajectory = parsed;
    s_core_link_user_car_trajectory_write_seq++;
    s_core_link_user_car_trajectory_valid = 1U;
    s_core_link_user_car_trajectory_age_ms = 0U;
    s_core_link_user_car_sequence_valid = 1U;
    s_core_link_user_car_last_sequence = parsed.sequence;
    s_core_link_user_car_last_sample_time_ms = parsed.sample_time_ms;
}

void core_link_on_frame_received (uint8 protocol_id, const uint8 *payload, uint8 length)
{
    if(CORE_LINK_PROTOCOL_CAR_TRAJECTORY == protocol_id)
    {
        core_link_user_parse_car_trajectory(payload, length);
    }
}
