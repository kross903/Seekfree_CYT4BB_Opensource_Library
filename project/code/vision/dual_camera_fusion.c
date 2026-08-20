#include "common/project_headfile.h"
#include "vision/vehicle_marker_localization.h"

#define DUAL_CAMERA_FUSION_CACHE_SIZE             (4U)
#define DUAL_CAMERA_FUSION_TRACK_COUNT            (12U)
#define DUAL_CAMERA_FUSION_MAX_CANDIDATES         (12U)
#define DUAL_CAMERA_FUSION_MAX_AGE_US             (300000U)
#define DUAL_CAMERA_FUSION_PAIR_GATE_US           (300000)
#define DUAL_CAMERA_FUSION_DEDUP_GATE_MIN_CM      (70.0f)
#define DUAL_CAMERA_FUSION_DEDUP_GATE_MAX_CM      (70.0f)
#define DUAL_CAMERA_FUSION_TIME_MARGIN_MAX_CM     (50.0f)
#define DUAL_CAMERA_FUSION_TRACK_GATE_MIN_CM      (30.0f)
#define DUAL_CAMERA_FUSION_TRACK_GATE_MAX_CM      (80.0f)
#define DUAL_CAMERA_FUSION_TRACK_MAX_MISS         (5U)
#define DUAL_CAMERA_FUSION_RESIDUAL_HISTORY_SIZE  (64U)
#define DUAL_CAMERA_FUSION_DEG_TO_RAD             (0.017453292519943295f)
/* Residual rear-board translation after each board's own camera extrinsic. */
#define DUAL_CAMERA_FUSION_REAR_RESIDUAL_X_CM     (20.0f)
#define DUAL_CAMERA_FUSION_REAR_RESIDUAL_Y_CM     (30.0f)

typedef struct
{
    uint8 valid;
    uint8 yaw_valid;
    uint8 car_valid;
    uint8 car_raw_valid;
    uint8 beacon_count;
    uint16 state_sequence;
    uint16 reset_generation;
    uint32 frame_seq;
    uint32 capture_time_us;
    float yaw_deg;
    float car_x_cm;
    float car_y_cm;
    CameraBoardBeaconObservation_t beacon[CAMERA_BOARD_MAX_BEACON_COUNT];
} DualCameraFusionFrame_t;

typedef struct
{
    uint8 valid;
    uint8 source;
    uint8 detection_confidence;
    uint8 position_sigma_cm;
    float x_cm;
    float y_cm;
    float dx_cm;
    float dy_cm;
} DualCameraFusionCandidate_t;

typedef struct
{
    uint8 valid;
    uint8 track_id;
    uint8 miss_count;
    uint8 position_sigma_cm;
    float x_cm;
    float y_cm;
} DualCameraFusionTrack_t;

static DualCameraFusionFrame_t s_front_cache[DUAL_CAMERA_FUSION_CACHE_SIZE];
static DualCameraFusionFrame_t s_rear_cache[DUAL_CAMERA_FUSION_CACHE_SIZE];
static uint8 s_front_head;
static uint8 s_front_count;
static uint8 s_rear_head;
static uint8 s_rear_count;
static uint32 s_last_front_service_seq;
static uint32 s_last_rear_service_seq;
static uint16 s_last_reset_generation;
static uint8 s_reset_generation_valid;
static DualCameraFusionTrack_t s_tracks[DUAL_CAMERA_FUSION_TRACK_COUNT];
static uint8 s_next_track_id;
static DualCameraFusionSnapshot_t s_snapshot;
static DualCameraFusionDiag_t s_diag;
static float s_residual_history[DUAL_CAMERA_FUSION_RESIDUAL_HISTORY_SIZE];
static uint8 s_residual_head;
static uint8 s_residual_count;

static float DualCameraFusion_Wrap180(float angle_deg)
{
    while(angle_deg > 180.0f) angle_deg -= 360.0f;
    while(angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static void DualCameraFusion_ApplySourceResidual(uint8 source_id,
                                                  float *x_cm,
                                                  float *y_cm)
{
    if((x_cm == NULL) || (y_cm == NULL)) return;
    if((source_id == DUAL_CAMERA_FUSION_SOURCE_REAR) ||
       (source_id == DUAL_CAMERA_FUSION_SOURCE_BOTH_REAR))
    {
        *x_cm -= DUAL_CAMERA_FUSION_REAR_RESIDUAL_X_CM;
        *y_cm -= DUAL_CAMERA_FUSION_REAR_RESIDUAL_Y_CM;
    }
}

static uint16 DualCameraFusion_AgeMs(uint32 now_us, uint32 capture_time_us)
{
    uint32 age_us;

    if(capture_time_us == 0U) return 0xFFFFU;
    age_us = (uint32)(now_us - capture_time_us);
    if(age_us / 1000U > 0xFFFFU) return 0xFFFFU;
    return (uint16)(age_us / 1000U);
}

static uint8 DualCameraFusion_FrameStrong(const DualCameraFusionFrame_t *frame,
                                           uint32 now_us)
{
    if((frame == NULL) || !frame->valid || !frame->yaw_valid ||
       (frame->capture_time_us == 0U) ||
       ((uint32)(now_us - frame->capture_time_us) >
        DUAL_CAMERA_FUSION_MAX_AGE_US))
    {
        return 0U;
    }
    return (uint8)isfinite(frame->yaw_deg);
}

static const DualCameraFusionFrame_t *DualCameraFusion_LatestFrame(
    const DualCameraFusionFrame_t cache[DUAL_CAMERA_FUSION_CACHE_SIZE],
    uint8 count,
    uint32 now_us,
    uint8 require_strong)
{
    const DualCameraFusionFrame_t *best = NULL;
    uint32 best_age = 0xFFFFFFFFU;
    uint8 i;

    for(i = 0U; i < count; i++)
    {
        const DualCameraFusionFrame_t *frame = &cache[i];
        uint32 age;

        if(!frame->valid || frame->capture_time_us == 0U) continue;
        age = (uint32)(now_us - frame->capture_time_us);
        if((age > DUAL_CAMERA_FUSION_MAX_AGE_US) ||
           (require_strong && !DualCameraFusion_FrameStrong(frame, now_us)))
        {
            continue;
        }
        if((best == NULL) || (age < best_age))
        {
            best = frame;
            best_age = age;
        }
    }
    return best;
}

static uint8 DualCameraFusion_AlignCarToReference(
    const DualCameraFusionFrame_t *source,
    const DualCameraFusionFrame_t *reference,
    uint8 source_id,
    float *aligned_x_cm, float *aligned_y_cm)
{
    float source_x_cm;
    float source_y_cm;
    float yaw_delta_rad;
    float cosine;
    float sine;

    if((source == NULL) || (reference == NULL) ||
       (aligned_x_cm == NULL) || (aligned_y_cm == NULL) ||
       !source->car_valid || !source->yaw_valid || !reference->yaw_valid ||
       !isfinite(source->car_x_cm) || !isfinite(source->car_y_cm) ||
       !isfinite(source->yaw_deg) || !isfinite(reference->yaw_deg))
    {
        return 0U;
    }

    yaw_delta_rad = DualCameraFusion_Wrap180(source->yaw_deg -
                                             reference->yaw_deg) *
                    DUAL_CAMERA_FUSION_DEG_TO_RAD;
    cosine = cosf(yaw_delta_rad);
    sine = sinf(yaw_delta_rad);
    source_x_cm = source->car_x_cm;
    source_y_cm = source->car_y_cm;
    DualCameraFusion_ApplySourceResidual(source_id,
                                         &source_x_cm, &source_y_cm);
    *aligned_x_cm = cosine * source_x_cm - sine * source_y_cm;
    *aligned_y_cm = sine * source_x_cm + cosine * source_y_cm;
    return 1U;
}

static void DualCameraFusion_ResetTracks(void)
{
    memset(s_tracks, 0, sizeof(s_tracks));
    s_next_track_id = 1U;
    s_diag.tracker_reset_count++;
}

static uint8 DualCameraFusion_TrackIdInUse(uint8 track_id)
{
    uint8 i;
    for(i = 0U; i < DUAL_CAMERA_FUSION_TRACK_COUNT; i++)
    {
        if(s_tracks[i].valid && (s_tracks[i].track_id == track_id)) return 1U;
    }
    return 0U;
}

static uint8 DualCameraFusion_AllocateTrackId(void)
{
    uint16 attempt;
    for(attempt = 0U; attempt < 255U; attempt++)
    {
        uint8 id = s_next_track_id++;
        if(s_next_track_id == 0U) s_next_track_id = 1U;
        if((id != 0U) && !DualCameraFusion_TrackIdInUse(id)) return id;
    }
    return 0U;
}

static void DualCameraFusion_AddResidual(float residual_cm)
{
    s_residual_history[s_residual_head] = residual_cm;
    s_residual_head = (uint8)((s_residual_head + 1U) %
                              DUAL_CAMERA_FUSION_RESIDUAL_HISTORY_SIZE);
    if(s_residual_count < DUAL_CAMERA_FUSION_RESIDUAL_HISTORY_SIZE)
    {
        s_residual_count++;
    }
    if(residual_cm > s_diag.duplicate_residual_max_cm)
    {
        s_diag.duplicate_residual_max_cm = residual_cm;
    }
}

static void DualCameraFusion_UpdateResidualPercentiles(void)
{
    float sorted[DUAL_CAMERA_FUSION_RESIDUAL_HISTORY_SIZE];
    uint8 i;

    if(s_residual_count == 0U)
    {
        s_diag.duplicate_residual_median_cm = 0.0f;
        s_diag.duplicate_residual_p95_cm = 0.0f;
        return;
    }
    memcpy(sorted, s_residual_history, sizeof(float) * s_residual_count);
    for(i = 1U; i < s_residual_count; i++)
    {
        float value = sorted[i];
        uint8 j = i;
        while((j > 0U) && (sorted[j - 1U] > value))
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = value;
    }
    s_diag.duplicate_residual_median_cm = sorted[s_residual_count / 2U];
    s_diag.duplicate_residual_p95_cm =
        sorted[(uint8)(((uint16)s_residual_count * 95U + 99U) / 100U - 1U)];
}

static void DualCameraFusion_AddFrameBeacon(
    const DualCameraFusionFrame_t *source,
    const DualCameraFusionFrame_t *reference,
    uint8 source_id,
    uint8 reference_source_id,
    uint8 beacon_index,
    DualCameraFusionCandidate_t *candidate)
{
    float source_x;
    float source_y;
    float reference_car_x;
    float reference_car_y;
    float yaw_delta_rad;
    float cosine;
    float sine;

    memset(candidate, 0, sizeof(*candidate));
    source_x = (float)source->beacon[beacon_index].x_mm * 0.1f;
    source_y = (float)source->beacon[beacon_index].y_mm * 0.1f;
    DualCameraFusion_ApplySourceResidual(source_id, &source_x, &source_y);
    yaw_delta_rad = DualCameraFusion_Wrap180(source->yaw_deg -
                                             reference->yaw_deg) *
                    DUAL_CAMERA_FUSION_DEG_TO_RAD;
    cosine = cosf(yaw_delta_rad);
    sine = sinf(yaw_delta_rad);
    /* Align axes by yaw; horizontal translation between captures is ignored. */
    candidate->x_cm = cosine * source_x - sine * source_y;
    candidate->y_cm = sine * source_x + cosine * source_y;
    if(reference->car_valid && isfinite(reference->car_x_cm) &&
       isfinite(reference->car_y_cm))
    {
        reference_car_x = reference->car_x_cm;
        reference_car_y = reference->car_y_cm;
        DualCameraFusion_ApplySourceResidual(reference_source_id,
                                             &reference_car_x,
                                             &reference_car_y);
        candidate->dx_cm = candidate->x_cm - reference_car_x;
        candidate->dy_cm = candidate->y_cm - reference_car_y;
    }
    candidate->detection_confidence =
        source->beacon[beacon_index].detection_confidence;
    candidate->position_sigma_cm =
        source->beacon[beacon_index].position_sigma_cm;
    if(candidate->position_sigma_cm == 0U)
    {
        candidate->position_sigma_cm = 15U;
    }
    candidate->source = source_id;
    candidate->valid = 1U;
}

static uint8 DualCameraFusion_IsFrontBetter(
    const DualCameraFusionCandidate_t *front,
    const DualCameraFusionCandidate_t *rear)
{
    if(front->position_sigma_cm != rear->position_sigma_cm)
    {
        return (front->position_sigma_cm < rear->position_sigma_cm) ? 1U : 0U;
    }
    if(front->detection_confidence != rear->detection_confidence)
    {
        return (front->detection_confidence > rear->detection_confidence) ? 1U : 0U;
    }
    return 1U;
}

static uint8 DualCameraFusion_QualityBefore(
    const DualCameraFusionCandidate_t *left,
    const DualCameraFusionCandidate_t *right)
{
    if(left->position_sigma_cm != right->position_sigma_cm)
    {
        return (left->position_sigma_cm < right->position_sigma_cm) ? 1U : 0U;
    }
    if(left->detection_confidence != right->detection_confidence)
    {
        return (left->detection_confidence > right->detection_confidence) ? 1U : 0U;
    }
    return (uint8)((left->source == DUAL_CAMERA_FUSION_SOURCE_FRONT) ||
                   (left->source == DUAL_CAMERA_FUSION_SOURCE_BOTH_FRONT));
}

static uint8 DualCameraFusion_BuildCandidates(
    const DualCameraFusionFrame_t *front,
    const DualCameraFusionFrame_t *rear,
    const DualCameraFusionFrame_t *reference,
    uint8 reference_source_id,
    uint32 pair_abs_dt_us,
    DualCameraFusionCandidate_t output[DUAL_CAMERA_FUSION_MAX_CANDIDATES])
{
    DualCameraFusionCandidate_t front_candidate[CAMERA_BOARD_MAX_BEACON_COUNT];
    DualCameraFusionCandidate_t rear_candidate[CAMERA_BOARD_MAX_BEACON_COUNT];
    uint8 front_used[CAMERA_BOARD_MAX_BEACON_COUNT] = {0};
    uint8 rear_used[CAMERA_BOARD_MAX_BEACON_COUNT] = {0};
    uint8 front_count = 0U;
    uint8 rear_count = 0U;
    uint8 output_count = 0U;
    uint8 i;

    if(front != NULL)
    {
        for(i = 0U; i < front->beacon_count; i++)
        {
            DualCameraFusion_AddFrameBeacon(front, reference,
                DUAL_CAMERA_FUSION_SOURCE_FRONT, reference_source_id, i,
                &front_candidate[front_count++]);
        }
    }
    if(rear != NULL)
    {
        for(i = 0U; i < rear->beacon_count; i++)
        {
            DualCameraFusion_AddFrameBeacon(rear, reference,
                DUAL_CAMERA_FUSION_SOURCE_REAR, reference_source_id, i,
                &rear_candidate[rear_count++]);
        }
    }

    for(;;)
    {
        uint8 best_front = 0xFFU;
        uint8 best_rear = 0xFFU;
        float best_distance_sq = 0.0f;
        uint8 r;

        for(i = 0U; i < front_count; i++)
        {
            if(front_used[i]) continue;
            for(r = 0U; r < rear_count; r++)
            {
                float dx;
                float dy;
                float distance_sq;
                float sigma_combined;
                float time_margin_cm;
                float gate_cm;

                if(rear_used[r]) continue;
                dx = front_candidate[i].x_cm - rear_candidate[r].x_cm;
                dy = front_candidate[i].y_cm - rear_candidate[r].y_cm;
                distance_sq = dx * dx + dy * dy;
                sigma_combined = sqrtf(
                    (float)front_candidate[i].position_sigma_cm *
                    (float)front_candidate[i].position_sigma_cm +
                    (float)rear_candidate[r].position_sigma_cm *
                    (float)rear_candidate[r].position_sigma_cm);
                gate_cm = 3.0f * sigma_combined;
                if(gate_cm < DUAL_CAMERA_FUSION_DEDUP_GATE_MIN_CM)
                    gate_cm = DUAL_CAMERA_FUSION_DEDUP_GATE_MIN_CM;
                /* Linearly absorb UAV translation over the accepted pair gap. */
                time_margin_cm = DUAL_CAMERA_FUSION_TIME_MARGIN_MAX_CM *
                    (float)pair_abs_dt_us /
                    (float)DUAL_CAMERA_FUSION_PAIR_GATE_US;
                gate_cm += time_margin_cm;
                if(gate_cm > DUAL_CAMERA_FUSION_DEDUP_GATE_MAX_CM)
                    gate_cm = DUAL_CAMERA_FUSION_DEDUP_GATE_MAX_CM;
                if((distance_sq <= gate_cm * gate_cm) &&
                   ((best_front == 0xFFU) || (distance_sq < best_distance_sq)))
                {
                    best_front = i;
                    best_rear = r;
                    best_distance_sq = distance_sq;
                }
            }
        }
        if(best_front == 0xFFU) break;

        front_used[best_front] = 1U;
        rear_used[best_rear] = 1U;
        if(DualCameraFusion_IsFrontBetter(&front_candidate[best_front],
                                          &rear_candidate[best_rear]))
        {
            output[output_count] = front_candidate[best_front];
            output[output_count].source = DUAL_CAMERA_FUSION_SOURCE_BOTH_FRONT;
        }
        else
        {
            output[output_count] = rear_candidate[best_rear];
            output[output_count].source = DUAL_CAMERA_FUSION_SOURCE_BOTH_REAR;
        }
        s_diag.duplicate_residual_cm = sqrtf(best_distance_sq);
        DualCameraFusion_AddResidual(s_diag.duplicate_residual_cm);
        s_diag.duplicate_count++;
        s_diag.duplicate_total_count++;
        output_count++;
    }

    for(i = 0U; i < front_count; i++)
    {
        if(!front_used[i]) output[output_count++] = front_candidate[i];
    }
    for(i = 0U; i < rear_count; i++)
    {
        if(!rear_used[i]) output[output_count++] = rear_candidate[i];
    }
    return output_count;
}

static void DualCameraFusion_SortCandidates(
    DualCameraFusionCandidate_t candidate[DUAL_CAMERA_FUSION_MAX_CANDIDATES],
    uint8 count)
{
    uint8 i;
    for(i = 1U; i < count; i++)
    {
        DualCameraFusionCandidate_t value = candidate[i];
        uint8 j = i;
        while((j > 0U) && DualCameraFusion_QualityBefore(&value,
                                                         &candidate[j - 1U]))
        {
            candidate[j] = candidate[j - 1U];
            j--;
        }
        candidate[j] = value;
    }
}

static void DualCameraFusion_UpdateTracks(
    const DualCameraFusionCandidate_t candidate[DUAL_CAMERA_FUSION_MAX_CANDIDATES],
    uint8 count,
    uint8 track_id[DUAL_CAMERA_FUSION_MAX_TARGETS])
{
    uint8 candidate_used[DUAL_CAMERA_FUSION_MAX_TARGETS] = {0};
    uint8 track_used[DUAL_CAMERA_FUSION_TRACK_COUNT] = {0};
    uint8 i;

    memset(track_id, 0, DUAL_CAMERA_FUSION_MAX_TARGETS);
    for(;;)
    {
        uint8 best_candidate = 0xFFU;
        uint8 best_track = 0xFFU;
        float best_distance_sq = 0.0f;
        uint8 track;

        for(i = 0U; i < count; i++)
        {
            if(candidate_used[i]) continue;
            for(track = 0U; track < DUAL_CAMERA_FUSION_TRACK_COUNT; track++)
            {
                float dx;
                float dy;
                float distance_sq;
                float gate_cm;
                float sigma;

                if(track_used[track] || !s_tracks[track].valid) continue;
                dx = candidate[i].x_cm - s_tracks[track].x_cm;
                dy = candidate[i].y_cm - s_tracks[track].y_cm;
                distance_sq = dx * dx + dy * dy;
                sigma = sqrtf((float)candidate[i].position_sigma_cm *
                              (float)candidate[i].position_sigma_cm +
                              (float)s_tracks[track].position_sigma_cm *
                              (float)s_tracks[track].position_sigma_cm);
                gate_cm = 3.0f * sigma;
                if(gate_cm < DUAL_CAMERA_FUSION_TRACK_GATE_MIN_CM)
                    gate_cm = DUAL_CAMERA_FUSION_TRACK_GATE_MIN_CM;
                if(gate_cm > DUAL_CAMERA_FUSION_TRACK_GATE_MAX_CM)
                    gate_cm = DUAL_CAMERA_FUSION_TRACK_GATE_MAX_CM;
                if((distance_sq <= gate_cm * gate_cm) &&
                   ((best_track == 0xFFU) || (distance_sq < best_distance_sq)))
                {
                    best_candidate = i;
                    best_track = track;
                    best_distance_sq = distance_sq;
                }
            }
        }
        if(best_track == 0xFFU) break;

        candidate_used[best_candidate] = 1U;
        track_used[best_track] = 1U;
        track_id[best_candidate] = s_tracks[best_track].track_id;
        s_tracks[best_track].x_cm = candidate[best_candidate].x_cm;
        s_tracks[best_track].y_cm = candidate[best_candidate].y_cm;
        s_tracks[best_track].position_sigma_cm =
            candidate[best_candidate].position_sigma_cm;
        s_tracks[best_track].miss_count = 0U;
    }

    for(i = 0U; i < DUAL_CAMERA_FUSION_TRACK_COUNT; i++)
    {
        if(s_tracks[i].valid && !track_used[i])
        {
            s_tracks[i].miss_count++;
            if(s_tracks[i].miss_count >= DUAL_CAMERA_FUSION_TRACK_MAX_MISS)
                memset(&s_tracks[i], 0, sizeof(s_tracks[i]));
        }
    }
    for(i = 0U; i < count; i++)
    {
        uint8 track;
        if(candidate_used[i]) continue;
        for(track = 0U; track < DUAL_CAMERA_FUSION_TRACK_COUNT; track++)
        {
            if(!s_tracks[track].valid) break;
        }
        if(track >= DUAL_CAMERA_FUSION_TRACK_COUNT) continue;
        s_tracks[track].track_id = DualCameraFusion_AllocateTrackId();
        if(s_tracks[track].track_id == 0U) continue;
        s_tracks[track].valid = 1U;
        s_tracks[track].x_cm = candidate[i].x_cm;
        s_tracks[track].y_cm = candidate[i].y_cm;
        s_tracks[track].position_sigma_cm = candidate[i].position_sigma_cm;
        track_id[i] = s_tracks[track].track_id;
    }
}

void DualCameraFusion_Init(void)
{
    memset(s_front_cache, 0, sizeof(s_front_cache));
    memset(s_rear_cache, 0, sizeof(s_rear_cache));
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(&s_diag, 0, sizeof(s_diag));
    memset(s_residual_history, 0, sizeof(s_residual_history));
    s_front_head = 0U;
    s_front_count = 0U;
    s_rear_head = 0U;
    s_rear_count = 0U;
    s_last_front_service_seq = 0U;
    s_last_rear_service_seq = 0U;
    s_last_reset_generation = 0U;
    s_reset_generation_valid = 0U;
    s_residual_head = 0U;
    s_residual_count = 0U;
    s_diag.output_mode = VISION_FUSION_OUTPUT_MODE;
    DualCameraFusion_ResetTracks();
}

void DualCameraFusion_PushFrontFrame(const CrossCoreCameraFrontFrame_t *frame)
{
    DualCameraFusionFrame_t *target;
    float follow_x_cm = 0.0f;
    float follow_y_cm = 0.0f;
    uint8 i;

    if((frame == NULL) || (frame->frame_seq == 0U)) return;
    target = &s_front_cache[s_front_head];
    memset(target, 0, sizeof(*target));
    target->valid = 1U;
    target->frame_seq = frame->frame_seq;
    target->capture_time_us = frame->capture_time_us;
    target->yaw_deg = frame->yaw_deg;
    target->yaw_valid = frame->yaw_valid;
    target->car_valid = (uint8)(frame->car_valid &&
        VehicleMarkerLocalization_GetFollowPoint(&frame->car_pose,
                                                  &follow_x_cm,
                                                  &follow_y_cm,
                                                  NULL, NULL));
    target->car_raw_valid = (uint8)(frame->car_raw_valid && target->car_valid);
    target->car_x_cm = follow_x_cm;
    target->car_y_cm = follow_y_cm;
    for(i = 0U; i < VISION_RUNTIME_BEACON_SLOT_COUNT; i++)
    {
        const VisionRuntimeBeaconSlot_t *source = &frame->beacon_slots[i];
        CameraBoardBeaconObservation_t *beacon;
        float x_mm;
        float y_mm;

        if(!source->valid || !isfinite(source->x_cm) ||
           !isfinite(source->y_cm) ||
           (target->beacon_count >= CAMERA_BOARD_MAX_BEACON_COUNT))
        {
            continue;
        }
        beacon = &target->beacon[target->beacon_count++];
        x_mm = source->x_cm * 10.0f;
        y_mm = source->y_cm * 10.0f;
        if(x_mm > 32767.0f) x_mm = 32767.0f;
        if(x_mm < -32768.0f) x_mm = -32768.0f;
        if(y_mm > 32767.0f) y_mm = 32767.0f;
        if(y_mm < -32768.0f) y_mm = -32768.0f;
        beacon->x_mm = (int16)x_mm;
        beacon->y_mm = (int16)y_mm;
        beacon->detection_confidence = source->detection_confidence;
        beacon->position_sigma_cm = source->position_sigma_cm;
        beacon->flags = CAMERA_BOARD_RESULT_FLAG_BEACON_VALID;
    }
    s_front_head = (uint8)((s_front_head + 1U) % DUAL_CAMERA_FUSION_CACHE_SIZE);
    if(s_front_count < DUAL_CAMERA_FUSION_CACHE_SIZE) s_front_count++;
}

void DualCameraFusion_PushRearFrame(const CameraBoardVisionFrame_t *frame)
{
    DualCameraFusionFrame_t *target;
    float yaw_deg = 0.0f;
    uint8 count;

    if((frame == NULL) || (frame->camera_frame_seq == 0U)) return;
    if(s_reset_generation_valid &&
       (frame->reset_generation != s_last_reset_generation))
    {
        DualCameraFusion_ResetTracks();
    }
    s_last_reset_generation = frame->reset_generation;
    s_reset_generation_valid = 1U;

    target = &s_rear_cache[s_rear_head];
    memset(target, 0, sizeof(*target));
    target->valid = 1U;
    target->frame_seq = frame->camera_frame_seq;
    target->capture_time_us = frame->capture_time_us;
    target->state_sequence = frame->state_seq_used;
    target->reset_generation = frame->reset_generation;
    target->yaw_valid = CameraBoardLink_GetStateYaw(frame->state_seq_used,
                                                    &yaw_deg);
    target->yaw_deg = yaw_deg;
    target->car_valid = (frame->result_flags &
                         CAMERA_BOARD_RESULT_FLAG_CAR_VALID) ? 1U : 0U;
    target->car_raw_valid = (frame->result_flags &
                             CAMERA_BOARD_RESULT_FLAG_CAR_RAW) ? 1U : 0U;
    target->car_x_cm = (float)frame->car_x_mm * 0.1f;
    target->car_y_cm = (float)frame->car_y_mm * 0.1f;
    count = frame->beacon_count;
    if(count > CAMERA_BOARD_MAX_BEACON_COUNT)
        count = CAMERA_BOARD_MAX_BEACON_COUNT;
    if((frame->result_flags & CAMERA_BOARD_RESULT_FLAG_BEACON_VALID) != 0U)
    {
        target->beacon_count = count;
        memcpy(target->beacon, frame->beacon,
               sizeof(CameraBoardBeaconObservation_t) * count);
    }
    if((frame->result_flags & (CAMERA_BOARD_RESULT_FLAG_TIME_SYNC |
                               CAMERA_BOARD_RESULT_FLAG_STATE_FRESH)) !=
       (CAMERA_BOARD_RESULT_FLAG_TIME_SYNC |
        CAMERA_BOARD_RESULT_FLAG_STATE_FRESH))
    {
        target->yaw_valid = 0U;
    }
    if(!target->yaw_valid) s_diag.state_lookup_miss_count++;
    s_rear_head = (uint8)((s_rear_head + 1U) % DUAL_CAMERA_FUSION_CACHE_SIZE);
    if(s_rear_count < DUAL_CAMERA_FUSION_CACHE_SIZE) s_rear_count++;
}

void DualCameraFusion_ServiceInputs(void)
{
    CrossCoreCameraFrontFrame_t front;
    CameraBoardVisionFrame_t rear;

    if(CrossCoreCamera_ControlCoreGetFrontFrame(&front) &&
       (front.frame_seq != s_last_front_service_seq))
    {
        s_last_front_service_seq = front.frame_seq;
        DualCameraFusion_PushFrontFrame(&front);
    }
    if(CameraBoardLink_GetLatestRearFrame(&rear) &&
       (rear.camera_frame_seq != s_last_rear_service_seq))
    {
        s_last_rear_service_seq = rear.camera_frame_seq;
        DualCameraFusion_PushRearFrame(&rear);
    }
}

void DualCameraFusion_BuildSnapshot(uint32 now_us)
{
    const DualCameraFusionFrame_t *front_any;
    const DualCameraFusionFrame_t *rear_any;
    const DualCameraFusionFrame_t *front_latest;
    const DualCameraFusionFrame_t *rear_latest;
    const DualCameraFusionFrame_t *paired_front = NULL;
    const DualCameraFusionFrame_t *paired_rear = NULL;
    const DualCameraFusionFrame_t *display_front = NULL;
    const DualCameraFusionFrame_t *display_rear = NULL;
    const DualCameraFusionFrame_t *reference = NULL;
    DualCameraFusionCandidate_t candidate[DUAL_CAMERA_FUSION_MAX_CANDIDATES];
    uint8 track_id[DUAL_CAMERA_FUSION_MAX_TARGETS] = {0};
    CameraBoardLinkDiag_t link_diag;
    uint32 best_common_age = 0xFFFFFFFFU;
    int32 best_abs_dt = 0x7FFFFFFF;
    uint8 candidate_count = 0U;
    uint8 output_count;
    uint8 i;

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(candidate, 0, sizeof(candidate));
    memset(&link_diag, 0, sizeof(link_diag));
    s_diag.build_count++;
    s_diag.pair_valid = 0U;
    s_diag.pair_dt_ms = 0;
    s_diag.pair_yaw_delta_deg = 0.0f;
    s_diag.duplicate_count = 0U;
    s_diag.duplicate_residual_cm = 0.0f;
    CameraBoardLink_GetDiag(&link_diag);
    s_diag.extrinsic_nominal = link_diag.rear_extrinsic_nominal;
    s_diag.extrinsic_valid = link_diag.rear_extrinsic_valid;
    s_diag.link_crc_error_count = link_diag.crc_error_count;
    s_diag.link_rx_overflow_count = link_diag.rx_overflow_count;
    s_diag.link_tx_busy_drop_count = link_diag.tx_busy_drop_count;
    s_diag.rear_dropped_frame_count = link_diag.rear_dropped_frame_count;
    s_diag.rear_crc_error_count = link_diag.rear_crc_error_count;
    s_diag.rear_rx_overflow_count = link_diag.rear_rx_overflow_count;
    s_diag.rear_tx_busy_drop_count = link_diag.rear_tx_busy_drop_count;

    front_any = DualCameraFusion_LatestFrame(s_front_cache, s_front_count,
                                             now_us, 0U);
    rear_any = DualCameraFusion_LatestFrame(s_rear_cache, s_rear_count,
                                            now_us, 0U);
    front_latest = DualCameraFusion_LatestFrame(s_front_cache, s_front_count,
                                                now_us, 1U);
    rear_latest = DualCameraFusion_LatestFrame(s_rear_cache, s_rear_count,
                                               now_us, 1U);
    if(!link_diag.mode_active || !link_diag.rear_extrinsic_valid)
    {
        rear_latest = NULL;
    }
    s_diag.front_age_ms = (front_any == NULL) ? 0xFFFFU :
        DualCameraFusion_AgeMs(now_us, front_any->capture_time_us);
    s_diag.rear_age_ms = (rear_any == NULL) ? 0xFFFFU :
        DualCameraFusion_AgeMs(now_us, rear_any->capture_time_us);
    s_diag.front_healthy = (front_latest != NULL) ? 1U : 0U;
    s_diag.rear_healthy = (rear_latest != NULL) ? 1U : 0U;
    s_diag.front_car_valid = (front_any != NULL) ? front_any->car_valid : 0U;
    s_diag.rear_car_valid = (rear_any != NULL) ? rear_any->car_valid : 0U;
    s_diag.front_beacon_count = (front_any != NULL) ? front_any->beacon_count : 0U;
    s_diag.rear_beacon_count = (rear_any != NULL) ? rear_any->beacon_count : 0U;
    if(front_latest == NULL) s_diag.front_stale_build_count++;
    if(rear_latest == NULL) s_diag.rear_stale_build_count++;
    if((front_latest != NULL) && (rear_latest != NULL))
    {
        uint8 f;
        uint8 r;
        s_diag.dual_healthy_build_count++;
        for(f = 0U; f < s_front_count; f++)
        {
            const DualCameraFusionFrame_t *front = &s_front_cache[f];
            if(!DualCameraFusion_FrameStrong(front, now_us)) continue;
            for(r = 0U; r < s_rear_count; r++)
            {
                const DualCameraFusionFrame_t *rear = &s_rear_cache[r];
                int32 dt;
                int32 abs_dt;
                uint32 front_age;
                uint32 rear_age;
                uint32 common_age;

                if(!DualCameraFusion_FrameStrong(rear, now_us)) continue;
                dt = (int32)(rear->capture_time_us - front->capture_time_us);
                abs_dt = (dt < 0) ? -dt : dt;
                if(abs_dt > DUAL_CAMERA_FUSION_PAIR_GATE_US) continue;
                front_age = (uint32)(now_us - front->capture_time_us);
                rear_age = (uint32)(now_us - rear->capture_time_us);
                common_age = (front_age > rear_age) ? front_age : rear_age;
                if((paired_front == NULL) || (common_age < best_common_age) ||
                   ((common_age == best_common_age) && (abs_dt < best_abs_dt)))
                {
                    paired_front = front;
                    paired_rear = rear;
                    best_common_age = common_age;
                    best_abs_dt = abs_dt;
                }
            }
        }
    }

    if((paired_front != NULL) && (paired_rear != NULL))
    {
        int32 dt_us = (int32)(paired_rear->capture_time_us -
                              paired_front->capture_time_us);
        reference = paired_front;
        s_snapshot.paired = 1U;
        s_snapshot.reference_source = DUAL_CAMERA_FUSION_SOURCE_FRONT;
        s_diag.pair_valid = 1U;
        s_diag.pair_dt_ms = (int16)(dt_us / 1000);
        s_diag.pair_yaw_delta_deg = DualCameraFusion_Wrap180(
            paired_rear->yaw_deg - paired_front->yaw_deg);
        s_diag.paired_build_count++;
        candidate_count = DualCameraFusion_BuildCandidates(
            paired_front, paired_rear, reference,
            DUAL_CAMERA_FUSION_SOURCE_FRONT, (uint32)best_abs_dt,
            candidate);
    }
    else if((front_latest != NULL) && ((rear_latest == NULL) ||
            ((uint32)(now_us - front_latest->capture_time_us) <=
             (uint32)(now_us - rear_latest->capture_time_us))))
    {
        reference = front_latest;
        s_snapshot.reference_source = DUAL_CAMERA_FUSION_SOURCE_FRONT;
        s_diag.front_only_build_count++;
        candidate_count = DualCameraFusion_BuildCandidates(
            front_latest, NULL, reference, DUAL_CAMERA_FUSION_SOURCE_FRONT,
            0U, candidate);
    }
    else if(rear_latest != NULL)
    {
        reference = rear_latest;
        s_snapshot.reference_source = DUAL_CAMERA_FUSION_SOURCE_REAR;
        s_diag.rear_only_build_count++;
        candidate_count = DualCameraFusion_BuildCandidates(
            NULL, rear_latest, reference, DUAL_CAMERA_FUSION_SOURCE_REAR,
            0U, candidate);
    }
    else
    {
        s_diag.empty_build_count++;
    }

    if(reference != NULL)
    {
        s_snapshot.valid = 1U;
        s_snapshot.reference_time_us = reference->capture_time_us;
        s_snapshot.reference_frame_seq = reference->frame_seq;
        s_snapshot.car_x_cm = reference->car_x_cm;
        s_snapshot.car_y_cm = reference->car_y_cm;
        DualCameraFusion_ApplySourceResidual(s_snapshot.reference_source,
                                             &s_snapshot.car_x_cm,
                                             &s_snapshot.car_y_cm);
        s_snapshot.car_valid = reference->car_valid;
        s_snapshot.car_raw_valid = reference->car_raw_valid;
        s_snapshot.yaw_deg = reference->yaw_deg;
        s_snapshot.yaw_valid = reference->yaw_valid;

        display_front = (paired_front != NULL) ? paired_front : front_latest;
        display_rear = (paired_rear != NULL) ? paired_rear : rear_latest;
        s_snapshot.front_car_valid = DualCameraFusion_AlignCarToReference(
            display_front, reference, DUAL_CAMERA_FUSION_SOURCE_FRONT,
            &s_snapshot.front_car_x_cm, &s_snapshot.front_car_y_cm);
        s_snapshot.rear_car_valid = DualCameraFusion_AlignCarToReference(
            display_rear, reference, DUAL_CAMERA_FUSION_SOURCE_REAR,
            &s_snapshot.rear_car_x_cm, &s_snapshot.rear_car_y_cm);
        if(display_front != NULL)
        {
            s_snapshot.front_frame_seq = display_front->frame_seq;
            s_snapshot.front_capture_time_us = display_front->capture_time_us;
            s_snapshot.front_car_raw_valid = display_front->car_raw_valid;
        }
        if(display_rear != NULL)
        {
            s_snapshot.rear_frame_seq = display_rear->frame_seq;
            s_snapshot.rear_capture_time_us = display_rear->capture_time_us;
            s_snapshot.rear_car_raw_valid = display_rear->car_raw_valid;
            s_snapshot.rear_observation_x_cm = display_rear->car_x_cm;
            s_snapshot.rear_observation_y_cm = display_rear->car_y_cm;
            s_snapshot.rear_yaw_deg = display_rear->yaw_deg;
            s_snapshot.rear_yaw_valid = display_rear->yaw_valid;
        }
    }

    DualCameraFusion_SortCandidates(candidate, candidate_count);
    output_count = candidate_count;
    if(output_count > DUAL_CAMERA_FUSION_MAX_TARGETS)
    {
        s_diag.output_overflow_count +=
            (uint32)(output_count - DUAL_CAMERA_FUSION_MAX_TARGETS);
        output_count = DUAL_CAMERA_FUSION_MAX_TARGETS;
    }
    DualCameraFusion_UpdateTracks(candidate, output_count, track_id);
    s_snapshot.target_count = output_count;
    s_snapshot.tracker_reset_count = s_diag.tracker_reset_count;
    for(i = 0U; i < output_count; i++)
    {
        s_snapshot.target[i].valid = 1U;
        s_snapshot.target[i].track_id = track_id[i];
        s_snapshot.target[i].source = candidate[i].source;
        s_snapshot.target[i].detection_confidence =
            candidate[i].detection_confidence;
        s_snapshot.target[i].position_sigma_cm =
            candidate[i].position_sigma_cm;
        s_snapshot.target[i].x_cm = candidate[i].x_cm;
        s_snapshot.target[i].y_cm = candidate[i].y_cm;
        s_snapshot.target[i].dx_cm = candidate[i].dx_cm;
        s_snapshot.target[i].dy_cm = candidate[i].dy_cm;
    }
    s_diag.output_count = output_count;
    s_diag.pair_rate_permille = (s_diag.dual_healthy_build_count == 0U) ? 0U :
        (uint16)(((uint64)s_diag.paired_build_count * 1000ULL) /
                 (uint64)s_diag.dual_healthy_build_count);
    DualCameraFusion_UpdateResidualPercentiles();
}

uint8 DualCameraFusion_GetSnapshot(DualCameraFusionSnapshot_t *snapshot)
{
    if(snapshot == NULL) return 0U;
    *snapshot = s_snapshot;
    return s_snapshot.valid;
}

void DualCameraFusion_GetDiag(DualCameraFusionDiag_t *diag)
{
    if(diag != NULL) *diag = s_diag;
}
