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
#define DUAL_CAMERA_FUSION_FRONT_WEIGHT           (0.75f)
#define DUAL_CAMERA_FUSION_REAR_WEIGHT            (0.25f)
#define DUAL_CAMERA_FUSION_CAR_VARIANCE_BOTH_CM2  (25.0f)
#define DUAL_CAMERA_FUSION_CAR_VARIANCE_ONE_CM2   (64.0f)
#define DUAL_CAMERA_FUSION_CAR_VARIANCE_MAX_CM2   (40000.0f)
#define DUAL_CAMERA_FUSION_CAR_VALID_VARIANCE_CM2 (10000.0f)
#define DUAL_CAMERA_FUSION_CAR_EXCLUSION_VARIANCE_CM2 (1225.0f)
#define DUAL_CAMERA_FUSION_CAR_SIGMA_MOTION_CMS   (80.0f)
#define DUAL_CAMERA_FUSION_CAR_SIGMA_STATIC_CMS   (35.0f)
#define DUAL_CAMERA_FUSION_CAR_SIGMA_VELOCITY_CMS (25.0f)
#define DUAL_CAMERA_FUSION_CAR_ROBUST_NIS         (9.0f)
#define DUAL_CAMERA_FUSION_CAR_WEIGHT_MIN         (0.05f)
#define DUAL_CAMERA_FUSION_CAR_YAW_RESET_DEG      (60.0f)
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

typedef struct
{
    uint8 initialized;
    uint8 motion_valid;
    uint8 car_velocity_valid;
    uint8 yaw_valid;
    uint8 observation_applied;
    uint8 observation_source;
    uint32 state_time_us;
    uint32 observation_time_us;
    uint32 last_front_frame_seq;
    uint32 last_rear_frame_seq;
    float x_earth_cm;
    float y_earth_cm;
    float covariance_x_cm2;
    float covariance_y_cm2;
    float relative_vx_earth_cms;
    float relative_vy_earth_cms;
    float yaw_deg;
    float innovation_cm;
    float adaptive_gate_cm;
    float observation_x_earth_cm;
    float observation_y_earth_cm;
    float observation_weight;
} DualCameraFusionCarEstimator_t;

static DualCameraFusionFrame_t s_front_cache[DUAL_CAMERA_FUSION_CACHE_SIZE];
static DualCameraFusionFrame_t s_rear_cache[DUAL_CAMERA_FUSION_CACHE_SIZE];
static uint8 s_front_head;
static uint8 s_front_count;
static uint8 s_rear_head;
static uint8 s_rear_count;
static uint32 s_last_front_service_seq;
static uint32 s_last_rear_service_seq;
static uint32 s_fusion_revision;
static uint16 s_last_reset_generation;
static uint8 s_reset_generation_valid;
static DualCameraFusionTrack_t s_tracks[DUAL_CAMERA_FUSION_TRACK_COUNT];
static uint8 s_next_track_id;
static DualCameraFusionSnapshot_t s_snapshot;
static DualCameraFusionDiag_t s_diag;
static float s_residual_history[DUAL_CAMERA_FUSION_RESIDUAL_HISTORY_SIZE];
static uint8 s_residual_head;
static uint8 s_residual_count;
static DualCameraFusionCarEstimator_t s_car_estimator;

static float DualCameraFusion_Wrap180(float angle_deg)
{
    while(angle_deg > 180.0f) angle_deg -= 360.0f;
    while(angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static float DualCameraFusion_LimitFloat(float value,
                                          float minimum,
                                          float maximum)
{
    if(value > maximum) return maximum;
    if(value < minimum) return minimum;
    return value;
}

static void DualCameraFusion_BodyToEarth(float body_x,
                                          float body_y,
                                          float yaw_deg,
                                          float *earth_x,
                                          float *earth_y)
{
    float yaw_rad = yaw_deg * DUAL_CAMERA_FUSION_DEG_TO_RAD;
    float cosine = cosf(yaw_rad);
    float sine = sinf(yaw_rad);

    *earth_x = cosine * body_x - sine * body_y;
    *earth_y = sine * body_x + cosine * body_y;
}

static void DualCameraFusion_EarthToBody(float earth_x,
                                          float earth_y,
                                          float yaw_deg,
                                          float *body_x,
                                          float *body_y)
{
    float yaw_rad = yaw_deg * DUAL_CAMERA_FUSION_DEG_TO_RAD;
    float cosine = cosf(yaw_rad);
    float sine = sinf(yaw_rad);

    *body_x = cosine * earth_x + sine * earth_y;
    *body_y = -sine * earth_x + cosine * earth_y;
}

static void DualCameraFusion_ResetCarEstimateUnlocked(void)
{
    memset(&s_car_estimator, 0, sizeof(s_car_estimator));
    s_car_estimator.covariance_x_cm2 =
        DUAL_CAMERA_FUSION_CAR_VARIANCE_ONE_CM2;
    s_car_estimator.covariance_y_cm2 =
        DUAL_CAMERA_FUSION_CAR_VARIANCE_ONE_CM2;
}

static float DualCameraFusion_CarProcessSigma(void)
{
    if(!s_car_estimator.motion_valid)
        return DUAL_CAMERA_FUSION_CAR_SIGMA_MOTION_CMS;
    if(!s_car_estimator.car_velocity_valid)
        return DUAL_CAMERA_FUSION_CAR_SIGMA_STATIC_CMS;
    return DUAL_CAMERA_FUSION_CAR_SIGMA_VELOCITY_CMS;
}

static void DualCameraFusion_PredictCarToUnlocked(uint32 now_us)
{
    uint32 elapsed_us;
    float dt_s;
    float position_dt_s;
    float sigma;
    float process_variance;

    if(s_car_estimator.state_time_us == 0U)
    {
        s_car_estimator.state_time_us = now_us;
        return;
    }
    elapsed_us = (uint32)(now_us - s_car_estimator.state_time_us);
    if(elapsed_us == 0U) return;
    dt_s = (float)elapsed_us * 0.000001f;
    position_dt_s = (dt_s > 10.0f) ? 10.0f : dt_s;

    if(s_car_estimator.initialized && s_car_estimator.motion_valid)
    {
        s_car_estimator.x_earth_cm +=
            s_car_estimator.relative_vx_earth_cms * position_dt_s;
        s_car_estimator.y_earth_cm +=
            s_car_estimator.relative_vy_earth_cms * position_dt_s;
    }
    if(s_car_estimator.initialized)
    {
        sigma = DualCameraFusion_CarProcessSigma();
        process_variance = sigma * sigma * dt_s;
        s_car_estimator.covariance_x_cm2 = DualCameraFusion_LimitFloat(
            s_car_estimator.covariance_x_cm2 + process_variance,
            0.0f, DUAL_CAMERA_FUSION_CAR_VARIANCE_MAX_CM2);
        s_car_estimator.covariance_y_cm2 = DualCameraFusion_LimitFloat(
            s_car_estimator.covariance_y_cm2 + process_variance,
            0.0f, DUAL_CAMERA_FUSION_CAR_VARIANCE_MAX_CM2);
    }
    s_car_estimator.state_time_us = now_us;
}

static void DualCameraFusion_UpdateCarObservation(
    uint32 now_us,
    uint32 capture_time_us,
    uint32 front_frame_seq,
    uint32 rear_frame_seq,
    float yaw_deg,
    float body_x_cm,
    float body_y_cm,
    float measurement_variance_cm2,
    uint8 source)
{
    uint32 irq_state;
    uint32 age_us;
    float observation_x;
    float observation_y;
    float dx;
    float dy;
    float innovation_squared;
    float normalized_innovation;
    float effective_variance;
    float gain_x;
    float gain_y;
    float weight;
    float inflation;
    float average_innovation_variance;

    if((capture_time_us == 0U) || !isfinite(yaw_deg) ||
       !isfinite(body_x_cm) || !isfinite(body_y_cm) ||
       !isfinite(measurement_variance_cm2) ||
       !(measurement_variance_cm2 > 0.0f))
    {
        return;
    }

    DualCameraFusion_BodyToEarth(body_x_cm, body_y_cm, yaw_deg,
                                 &observation_x, &observation_y);

    irq_state = __get_PRIMASK();
    __disable_irq();
    DualCameraFusion_PredictCarToUnlocked(now_us);
    s_car_estimator.observation_applied = 0U;
    if((front_frame_seq == s_car_estimator.last_front_frame_seq) &&
       (rear_frame_seq == s_car_estimator.last_rear_frame_seq))
    {
        if(irq_state == 0U) __enable_irq();
        return;
    }

    age_us = ((int32)(now_us - capture_time_us) > 0) ?
             (uint32)(now_us - capture_time_us) : 0U;
    if(s_car_estimator.motion_valid)
    {
        float age_s = (float)age_us * 0.000001f;
        observation_x += s_car_estimator.relative_vx_earth_cms * age_s;
        observation_y += s_car_estimator.relative_vy_earth_cms * age_s;
    }
    s_car_estimator.observation_x_earth_cm = observation_x;
    s_car_estimator.observation_y_earth_cm = observation_y;

    if(!s_car_estimator.initialized ||
       (0.5f * (s_car_estimator.covariance_x_cm2 +
                s_car_estimator.covariance_y_cm2) >
        DUAL_CAMERA_FUSION_CAR_VALID_VARIANCE_CM2))
    {
        s_car_estimator.x_earth_cm = observation_x;
        s_car_estimator.y_earth_cm = observation_y;
        s_car_estimator.covariance_x_cm2 = measurement_variance_cm2;
        s_car_estimator.covariance_y_cm2 = measurement_variance_cm2;
        s_car_estimator.innovation_cm = 0.0f;
        s_car_estimator.adaptive_gate_cm =
            3.0f * sqrtf(measurement_variance_cm2);
        s_car_estimator.observation_weight = 1.0f;
        s_car_estimator.initialized = 1U;
    }
    else
    {
        dx = observation_x - s_car_estimator.x_earth_cm;
        dy = observation_y - s_car_estimator.y_earth_cm;
        innovation_squared = dx * dx + dy * dy;
        average_innovation_variance = 0.5f *
            (s_car_estimator.covariance_x_cm2 +
             s_car_estimator.covariance_y_cm2) + measurement_variance_cm2;
        if(average_innovation_variance < 1.0f)
            average_innovation_variance = 1.0f;
        normalized_innovation = innovation_squared /
                                average_innovation_variance;
        weight = (normalized_innovation <=
                  DUAL_CAMERA_FUSION_CAR_ROBUST_NIS) ? 1.0f :
            DUAL_CAMERA_FUSION_CAR_ROBUST_NIS / normalized_innovation;
        weight = DualCameraFusion_LimitFloat(
            weight, DUAL_CAMERA_FUSION_CAR_WEIGHT_MIN, 1.0f);
        effective_variance = measurement_variance_cm2 / weight;
        gain_x = s_car_estimator.covariance_x_cm2 /
            (s_car_estimator.covariance_x_cm2 + effective_variance);
        gain_y = s_car_estimator.covariance_y_cm2 /
            (s_car_estimator.covariance_y_cm2 + effective_variance);
        s_car_estimator.x_earth_cm += gain_x * dx;
        s_car_estimator.y_earth_cm += gain_y * dy;
        inflation = (1.0f - weight) * measurement_variance_cm2;
        s_car_estimator.covariance_x_cm2 = DualCameraFusion_LimitFloat(
            (1.0f - gain_x) * s_car_estimator.covariance_x_cm2 + inflation,
            0.0f, DUAL_CAMERA_FUSION_CAR_VARIANCE_MAX_CM2);
        s_car_estimator.covariance_y_cm2 = DualCameraFusion_LimitFloat(
            (1.0f - gain_y) * s_car_estimator.covariance_y_cm2 + inflation,
            0.0f, DUAL_CAMERA_FUSION_CAR_VARIANCE_MAX_CM2);
        s_car_estimator.innovation_cm = sqrtf(innovation_squared);
        s_car_estimator.adaptive_gate_cm =
            3.0f * sqrtf(average_innovation_variance);
        s_car_estimator.observation_weight = weight;
    }

    s_car_estimator.observation_time_us = capture_time_us;
    s_car_estimator.last_front_frame_seq = front_frame_seq;
    s_car_estimator.last_rear_frame_seq = rear_frame_seq;
    s_car_estimator.observation_source = source;
    s_car_estimator.observation_applied = 1U;
    if(irq_state == 0U) __enable_irq();
}

void DualCameraFusion_ResetCarEstimate(void)
{
    uint32 irq_state = __get_PRIMASK();

    __disable_irq();
    DualCameraFusion_ResetCarEstimateUnlocked();
    if(irq_state == 0U) __enable_irq();
}

void DualCameraFusion_UpdateCarPrediction(uint32 now_us,
                                          float yaw_deg,
                                          uint8 motion_valid,
                                          float relative_vx_earth_cms,
                                          float relative_vy_earth_cms,
                                          uint8 car_velocity_valid)
{
    uint32 irq_state;

    if(now_us == 0U) return;
    irq_state = __get_PRIMASK();
    __disable_irq();
    DualCameraFusion_PredictCarToUnlocked(now_us);

    if(isfinite(yaw_deg))
    {
        if(s_car_estimator.yaw_valid &&
           (fabsf(DualCameraFusion_Wrap180(
                yaw_deg - s_car_estimator.yaw_deg)) >
            DUAL_CAMERA_FUSION_CAR_YAW_RESET_DEG))
        {
            DualCameraFusion_ResetCarEstimateUnlocked();
            s_car_estimator.state_time_us = now_us;
        }
        s_car_estimator.yaw_deg = yaw_deg;
        s_car_estimator.yaw_valid = 1U;
    }
    else
    {
        s_car_estimator.yaw_valid = 0U;
    }

    s_car_estimator.motion_valid = (uint8)(motion_valid &&
        isfinite(relative_vx_earth_cms) &&
        isfinite(relative_vy_earth_cms));
    s_car_estimator.car_velocity_valid =
        (uint8)(s_car_estimator.motion_valid && car_velocity_valid);
    if(s_car_estimator.motion_valid)
    {
        s_car_estimator.relative_vx_earth_cms = relative_vx_earth_cms;
        s_car_estimator.relative_vy_earth_cms = relative_vy_earth_cms;
    }
    else
    {
        s_car_estimator.relative_vx_earth_cms = 0.0f;
        s_car_estimator.relative_vy_earth_cms = 0.0f;
    }
    if(irq_state == 0U) __enable_irq();
}

uint8 DualCameraFusion_GetCarEstimate(DualCameraFusionCarEstimate_t *estimate)
{
    DualCameraFusionCarEstimator_t state;
    uint32 irq_state;
    uint32 now_us;
    uint32 elapsed_us;
    float dt_s;
    float position_dt_s;
    float sigma;
    float average_variance;

    if(estimate == NULL) return 0U;
    memset(estimate, 0, sizeof(*estimate));
    irq_state = __get_PRIMASK();
    __disable_irq();
    state = s_car_estimator;
    if(irq_state == 0U) __enable_irq();
    if(!state.initialized) return 0U;

    now_us = CameraBoardLink_TimeNowUs();
    elapsed_us = (state.state_time_us == 0U) ? 0U :
                 (uint32)(now_us - state.state_time_us);
    dt_s = (float)elapsed_us * 0.000001f;
    position_dt_s = (dt_s > 10.0f) ? 10.0f : dt_s;
    estimate->x_earth_cm = state.x_earth_cm;
    estimate->y_earth_cm = state.y_earth_cm;
    if(state.motion_valid)
    {
        estimate->x_earth_cm += state.relative_vx_earth_cms * position_dt_s;
        estimate->y_earth_cm += state.relative_vy_earth_cms * position_dt_s;
    }
    sigma = !state.motion_valid ? DUAL_CAMERA_FUSION_CAR_SIGMA_MOTION_CMS :
        (!state.car_velocity_valid ?
         DUAL_CAMERA_FUSION_CAR_SIGMA_STATIC_CMS :
         DUAL_CAMERA_FUSION_CAR_SIGMA_VELOCITY_CMS);
    estimate->covariance_x_cm2 = DualCameraFusion_LimitFloat(
        state.covariance_x_cm2 + sigma * sigma * dt_s,
        0.0f, DUAL_CAMERA_FUSION_CAR_VARIANCE_MAX_CM2);
    estimate->covariance_y_cm2 = DualCameraFusion_LimitFloat(
        state.covariance_y_cm2 + sigma * sigma * dt_s,
        0.0f, DUAL_CAMERA_FUSION_CAR_VARIANCE_MAX_CM2);
    average_variance = 0.5f * (estimate->covariance_x_cm2 +
                               estimate->covariance_y_cm2);
    estimate->valid = (uint8)(isfinite(estimate->x_earth_cm) &&
        isfinite(estimate->y_earth_cm) &&
        (average_variance <= DUAL_CAMERA_FUSION_CAR_VALID_VARIANCE_CM2));
    estimate->exclusion_valid = (uint8)(estimate->valid &&
        (average_variance <=
         DUAL_CAMERA_FUSION_CAR_EXCLUSION_VARIANCE_CM2));
    estimate->state_time_us = now_us;
    estimate->observation_time_us = state.observation_time_us;
    if(state.observation_time_us != 0U)
    {
        uint32 age_ms = ((int32)(now_us - state.observation_time_us) > 0) ?
            (uint32)(now_us - state.observation_time_us) / 1000U : 0U;
        estimate->prediction_age_ms = (uint16)(
            (age_ms > 0xFFFFU) ? 0xFFFFU : age_ms);
    }
    else
    {
        estimate->prediction_age_ms = 0xFFFFU;
    }
    estimate->innovation_cm = state.innovation_cm;
    estimate->adaptive_gate_cm = state.adaptive_gate_cm;
    estimate->observation_x_earth_cm = state.observation_x_earth_cm;
    estimate->observation_y_earth_cm = state.observation_y_earth_cm;
    estimate->observation_weight = state.observation_weight;
    estimate->observation_applied = state.observation_applied;
    estimate->observation_source = state.observation_source;
    estimate->initialized = state.initialized;
    return estimate->valid;
}

static void DualCameraFusion_ApplyCarEstimateToSnapshot(
    DualCameraFusionSnapshot_t *snapshot,
    uint32 now_us)
{
    DualCameraFusionCarEstimate_t estimate;
    DualCameraFusionCarEstimator_t state;
    uint32 irq_state;
    uint32 reference_age_us;
    float reference_x_earth;
    float reference_y_earth;
    float average_variance;

    if(snapshot == NULL) return;
    (void)DualCameraFusion_GetCarEstimate(&estimate);
    snapshot->car_estimate_valid = estimate.valid;
    snapshot->car_exclusion_valid = estimate.exclusion_valid;
    snapshot->car_estimate_source = estimate.observation_source;
    snapshot->car_prediction_age_ms = estimate.prediction_age_ms;
    snapshot->car_innovation_cm = estimate.innovation_cm;
    snapshot->car_observation_weight = estimate.observation_weight;
    average_variance = 0.5f * (estimate.covariance_x_cm2 +
                               estimate.covariance_y_cm2);
    snapshot->car_position_sigma_cm =
        (average_variance > 0.0f) ? sqrtf(average_variance) : 0.0f;
    snapshot->car_valid = estimate.valid;
    if(!estimate.valid || !snapshot->yaw_valid ||
       (snapshot->reference_time_us == 0U))
    {
        return;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    state = s_car_estimator;
    if(irq_state == 0U) __enable_irq();
    reference_x_earth = estimate.x_earth_cm;
    reference_y_earth = estimate.y_earth_cm;
    reference_age_us =
        ((int32)(now_us - snapshot->reference_time_us) > 0) ?
        (uint32)(now_us - snapshot->reference_time_us) : 0U;
    if(state.motion_valid)
    {
        float reference_age_s = (float)reference_age_us * 0.000001f;
        reference_x_earth -= state.relative_vx_earth_cms * reference_age_s;
        reference_y_earth -= state.relative_vy_earth_cms * reference_age_s;
    }
    DualCameraFusion_EarthToBody(reference_x_earth, reference_y_earth,
                                 snapshot->yaw_deg,
                                 &snapshot->car_x_cm,
                                 &snapshot->car_y_cm);
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
                if((distance_sq < gate_cm * gate_cm) &&
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
        output[output_count].x_cm =
            DUAL_CAMERA_FUSION_FRONT_WEIGHT * front_candidate[best_front].x_cm +
            DUAL_CAMERA_FUSION_REAR_WEIGHT * rear_candidate[best_rear].x_cm;
        output[output_count].y_cm =
            DUAL_CAMERA_FUSION_FRONT_WEIGHT * front_candidate[best_front].y_cm +
            DUAL_CAMERA_FUSION_REAR_WEIGHT * rear_candidate[best_rear].y_cm;
        output[output_count].dx_cm =
            DUAL_CAMERA_FUSION_FRONT_WEIGHT * front_candidate[best_front].dx_cm +
            DUAL_CAMERA_FUSION_REAR_WEIGHT * rear_candidate[best_rear].dx_cm;
        output[output_count].dy_cm =
            DUAL_CAMERA_FUSION_FRONT_WEIGHT * front_candidate[best_front].dy_cm +
            DUAL_CAMERA_FUSION_REAR_WEIGHT * rear_candidate[best_rear].dy_cm;
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
    s_fusion_revision = 0U;
    s_last_reset_generation = 0U;
    s_reset_generation_valid = 0U;
    s_residual_head = 0U;
    s_residual_count = 0U;
    DualCameraFusion_ResetCarEstimateUnlocked();
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
    uint8 input_changed = 0U;

    if(CrossCoreCamera_ControlCoreGetFrontFrame(&front) &&
       (front.frame_seq != s_last_front_service_seq))
    {
        s_last_front_service_seq = front.frame_seq;
        DualCameraFusion_PushFrontFrame(&front);
        input_changed = 1U;
    }
    if(CameraBoardLink_GetLatestRearFrame(&rear) &&
       (rear.camera_frame_seq != s_last_rear_service_seq))
    {
        s_last_rear_service_seq = rear.camera_frame_seq;
        DualCameraFusion_PushRearFrame(&rear);
        input_changed = 1U;
    }
    if(input_changed)
    {
        s_fusion_revision++;
        if(0U == s_fusion_revision)
        {
            s_fusion_revision = 1U;
        }
        DualCameraFusion_BuildSnapshot(CameraBoardLink_TimeNowUs());
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
    uint8 front_car_observation_valid;
    uint8 rear_car_observation_valid;
    uint8 car_observation_source;
    float car_measurement_variance_cm2;

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.fusion_revision = s_fusion_revision;
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

        front_car_observation_valid = (uint8)(
            s_snapshot.front_car_valid && s_snapshot.front_car_raw_valid);
        rear_car_observation_valid = (uint8)(
            s_snapshot.rear_car_valid && s_snapshot.rear_car_raw_valid);
        car_observation_source = DUAL_CAMERA_FUSION_SOURCE_NONE;
        car_measurement_variance_cm2 =
            DUAL_CAMERA_FUSION_CAR_VARIANCE_ONE_CM2;

        /* Only raw detections update the estimator. Board-side held poses must
         * not be counted as new measurements. */
        if(front_car_observation_valid && rear_car_observation_valid)
        {
            float board_dx = s_snapshot.front_car_x_cm -
                             s_snapshot.rear_car_x_cm;
            float board_dy = s_snapshot.front_car_y_cm -
                             s_snapshot.rear_car_y_cm;

            s_snapshot.car_x_cm =
                DUAL_CAMERA_FUSION_FRONT_WEIGHT * s_snapshot.front_car_x_cm +
                DUAL_CAMERA_FUSION_REAR_WEIGHT * s_snapshot.rear_car_x_cm;
            s_snapshot.car_y_cm =
                DUAL_CAMERA_FUSION_FRONT_WEIGHT * s_snapshot.front_car_y_cm +
                DUAL_CAMERA_FUSION_REAR_WEIGHT * s_snapshot.rear_car_y_cm;
            s_snapshot.car_observation_valid = 1U;
            car_observation_source =
                DUAL_CAMERA_FUSION_SOURCE_BOTH_FRONT;
            car_measurement_variance_cm2 = DualCameraFusion_LimitFloat(
                DUAL_CAMERA_FUSION_CAR_VARIANCE_BOTH_CM2 +
                0.0625f * (board_dx * board_dx + board_dy * board_dy),
                DUAL_CAMERA_FUSION_CAR_VARIANCE_BOTH_CM2,
                400.0f);
        }
        else if(front_car_observation_valid)
        {
            s_snapshot.car_x_cm = s_snapshot.front_car_x_cm;
            s_snapshot.car_y_cm = s_snapshot.front_car_y_cm;
            s_snapshot.car_observation_valid = 1U;
            car_observation_source = DUAL_CAMERA_FUSION_SOURCE_FRONT;
        }
        else if(rear_car_observation_valid)
        {
            s_snapshot.car_x_cm = s_snapshot.rear_car_x_cm;
            s_snapshot.car_y_cm = s_snapshot.rear_car_y_cm;
            s_snapshot.car_observation_valid = 1U;
            car_observation_source = DUAL_CAMERA_FUSION_SOURCE_REAR;
        }
        else
        {
            s_snapshot.car_observation_valid = 0U;
        }
        s_snapshot.car_raw_valid = s_snapshot.car_observation_valid;
        if(s_snapshot.car_observation_valid)
        {
            s_snapshot.car_observation_x_cm = s_snapshot.car_x_cm;
            s_snapshot.car_observation_y_cm = s_snapshot.car_y_cm;
            DualCameraFusion_UpdateCarObservation(
                now_us,
                s_snapshot.reference_time_us,
                front_car_observation_valid ? s_snapshot.front_frame_seq : 0U,
                rear_car_observation_valid ? s_snapshot.rear_frame_seq : 0U,
                s_snapshot.yaw_deg,
                s_snapshot.car_x_cm,
                s_snapshot.car_y_cm,
                car_measurement_variance_cm2,
                car_observation_source);
        }
        DualCameraFusion_ApplyCarEstimateToSnapshot(&s_snapshot, now_us);

        if(s_snapshot.car_valid)
        {
            for(i = 0U; i < candidate_count; i++)
            {
                candidate[i].dx_cm = candidate[i].x_cm - s_snapshot.car_x_cm;
                candidate[i].dy_cm = candidate[i].y_cm - s_snapshot.car_y_cm;
            }
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
    DualCameraFusion_ApplyCarEstimateToSnapshot(
        snapshot, CameraBoardLink_TimeNowUs());
    return snapshot->valid;
}

void DualCameraFusion_GetDiag(DualCameraFusionDiag_t *diag)
{
    if(diag != NULL) *diag = s_diag;
}
