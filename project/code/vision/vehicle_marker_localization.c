#include "vehicle_marker_localization.h"
#include "common/project_headfile.h"
#include <math.h>
#include <string.h>

VehicleMarkerPose_t g_vehicle_marker_pose = {0};

volatile uint32 g_vehicle_marker_pose_update_seq = 0;
volatile uint8 g_vehicle_marker_position_lost_flag = 1U;
volatile uint16 g_vehicle_marker_position_lost_elapsed_ms = VEHICLE_MARKER_POSITION_LOST_HOLD_MS;

#define VEHICLE_MARKER_CENTER_KALMAN_Q          (0.20f)
#define VEHICLE_MARKER_CENTER_KALMAN_R          (4.0f)
#define VEHICLE_MARKER_CENTER_KALMAN_INITIAL_P  (10.0f)

typedef struct
{
    float x;
    float p;
    float q;
    float r;
    float k;
    uint8 initialized;
} VehicleMarkerKalman1D_t;

static uint8 s_vehicle_marker_has_trackable_pose = 0U;
static uint16 s_vehicle_marker_lost_elapsed_ms = VEHICLE_MARKER_POSITION_LOST_HOLD_MS;
static VehicleMarkerKalman1D_t s_vehicle_marker_center_kalman_x = {0};
static VehicleMarkerKalman1D_t s_vehicle_marker_center_kalman_y = {0};

static volatile uint8 s_vehicle_marker_debug_raw_detect_valid = 0U;
static volatile uint32 s_vehicle_marker_debug_task_5ms_count = 0U;
static volatile uint32 s_vehicle_marker_debug_process_count = 0U;
static volatile uint32 s_vehicle_marker_debug_process_success_count = 0U;
static volatile uint32 s_vehicle_marker_debug_process_fail_count = 0U;
static volatile uint32 s_vehicle_marker_debug_mark_lost_count = 0U;
static volatile uint32 s_vehicle_marker_debug_timeout_count = 0U;

static VehicleMarkerChevronDebug_t s_vehicle_marker_chevron_debug = {0};

static void vehicle_marker_chevron_reset_debug(void);

static void vehicle_marker_center_kalman_reset_axis(VehicleMarkerKalman1D_t *kf)
{
    if(kf == NULL)
    {
        return;
    }

    kf->x = 0.0f;
    kf->p = VEHICLE_MARKER_CENTER_KALMAN_INITIAL_P;
    kf->q = VEHICLE_MARKER_CENTER_KALMAN_Q;
    kf->r = VEHICLE_MARKER_CENTER_KALMAN_R;
    kf->k = 0.0f;
    kf->initialized = 0U;
}

static void vehicle_marker_center_kalman_reset(void)
{
    vehicle_marker_center_kalman_reset_axis(&s_vehicle_marker_center_kalman_x);
    vehicle_marker_center_kalman_reset_axis(&s_vehicle_marker_center_kalman_y);
}

static float vehicle_marker_center_kalman_update_axis(VehicleMarkerKalman1D_t *kf, float measurement)
{
    if(kf == NULL)
    {
        return measurement;
    }

    if(!kf->initialized)
    {
        kf->x = measurement;
        kf->p = VEHICLE_MARKER_CENTER_KALMAN_INITIAL_P;
        kf->q = VEHICLE_MARKER_CENTER_KALMAN_Q;
        kf->r = VEHICLE_MARKER_CENTER_KALMAN_R;
        kf->k = 0.0f;
        kf->initialized = 1U;
        return kf->x;
    }

    kf->p += kf->q;
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x);
    kf->p = (1.0f - kf->k) * kf->p;

    return kf->x;
}

static void vehicle_marker_center_kalman_update(float raw_x, float raw_y,
                                                 float *filtered_x, float *filtered_y)
{
    if(filtered_x != NULL)
    {
        *filtered_x = vehicle_marker_center_kalman_update_axis(&s_vehicle_marker_center_kalman_x, raw_x);
    }
    if(filtered_y != NULL)
    {
        *filtered_y = vehicle_marker_center_kalman_update_axis(&s_vehicle_marker_center_kalman_y, raw_y);
    }
}

static void vehicle_marker_clear_pose(void)
{
    memset(&g_vehicle_marker_pose, 0, sizeof(g_vehicle_marker_pose));
}

static void vehicle_marker_clear_tracked_center(void)
{
    g_vehicle_marker_pose.x = 0.0f;
    g_vehicle_marker_pose.y = 0.0f;
    g_vehicle_marker_pose.raw_x = 0.0f;
    g_vehicle_marker_pose.raw_y = 0.0f;
    g_vehicle_marker_pose.detected_point_count = 0U;
    g_vehicle_marker_pose.reconstructed_point = 0U;
    g_vehicle_marker_pose.valid = 0U;
}

static void vehicle_marker_mark_position_lost(void)
{
    s_vehicle_marker_debug_mark_lost_count++;
    s_vehicle_marker_has_trackable_pose = 0U;
    s_vehicle_marker_lost_elapsed_ms = VEHICLE_MARKER_POSITION_LOST_HOLD_MS;
    g_vehicle_marker_position_lost_elapsed_ms = s_vehicle_marker_lost_elapsed_ms;
    g_vehicle_marker_position_lost_flag = 1U;
    vehicle_marker_center_kalman_reset();
    vehicle_marker_clear_tracked_center();
}

static void vehicle_marker_set_position_lost(void)
{
    vehicle_marker_mark_position_lost();
    vehicle_marker_clear_pose();
}

static void vehicle_marker_copy_pose(VehicleMarkerPose_t *out_pose)
{
    if(out_pose != NULL)
    {
        *out_pose = g_vehicle_marker_pose;
    }
}

static void vehicle_marker_reset_loss_debug(void)
{
    s_vehicle_marker_debug_raw_detect_valid = 0U;
    s_vehicle_marker_debug_task_5ms_count = 0U;
    s_vehicle_marker_debug_process_count = 0U;
    s_vehicle_marker_debug_process_success_count = 0U;
    s_vehicle_marker_debug_process_fail_count = 0U;
    s_vehicle_marker_debug_mark_lost_count = 0U;
    s_vehicle_marker_debug_timeout_count = 0U;
}

void VehicleMarkerLocalization_GetLossDebug(VehicleMarkerLossDebug_t *debug)
{
    if(debug == NULL)
    {
        return;
    }

    debug->raw_detect_valid = s_vehicle_marker_debug_raw_detect_valid;
    debug->has_trackable_pose = s_vehicle_marker_has_trackable_pose;
    debug->lost_flag = g_vehicle_marker_position_lost_flag;
    debug->pose_valid = g_vehicle_marker_pose.valid;
    debug->lost_elapsed_ms = g_vehicle_marker_position_lost_elapsed_ms;
    debug->task_5ms_count = s_vehicle_marker_debug_task_5ms_count;
    debug->process_count = s_vehicle_marker_debug_process_count;
    debug->process_success_count = s_vehicle_marker_debug_process_success_count;
    debug->process_fail_count = s_vehicle_marker_debug_process_fail_count;
    debug->mark_lost_count = s_vehicle_marker_debug_mark_lost_count;
    debug->timeout_count = s_vehicle_marker_debug_timeout_count;
}

void VehicleMarkerLocalization_SetLossDebug(const VehicleMarkerLossDebug_t *debug)
{
    if(debug == NULL)
    {
        return;
    }

    s_vehicle_marker_debug_raw_detect_valid = debug->raw_detect_valid;
    s_vehicle_marker_has_trackable_pose = debug->has_trackable_pose;
    g_vehicle_marker_position_lost_flag = debug->lost_flag;
    g_vehicle_marker_pose.valid = debug->pose_valid;
    s_vehicle_marker_lost_elapsed_ms = debug->lost_elapsed_ms;
    g_vehicle_marker_position_lost_elapsed_ms = debug->lost_elapsed_ms;
    s_vehicle_marker_debug_task_5ms_count = debug->task_5ms_count;
    s_vehicle_marker_debug_process_count = debug->process_count;
    s_vehicle_marker_debug_process_success_count = debug->process_success_count;
    s_vehicle_marker_debug_process_fail_count = debug->process_fail_count;
    s_vehicle_marker_debug_mark_lost_count = debug->mark_lost_count;
    s_vehicle_marker_debug_timeout_count = debug->timeout_count;
}

void VehicleMarkerLocalization_GetChevronDebug(VehicleMarkerChevronDebug_t *debug)
{
    if(debug == NULL)
    {
        return;
    }

    *debug = s_vehicle_marker_chevron_debug;
}

void VehicleMarkerLocalization_Init(void)
{
    vehicle_marker_set_position_lost();
    g_vehicle_marker_pose_update_seq = 0U;
    vehicle_marker_reset_loss_debug();
    vehicle_marker_chevron_reset_debug();
}

void VehicleMarkerLocalization_Task5ms(void)
{
    uint32 next_elapsed_ms = 0U;

    s_vehicle_marker_debug_task_5ms_count++;

    if(!s_vehicle_marker_has_trackable_pose)
    {
        if(!g_vehicle_marker_position_lost_flag)
        {
            vehicle_marker_set_position_lost();
        }
        return;
    }

    next_elapsed_ms = (uint32)s_vehicle_marker_lost_elapsed_ms +
                      VEHICLE_MARKER_TASK_PERIOD_MS;
    if(next_elapsed_ms >= VEHICLE_MARKER_POSITION_LOST_HOLD_MS)
    {
        s_vehicle_marker_debug_timeout_count++;
        vehicle_marker_mark_position_lost();
        return;
    }

    s_vehicle_marker_lost_elapsed_ms = (uint16)next_elapsed_ms;
    g_vehicle_marker_position_lost_elapsed_ms = s_vehicle_marker_lost_elapsed_ms;
    g_vehicle_marker_position_lost_flag = 0U;
    g_vehicle_marker_pose.valid = 1U;
}

void VehicleMarkerLocalization_ServiceLostProtection(void)
{
    if(g_vehicle_marker_position_lost_flag)
    {
        vehicle_marker_clear_tracked_center();
    }
}

#define VEHICLE_MARKER_CHEVRON_REF_HEIGHT_CM              (100.0f)
#define VEHICLE_MARKER_CHEVRON_HEIGHT_MIN_CM              (35.0f)
#define VEHICLE_MARKER_CHEVRON_HEIGHT_MAX_CM              (300.0f)
#define VEHICLE_MARKER_CHEVRON_MIN_AREA_PX                (8)
#define VEHICLE_MARKER_CHEVRON_MAX_AREA_PX                (2200)
#define VEHICLE_MARKER_CHEVRON_AREA_MIN_FLOOR_PX          (4)
#define VEHICLE_MARKER_CHEVRON_AREA_MAX_CEILING_PX        (4000)
#define VEHICLE_MARKER_CHEVRON_MIN_OCCUPANCY              (0.10f)
#define VEHICLE_MARKER_CHEVRON_MAX_OCCUPANCY              (0.85f)
#define VEHICLE_MARKER_CHEVRON_MAX_CIRCULARITY            (0.82f)
#define VEHICLE_MARKER_CHEVRON_SCORE_AREA_WEIGHT           (1.5f)
#define VEHICLE_MARKER_CHEVRON_SCORE_OCCUPANCY_WEIGHT      (1.2f)
#define VEHICLE_MARKER_CHEVRON_SCORE_CIRCULARITY_WEIGHT    (3.0f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_MIN_AREA_CM2       (160.0f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_MAX_AREA_CM2       (900.0f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_EXPECTED_AREA_CM2  (500.0f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_MIN_OCCUPANCY      (0.30f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_MAX_OCCUPANCY      (1.0f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_EXPECTED_OCCUPANCY (0.78f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_MAX_CIRCULARITY    (0.55f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_EXPECTED_CIRCULARITY (0.30f)
#define VEHICLE_MARKER_CHEVRON_MIN_ARM_CM                  (10.5f)
#define VEHICLE_MARKER_CHEVRON_MAX_ARM_CM                  (42.0f)
#define VEHICLE_MARKER_CHEVRON_MIN_ARM_RATIO               (0.55f)
#define VEHICLE_MARKER_CHEVRON_MAX_ARM_RATIO               (2.90f)
#define VEHICLE_MARKER_CHEVRON_EXPECTED_ANGLE_DEG          (70.0f)
#define VEHICLE_MARKER_CHEVRON_EXPECTED_ANGLE_COS          (0.34202014f)
/* 1/sin(70 deg)，将目标附近的余弦差近似换算为弧度误差。 */
#define VEHICLE_MARKER_CHEVRON_COS_ERROR_TO_RAD            (1.06417777f)
#define VEHICLE_MARKER_CHEVRON_MIN_ANGLE_DEG               (30.0f)
#define VEHICLE_MARKER_CHEVRON_MAX_ANGLE_DEG               (90.0f)
#define VEHICLE_MARKER_CHEVRON_MIN_BASE_CM                 (15.0f)
#define VEHICLE_MARKER_CHEVRON_MAX_BASE_CM                 (46.0f)
#define VEHICLE_MARKER_CHEVRON_MIN_TRIANGLE_HEIGHT_CM      (8.0f)
#define VEHICLE_MARKER_CHEVRON_MAX_TRIANGLE_HEIGHT_CM      (24.0f)
#define VEHICLE_MARKER_CHEVRON_MIN_CONTOUR_POINTS          (8U)
#define VEHICLE_MARKER_CHEVRON_POINT_EPS                   (1e-4f)
#define VEHICLE_MARKER_CHEVRON_SEED_SAMPLE_POINTS          (64U)
#define VEHICLE_MARKER_CHEVRON_OPEN_V_SAMPLE_COUNT         (4U)
#define VEHICLE_MARKER_CHEVRON_OPEN_V_WINDOW_RADIUS        (1)
#define VEHICLE_MARKER_CHEVRON_OPEN_V_ANGLE_MARGIN_RAD     (0.15f)
#define VEHICLE_MARKER_CHEVRON_MAX_FRAME_CANDIDATES        (4U)

#if VEHICLE_MARKER_CHEVRON_SEED_SAMPLE_POINTS < 3U || \
    VEHICLE_MARKER_CHEVRON_SEED_SAMPLE_POINTS > VISION_RUNTIME_CONTOUR_MAX_POINTS
#error "Invalid chevron seed contour sample count"
#endif

typedef struct
{
    float min_area;
    float max_area;
    float expected_area;
    float min_occupancy;
    float max_occupancy;
    float max_circularity;
    uint8 physical;
} VehicleMarkerChevronGate_t;

typedef struct
{
    VisionBlob_t blob;
    VisionContour_t contour;
    float score;
    uint8 valid;
    uint8 geometry_checked;
    uint8 geometry_valid;
} VehicleMarkerChevronCandidate_t;

typedef struct
{
    float arm_support;
    float center_gap;
    float score;
} VehicleMarkerOpenVMetrics_t;

#pragma location = 0x20000000
__no_init static VehicleMarkerChevronCandidate_t
    s_vehicle_marker_chevron_candidates[VEHICLE_MARKER_CHEVRON_MAX_FRAME_CANDIDATES];
static uint8 s_vehicle_marker_chevron_candidate_count = 0U;
static uint8 s_vehicle_marker_chevron_last_candidate_index = 0U;
static uint8 s_vehicle_marker_chevron_last_candidate_stored = 0U;
static uint8 s_vehicle_marker_chevron_cached_pose_valid = 0U;
static volatile uint32 s_vehicle_marker_chevron_reject_area_low_count = 0U;
static volatile uint32 s_vehicle_marker_chevron_reject_area_high_count = 0U;
static volatile uint32 s_vehicle_marker_chevron_reject_size_count = 0U;
static volatile uint32 s_vehicle_marker_chevron_reject_circularity_count = 0U;
static volatile uint32 s_vehicle_marker_chevron_accept_count = 0U;
static volatile uint32 s_vehicle_marker_chevron_geometry_reject_count = 0U;

static float vehicle_marker_chevron_clamp_float(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

static int vehicle_marker_chevron_clamp_int(int value, int min_value, int max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

static int vehicle_marker_chevron_round_to_int(float value)
{
    return (int)(value + 0.5f);
}

static void vehicle_marker_chevron_reset_debug(void)
{
    memset(s_vehicle_marker_chevron_candidates, 0, sizeof(s_vehicle_marker_chevron_candidates));
    s_vehicle_marker_chevron_candidate_count = 0U;
    s_vehicle_marker_chevron_last_candidate_index = 0U;
    s_vehicle_marker_chevron_last_candidate_stored = 0U;
    s_vehicle_marker_chevron_cached_pose_valid = 0U;
    memset(&s_vehicle_marker_chevron_debug, 0, sizeof(s_vehicle_marker_chevron_debug));
    s_vehicle_marker_chevron_reject_area_low_count = 0U;
    s_vehicle_marker_chevron_reject_area_high_count = 0U;
    s_vehicle_marker_chevron_reject_size_count = 0U;
    s_vehicle_marker_chevron_reject_circularity_count = 0U;
    s_vehicle_marker_chevron_accept_count = 0U;
    s_vehicle_marker_chevron_geometry_reject_count = 0U;
}

static void vehicle_marker_chevron_get_gate(const VisionBlob_t *blob,
                                            VehicleMarkerChevronGate_t *gate)
{
    float height_cm = g_ipm.height;
    float linear_scale = 1.0f;
    float area_scale = 1.0f;
    int min_area = VEHICLE_MARKER_CHEVRON_MIN_AREA_PX;
    int max_area = VEHICLE_MARKER_CHEVRON_MAX_AREA_PX;

    if(blob == NULL || gate == NULL)
    {
        return;
    }

    memset(gate, 0, sizeof(*gate));
    if(blob->physical_valid)
    {
        gate->min_area = VEHICLE_MARKER_CHEVRON_PHYSICAL_MIN_AREA_CM2;
        gate->max_area = VEHICLE_MARKER_CHEVRON_PHYSICAL_MAX_AREA_CM2;
        gate->expected_area = VEHICLE_MARKER_CHEVRON_PHYSICAL_EXPECTED_AREA_CM2;
        gate->min_occupancy = VEHICLE_MARKER_CHEVRON_PHYSICAL_MIN_OCCUPANCY;
        gate->max_occupancy = VEHICLE_MARKER_CHEVRON_PHYSICAL_MAX_OCCUPANCY;
        gate->max_circularity = VEHICLE_MARKER_CHEVRON_PHYSICAL_MAX_CIRCULARITY;
        gate->physical = 1U;
        return;
    }

    if(height_cm <= 1.0f)
    {
        height_cm = VEHICLE_MARKER_CHEVRON_REF_HEIGHT_CM;
    }

    height_cm = vehicle_marker_chevron_clamp_float(height_cm,
                                                    VEHICLE_MARKER_CHEVRON_HEIGHT_MIN_CM,
                                                    VEHICLE_MARKER_CHEVRON_HEIGHT_MAX_CM);
    linear_scale = VEHICLE_MARKER_CHEVRON_REF_HEIGHT_CM / height_cm;
    area_scale = linear_scale * linear_scale;

    min_area = vehicle_marker_chevron_round_to_int((float)VEHICLE_MARKER_CHEVRON_MIN_AREA_PX * area_scale);
    max_area = vehicle_marker_chevron_round_to_int((float)VEHICLE_MARKER_CHEVRON_MAX_AREA_PX * area_scale);

    min_area = vehicle_marker_chevron_clamp_int(min_area,
                                                VEHICLE_MARKER_CHEVRON_AREA_MIN_FLOOR_PX,
                                                VEHICLE_MARKER_CHEVRON_MAX_AREA_PX);
    max_area = vehicle_marker_chevron_clamp_int(max_area,
                                                min_area,
                                                VEHICLE_MARKER_CHEVRON_AREA_MAX_CEILING_PX);
    gate->min_area = (float)min_area;
    gate->max_area = (float)max_area;
    gate->expected_area = sqrtf(gate->min_area * gate->max_area);
    if(gate->expected_area < 1.0f)
    {
        gate->expected_area = 1.0f;
    }
    gate->min_occupancy = VEHICLE_MARKER_CHEVRON_MIN_OCCUPANCY;
    gate->max_occupancy = VEHICLE_MARKER_CHEVRON_MAX_OCCUPANCY;
    gate->max_circularity = VEHICLE_MARKER_CHEVRON_MAX_CIRCULARITY;
}

static float vehicle_marker_chevron_point_distance_sq(const VehicleMarkerWorldPoint_t *a,
                                                      const VehicleMarkerWorldPoint_t *b)
{
    float dx = b->x - a->x;
    float dy = b->y - a->y;
    return dx * dx + dy * dy;
}

static uint8 vehicle_marker_chevron_world_to_image(
    const VehicleMarkerWorldPoint_t *world, VehicleMarkerPoint2D_t *img)
{
    int img_x = 0;
    int img_y = 0;

    if(world == NULL || img == NULL)
    {
        return 0U;
    }
    if(!IPM_Plane2FullRectifiedImage(
           world->x, world->y, VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
           &img_x, &img_y))
    {
        return 0U;
    }
    if(img_x < 0) img_x = 0;
    else if(img_x >= (int)VISION_RUNTIME_FINE_WIDTH)
        img_x = VISION_RUNTIME_FINE_WIDTH - 1;
    if(img_y < 0) img_y = 0;
    else if(img_y >= (int)VISION_RUNTIME_FINE_HEIGHT)
        img_y = VISION_RUNTIME_FINE_HEIGHT - 1;
    img->u = (float)img_x;
    img->v = (float)img_y;
    img->intensity = 0.0f;
    return 1U;
}

static float vehicle_marker_chevron_score_blob(const VisionBlob_t *blob,
                                               const VehicleMarkerChevronGate_t *gate)
{
    float area = 0.0f;
    float occupancy = 0.0f;
    float circularity = 0.0f;
    float area_error = 0.0f;
    float occupancy_error = 0.0f;
    float circularity_error = 0.0f;

    if(blob == NULL || gate == NULL)
    {
        return 1e30f;
    }

    area = gate->physical ? blob->physical_area_cm2 : (float)blob->area;
    occupancy = gate->physical ? blob->physical_occupancy_ratio : blob->occupancy_ratio;
    circularity = gate->physical ? blob->physical_circularity : blob->circularity;
    area_error = fabsf(area - gate->expected_area) / gate->expected_area;
    if(gate->physical)
    {
        occupancy_error = fabsf(occupancy - VEHICLE_MARKER_CHEVRON_PHYSICAL_EXPECTED_OCCUPANCY);
        circularity_error = fabsf(circularity - VEHICLE_MARKER_CHEVRON_PHYSICAL_EXPECTED_CIRCULARITY);
    }
    else if(occupancy < gate->min_occupancy)
    {
        occupancy_error = gate->min_occupancy - occupancy;
    }
    else if(occupancy > gate->max_occupancy)
    {
        occupancy_error = occupancy - gate->max_occupancy;
    }
    if(!gate->physical)
    {
        circularity_error = circularity;
    }

    return area_error * VEHICLE_MARKER_CHEVRON_SCORE_AREA_WEIGHT +
           occupancy_error * VEHICLE_MARKER_CHEVRON_SCORE_OCCUPANCY_WEIGHT +
           circularity_error * VEHICLE_MARKER_CHEVRON_SCORE_CIRCULARITY_WEIGHT;
}

static uint8 vehicle_marker_chevron_blob_pass(const VisionBlob_t *blob,
                                              const VehicleMarkerChevronGate_t *gate)
{
    float area = 0.0f;
    float occupancy = 0.0f;
    float circularity = 0.0f;
    int width = 0;
    int height = 0;

    if(blob == NULL || gate == NULL)
    {
        return 0U;
    }

    width = blob->max_col - blob->min_col + 1;
    height = blob->max_row - blob->min_row + 1;
    area = gate->physical ? blob->physical_area_cm2 : (float)blob->area;
    occupancy = gate->physical ? blob->physical_occupancy_ratio : blob->occupancy_ratio;
    circularity = gate->physical ? blob->physical_circularity : blob->circularity;

    if(area < gate->min_area)
    {
        s_vehicle_marker_chevron_reject_area_low_count++;
        return 0U;
    }
    if(area > gate->max_area)
    {
        s_vehicle_marker_chevron_reject_area_high_count++;
        return 0U;
    }
    if(width < 2 || height < 2)
    {
        s_vehicle_marker_chevron_reject_size_count++;
        return 0U;
    }
    if(occupancy < gate->min_occupancy || occupancy > gate->max_occupancy)
    {
        s_vehicle_marker_chevron_reject_size_count++;
        return 0U;
    }
    if(circularity > gate->max_circularity)
    {
        s_vehicle_marker_chevron_reject_circularity_count++;
        return 0U;
    }

    return 1U;
}

static uint8 vehicle_marker_chevron_store_pose(const VehicleMarkerWorldPoint_t *apex,
                                               const VehicleMarkerWorldPoint_t *left_tip,
                                               const VehicleMarkerWorldPoint_t *right_tip,
                                               const VehicleMarkerPoint2D_t *left_line_start_img,
                                               const VehicleMarkerPoint2D_t *left_line_end_img,
                                               const VehicleMarkerPoint2D_t *right_line_start_img,
                                               const VehicleMarkerPoint2D_t *right_line_end_img,
                                               float heading_deg,
                                               float tip_span_cm,
                                               float estimated_z,
                                               VehicleMarkerPose_t *out_pose)
{
    float center_x = 0.0f;
    float center_y = 0.0f;
    float filtered_center_x = 0.0f;
    float filtered_center_y = 0.0f;
    VehicleMarkerPoint2D_t apex_img = {0};
    VehicleMarkerPoint2D_t left_tip_img = {0};
    VehicleMarkerPoint2D_t right_tip_img = {0};

    if(apex == NULL || left_tip == NULL || right_tip == NULL)
    {
        return 0U;
    }

    if(left_line_start_img == NULL || left_line_end_img == NULL ||
       right_line_start_img == NULL || right_line_end_img == NULL)
    {
        return 0U;
    }

    /* The world projection can land exactly on an inverse-IPM boundary even
     * when the original contour point was valid.  Prefer the already sampled
     * image endpoints in that case instead of discarding an otherwise valid
     * chevron. */
    if(!vehicle_marker_chevron_world_to_image(apex, &apex_img))
    {
        apex_img = *left_line_start_img;
    }
    if(!vehicle_marker_chevron_world_to_image(left_tip, &left_tip_img))
    {
        left_tip_img = *left_line_end_img;
    }
    if(!vehicle_marker_chevron_world_to_image(right_tip, &right_tip_img))
    {
        right_tip_img = *right_line_end_img;
    }

    /* 小车中心使用箭头三顶点的几何重心；无人机跟随偏移由位置环单独计算。 */
    center_x = (apex->x + left_tip->x + right_tip->x) / 3.0f;
    center_y = (apex->y + left_tip->y + right_tip->y) / 3.0f;
    vehicle_marker_center_kalman_update(center_x, center_y,
                                        &filtered_center_x, &filtered_center_y);

    g_vehicle_marker_pose.raw_x = center_x;
    g_vehicle_marker_pose.raw_y = center_y;
    g_vehicle_marker_pose.x = filtered_center_x;
    g_vehicle_marker_pose.y = filtered_center_y;
    g_vehicle_marker_pose.z = estimated_z;
    g_vehicle_marker_pose.heading_deg = heading_deg;
    g_vehicle_marker_pose.square_side_cm = tip_span_cm;
    g_vehicle_marker_pose.detected_point_count = 3U;
    g_vehicle_marker_pose.reconstructed_point = 0U;
    g_vehicle_marker_pose.valid = 1U;
    g_vehicle_marker_pose.img_pts[0] = apex_img;
    g_vehicle_marker_pose.img_pts[1] = left_tip_img;
    g_vehicle_marker_pose.img_pts[2] = right_tip_img;
    g_vehicle_marker_pose.img_pts[3].u = 0.0f;
    g_vehicle_marker_pose.img_pts[3].v = 0.0f;
    g_vehicle_marker_pose.img_pts[3].intensity = 0.0f;
    g_vehicle_marker_pose.world_pts[0] = *apex;
    g_vehicle_marker_pose.world_pts[1] = *left_tip;
    g_vehicle_marker_pose.world_pts[2] = *right_tip;
    g_vehicle_marker_pose.world_pts[3].x = 0.0f;
    g_vehicle_marker_pose.world_pts[3].y = 0.0f;

    s_vehicle_marker_chevron_debug.valid = 1U;
    s_vehicle_marker_chevron_debug.left_valid = 1U;
    s_vehicle_marker_chevron_debug.right_valid = 1U;
    s_vehicle_marker_chevron_debug.apex_img = apex_img;
    s_vehicle_marker_chevron_debug.left_tip_img = left_tip_img;
    s_vehicle_marker_chevron_debug.right_tip_img = right_tip_img;
    s_vehicle_marker_chevron_debug.left_line_start_img = *left_line_start_img;
    s_vehicle_marker_chevron_debug.left_line_end_img = *left_line_end_img;
    s_vehicle_marker_chevron_debug.right_line_start_img = *right_line_start_img;
    s_vehicle_marker_chevron_debug.right_line_end_img = *right_line_end_img;
    s_vehicle_marker_chevron_debug.heading_deg = heading_deg;

    s_vehicle_marker_has_trackable_pose = 1U;
    s_vehicle_marker_lost_elapsed_ms = 0U;
    g_vehicle_marker_position_lost_elapsed_ms = 0U;
    g_vehicle_marker_position_lost_flag = 0U;
    g_vehicle_marker_pose_update_seq++;

    if(out_pose != NULL)
    {
        *out_pose = g_vehicle_marker_pose;
    }

    return 1U;
}

uint8 VehicleMarkerLocalization_GetFollowPoint(
    const VehicleMarkerPose_t *pose,
    float *filtered_x, float *filtered_y,
    float *raw_x, float *raw_y)
{
    float rear_x;
    float rear_y;
    float rear_length;
    float offset_x;
    float offset_y;

    if((pose == NULL) || (filtered_x == NULL) || (filtered_y == NULL) ||
       ((raw_x == NULL) != (raw_y == NULL)) ||
       (pose->detected_point_count < 3U))
    {
        return 0U;
    }

    rear_x = 0.5f * (pose->world_pts[1].x + pose->world_pts[2].x) -
             pose->world_pts[0].x;
    rear_y = 0.5f * (pose->world_pts[1].y + pose->world_pts[2].y) -
             pose->world_pts[0].y;
    rear_length = sqrtf(rear_x * rear_x + rear_y * rear_y);
    if(!isfinite(rear_length) ||
       (rear_length <= VEHICLE_MARKER_CHEVRON_POINT_EPS))
    {
        return 0U;
    }

    offset_x = VEHICLE_MARKER_FOLLOW_REAR_OFFSET_CM * rear_x / rear_length;
    offset_y = VEHICLE_MARKER_FOLLOW_REAR_OFFSET_CM * rear_y / rear_length;
    *filtered_x = pose->x + offset_x;
    *filtered_y = pose->y + offset_y;
    if(raw_x != NULL)
    {
        *raw_x = pose->raw_x + offset_x;
        *raw_y = pose->raw_y + offset_y;
        if(!isfinite(*raw_x) || !isfinite(*raw_y))
        {
            return 0U;
        }
    }
    return (uint8)(isfinite(*filtered_x) && isfinite(*filtered_y));
}

void VehicleMarkerLocalization_BeginBlobFrame(void)
{
    vehicle_marker_chevron_reset_debug();
}

void VehicleMarkerLocalization_GetBlobRejectDebug(int *area_low_count,
                                                  int *area_high_count,
                                                  int *size_count,
                                                  int *aspect_count,
                                                  int *insert_count)
{
    if(area_low_count != NULL)
    {
        *area_low_count = (int)s_vehicle_marker_chevron_reject_area_low_count;
    }
    if(area_high_count != NULL)
    {
        *area_high_count = (int)s_vehicle_marker_chevron_reject_area_high_count;
    }
    if(size_count != NULL)
    {
        *size_count = (int)s_vehicle_marker_chevron_reject_size_count;
    }
    if(aspect_count != NULL)
    {
        *aspect_count = (int)s_vehicle_marker_chevron_reject_circularity_count;
    }
    if(insert_count != NULL)
    {
        *insert_count = (int)s_vehicle_marker_chevron_accept_count;
    }
}

uint8 VehicleMarkerLocalization_AppendBlob(const VisionBlob_t *blob, const VisionContour_t *contour)
{
    VehicleMarkerChevronGate_t gate;
    float score = 0.0f;
    uint8 candidate_index = 0U;

    s_vehicle_marker_chevron_last_candidate_stored = 0U;
    if(blob == NULL || contour == NULL || contour->point_count < VEHICLE_MARKER_CHEVRON_MIN_CONTOUR_POINTS)
    {
        s_vehicle_marker_chevron_reject_size_count++;
        return 0U;
    }
    if(!blob->world_valid)
    {
        s_vehicle_marker_chevron_reject_size_count++;
        return 0U;
    }

    vehicle_marker_chevron_get_gate(blob, &gate);
    if(!vehicle_marker_chevron_blob_pass(blob, &gate))
    {
        return 0U;
    }

    score = vehicle_marker_chevron_score_blob(blob, &gate);
    if(s_vehicle_marker_chevron_candidate_count < VEHICLE_MARKER_CHEVRON_MAX_FRAME_CANDIDATES)
    {
        candidate_index = s_vehicle_marker_chevron_candidate_count++;
    }
    else
    {
        uint8 i;
        float worst_score = s_vehicle_marker_chevron_candidates[0].score;

        candidate_index = 0U;
        for(i = 1U; i < VEHICLE_MARKER_CHEVRON_MAX_FRAME_CANDIDATES; ++i)
        {
            if(s_vehicle_marker_chevron_candidates[i].score > worst_score)
            {
                candidate_index = i;
                worst_score = s_vehicle_marker_chevron_candidates[i].score;
            }
        }
        if(score >= worst_score)
        {
            s_vehicle_marker_chevron_accept_count++;
            return 1U;
        }
    }

    s_vehicle_marker_chevron_candidates[candidate_index].blob = *blob;
    s_vehicle_marker_chevron_candidates[candidate_index].contour = *contour;
    s_vehicle_marker_chevron_candidates[candidate_index].score = score;
    s_vehicle_marker_chevron_candidates[candidate_index].valid = 1U;
    s_vehicle_marker_chevron_candidates[candidate_index].geometry_checked = 0U;
    s_vehicle_marker_chevron_candidates[candidate_index].geometry_valid = 0U;
    s_vehicle_marker_chevron_last_candidate_index = candidate_index;
    s_vehicle_marker_chevron_last_candidate_stored = 1U;
    s_vehicle_marker_chevron_accept_count++;
    return 1U;
}

static float vehicle_marker_chevron_image_distance_sq(const VehicleMarkerPoint2D_t *a,
                                                      const VehicleMarkerPoint2D_t *b)
{
    float dx = b->u - a->u;
    float dy = b->v - a->v;
    return dx * dx + dy * dy;
}

static float vehicle_marker_chevron_image_point_to_line_distance_sq(
    const VehicleMarkerPoint2D_t *point,
    const VehicleMarkerPoint2D_t *line_start,
    const VehicleMarkerPoint2D_t *line_end)
{
    float line_x = line_end->u - line_start->u;
    float line_y = line_end->v - line_start->v;
    float point_x = point->u - line_start->u;
    float point_y = point->v - line_start->v;
    float cross = line_x * point_y - line_y * point_x;
    float length_sq = line_x * line_x + line_y * line_y;

    if(length_sq <= VEHICLE_MARKER_CHEVRON_POINT_EPS) return 1e30f;
    return (cross * cross) / length_sq;
}

static uint8 vehicle_marker_chevron_binary_get(
    const VehicleMarkerBinaryView_t *view, int x, int y)
{
    uint32 byte_index;
    uint8 mask;

    if(view == NULL || view->data == NULL || view->stride == 0U ||
       x < 0 || y < 0 || x >= (int)view->width || y >= (int)view->height)
    {
        return 0U;
    }
    byte_index = (uint32)y * (uint32)view->stride + (uint32)(x >> 3);
    mask = (uint8)(0x80U >> (x & 7));
    return (view->data[byte_index] & mask) ? 1U : 0U;
}

static void vehicle_marker_chevron_refine_point_3x3(
    const VehicleMarkerBinaryView_t *view,
    const VehicleMarkerPoint2D_t *input,
    VehicleMarkerPoint2D_t *output)
{
    int center_x;
    int center_y;
    int sum_x = 0;
    int sum_y = 0;
    int count = 0;
    int dx;
    int dy;

    if(input == NULL || output == NULL)
    {
        return;
    }
    *output = *input;
    if(view == NULL || view->data == NULL)
    {
        return;
    }
    center_x = (int)(input->u + 0.5f);
    center_y = (int)(input->v + 0.5f);
    for(dy = -1; dy <= 1; ++dy)
    {
        for(dx = -1; dx <= 1; ++dx)
        {
            int x = center_x + dx;
            int y = center_y + dy;
            if(vehicle_marker_chevron_binary_get(view, x, y))
            {
                sum_x += x;
                sum_y += y;
                ++count;
            }
        }
    }
    if(count > 0)
    {
        output->u = (float)sum_x / (float)count;
        output->v = (float)sum_y / (float)count;
        output->intensity = 0.0f;
    }
}

static float vehicle_marker_chevron_sample_foreground(
    const VehicleMarkerBinaryView_t *view, float u, float v)
{
    int center_x = (int)(u + 0.5f);
    int center_y = (int)(v + 0.5f);
    int foreground = 0;
    int valid = 0;
    int dy;
    int dx;

    for(dy = -VEHICLE_MARKER_CHEVRON_OPEN_V_WINDOW_RADIUS;
        dy <= VEHICLE_MARKER_CHEVRON_OPEN_V_WINDOW_RADIUS; ++dy)
    {
        for(dx = -VEHICLE_MARKER_CHEVRON_OPEN_V_WINDOW_RADIUS;
            dx <= VEHICLE_MARKER_CHEVRON_OPEN_V_WINDOW_RADIUS; ++dx)
        {
            int x = center_x + dx;
            int y = center_y + dy;
            if(x < 0 || y < 0 || x >= (int)view->width || y >= (int)view->height)
            {
                continue;
            }
            foreground += vehicle_marker_chevron_binary_get(view, x, y);
            ++valid;
        }
    }
    return (valid > 0) ? (float)foreground / (float)valid : 0.0f;
}

static void vehicle_marker_chevron_measure_open_v(
    const VehicleMarkerPoint2D_t *apex,
    const VehicleMarkerPoint2D_t *left_tip,
    const VehicleMarkerPoint2D_t *right_tip,
    const VehicleMarkerBinaryView_t *view,
    VehicleMarkerOpenVMetrics_t *metrics)
{
    static const float sample_t[VEHICLE_MARKER_CHEVRON_OPEN_V_SAMPLE_COUNT] = {
        0.40f, 0.55f, 0.70f, 0.85f
    };
    float arm_support_sum = 0.0f;
    float center_gap_sum = 0.0f;
    float score_sum = 0.0f;
    uint8 i;

    if(metrics == NULL)
    {
        return;
    }
    memset(metrics, 0, sizeof(*metrics));
    if(apex == NULL || left_tip == NULL || right_tip == NULL ||
       view == NULL || view->data == NULL)
    {
        return;
    }

    for(i = 0U; i < VEHICLE_MARKER_CHEVRON_OPEN_V_SAMPLE_COUNT; ++i)
    {
        float t = sample_t[i];
        float left_u = apex->u + t * (left_tip->u - apex->u);
        float left_v = apex->v + t * (left_tip->v - apex->v);
        float right_u = apex->u + t * (right_tip->u - apex->u);
        float right_v = apex->v + t * (right_tip->v - apex->v);
        float center_u = 0.5f * (left_u + right_u);
        float center_v = 0.5f * (left_v + right_v);
        float left_support = vehicle_marker_chevron_sample_foreground(
            view, left_u, left_v);
        float right_support = vehicle_marker_chevron_sample_foreground(
            view, right_u, right_v);
        float arm_support = (left_support < right_support) ?
                            left_support : right_support;
        float center_gap = 1.0f - vehicle_marker_chevron_sample_foreground(
            view, center_u, center_v);

        arm_support_sum += arm_support;
        center_gap_sum += center_gap;
        score_sum += arm_support * center_gap;
    }

    metrics->arm_support =
        arm_support_sum / (float)VEHICLE_MARKER_CHEVRON_OPEN_V_SAMPLE_COUNT;
    metrics->center_gap =
        center_gap_sum / (float)VEHICLE_MARKER_CHEVRON_OPEN_V_SAMPLE_COUNT;
    metrics->score =
        score_sum / (float)VEHICLE_MARKER_CHEVRON_OPEN_V_SAMPLE_COUNT;
}

static uint8 vehicle_marker_chevron_image_shape_projects_valid(
    const VehicleMarkerPoint2D_t *apex_img,
    const VehicleMarkerPoint2D_t *left_tip_img,
    const VehicleMarkerPoint2D_t *right_tip_img)
{
    VehicleMarkerWorldPoint_t apex;
    VehicleMarkerWorldPoint_t left_tip;
    VehicleMarkerWorldPoint_t right_tip;
    float left_dx;
    float left_dy;
    float right_dx;
    float right_dy;
    float base_dx;
    float base_dy;
    float left_span;
    float right_span;
    float arm_ratio;
    float arm_angle;
    float base;
    float triangle_height;

    if(apex_img == NULL || left_tip_img == NULL || right_tip_img == NULL)
    {
        return 0U;
    }
    if(!IPM_FullRectifiedImage2PlaneF(
           apex_img->u, apex_img->v, VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
           &apex.x, &apex.y) ||
       !IPM_FullRectifiedImage2PlaneF(
           left_tip_img->u, left_tip_img->v,
           VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
           &left_tip.x, &left_tip.y) ||
       !IPM_FullRectifiedImage2PlaneF(
           right_tip_img->u, right_tip_img->v,
           VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
           &right_tip.x, &right_tip.y))
    {
        return 0U;
    }

    left_dx = left_tip.x - apex.x;
    left_dy = left_tip.y - apex.y;
    right_dx = right_tip.x - apex.x;
    right_dy = right_tip.y - apex.y;
    base_dx = right_tip.x - left_tip.x;
    base_dy = right_tip.y - left_tip.y;
    left_span = sqrtf(left_dx * left_dx + left_dy * left_dy);
    right_span = sqrtf(right_dx * right_dx + right_dy * right_dy);
    base = sqrtf(base_dx * base_dx + base_dy * base_dy);
    if(left_span <= VEHICLE_MARKER_CHEVRON_POINT_EPS ||
       right_span <= VEHICLE_MARKER_CHEVRON_POINT_EPS ||
       base <= VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        return 0U;
    }
    arm_ratio = left_span / right_span;
    arm_angle = acosf(vehicle_marker_chevron_clamp_float(
        (left_dx * right_dx + left_dy * right_dy) /
        (left_span * right_span), -1.0f, 1.0f)) *
        (180.0f / 3.14159265f);
    triangle_height = fabsf(base_dx * (apex.y - left_tip.y) -
                            base_dy * (apex.x - left_tip.x)) / base;

    return (uint8)(
        left_span >= VEHICLE_MARKER_CHEVRON_MIN_ARM_CM &&
        left_span <= VEHICLE_MARKER_CHEVRON_MAX_ARM_CM &&
        right_span >= VEHICLE_MARKER_CHEVRON_MIN_ARM_CM &&
        right_span <= VEHICLE_MARKER_CHEVRON_MAX_ARM_CM &&
        arm_ratio >= VEHICLE_MARKER_CHEVRON_MIN_ARM_RATIO &&
        arm_ratio <= VEHICLE_MARKER_CHEVRON_MAX_ARM_RATIO &&
        arm_angle >= VEHICLE_MARKER_CHEVRON_MIN_ANGLE_DEG &&
        arm_angle <= VEHICLE_MARKER_CHEVRON_MAX_ANGLE_DEG &&
        base >= VEHICLE_MARKER_CHEVRON_MIN_BASE_CM &&
        base <= VEHICLE_MARKER_CHEVRON_MAX_BASE_CM &&
        triangle_height >= VEHICLE_MARKER_CHEVRON_MIN_TRIANGLE_HEIGHT_CM &&
        triangle_height <= VEHICLE_MARKER_CHEVRON_MAX_TRIANGLE_HEIGHT_CM);
}

/* Build an extreme triangle, score all three apex hypotheses directly on the
 * packed binary image, then keep the best physically valid open-V geometry.
 * No contour-line fitting is performed. */
static uint8 vehicle_marker_chevron_extract_image_geometry(
    const VehicleMarkerChevronCandidate_t *candidate,
    VehicleMarkerPoint2D_t *apex,
    VehicleMarkerPoint2D_t *left_tip,
    VehicleMarkerPoint2D_t *right_tip,
    const VehicleMarkerBinaryView_t *binary_view)
{
    const VisionContour_t *contour;
    VehicleMarkerPoint2D_t triangle[3] = {{0}};
    VehicleMarkerPoint2D_t raw_apex;
    VehicleMarkerPoint2D_t raw_left_tip;
    VehicleMarkerPoint2D_t raw_right_tip;
    VehicleMarkerPoint2D_t refined_apex;
    VehicleMarkerPoint2D_t refined_left_tip;
    VehicleMarkerPoint2D_t refined_right_tip;
    uint16 seed_indices[VEHICLE_MARKER_CHEVRON_SEED_SAMPLE_POINTS];
    uint16 seed_count = 0U;
    uint16 pair_i = 0U;
    uint16 pair_j = 0U;
    uint16 third_i = 0U;
    uint16 i;
    uint16 j;
    int best_apex = 0;
    int angle_best_apex = -1;
    uint8 physical_valid[3] = {0U, 0U, 0U};
    float best_pair_dist_sq = -1.0f;
    float best_third_dist_sq = -1.0f;
    float best_angle_error = 1e30f;
    float angle_error[3] = {1e30f, 1e30f, 1e30f};
    VehicleMarkerOpenVMetrics_t open_v[3] = {{0}};
    float cross;

    if(candidate == NULL || apex == NULL || left_tip == NULL ||
       right_tip == NULL) return 0U;
    contour = &candidate->contour;
    if(contour->point_count < VEHICLE_MARKER_CHEVRON_MIN_CONTOUR_POINTS)
        return 0U;

    seed_count = contour->point_count;
    if(seed_count > VEHICLE_MARKER_CHEVRON_SEED_SAMPLE_POINTS)
        seed_count = VEHICLE_MARKER_CHEVRON_SEED_SAMPLE_POINTS;
    for(i = 0U; i < seed_count; ++i)
    {
        seed_indices[i] = (seed_count == contour->point_count) ? i :
            (uint16)(((uint32)i * (uint32)(contour->point_count - 1U)) /
                     (uint32)(seed_count - 1U));
    }

    for(i = 0U; i + 1U < seed_count; ++i)
    {
        uint16 contour_i = seed_indices[i];
        VehicleMarkerPoint2D_t point_i = {
            (float)contour->point_x[contour_i],
            (float)contour->point_y[contour_i], 0.0f};
        for(j = (uint16)(i + 1U); j < seed_count; ++j)
        {
            uint16 contour_j = seed_indices[j];
            VehicleMarkerPoint2D_t point_j = {
                (float)contour->point_x[contour_j],
                (float)contour->point_y[contour_j], 0.0f};
            float dist_sq =
                vehicle_marker_chevron_image_distance_sq(&point_i, &point_j);
            if(dist_sq > best_pair_dist_sq)
            {
                best_pair_dist_sq = dist_sq;
                pair_i = contour_i;
                pair_j = contour_j;
            }
        }
    }
    if(best_pair_dist_sq <= VEHICLE_MARKER_CHEVRON_POINT_EPS) return 0U;
    triangle[1].u = (float)contour->point_x[pair_i];
    triangle[1].v = (float)contour->point_y[pair_i];
    triangle[2].u = (float)contour->point_x[pair_j];
    triangle[2].v = (float)contour->point_y[pair_j];
    for(i = 0U; i < seed_count; ++i)
    {
        uint16 contour_i = seed_indices[i];
        VehicleMarkerPoint2D_t point = {
            (float)contour->point_x[contour_i],
            (float)contour->point_y[contour_i], 0.0f};
        float dist_sq = vehicle_marker_chevron_image_point_to_line_distance_sq(
            &point, &triangle[1], &triangle[2]);
        if(dist_sq > best_third_dist_sq)
        {
            best_third_dist_sq = dist_sq;
            third_i = contour_i;
        }
    }
    if(best_third_dist_sq <= VEHICLE_MARKER_CHEVRON_POINT_EPS) return 0U;
    triangle[0].u = (float)contour->point_x[third_i];
    triangle[0].v = (float)contour->point_y[third_i];

    for(i = 0U; i < 3U; ++i)
    {
        int a = ((int)i + 1) % 3;
        int b = ((int)i + 2) % 3;
        float ax = triangle[a].u - triangle[i].u;
        float ay = triangle[a].v - triangle[i].v;
        float bx = triangle[b].u - triangle[i].u;
        float by = triangle[b].v - triangle[i].v;
        float len_a_sq = ax * ax + ay * ay;
        float len_b_sq = bx * bx + by * by;
        float angle_cos;
        float error;
        if(len_a_sq <= 1.0f || len_b_sq <= 1.0f) continue;
        angle_cos = vehicle_marker_chevron_clamp_float(
            (ax * bx + ay * by) / sqrtf(len_a_sq * len_b_sq),
            -1.0f, 1.0f);
        error = fabsf(angle_cos - VEHICLE_MARKER_CHEVRON_EXPECTED_ANGLE_COS) *
                VEHICLE_MARKER_CHEVRON_COS_ERROR_TO_RAD;
        angle_error[i] = error;
        vehicle_marker_chevron_measure_open_v(
            &triangle[i], &triangle[a], &triangle[b], binary_view,
            &open_v[i]);
        physical_valid[i] = vehicle_marker_chevron_image_shape_projects_valid(
            &triangle[i], &triangle[a], &triangle[b]);
        if(physical_valid[i] && error < best_angle_error)
        {
            best_angle_error = error;
            angle_best_apex = (int)i;
        }
    }
    if(angle_best_apex < 0)
    {
        for(i = 0U; i < 3U; ++i)
        {
            if(angle_error[i] < best_angle_error)
            {
                best_angle_error = angle_error[i];
                angle_best_apex = (int)i;
            }
        }
    }
    if(angle_best_apex < 0)
    {
        return 0U;
    }
    best_apex = angle_best_apex;
    if(binary_view != NULL && binary_view->data != NULL)
    {
        float best_open_v_score = open_v[best_apex].score;

        for(i = 0U; i < 3U; ++i)
        {
            if(!physical_valid[i] ||
               angle_error[i] > best_angle_error +
                                  VEHICLE_MARKER_CHEVRON_OPEN_V_ANGLE_MARGIN_RAD)
            {
                continue;
            }
            if(open_v[i].score > best_open_v_score)
            {
                best_open_v_score = open_v[i].score;
                best_apex = (int)i;
            }
        }
    }

    raw_apex = triangle[best_apex];
    raw_left_tip = triangle[(best_apex + 1) % 3];
    raw_right_tip = triangle[(best_apex + 2) % 3];
    vehicle_marker_chevron_refine_point_3x3(
        binary_view, &raw_apex, &refined_apex);
    vehicle_marker_chevron_refine_point_3x3(
        binary_view, &raw_left_tip, &refined_left_tip);
    vehicle_marker_chevron_refine_point_3x3(
        binary_view, &raw_right_tip, &refined_right_tip);
    if(vehicle_marker_chevron_image_shape_projects_valid(
           &refined_apex, &refined_left_tip, &refined_right_tip))
    {
        *apex = refined_apex;
        *left_tip = refined_left_tip;
        *right_tip = refined_right_tip;
    }
    else
    {
        *apex = raw_apex;
        *left_tip = raw_left_tip;
        *right_tip = raw_right_tip;
    }

    cross = (left_tip->u - apex->u) * (right_tip->v - apex->v) -
            (left_tip->v - apex->v) * (right_tip->u - apex->u);
    if(cross < 0.0f)
    {
        VehicleMarkerPoint2D_t temp = *left_tip;
        *left_tip = *right_tip;
        *right_tip = temp;
    }
    {
        float selected_score = open_v[best_apex].score;
        float second_score = 0.0f;
        for(i = 0U; i < 3U; ++i)
        {
            if((int)i == best_apex || !physical_valid[i] ||
               angle_error[i] > best_angle_error +
                                  VEHICLE_MARKER_CHEVRON_OPEN_V_ANGLE_MARGIN_RAD)
            {
                continue;
            }
            if(open_v[i].score > second_score)
            {
                second_score = open_v[i].score;
            }
        }
        s_vehicle_marker_chevron_debug.open_v_hypothesis_score =
            selected_score;
        s_vehicle_marker_chevron_debug.open_v_hypothesis_second_score =
            second_score;
        s_vehicle_marker_chevron_debug.open_v_hypothesis_margin =
            selected_score - second_score;
        s_vehicle_marker_chevron_debug.open_v_apex_index = (uint8)best_apex;
    }
    return vehicle_marker_chevron_image_distance_sq(apex, left_tip) > 1.0f &&
           vehicle_marker_chevron_image_distance_sq(apex, right_tip) > 1.0f;
}

static uint8 vehicle_marker_chevron_process_candidate(
    const VehicleMarkerChevronCandidate_t *candidate,
    VehicleMarkerPose_t *out_pose, uint8 commit_pose,
    const VehicleMarkerBinaryView_t *binary_view,
    const float *open_v_gate_score)
{
    VehicleMarkerPoint2D_t apex_img = {0};
    VehicleMarkerPoint2D_t left_tip_img = {0};
    VehicleMarkerPoint2D_t right_tip_img = {0};
    VehicleMarkerWorldPoint_t apex = {0};
    VehicleMarkerWorldPoint_t left_tip = {0};
    VehicleMarkerWorldPoint_t right_tip = {0};
    VehicleMarkerPoint2D_t left_line_start_img = {0};
    VehicleMarkerPoint2D_t left_line_end_img = {0};
    VehicleMarkerPoint2D_t right_line_start_img = {0};
    VehicleMarkerPoint2D_t right_line_end_img = {0};
    float left_span_cm = 0.0f;
    float right_span_cm = 0.0f;
    float heading_x = 0.0f;
    float heading_y = 0.0f;
    float heading_deg = 0.0f;
    float estimated_z = g_ipm.height;
    float left_dx = 0.0f;
    float left_dy = 0.0f;
    float right_dx = 0.0f;
    float right_dy = 0.0f;
    float arm_ratio = 0.0f;
    float arm_dot = 0.0f;
    float arm_angle_deg = 0.0f;
    float base_dx = 0.0f;
    float base_dy = 0.0f;
    float base_cm = 0.0f;
    float triangle_height_cm = 0.0f;
    float effective_open_v_score = 0.0f;

    if(candidate == NULL)
    {
        return 0U;
    }

    s_vehicle_marker_chevron_debug.physical_area_cm2 = candidate->blob.physical_area_cm2;
    s_vehicle_marker_chevron_debug.physical_occupancy_ratio =
        candidate->blob.physical_occupancy_ratio;
    s_vehicle_marker_chevron_debug.physical_circularity =
        candidate->blob.physical_circularity;
    s_vehicle_marker_chevron_debug.reject_reason =
        VEHICLE_MARKER_CHEVRON_REJECT_IMAGE_GEOMETRY;

    if(!vehicle_marker_chevron_extract_image_geometry(candidate,
                                                      &apex_img,
                                                      &left_tip_img,
                                                      &right_tip_img,
                                                      binary_view))
    {
        return 0U;
    }
    s_vehicle_marker_chevron_debug.apex_img = apex_img;
    s_vehicle_marker_chevron_debug.left_tip_img = left_tip_img;
    s_vehicle_marker_chevron_debug.right_tip_img = right_tip_img;

    {
        VehicleMarkerOpenVMetrics_t fitted_open_v;
        vehicle_marker_chevron_measure_open_v(
            &apex_img, &left_tip_img, &right_tip_img, binary_view,
            &fitted_open_v);
        s_vehicle_marker_chevron_debug.open_v_arm_support =
            fitted_open_v.arm_support;
        s_vehicle_marker_chevron_debug.open_v_center_gap =
            fitted_open_v.center_gap;
        s_vehicle_marker_chevron_debug.open_v_score = fitted_open_v.score;
        effective_open_v_score = (open_v_gate_score != NULL) ?
                                 *open_v_gate_score : fitted_open_v.score;
        s_vehicle_marker_chevron_debug.open_v_gate_score =
            effective_open_v_score;
        s_vehicle_marker_chevron_debug.open_v_gate_from_primary =
            (open_v_gate_score != NULL) ? 1U : 0U;
        if((binary_view != NULL || open_v_gate_score != NULL) &&
           effective_open_v_score <
               VEHICLE_MARKER_CHEVRON_MIN_OPEN_V_SCORE)
        {
            s_vehicle_marker_chevron_debug.reject_reason =
                VEHICLE_MARKER_CHEVRON_REJECT_OPEN_V;
            return 0U;
        }
    }

    if(!IPM_FullRectifiedImage2PlaneF(
           apex_img.u, apex_img.v, VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
           &apex.x, &apex.y) ||
       !IPM_FullRectifiedImage2PlaneF(
           left_tip_img.u, left_tip_img.v,
           VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
           &left_tip.x, &left_tip.y) ||
       !IPM_FullRectifiedImage2PlaneF(
           right_tip_img.u, right_tip_img.v,
           VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
           &right_tip.x, &right_tip.y))
    {
        s_vehicle_marker_chevron_debug.reject_reason =
            VEHICLE_MARKER_CHEVRON_REJECT_WORLD_PROJECTION;
        return 0U;
    }

    left_line_start_img = apex_img;
    left_line_end_img = left_tip_img;
    right_line_start_img = apex_img;
    right_line_end_img = right_tip_img;
    left_span_cm = sqrtf(vehicle_marker_chevron_point_distance_sq(&apex, &left_tip));
    right_span_cm = sqrtf(vehicle_marker_chevron_point_distance_sq(&apex, &right_tip));

    left_dx = left_tip.x - apex.x;
    left_dy = left_tip.y - apex.y;
    right_dx = right_tip.x - apex.x;
    right_dy = right_tip.y - apex.y;
    base_dx = right_tip.x - left_tip.x;
    base_dy = right_tip.y - left_tip.y;
    base_cm = sqrtf(base_dx * base_dx + base_dy * base_dy);
    if(left_span_cm <= VEHICLE_MARKER_CHEVRON_POINT_EPS ||
       right_span_cm <= VEHICLE_MARKER_CHEVRON_POINT_EPS ||
       base_cm <= VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        s_vehicle_marker_chevron_debug.reject_reason =
            VEHICLE_MARKER_CHEVRON_REJECT_PHYSICAL_GEOMETRY;
        return 0U;
    }

    arm_ratio = left_span_cm / right_span_cm;
    arm_dot = left_dx * right_dx + left_dy * right_dy;
    arm_angle_deg = acosf(vehicle_marker_chevron_clamp_float(
        arm_dot / (left_span_cm * right_span_cm), -1.0f, 1.0f)) *
        (180.0f / 3.14159265f);
    triangle_height_cm = fabsf(base_dx * (apex.y - left_tip.y) -
                                  base_dy * (apex.x - left_tip.x)) / base_cm;

    s_vehicle_marker_chevron_debug.left_arm_cm = left_span_cm;
    s_vehicle_marker_chevron_debug.right_arm_cm = right_span_cm;
    s_vehicle_marker_chevron_debug.arm_angle_deg = arm_angle_deg;
    s_vehicle_marker_chevron_debug.base_cm = base_cm;
    s_vehicle_marker_chevron_debug.triangle_height_cm = triangle_height_cm;

    if(left_span_cm < VEHICLE_MARKER_CHEVRON_MIN_ARM_CM ||
       left_span_cm > VEHICLE_MARKER_CHEVRON_MAX_ARM_CM ||
       right_span_cm < VEHICLE_MARKER_CHEVRON_MIN_ARM_CM ||
       right_span_cm > VEHICLE_MARKER_CHEVRON_MAX_ARM_CM ||
       arm_ratio < VEHICLE_MARKER_CHEVRON_MIN_ARM_RATIO ||
       arm_ratio > VEHICLE_MARKER_CHEVRON_MAX_ARM_RATIO ||
       arm_angle_deg < VEHICLE_MARKER_CHEVRON_MIN_ANGLE_DEG ||
       arm_angle_deg > VEHICLE_MARKER_CHEVRON_MAX_ANGLE_DEG ||
       base_cm < VEHICLE_MARKER_CHEVRON_MIN_BASE_CM ||
       base_cm > VEHICLE_MARKER_CHEVRON_MAX_BASE_CM ||
       triangle_height_cm < VEHICLE_MARKER_CHEVRON_MIN_TRIANGLE_HEIGHT_CM ||
       triangle_height_cm > VEHICLE_MARKER_CHEVRON_MAX_TRIANGLE_HEIGHT_CM)
    {
        s_vehicle_marker_chevron_debug.reject_reason =
            VEHICLE_MARKER_CHEVRON_REJECT_PHYSICAL_GEOMETRY;
        return 0U;
    }

    s_vehicle_marker_chevron_debug.reject_reason = VEHICLE_MARKER_CHEVRON_REJECT_NONE;
    heading_x = left_dx + right_dx;
    heading_y = left_dy + right_dy;
    if((heading_x * heading_x + heading_y * heading_y) < VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        heading_x = left_dx;
        heading_y = left_dy;
    }
    heading_deg = atan2f(heading_y, heading_x) * (180.0f / 3.14159265f);

    if(estimated_z <= 1.0f)
    {
        estimated_z = VEHICLE_MARKER_CHEVRON_REF_HEIGHT_CM;
    }

    if(!commit_pose)
    {
        return 1U;
    }

    if(!vehicle_marker_chevron_store_pose(&apex,
                                          &left_tip,
                                          &right_tip,
                                          &left_line_start_img,
                                          &left_line_end_img,
                                          &right_line_start_img,
                                          &right_line_end_img,
                                          heading_deg,
                                          0.5f * (left_span_cm + right_span_cm),
                                          estimated_z,
                                          out_pose))
    {
        return 0U;
    }

    /* Preserve the exact contour geometry for the UI instead of the rounded
     * world-to-image reprojection performed by vehicle_marker_chevron_store_pose. */
    g_vehicle_marker_pose.img_pts[0] = apex_img;
    g_vehicle_marker_pose.img_pts[1] = left_tip_img;
    g_vehicle_marker_pose.img_pts[2] = right_tip_img;
    s_vehicle_marker_chevron_debug.apex_img = apex_img;
    s_vehicle_marker_chevron_debug.left_tip_img = left_tip_img;
    s_vehicle_marker_chevron_debug.right_tip_img = right_tip_img;
    s_vehicle_marker_chevron_debug.left_line_start_img = left_line_start_img;
    s_vehicle_marker_chevron_debug.left_line_end_img = left_line_end_img;
    s_vehicle_marker_chevron_debug.right_line_start_img = right_line_start_img;
    s_vehicle_marker_chevron_debug.right_line_end_img = right_line_end_img;

    if(out_pose != NULL)
    {
        *out_pose = g_vehicle_marker_pose;
    }
    return 1U;
}

uint8 VehicleMarkerLocalization_AppendAndValidateBlob(
    const VisionBlob_t *blob, const VisionContour_t *contour,
    const VehicleMarkerBinaryView_t *binary_view,
    const float *open_v_gate_score, float *measured_open_v_score)
{
    VehicleMarkerChevronCandidate_t candidate;
    uint8 valid;

    if(measured_open_v_score != NULL)
    {
        *measured_open_v_score = 0.0f;
    }
    if(!VehicleMarkerLocalization_AppendBlob(blob, contour))
    {
        return 0U;
    }
    memset(&candidate, 0, sizeof(candidate));
    candidate.blob = *blob;
    candidate.contour = *contour;
    candidate.valid = 1U;
    valid = vehicle_marker_chevron_process_candidate(
        &candidate, NULL, 1U, binary_view, open_v_gate_score);
    if(measured_open_v_score != NULL)
    {
        *measured_open_v_score = s_vehicle_marker_chevron_debug.open_v_score;
    }
    if(s_vehicle_marker_chevron_last_candidate_stored)
    {
        VehicleMarkerChevronCandidate_t *stored =
            &s_vehicle_marker_chevron_candidates[
                s_vehicle_marker_chevron_last_candidate_index];
        stored->geometry_checked = 1U;
        stored->geometry_valid = valid;
    }
    if(valid)
    {
        s_vehicle_marker_chevron_cached_pose_valid = 1U;
    }
    return valid;
}

uint8 VehicleMarkerLocalization_EndBlobFrame(VehicleMarkerPose_t *out_pose)
{
    uint8 tried[VEHICLE_MARKER_CHEVRON_MAX_FRAME_CANDIDATES] = {0};
    uint8 attempt;

    s_vehicle_marker_debug_raw_detect_valid = 0U;
    s_vehicle_marker_debug_process_count++;
    s_vehicle_marker_chevron_debug.candidate_count = s_vehicle_marker_chevron_candidate_count;

    if(s_vehicle_marker_chevron_cached_pose_valid)
    {
        s_vehicle_marker_debug_raw_detect_valid = 1U;
        s_vehicle_marker_debug_process_success_count++;
        s_vehicle_marker_chevron_debug.geometry_reject_count =
            (s_vehicle_marker_chevron_geometry_reject_count > 255U) ?
            255U : (uint8)s_vehicle_marker_chevron_geometry_reject_count;
        vehicle_marker_copy_pose(out_pose);
        return 1U;
    }

    if(s_vehicle_marker_chevron_candidate_count == 0U)
    {
        s_vehicle_marker_chevron_debug.reject_reason =
            VEHICLE_MARKER_CHEVRON_REJECT_NO_CANDIDATE;
        s_vehicle_marker_debug_process_fail_count++;
        s_vehicle_marker_chevron_debug.valid = 0U;
        /* 单帧漏检不清除最后可信位置，由5 ms看门狗统一判定150 ms超时。 */
        if(!s_vehicle_marker_has_trackable_pose &&
           !g_vehicle_marker_position_lost_flag)
        {
            vehicle_marker_set_position_lost();
        }
        if(out_pose != NULL)
        {
            vehicle_marker_copy_pose(out_pose);
        }
        return 0U;
    }

    for(attempt = 0U; attempt < s_vehicle_marker_chevron_candidate_count; ++attempt)
    {
        uint8 i;
        int best_index = -1;
        float best_score = 0.0f;

        for(i = 0U; i < s_vehicle_marker_chevron_candidate_count; ++i)
        {
            if(tried[i] || !s_vehicle_marker_chevron_candidates[i].valid ||
               s_vehicle_marker_chevron_candidates[i].geometry_checked)
            {
                continue;
            }
            if(best_index < 0 || s_vehicle_marker_chevron_candidates[i].score < best_score)
            {
                best_index = (int)i;
                best_score = s_vehicle_marker_chevron_candidates[i].score;
            }
        }

        if(best_index < 0)
        {
            break;
        }
        tried[best_index] = 1U;
        if(vehicle_marker_chevron_process_candidate(
               &s_vehicle_marker_chevron_candidates[best_index], out_pose, 1U,
               NULL, NULL))
        {
            s_vehicle_marker_debug_raw_detect_valid = 1U;
            s_vehicle_marker_debug_process_success_count++;
            s_vehicle_marker_chevron_debug.geometry_reject_count =
                (s_vehicle_marker_chevron_geometry_reject_count > 255U) ?
                255U : (uint8)s_vehicle_marker_chevron_geometry_reject_count;
            vehicle_marker_copy_pose(out_pose);
            return 1U;
        }
        s_vehicle_marker_chevron_geometry_reject_count++;
    }

    s_vehicle_marker_chevron_debug.geometry_reject_count =
        (s_vehicle_marker_chevron_geometry_reject_count > 255U) ?
        255U : (uint8)s_vehicle_marker_chevron_geometry_reject_count;
    s_vehicle_marker_debug_process_fail_count++;
    s_vehicle_marker_chevron_debug.valid = 0U;
    if(!s_vehicle_marker_has_trackable_pose &&
       !g_vehicle_marker_position_lost_flag)
    {
        vehicle_marker_set_position_lost();
    }
    if(out_pose != NULL)
    {
        vehicle_marker_copy_pose(out_pose);
    }
    return 0U;
}
