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

static uint8 g_vehicle_marker_binary_image[MT9V03X_H * MT9V03X_W];
static VehicleMarkerChevronDebug_t s_vehicle_marker_chevron_debug = {0};

#if VEHICLE_MARKER_MODE == VEHICLE_MARKER_MODE_CHEVRON
static void vehicle_marker_chevron_reset_debug(void);
#endif

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

static void vehicle_marker_copy_binary_image(const uint8 *src, uint8 *dst,
                                             int pixel_count)
{
    int i = 0;

    if(src == NULL || dst == NULL)
    {
        return;
    }

    for(i = 0; i < pixel_count; i++)
    {
        dst[i] = src[i];
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

static uint8 vehicle_marker_process_fail(VehicleMarkerPose_t *out_pose)
{
    s_vehicle_marker_debug_raw_detect_valid = 0U;
    s_vehicle_marker_debug_process_fail_count++;

    if(s_vehicle_marker_has_trackable_pose && !g_vehicle_marker_position_lost_flag)
    {
        g_vehicle_marker_pose.valid = 1U;
    }
    else
    {
        vehicle_marker_set_position_lost();
    }

    vehicle_marker_copy_pose(out_pose);
    return 0U;
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
    memset(g_vehicle_marker_binary_image, 0, sizeof(g_vehicle_marker_binary_image));
    vehicle_marker_reset_loss_debug();
#if VEHICLE_MARKER_MODE == VEHICLE_MARKER_MODE_CHEVRON
    vehicle_marker_chevron_reset_debug();
#endif
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

    next_elapsed_ms = (uint32)s_vehicle_marker_lost_elapsed_ms + VEHICLE_MARKER_TASK_PERIOD_MS;

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

#if VEHICLE_MARKER_MODE == VEHICLE_MARKER_MODE_FOUR_POINT

#define VEHICLE_MARKER_THREE_POINT_MIN_SIDE_CM          (2.0f)
#define VEHICLE_MARKER_THREE_POINT_SIDE_RATIO_MAX       (0.45f)
#define VEHICLE_MARKER_THREE_POINT_DIAG_RATIO_MAX       (0.45f)
#define VEHICLE_MARKER_BLOB_MIN_AREA_PX                 (3)
#define VEHICLE_MARKER_BLOB_MAX_AREA_PX                 (260)
#define VEHICLE_MARKER_BLOB_MAX_WIDTH_PX                (32)
#define VEHICLE_MARKER_BLOB_MAX_HEIGHT_PX               (32)
#define VEHICLE_MARKER_BLOB_MAX_ASPECT_RATIO            (2.5f)
#define VEHICLE_MARKER_BLOB_REF_HEIGHT_CM               (100.0f)
#define VEHICLE_MARKER_BLOB_HEIGHT_MIN_CM               (35.0f)
#define VEHICLE_MARKER_BLOB_HEIGHT_MAX_CM               (300.0f)
#define VEHICLE_MARKER_BLOB_MIN_AREA_FLOOR_PX           (1)
#define VEHICLE_MARKER_BLOB_MAX_AREA_CEILING_PX         (1200)
#define VEHICLE_MARKER_BLOB_MAX_WIDTH_CEILING_PX        (80)
#define VEHICLE_MARKER_BLOB_MAX_HEIGHT_CEILING_PX       (80)
#define VEHICLE_MARKER_SQUARE_SIDE_TARGET_CM            (VEHICLE_MARKER_SQUARE_SIDE_M * 100.0f)
#define VEHICLE_MARKER_SQUARE_SIDE_MIN_CM               (5.0f)
#define VEHICLE_MARKER_SQUARE_SIDE_MAX_CM               (18.0f)
#define VEHICLE_MARKER_SQUARE_SIDE_SPREAD_MAX           (0.50f)
#define VEHICLE_MARKER_SQUARE_DIAG_SPREAD_MAX           (0.35f)
#define VEHICLE_MARKER_SQUARE_DIAG_RATIO_MAX            (0.35f)
#define VEHICLE_MARKER_SQUARE_GEOMETRY_MAX_SCORE        (3.5f)
#define VEHICLE_MARKER_SQUARE_CENTER_JUMP_SOFT_CM       (40.0f)
#define VEHICLE_MARKER_SQUARE_CENTER_JUMP_HARD_CM       (100.0f)
#define VEHICLE_MARKER_THREE_POINT_SCORE_PENALTY        (0.60f)
#define VEHICLE_MARKER_SQRT2                            (1.41421356f)

typedef struct
{
    float sum_x;
    float sum_y;
    int count;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
} VehicleMarkerRegion_t;

typedef struct
{
    int min_area;
    int max_area;
    int max_width;
    int max_height;
    float expected_area;
} VehicleMarkerBlobGate_t;

typedef struct
{
    VehicleMarkerPoint2D_t img;
    VehicleMarkerWorldPoint_t world;
    float score;
} VehicleMarkerCandidate_t;

static uint8 vehicle_marker_find_points_binary_with_count(
    const uint8 *binary_image,
    VehicleMarkerPoint2D_t out_pts[VEHICLE_MARKER_POINT_COUNT],
    uint8 *out_point_count);

static float vehicle_marker_clamp_float(float value, float min_value, float max_value)
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

static int vehicle_marker_clamp_int(int value, int min_value, int max_value)
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

static int vehicle_marker_round_to_int(float value)
{
    return (int)(value + 0.5f);
}

static void vehicle_marker_get_blob_gate(VehicleMarkerBlobGate_t *gate)
{
    float height_cm = g_ipm.height;
    float linear_scale = 1.0f;
    float area_scale = 1.0f;
    int min_area = VEHICLE_MARKER_BLOB_MIN_AREA_PX;
    int max_area = VEHICLE_MARKER_BLOB_MAX_AREA_PX;
    int max_width = VEHICLE_MARKER_BLOB_MAX_WIDTH_PX;
    int max_height = VEHICLE_MARKER_BLOB_MAX_HEIGHT_PX;

    if(gate == NULL)
    {
        return;
    }

    if(height_cm <= 1.0f)
    {
        height_cm = VEHICLE_MARKER_BLOB_REF_HEIGHT_CM;
    }

    height_cm = vehicle_marker_clamp_float(height_cm,
                                           VEHICLE_MARKER_BLOB_HEIGHT_MIN_CM,
                                           VEHICLE_MARKER_BLOB_HEIGHT_MAX_CM);
    linear_scale = VEHICLE_MARKER_BLOB_REF_HEIGHT_CM / height_cm;
    area_scale = linear_scale * linear_scale;

    min_area = vehicle_marker_round_to_int((float)VEHICLE_MARKER_BLOB_MIN_AREA_PX * area_scale);
    max_area = vehicle_marker_round_to_int((float)VEHICLE_MARKER_BLOB_MAX_AREA_PX * area_scale);
    max_width = vehicle_marker_round_to_int((float)VEHICLE_MARKER_BLOB_MAX_WIDTH_PX * linear_scale);
    max_height = vehicle_marker_round_to_int((float)VEHICLE_MARKER_BLOB_MAX_HEIGHT_PX * linear_scale);

    gate->min_area = vehicle_marker_clamp_int(min_area,
                                              VEHICLE_MARKER_BLOB_MIN_AREA_FLOOR_PX,
                                              VEHICLE_MARKER_BLOB_MAX_AREA_PX);
    gate->max_area = vehicle_marker_clamp_int(max_area,
                                              gate->min_area,
                                              VEHICLE_MARKER_BLOB_MAX_AREA_CEILING_PX);
    gate->max_width = vehicle_marker_clamp_int(max_width, 1, VEHICLE_MARKER_BLOB_MAX_WIDTH_CEILING_PX);
    gate->max_height = vehicle_marker_clamp_int(max_height, 1, VEHICLE_MARKER_BLOB_MAX_HEIGHT_CEILING_PX);
    gate->expected_area = sqrtf((float)gate->min_area * (float)gate->max_area);
    if(gate->expected_area < 1.0f)
    {
        gate->expected_area = 1.0f;
    }
}

static float vehicle_marker_region_aspect_ratio(const VehicleMarkerRegion_t *region)
{
    float width = (float)(region->max_x - region->min_x + 1);
    float height = (float)(region->max_y - region->min_y + 1);

    if(width < 1.0f)
    {
        width = 1.0f;
    }
    if(height < 1.0f)
    {
        height = 1.0f;
    }

    return (width > height) ? (width / height) : (height / width);
}

static uint8 vehicle_marker_region_is_small_light_candidate(const VehicleMarkerRegion_t *region,
                                                            const VehicleMarkerBlobGate_t *gate)
{
    int width = region->max_x - region->min_x + 1;
    int height = region->max_y - region->min_y + 1;
    float aspect_ratio = vehicle_marker_region_aspect_ratio(region);

    if(gate == NULL)
    {
        return 0U;
    }

    if(region->count < gate->min_area || region->count > gate->max_area)
    {
        return 0U;
    }

    if(width > gate->max_width || height > gate->max_height)
    {
        return 0U;
    }

    if(aspect_ratio > VEHICLE_MARKER_BLOB_MAX_ASPECT_RATIO)
    {
        return 0U;
    }

    return 1U;
}

static uint8 vehicle_marker_insert_candidate(
    VehicleMarkerCandidate_t candidates[VEHICLE_MARKER_MAX_CANDIDATES],
    uint8 *candidate_count,
    const VehicleMarkerRegion_t *region,
    const VehicleMarkerBlobGate_t *gate)
{
    VehicleMarkerCandidate_t candidate = {0};
    float world_x = 0.0f;
    float world_y = 0.0f;
    float aspect_ratio = vehicle_marker_region_aspect_ratio(region);
    float area_error = 0.0f;
    int replace_index = -1;
    float worst_score = -1.0f;
    int i = 0;

    candidate.img.u = region->sum_x / (float)region->count;
    candidate.img.v = region->sum_y / (float)region->count;
    candidate.img.intensity = (float)region->count;
    area_error = fabsf((float)region->count - gate->expected_area) / gate->expected_area;
    candidate.score = area_error * 20.0f + (aspect_ratio - 1.0f) * 10.0f;

    if(!IPM_Image2WorldF(candidate.img.u, candidate.img.v, &world_x, &world_y))
    {
        return 0U;
    }
    candidate.world.x = world_x;
    candidate.world.y = world_y;

    if(*candidate_count < VEHICLE_MARKER_MAX_CANDIDATES)
    {
        candidates[*candidate_count] = candidate;
        (*candidate_count)++;
        return 1U;
    }

    for(i = 0; i < VEHICLE_MARKER_MAX_CANDIDATES; i++)
    {
        if(candidates[i].score > worst_score)
        {
            worst_score = candidates[i].score;
            replace_index = i;
        }
    }

    if(replace_index >= 0 && candidate.score < worst_score)
    {
        candidates[replace_index] = candidate;
        return 1U;
    }

    return 0U;
}

static float vehicle_marker_world_point_distance_cm(const VehicleMarkerWorldPoint_t *a,
                                                    const VehicleMarkerWorldPoint_t *b)
{
    return sqrtf((b->x - a->x) * (b->x - a->x) +
                 (b->y - a->y) * (b->y - a->y));
}

static void vehicle_marker_sort_float_ascending(float *values, int count)
{
    int i = 0;
    int j = 0;

    for(i = 0; i < count - 1; i++)
    {
        for(j = i + 1; j < count; j++)
        {
            if(values[j] < values[i])
            {
                float tmp = values[i];
                values[i] = values[j];
                values[j] = tmp;
            }
        }
    }
}

static float vehicle_marker_calculate_square_side(
    const VehicleMarkerWorldPoint_t pts[VEHICLE_MARKER_POINT_COUNT])
{
    float distances[6];
    float side_sum = 0.0f;
    int dist_count = 0;
    int i = 0;
    int j = 0;

    for(i = 0; i < VEHICLE_MARKER_POINT_COUNT - 1; i++)
    {
        for(j = i + 1; j < VEHICLE_MARKER_POINT_COUNT; j++)
        {
            distances[dist_count] = vehicle_marker_world_point_distance_cm(&pts[i], &pts[j]);
            dist_count++;
        }
    }

    vehicle_marker_sort_float_ascending(distances, dist_count);

    for(i = 0; i < 4; i++)
    {
        side_sum += distances[i];
    }

    return side_sum / 4.0f;
}

static void vehicle_marker_calculate_square_center(
    const VehicleMarkerWorldPoint_t pts[VEHICLE_MARKER_POINT_COUNT],
    float *center_x,
    float *center_y)
{
    int i = 0;

    *center_x = 0.0f;
    *center_y = 0.0f;

    for(i = 0; i < VEHICLE_MARKER_POINT_COUNT; i++)
    {
        *center_x += pts[i].x;
        *center_y += pts[i].y;
    }

    *center_x /= (float)VEHICLE_MARKER_POINT_COUNT;
    *center_y /= (float)VEHICLE_MARKER_POINT_COUNT;
}

static uint8 vehicle_marker_reconstruct_square_from_three_points(
    VehicleMarkerWorldPoint_t world_pts[VEHICLE_MARKER_POINT_COUNT],
    VehicleMarkerPoint2D_t img_pts[VEHICLE_MARKER_POINT_COUNT])
{
    float d01 = vehicle_marker_world_point_distance_cm(&world_pts[0], &world_pts[1]);
    float d02 = vehicle_marker_world_point_distance_cm(&world_pts[0], &world_pts[2]);
    float d12 = vehicle_marker_world_point_distance_cm(&world_pts[1], &world_pts[2]);
    float diag = d01;
    int diag_a = 0;
    int diag_b = 1;
    int right = 2;
    float side_a = 0.0f;
    float side_b = 0.0f;
    float side_avg = 0.0f;
    float diag_expected = 0.0f;
    float side_ratio = 0.0f;
    float diag_ratio = 0.0f;

    if(d02 > diag)
    {
        diag = d02;
        diag_a = 0;
        diag_b = 2;
        right = 1;
    }
    if(d12 > diag)
    {
        diag = d12;
        diag_a = 1;
        diag_b = 2;
        right = 0;
    }

    side_a = vehicle_marker_world_point_distance_cm(&world_pts[right], &world_pts[diag_a]);
    side_b = vehicle_marker_world_point_distance_cm(&world_pts[right], &world_pts[diag_b]);
    side_avg = 0.5f * (side_a + side_b);

    if(side_avg < VEHICLE_MARKER_THREE_POINT_MIN_SIDE_CM || side_avg > VEHICLE_MARKER_SQUARE_SIDE_MAX_CM)
    {
        return 0U;
    }

    diag_expected = sqrtf(side_a * side_a + side_b * side_b);
    if(diag_expected < VEHICLE_MARKER_THREE_POINT_MIN_SIDE_CM)
    {
        return 0U;
    }

    side_ratio = fabsf(side_a - side_b) / side_avg;
    diag_ratio = fabsf(diag - diag_expected) / diag_expected;

    if(side_ratio > VEHICLE_MARKER_THREE_POINT_SIDE_RATIO_MAX ||
       diag_ratio > VEHICLE_MARKER_THREE_POINT_DIAG_RATIO_MAX)
    {
        return 0U;
    }

    world_pts[3].x = world_pts[diag_a].x + world_pts[diag_b].x - world_pts[right].x;
    world_pts[3].y = world_pts[diag_a].y + world_pts[diag_b].y - world_pts[right].y;

    img_pts[3].u = img_pts[diag_a].u + img_pts[diag_b].u - img_pts[right].u;
    img_pts[3].v = img_pts[diag_a].v + img_pts[diag_b].v - img_pts[right].v;
    img_pts[3].intensity = 0.0f;

    return 1U;
}

static float vehicle_marker_square_center_jump_penalty(
    const VehicleMarkerWorldPoint_t world_pts[VEHICLE_MARKER_POINT_COUNT])
{
    float center_x = 0.0f;
    float center_y = 0.0f;
    float dx = 0.0f;
    float dy = 0.0f;
    float distance = 0.0f;

    if(!s_vehicle_marker_has_trackable_pose ||
       g_vehicle_marker_position_lost_flag ||
       !g_vehicle_marker_pose.valid)
    {
        return 0.0f;
    }

    vehicle_marker_calculate_square_center(world_pts, &center_x, &center_y);
    dx = center_x - g_vehicle_marker_pose.raw_x;
    dy = center_y - g_vehicle_marker_pose.raw_y;
    distance = sqrtf(dx * dx + dy * dy);

    if(distance > VEHICLE_MARKER_SQUARE_CENTER_JUMP_HARD_CM)
    {
        return VEHICLE_MARKER_SQUARE_GEOMETRY_MAX_SCORE + 1.0f;
    }

    return distance / VEHICLE_MARKER_SQUARE_CENTER_JUMP_SOFT_CM;
}

static uint8 vehicle_marker_evaluate_square_geometry(
    const VehicleMarkerWorldPoint_t world_pts[VEHICLE_MARKER_POINT_COUNT],
    float *out_score)
{
    float distances[6];
    float side_sum = 0.0f;
    float side_avg = 0.0f;
    float diag_avg = 0.0f;
    float side_spread = 0.0f;
    float diag_spread = 0.0f;
    float diag_expected = 0.0f;
    float diag_ratio = 0.0f;
    float target_ratio = 0.0f;
    int dist_count = 0;
    int i = 0;
    int j = 0;

    for(i = 0; i < VEHICLE_MARKER_POINT_COUNT - 1; i++)
    {
        for(j = i + 1; j < VEHICLE_MARKER_POINT_COUNT; j++)
        {
            distances[dist_count] = vehicle_marker_world_point_distance_cm(&world_pts[i], &world_pts[j]);
            dist_count++;
        }
    }

    vehicle_marker_sort_float_ascending(distances, dist_count);

    for(i = 0; i < 4; i++)
    {
        side_sum += distances[i];
    }

    side_avg = side_sum / 4.0f;
    diag_avg = 0.5f * (distances[4] + distances[5]);

    if(side_avg < VEHICLE_MARKER_SQUARE_SIDE_MIN_CM ||
       side_avg > VEHICLE_MARKER_SQUARE_SIDE_MAX_CM ||
       diag_avg < VEHICLE_MARKER_SQUARE_SIDE_MIN_CM)
    {
        return 0U;
    }

    side_spread = (distances[3] - distances[0]) / side_avg;
    diag_spread = fabsf(distances[5] - distances[4]) / diag_avg;
    diag_expected = side_avg * VEHICLE_MARKER_SQRT2;
    diag_ratio = fabsf(diag_avg - diag_expected) / diag_expected;
    target_ratio = fabsf(side_avg - VEHICLE_MARKER_SQUARE_SIDE_TARGET_CM) /
                   VEHICLE_MARKER_SQUARE_SIDE_TARGET_CM;

    if(side_spread > VEHICLE_MARKER_SQUARE_SIDE_SPREAD_MAX ||
       diag_spread > VEHICLE_MARKER_SQUARE_DIAG_SPREAD_MAX ||
       diag_ratio > VEHICLE_MARKER_SQUARE_DIAG_RATIO_MAX)
    {
        return 0U;
    }

    if(out_score != NULL)
    {
        *out_score = target_ratio * 1.5f +
                     side_spread * 2.5f +
                     diag_spread * 2.0f +
                     diag_ratio * 2.0f;
    }

    return 1U;
}

static uint8 vehicle_marker_select_square_points_from_candidates(
    const VehicleMarkerCandidate_t candidates[VEHICLE_MARKER_MAX_CANDIDATES],
    uint8 candidate_count,
    VehicleMarkerPoint2D_t out_pts[VEHICLE_MARKER_POINT_COUNT],
    uint8 *out_point_count)
{
    VehicleMarkerPoint2D_t best_img_pts[VEHICLE_MARKER_POINT_COUNT];
    float best_score = VEHICLE_MARKER_SQUARE_GEOMETRY_MAX_SCORE;
    uint8 best_count = 0U;
    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;
    int i = 0;

    memset(best_img_pts, 0, sizeof(best_img_pts));
    memset(out_pts, 0, sizeof(VehicleMarkerPoint2D_t) * VEHICLE_MARKER_POINT_COUNT);
    if(out_point_count != NULL)
    {
        *out_point_count = 0U;
    }

    for(a = 0; a < candidate_count - 3; a++)
    {
        for(b = a + 1; b < candidate_count - 2; b++)
        {
            for(c = b + 1; c < candidate_count - 1; c++)
            {
                for(d = c + 1; d < candidate_count; d++)
                {
                    VehicleMarkerWorldPoint_t world_pts[VEHICLE_MARKER_POINT_COUNT];
                    float score = 0.0f;

                    world_pts[0] = candidates[a].world;
                    world_pts[1] = candidates[b].world;
                    world_pts[2] = candidates[c].world;
                    world_pts[3] = candidates[d].world;

                    if(!vehicle_marker_evaluate_square_geometry(world_pts, &score))
                    {
                        continue;
                    }

                    score += vehicle_marker_square_center_jump_penalty(world_pts);

                    if(score < best_score)
                    {
                        best_score = score;
                        best_count = VEHICLE_MARKER_POINT_COUNT;
                        best_img_pts[0] = candidates[a].img;
                        best_img_pts[1] = candidates[b].img;
                        best_img_pts[2] = candidates[c].img;
                        best_img_pts[3] = candidates[d].img;
                    }
                }
            }
        }
    }

    for(a = 0; a < candidate_count - 2; a++)
    {
        for(b = a + 1; b < candidate_count - 1; b++)
        {
            for(c = b + 1; c < candidate_count; c++)
            {
                VehicleMarkerWorldPoint_t world_pts[VEHICLE_MARKER_POINT_COUNT];
                VehicleMarkerPoint2D_t img_pts[VEHICLE_MARKER_POINT_COUNT];
                float score = 0.0f;

                memset(world_pts, 0, sizeof(world_pts));
                memset(img_pts, 0, sizeof(img_pts));

                world_pts[0] = candidates[a].world;
                world_pts[1] = candidates[b].world;
                world_pts[2] = candidates[c].world;
                img_pts[0] = candidates[a].img;
                img_pts[1] = candidates[b].img;
                img_pts[2] = candidates[c].img;

                if(!vehicle_marker_reconstruct_square_from_three_points(world_pts, img_pts))
                {
                    continue;
                }

                if(!vehicle_marker_evaluate_square_geometry(world_pts, &score))
                {
                    continue;
                }

                score += VEHICLE_MARKER_THREE_POINT_SCORE_PENALTY +
                         vehicle_marker_square_center_jump_penalty(world_pts);

                if(score < best_score)
                {
                    best_score = score;
                    best_count = VEHICLE_MARKER_MIN_DETECT_POINTS;
                    best_img_pts[0] = candidates[a].img;
                    best_img_pts[1] = candidates[b].img;
                    best_img_pts[2] = candidates[c].img;
                    best_img_pts[3].u = 0.0f;
                    best_img_pts[3].v = 0.0f;
                    best_img_pts[3].intensity = 0.0f;
                }
            }
        }
    }

    if(best_count < VEHICLE_MARKER_MIN_DETECT_POINTS)
    {
        return 0U;
    }

    for(i = 0; i < best_count; i++)
    {
        out_pts[i] = best_img_pts[i];
    }
    if(out_point_count != NULL)
    {
        *out_point_count = best_count;
    }

    return 1U;
}

static uint8 vehicle_marker_find_points_with_count(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    VehicleMarkerPoint2D_t out_pts[VEHICLE_MARKER_POINT_COUNT],
    uint8 *out_point_count)
{
    if(image == NULL)
    {
        return 0U;
    }

    return vehicle_marker_find_points_binary_with_count((const uint8 *)image,
                                                        out_pts,
                                                        out_point_count);
}

static uint8 vehicle_marker_find_points_binary_with_count(
    const uint8 *binary_image,
    VehicleMarkerPoint2D_t out_pts[VEHICLE_MARKER_POINT_COUNT],
    uint8 *out_point_count)
{
    VehicleMarkerCandidate_t candidates[VEHICLE_MARKER_MAX_CANDIDATES];
    VehicleMarkerBlobGate_t blob_gate;
    static uint8 visited[MT9V03X_H * MT9V03X_W];
    uint8 candidate_count = 0U;
    int y = 0;
    int x = 0;

    if(binary_image == NULL)
    {
        return 0U;
    }

    vehicle_marker_copy_binary_image(binary_image,
                                     g_vehicle_marker_binary_image,
                                     MT9V03X_W * MT9V03X_H);
    vehicle_marker_get_blob_gate(&blob_gate);

    memset(visited, 0, sizeof(visited));
    memset(candidates, 0, sizeof(candidates));
    memset(out_pts, 0, sizeof(VehicleMarkerPoint2D_t) * VEHICLE_MARKER_POINT_COUNT);
    if(out_point_count != NULL)
    {
        *out_point_count = 0U;
    }

    for(y = 0; y < MT9V03X_H; y++)
    {
        for(x = 0; x < MT9V03X_W; x++)
        {
            int idx = y * MT9V03X_W + x;

            if(g_vehicle_marker_binary_image[idx] == 0 || visited[idx])
            {
                continue;
            }

            {
                VehicleMarkerRegion_t region = {0};
                static int stack_x[1000];
                static int stack_y[1000];
                int stack_top = 0;

                region.sum_x = 0.0f;
                region.sum_y = 0.0f;
                region.count = 0;
                region.min_x = MT9V03X_W;
                region.max_x = 0;
                region.min_y = MT9V03X_H;
                region.max_y = 0;

                stack_x[stack_top] = x;
                stack_y[stack_top] = y;
                stack_top++;
                visited[idx] = 1U;

                while(stack_top > 0)
                {
                    int cx = 0;
                    int cy = 0;
                    int dy = 0;
                    int dx = 0;

                    stack_top--;
                    cx = stack_x[stack_top];
                    cy = stack_y[stack_top];

                    region.sum_x += (float)cx;
                    region.sum_y += (float)cy;
                    region.count++;

                    if(cx < region.min_x) region.min_x = cx;
                    if(cx > region.max_x) region.max_x = cx;
                    if(cy < region.min_y) region.min_y = cy;
                    if(cy > region.max_y) region.max_y = cy;

                    for(dy = -1; dy <= 1; dy++)
                    {
                        for(dx = -1; dx <= 1; dx++)
                        {
                            int nx = 0;
                            int ny = 0;
                            int nidx = 0;

                            if(dx == 0 && dy == 0)
                            {
                                continue;
                            }

                            nx = cx + dx;
                            ny = cy + dy;

                            if(nx < 0 || nx >= MT9V03X_W || ny < 0 || ny >= MT9V03X_H)
                            {
                                continue;
                            }

                            nidx = ny * MT9V03X_W + nx;
                            if(g_vehicle_marker_binary_image[nidx] != 0 && !visited[nidx])
                            {
                                visited[nidx] = 1U;
                                if(stack_top < 1000)
                                {
                                    stack_x[stack_top] = nx;
                                    stack_y[stack_top] = ny;
                                    stack_top++;
                                }
                            }
                        }
                    }
                }

                if(vehicle_marker_region_is_small_light_candidate(&region, &blob_gate))
                {
                    (void)vehicle_marker_insert_candidate(candidates, &candidate_count, &region, &blob_gate);
                }
            }
        }
    }

    if(candidate_count < VEHICLE_MARKER_MIN_DETECT_POINTS)
    {
        return 0U;
    }

    return vehicle_marker_select_square_points_from_candidates(candidates,
                                                               candidate_count,
                                                               out_pts,
                                                               out_point_count);
}

uint8 VehicleMarkerLocalization_FindPoints(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    VehicleMarkerPoint2D_t out_pts[VEHICLE_MARKER_POINT_COUNT])
{
    uint8 point_count = 0U;

    return vehicle_marker_find_points_with_count(image, out_pts, &point_count);
}

uint8 VehicleMarkerLocalization_FindPointsBinary(
    const uint8 *binary_image,
    VehicleMarkerPoint2D_t out_pts[VEHICLE_MARKER_POINT_COUNT])
{
    uint8 point_count = 0U;

    return vehicle_marker_find_points_binary_with_count(binary_image, out_pts, &point_count);
}

uint8 VehicleMarkerLocalization_Process(const uint8 image[MT9V03X_H][MT9V03X_W],
                                        VehicleMarkerPose_t *out_pose)
{
    if(image == NULL)
    {
        return vehicle_marker_process_fail(out_pose);
    }

    return VehicleMarkerLocalization_ProcessBinary((const uint8 *)image, out_pose);
}

uint8 VehicleMarkerLocalization_ProcessBinary(const uint8 *binary_image,
                                              VehicleMarkerPose_t *out_pose)
{
    VehicleMarkerPoint2D_t img_pts[VEHICLE_MARKER_POINT_COUNT];
    VehicleMarkerWorldPoint_t world_pts[VEHICLE_MARKER_POINT_COUNT];
    float center_x = 0.0f;
    float center_y = 0.0f;
    float filtered_center_x = 0.0f;
    float filtered_center_y = 0.0f;
    float square_side_cm = 0.0f;
    float real_side_cm = VEHICLE_MARKER_SQUARE_SIDE_M * 100.0f;
    float current_height = g_ipm.height;
    float estimated_z = 0.0f;
    uint8 detected_point_count = 0U;
    uint8 reconstructed_point = 0U;
    int i = 0;

    s_vehicle_marker_debug_process_count++;

    if(!vehicle_marker_find_points_binary_with_count(binary_image, img_pts, &detected_point_count))
    {
        return vehicle_marker_process_fail(out_pose);
    }

    s_vehicle_marker_debug_raw_detect_valid = 1U;

    for(i = 0; i < detected_point_count; i++)
    {
        float world_x = 0.0f;
        float world_y = 0.0f;

        if(!IPM_Image2WorldF(img_pts[i].u, img_pts[i].v, &world_x, &world_y))
        {
            return vehicle_marker_process_fail(out_pose);
        }
        world_pts[i].x = world_x;
        world_pts[i].y = world_y;
    }

    if(detected_point_count == VEHICLE_MARKER_MIN_DETECT_POINTS)
    {
        if(!vehicle_marker_reconstruct_square_from_three_points(world_pts, img_pts))
        {
            return vehicle_marker_process_fail(out_pose);
        }
        reconstructed_point = 1U;
    }

    s_vehicle_marker_debug_process_success_count++;

    for(i = 0; i < VEHICLE_MARKER_POINT_COUNT; i++)
    {
        g_vehicle_marker_pose.img_pts[i] = img_pts[i];
        g_vehicle_marker_pose.world_pts[i] = world_pts[i];
    }

    vehicle_marker_calculate_square_center(world_pts, &center_x, &center_y);
    square_side_cm = vehicle_marker_calculate_square_side(world_pts);
    vehicle_marker_center_kalman_update(center_x, center_y,
                                        &filtered_center_x, &filtered_center_y);

    if(square_side_cm > 0.5f && current_height > 1.0f)
    {
        estimated_z = current_height * (real_side_cm / square_side_cm);

        if(estimated_z < 5.0f)
        {
            estimated_z = 5.0f;
        }
        if(estimated_z > 300.0f)
        {
            estimated_z = 300.0f;
        }
    }
    else
    {
        estimated_z = g_vehicle_marker_pose.z;
        if(estimated_z < 5.0f)
        {
            estimated_z = current_height;
        }
    }

    g_vehicle_marker_pose.raw_x = center_x;
    g_vehicle_marker_pose.raw_y = center_y;
    g_vehicle_marker_pose.x = filtered_center_x;
    g_vehicle_marker_pose.y = filtered_center_y;
    g_vehicle_marker_pose.z = estimated_z;
    g_vehicle_marker_pose.square_side_cm = square_side_cm;
    g_vehicle_marker_pose.detected_point_count = detected_point_count;
    g_vehicle_marker_pose.reconstructed_point = reconstructed_point;
    g_vehicle_marker_pose.valid = 1U;
    s_vehicle_marker_has_trackable_pose = 1U;
    s_vehicle_marker_lost_elapsed_ms = 0U;
    g_vehicle_marker_position_lost_elapsed_ms = 0U;
    g_vehicle_marker_position_lost_flag = 0U;
    g_vehicle_marker_pose_update_seq++;

    vehicle_marker_copy_pose(out_pose);
    return 1U;
}

void VehicleMarkerLocalization_BeginBlobFrame(void)
{
}

// 四点模式下仍保留同名接口，供 runtime 统一调用；实际定位走 Process()。
void VehicleMarkerLocalization_GetBlobRejectDebug(int *area_low_count,
                                                  int *area_high_count,
                                                  int *size_count,
                                                  int *aspect_count,
                                                  int *insert_count)
{
    if(area_low_count != NULL)
    {
        *area_low_count = 0;
    }
    if(area_high_count != NULL)
    {
        *area_high_count = 0;
    }
    if(size_count != NULL)
    {
        *size_count = 0;
    }
    if(aspect_count != NULL)
    {
        *aspect_count = 0;
    }
    if(insert_count != NULL)
    {
        *insert_count = 0;
    }
}

uint8 VehicleMarkerLocalization_AppendBlob(const VisionBlob_t *blob, const VisionContour_t *contour)
{
    (void)blob;
    (void)contour;
    return 0U;
}

uint8 VehicleMarkerLocalization_EndBlobFrame(VehicleMarkerPose_t *out_pose)
{
    (void)out_pose;
    return 0U;
}

#elif VEHICLE_MARKER_MODE == VEHICLE_MARKER_MODE_CHEVRON

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
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_MIN_OCCUPANCY      (0.50f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_MAX_OCCUPANCY      (1.0f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_EXPECTED_OCCUPANCY (0.78f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_MAX_CIRCULARITY    (0.55f)
#define VEHICLE_MARKER_CHEVRON_PHYSICAL_EXPECTED_CIRCULARITY (0.30f)
#define VEHICLE_MARKER_CHEVRON_MIN_ARM_CM                  (10.5f)
#define VEHICLE_MARKER_CHEVRON_MAX_ARM_CM                  (34.0f)
#define VEHICLE_MARKER_CHEVRON_MIN_ARM_RATIO               (0.65f)
#define VEHICLE_MARKER_CHEVRON_MAX_ARM_RATIO               (1.55f)
#define VEHICLE_MARKER_CHEVRON_MIN_ANGLE_DEG               (75.0f)
#define VEHICLE_MARKER_CHEVRON_MAX_ANGLE_DEG               (115.0f)
#define VEHICLE_MARKER_CHEVRON_MIN_BASE_CM                 (15.0f)
#define VEHICLE_MARKER_CHEVRON_MAX_BASE_CM                 (46.0f)
#define VEHICLE_MARKER_CHEVRON_MIN_TRIANGLE_HEIGHT_CM      (8.0f)
#define VEHICLE_MARKER_CHEVRON_MAX_TRIANGLE_HEIGHT_CM      (20.0f)
#define VEHICLE_MARKER_CHEVRON_MIN_CONTOUR_POINTS          (8U)
#define VEHICLE_MARKER_CHEVRON_POINT_EPS                   (1e-4f)
#define VEHICLE_MARKER_CHEVRON_GROUP_MAX_POINTS            (VISION_RUNTIME_CONTOUR_MAX_POINTS + 1U)
#define VEHICLE_MARKER_CHEVRON_MAX_FRAME_CANDIDATES        (4U)

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
    float mean_x;
    float mean_y;
    float dir_x;
    float dir_y;
    uint8 valid;
} VehicleMarkerChevronLine_t;

typedef struct
{
    VisionBlob_t blob;
    VisionContour_t contour;
    float score;
    uint8 valid;
} VehicleMarkerChevronCandidate_t;

#pragma location = 0x20000000
__no_init static VehicleMarkerChevronCandidate_t
    s_vehicle_marker_chevron_candidates[VEHICLE_MARKER_CHEVRON_MAX_FRAME_CANDIDATES];
static uint8 s_vehicle_marker_chevron_candidate_count = 0U;
static VehicleMarkerWorldPoint_t s_vehicle_marker_chevron_world_pts[VISION_RUNTIME_CONTOUR_MAX_POINTS];
static VehicleMarkerWorldPoint_t s_vehicle_marker_chevron_left_pts[VEHICLE_MARKER_CHEVRON_GROUP_MAX_POINTS];
static VehicleMarkerWorldPoint_t s_vehicle_marker_chevron_right_pts[VEHICLE_MARKER_CHEVRON_GROUP_MAX_POINTS];
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

static float vehicle_marker_chevron_point_to_line_distance_sq(const VehicleMarkerWorldPoint_t *p,
                                                              const VehicleMarkerWorldPoint_t *a,
                                                              const VehicleMarkerWorldPoint_t *b)
{
    float ax = b->x - a->x;
    float ay = b->y - a->y;
    float px = p->x - a->x;
    float py = p->y - a->y;
    float cross = ax * py - ay * px;
    float len_sq = ax * ax + ay * ay;

    if(len_sq < VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        return vehicle_marker_chevron_point_distance_sq(p, a);
    }

    return (cross * cross) / len_sq;
}

static void vehicle_marker_chevron_fit_line(const VehicleMarkerWorldPoint_t *pts,
                                            uint16 count,
                                            VehicleMarkerChevronLine_t *line)
{
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float sxx = 0.0f;
    float sxy = 0.0f;
    float syy = 0.0f;
    float theta = 0.0f;
    uint16 i = 0U;

    if(line == NULL)
    {
        return;
    }

    memset(line, 0, sizeof(*line));
    if(pts == NULL || count < 2U)
    {
        return;
    }

    for(i = 0U; i < count; i++)
    {
        mean_x += pts[i].x;
        mean_y += pts[i].y;
    }
    mean_x /= (float)count;
    mean_y /= (float)count;

    for(i = 0U; i < count; i++)
    {
        float dx = pts[i].x - mean_x;
        float dy = pts[i].y - mean_y;

        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
    }

    theta = 0.5f * atan2f(2.0f * sxy, sxx - syy);
    line->mean_x = mean_x;
    line->mean_y = mean_y;
    line->dir_x = cosf(theta);
    line->dir_y = sinf(theta);
    line->valid = 1U;
}

static void vehicle_marker_chevron_project_to_line(const VehicleMarkerWorldPoint_t *p,
                                                   const VehicleMarkerChevronLine_t *line,
                                                   VehicleMarkerWorldPoint_t *out)
{
    float dx = 0.0f;
    float dy = 0.0f;
    float t = 0.0f;

    if(out == NULL || p == NULL || line == NULL || !line->valid)
    {
        return;
    }

    dx = p->x - line->mean_x;
    dy = p->y - line->mean_y;
    t = dx * line->dir_x + dy * line->dir_y;
    out->x = line->mean_x + t * line->dir_x;
    out->y = line->mean_y + t * line->dir_y;
}

static uint8 vehicle_marker_chevron_world_to_image(const VehicleMarkerWorldPoint_t *world,
                                                   VehicleMarkerPoint2D_t *img)
{
    int img_x = 0;
    int img_y = 0;

    if(world == NULL || img == NULL)
    {
        return 0U;
    }

    if(!IPM_World2Image(world->x, world->y, &img_x, &img_y))
    {
        return 0U;
    }

    if(img_x < 0)
    {
        img_x = 0;
    }
    else if(img_x >= MT9V03X_W)
    {
        img_x = MT9V03X_W - 1;
    }

    if(img_y < 0)
    {
        img_y = 0;
    }
    else if(img_y >= MT9V03X_H)
    {
        img_y = MT9V03X_H - 1;
    }

    img->u = (float)img_x;
    img->v = (float)img_y;
    img->intensity = 0.0f;
    return 1U;
}

static uint8 vehicle_marker_chevron_copy_line_debug(const VehicleMarkerWorldPoint_t *apex,
                                                    const VehicleMarkerWorldPoint_t *tip,
                                                    const VehicleMarkerChevronLine_t *line,
                                                    VehicleMarkerPoint2D_t *line_start_img,
                                                    VehicleMarkerPoint2D_t *line_end_img,
                                                    VehicleMarkerPoint2D_t *tip_img)
{
    VehicleMarkerWorldPoint_t start_world = {0};
    VehicleMarkerWorldPoint_t end_world = {0};
    uint8 start_ok = 0U;
    uint8 end_ok = 0U;
    uint8 tip_ok = 0U;

    if(apex == NULL || tip == NULL || line == NULL || !line->valid)
    {
        return 0U;
    }

    vehicle_marker_chevron_project_to_line(apex, line, &start_world);
    vehicle_marker_chevron_project_to_line(tip, line, &end_world);
    start_ok = vehicle_marker_chevron_world_to_image(&start_world, line_start_img);
    end_ok = vehicle_marker_chevron_world_to_image(&end_world, line_end_img);
    tip_ok = vehicle_marker_chevron_world_to_image(tip, tip_img);
    return (uint8)((start_ok && end_ok && tip_ok) ? 1U : 0U);
}

static uint8 vehicle_marker_chevron_fit_group(const VehicleMarkerWorldPoint_t *pts,
                                              uint16 count,
                                              const VehicleMarkerWorldPoint_t *apex,
                                              VehicleMarkerWorldPoint_t *tip_world,
                                              VehicleMarkerChevronLine_t *fit,
                                              VehicleMarkerPoint2D_t *line_start_img,
                                              VehicleMarkerPoint2D_t *line_end_img,
                                              VehicleMarkerPoint2D_t *tip_img)
{
    float best_proj = -1e30f;
    float worst_proj = 1e30f;
    VehicleMarkerWorldPoint_t best_tip = {0};
    uint16 i = 0U;
    VehicleMarkerWorldPoint_t line_tip_world = {0};
    float apex_proj = 0.0f;
    float dx = 0.0f;
    float dy = 0.0f;

    if(pts == NULL || apex == NULL || tip_world == NULL || fit == NULL || count < 2U || !fit->valid)
    {
        return 0U;
    }

    for(i = 0U; i < count; i++)
    {
        float proj = (pts[i].x - fit->mean_x) * fit->dir_x +
                     (pts[i].y - fit->mean_y) * fit->dir_y;

        if(proj > best_proj)
        {
            best_proj = proj;
            best_tip = pts[i];
        }
        if(proj < worst_proj)
        {
            worst_proj = proj;
        }
    }

    dx = apex->x - fit->mean_x;
    dy = apex->y - fit->mean_y;
    apex_proj = dx * fit->dir_x + dy * fit->dir_y;

    if(best_proj < apex_proj)
    {
        fit->dir_x = -fit->dir_x;
        fit->dir_y = -fit->dir_y;
        best_proj = -1e30f;
        for(i = 0U; i < count; i++)
        {
            float proj = (pts[i].x - fit->mean_x) * fit->dir_x +
                         (pts[i].y - fit->mean_y) * fit->dir_y;

            if(proj > best_proj)
            {
                best_proj = proj;
                best_tip = pts[i];
            }
        }
    }

    (void)worst_proj;
    line_tip_world = best_tip;
    *tip_world = line_tip_world;
    if(line_start_img != NULL || line_end_img != NULL || tip_img != NULL)
    {
        return vehicle_marker_chevron_copy_line_debug(apex,
                                                      tip_world,
                                                      fit,
                                                      line_start_img,
                                                      line_end_img,
                                                      tip_img);
    }

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

    if(!vehicle_marker_chevron_world_to_image(apex, &apex_img) ||
       !vehicle_marker_chevron_world_to_image(left_tip, &left_tip_img) ||
       !vehicle_marker_chevron_world_to_image(right_tip, &right_tip_img))
    {
        return 0U;
    }

    if(left_line_start_img == NULL || left_line_end_img == NULL ||
       right_line_start_img == NULL || right_line_end_img == NULL)
    {
        return 0U;
    }

    center_x = apex->x;
    center_y = apex->y;
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

static float vehicle_marker_chevron_image_point_to_line_distance_sq(const VehicleMarkerPoint2D_t *p,
                                                                    const VehicleMarkerPoint2D_t *a,
                                                                    const VehicleMarkerPoint2D_t *b)
{
    float ax = b->u - a->u;
    float ay = b->v - a->v;
    float px = p->u - a->u;
    float py = p->v - a->v;
    float cross = ax * py - ay * px;
    float len_sq = ax * ax + ay * ay;

    if(len_sq < VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        return vehicle_marker_chevron_image_distance_sq(p, a);
    }

    return (cross * cross) / len_sq;
}

/* Extract the V geometry in image space. World-coordinate projection is not
 * used here because its non-uniform scale can change which contour point is
 * the tip. */
static uint8 vehicle_marker_chevron_extract_image_geometry(const VehicleMarkerChevronCandidate_t *candidate,
                                                           VehicleMarkerPoint2D_t *apex,
                                                           VehicleMarkerPoint2D_t *left_tip,
                                                           VehicleMarkerPoint2D_t *right_tip)
{
    const VisionContour_t *contour;
    VehicleMarkerPoint2D_t pair_a = {0};
    VehicleMarkerPoint2D_t pair_b = {0};
    VehicleMarkerPoint2D_t apex_point = {0};
    uint16 point_count = 0U;
    uint16 pair_i = 0U;
    uint16 pair_j = 0U;
    uint16 apex_i = 0U;
    uint16 i = 0U;
    uint16 j = 0U;
    float best_pair_dist_sq = -1.0f;
    float best_apex_dist_sq = -1.0f;
    float cross = 0.0f;
    float dot = 0.0f;
    float len_a = 0.0f;
    float len_b = 0.0f;
    float angle = 0.0f;

    if(candidate == NULL || apex == NULL || left_tip == NULL || right_tip == NULL)
    {
        return 0U;
    }
    contour = &candidate->contour;
    point_count = contour->point_count;
    if(point_count < VEHICLE_MARKER_CHEVRON_MIN_CONTOUR_POINTS)
    {
        return 0U;
    }

    /* The diameter of the component gives the two arm endpoints. */
    for(i = 0U; i + 1U < point_count; i++)
    {
        VehicleMarkerPoint2D_t point_i = {(float)contour->point_x[i],
                                          (float)contour->point_y[i], 0.0f};
        for(j = (uint16)(i + 1U); j < point_count; j++)
        {
            VehicleMarkerPoint2D_t point_j = {(float)contour->point_x[j],
                                              (float)contour->point_y[j], 0.0f};
            float dist_sq = vehicle_marker_chevron_image_distance_sq(&point_i, &point_j);
            if(dist_sq > best_pair_dist_sq)
            {
                best_pair_dist_sq = dist_sq;
                pair_i = i;
                pair_j = j;
            }
        }
    }

    if(best_pair_dist_sq <= VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        return 0U;
    }

    pair_a.u = (float)contour->point_x[pair_i];
    pair_a.v = (float)contour->point_y[pair_i];
    pair_a.intensity = 0.0f;
    pair_b.u = (float)contour->point_x[pair_j];
    pair_b.v = (float)contour->point_y[pair_j];
    pair_b.intensity = 0.0f;

    /* The apex is the contour point furthest from the tip-to-tip chord. */
    for(i = 0U; i < point_count; i++)
    {
        VehicleMarkerPoint2D_t point = {(float)contour->point_x[i],
                                        (float)contour->point_y[i], 0.0f};
        float dist_sq = vehicle_marker_chevron_image_point_to_line_distance_sq(&point,
                                                                                 &pair_a,
                                                                                 &pair_b);
        if(dist_sq > best_apex_dist_sq)
        {
            best_apex_dist_sq = dist_sq;
            apex_i = i;
        }
    }

    if(best_apex_dist_sq <= VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        return 0U;
    }

    apex_point.u = (float)contour->point_x[apex_i];
    apex_point.v = (float)contour->point_y[apex_i];
    apex_point.intensity = 0.0f;
    if(vehicle_marker_chevron_image_distance_sq(&apex_point, &pair_a) <= VEHICLE_MARKER_CHEVRON_POINT_EPS ||
       vehicle_marker_chevron_image_distance_sq(&apex_point, &pair_b) <= VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        return 0U;
    }

    len_a = vehicle_marker_chevron_image_distance_sq(&apex_point, &pair_a);
    len_b = vehicle_marker_chevron_image_distance_sq(&apex_point, &pair_b);
    cross = (pair_a.u - apex_point.u) * (pair_b.v - apex_point.v) -
            (pair_a.v - apex_point.v) * (pair_b.u - apex_point.u);
    if(cross * cross <= VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        return 0U;
    }

    dot = (pair_a.u - apex_point.u) * (pair_b.u - apex_point.u) +
          (pair_a.v - apex_point.v) * (pair_b.v - apex_point.v);
    angle = acosf(vehicle_marker_chevron_clamp_float(dot / sqrtf(len_a * len_b), -1.0f, 1.0f));
    if(angle < 0.35f || angle > 3.00f)
    {
        return 0U;
    }

    *apex = apex_point;
    /* Keep the labels deterministic. In image coordinates a positive cross
     * product is assigned to the left arm; labels do not affect geometry. */
    if(cross >= 0.0f)
    {
        *left_tip = pair_a;
        *right_tip = pair_b;
    }
    else
    {
        *left_tip = pair_b;
        *right_tip = pair_a;
    }
    return 1U;
}

static uint8 vehicle_marker_chevron_process_candidate(const VehicleMarkerChevronCandidate_t *candidate,
                                                      VehicleMarkerPose_t *out_pose)
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
                                                      &right_tip_img))
    {
        return 0U;
    }

    if(!IPM_Image2WorldF(apex_img.u, apex_img.v, &apex.x, &apex.y) ||
       !IPM_Image2WorldF(left_tip_img.u, left_tip_img.v, &left_tip.x, &left_tip.y) ||
       !IPM_Image2WorldF(right_tip_img.u, right_tip_img.v, &right_tip.x, &right_tip.y))
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

static uint8 vehicle_marker_chevron_process_candidate_legacy(VehicleMarkerPose_t *out_pose)
{
    VehicleMarkerWorldPoint_t apex = {0};
    VehicleMarkerWorldPoint_t left_tip = {0};
    VehicleMarkerWorldPoint_t right_tip = {0};
    VehicleMarkerPoint2D_t left_line_start_img = {0};
    VehicleMarkerPoint2D_t left_line_end_img = {0};
    VehicleMarkerPoint2D_t right_line_start_img = {0};
    VehicleMarkerPoint2D_t right_line_end_img = {0};
    VehicleMarkerWorldPoint_t pair_a = {0};
    VehicleMarkerWorldPoint_t pair_b = {0};
    VehicleMarkerWorldPoint_t centroid = {0};
    VehicleMarkerChevronLine_t left_fit;
    VehicleMarkerChevronLine_t right_fit;
    uint16 input_point_count = s_vehicle_marker_chevron_candidates[0].contour.point_count;
    uint16 point_count = 0U;
    uint16 left_count = 0U;
    uint16 right_count = 0U;
    uint16 i = 0U;
    uint16 j = 0U;
    uint16 apex_index = 0U;
    uint16 pair_i = 0U;
    uint16 pair_j = 1U;
    float best_pair_dist_sq = -1.0f;
    float best_apex_dist_sq = -1.0f;
    float centroid_x = 0.0f;
    float centroid_y = 0.0f;
    float axis_x = 0.0f;
    float axis_y = 0.0f;
    float axis_len = 0.0f;
    float left_span_cm = 0.0f;
    float right_span_cm = 0.0f;
    float heading_x = 0.0f;
    float heading_y = 0.0f;
    float heading_deg = 0.0f;
    float estimated_z = 0.0f;
    float current_height = g_ipm.height;
    uint8 left_fit_ok = 0U;
    uint8 right_fit_ok = 0U;

    if(input_point_count < VEHICLE_MARKER_CHEVRON_MIN_CONTOUR_POINTS)
    {
        return 0U;
    }

    for(i = 0U; i < input_point_count; i++)
    {
        float world_x = 0.0f;
        float world_y = 0.0f;

        if(!IPM_Image2WorldF((float)s_vehicle_marker_chevron_candidates[0].contour.point_x[i],
                             (float)s_vehicle_marker_chevron_candidates[0].contour.point_y[i],
                             &world_x,
                             &world_y))
        {
            continue;
        }

        if(point_count >= VISION_RUNTIME_CONTOUR_MAX_POINTS)
        {
            break;
        }

        s_vehicle_marker_chevron_world_pts[point_count].x = world_x;
        s_vehicle_marker_chevron_world_pts[point_count].y = world_y;
        centroid_x += world_x;
        centroid_y += world_y;
        point_count++;
    }
    if(point_count < VEHICLE_MARKER_CHEVRON_MIN_CONTOUR_POINTS)
    {
        return 0U;
    }
    centroid_x /= (float)point_count;
    centroid_y /= (float)point_count;
    centroid.x = centroid_x;
    centroid.y = centroid_y;

    for(i = 0U; i + 1U < point_count; i++)
    {
        for(j = (uint16)(i + 1U); j < point_count; j++)
        {
            float dist_sq = vehicle_marker_chevron_point_distance_sq(&s_vehicle_marker_chevron_world_pts[i],
                                                                     &s_vehicle_marker_chevron_world_pts[j]);

            if(dist_sq > best_pair_dist_sq)
            {
                best_pair_dist_sq = dist_sq;
                pair_i = i;
                pair_j = j;
            }
        }
    }
    pair_a = s_vehicle_marker_chevron_world_pts[pair_i];
    pair_b = s_vehicle_marker_chevron_world_pts[pair_j];

    for(i = 0U; i < point_count; i++)
    {
        float dist_sq = vehicle_marker_chevron_point_to_line_distance_sq(&s_vehicle_marker_chevron_world_pts[i],
                                                                         &pair_a,
                                                                         &pair_b);

        if(dist_sq > best_apex_dist_sq)
        {
            best_apex_dist_sq = dist_sq;
            apex_index = i;
        }
    }
    apex = s_vehicle_marker_chevron_world_pts[apex_index];

    axis_x = centroid.x - apex.x;
    axis_y = centroid.y - apex.y;
    axis_len = sqrtf(axis_x * axis_x + axis_y * axis_y);
    if(axis_len < VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        axis_x = ((pair_a.x + pair_b.x) * 0.5f) - apex.x;
        axis_y = ((pair_a.y + pair_b.y) * 0.5f) - apex.y;
        axis_len = sqrtf(axis_x * axis_x + axis_y * axis_y);
    }
    if(axis_len < VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        return 0U;
    }

    for(i = 0U; i < point_count; i++)
    {
        float side = axis_x * (s_vehicle_marker_chevron_world_pts[i].y - apex.y) -
                     axis_y * (s_vehicle_marker_chevron_world_pts[i].x - apex.x);

        if(side >= 0.0f)
        {
            if(left_count >= VISION_RUNTIME_CONTOUR_MAX_POINTS)
            {
                return 0U;
            }
            s_vehicle_marker_chevron_left_pts[left_count++] = s_vehicle_marker_chevron_world_pts[i];
        }
        else
        {
            if(right_count >= VISION_RUNTIME_CONTOUR_MAX_POINTS)
            {
                return 0U;
            }
            s_vehicle_marker_chevron_right_pts[right_count++] = s_vehicle_marker_chevron_world_pts[i];
        }
    }

    if(left_count >= VEHICLE_MARKER_CHEVRON_GROUP_MAX_POINTS ||
       right_count >= VEHICLE_MARKER_CHEVRON_GROUP_MAX_POINTS)
    {
        return 0U;
    }
    s_vehicle_marker_chevron_left_pts[left_count++] = apex;
    s_vehicle_marker_chevron_right_pts[right_count++] = apex;

    left_fit_ok = 0U;
    right_fit_ok = 0U;
    vehicle_marker_chevron_fit_line(s_vehicle_marker_chevron_left_pts, left_count, &left_fit);
    vehicle_marker_chevron_fit_line(s_vehicle_marker_chevron_right_pts, right_count, &right_fit);
    left_fit_ok = left_fit.valid;
    right_fit_ok = right_fit.valid;
    if(!left_fit_ok || !right_fit_ok)
    {
        return 0U;
    }

    if(!vehicle_marker_chevron_fit_group(s_vehicle_marker_chevron_left_pts,
                                         left_count,
                                         &apex,
                                         &left_tip,
                                         &left_fit,
                                         &left_line_start_img,
                                         &left_line_end_img,
                                         &s_vehicle_marker_chevron_debug.left_tip_img))
    {
        return 0U;
    }

    if(!vehicle_marker_chevron_fit_group(s_vehicle_marker_chevron_right_pts,
                                         right_count,
                                         &apex,
                                         &right_tip,
                                         &right_fit,
                                         &right_line_start_img,
                                         &right_line_end_img,
                                         &s_vehicle_marker_chevron_debug.right_tip_img))
    {
        return 0U;
    }

    left_span_cm = sqrtf(vehicle_marker_chevron_point_distance_sq(&apex, &left_tip));
    right_span_cm = sqrtf(vehicle_marker_chevron_point_distance_sq(&apex, &right_tip));
    heading_x = left_fit.dir_x + right_fit.dir_x;
    heading_y = left_fit.dir_y + right_fit.dir_y;
    if((heading_x * heading_x + heading_y * heading_y) < VEHICLE_MARKER_CHEVRON_POINT_EPS)
    {
        heading_x = left_fit.dir_x;
        heading_y = left_fit.dir_y;
    }
    heading_deg = atan2f(heading_y, heading_x) * (180.0f / 3.14159265f);

    if(current_height <= 1.0f)
    {
        current_height = VEHICLE_MARKER_CHEVRON_REF_HEIGHT_CM;
    }
    estimated_z = current_height;

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

    return 1U;
}
uint8 VehicleMarkerLocalization_EndBlobFrame(VehicleMarkerPose_t *out_pose)
{
    uint8 tried[VEHICLE_MARKER_CHEVRON_MAX_FRAME_CANDIDATES] = {0};
    uint8 attempt;

    s_vehicle_marker_debug_raw_detect_valid = 0U;
    s_vehicle_marker_debug_process_count++;
    s_vehicle_marker_chevron_debug.candidate_count = s_vehicle_marker_chevron_candidate_count;

    if(s_vehicle_marker_chevron_candidate_count == 0U)
    {
        s_vehicle_marker_chevron_debug.reject_reason =
            VEHICLE_MARKER_CHEVRON_REJECT_NO_CANDIDATE;
        s_vehicle_marker_debug_process_fail_count++;
        s_vehicle_marker_chevron_debug.valid = 0U;
        vehicle_marker_set_position_lost();
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
            if(tried[i] || !s_vehicle_marker_chevron_candidates[i].valid)
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
               &s_vehicle_marker_chevron_candidates[best_index], out_pose))
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
    vehicle_marker_set_position_lost();
    if(out_pose != NULL)
    {
        vehicle_marker_copy_pose(out_pose);
    }
    return 0U;
}

uint8 VehicleMarkerLocalization_FindPoints(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    VehicleMarkerPoint2D_t out_pts[VEHICLE_MARKER_POINT_COUNT])
{
    (void)image;
    if(out_pts != NULL)
    {
        memset(out_pts, 0, sizeof(VehicleMarkerPoint2D_t) * VEHICLE_MARKER_POINT_COUNT);
    }
    return 0U;
}

uint8 VehicleMarkerLocalization_FindPointsBinary(
    const uint8 *binary_image,
    VehicleMarkerPoint2D_t out_pts[VEHICLE_MARKER_POINT_COUNT])
{
    (void)binary_image;
    if(out_pts != NULL)
    {
        memset(out_pts, 0, sizeof(VehicleMarkerPoint2D_t) * VEHICLE_MARKER_POINT_COUNT);
    }
    return 0U;
}

uint8 VehicleMarkerLocalization_Process(const uint8 image[MT9V03X_H][MT9V03X_W],
                                        VehicleMarkerPose_t *out_pose)
{
    (void)image;
    vehicle_marker_set_position_lost();
    if(out_pose != NULL)
    {
        vehicle_marker_copy_pose(out_pose);
    }
    return 0U;
}

uint8 VehicleMarkerLocalization_ProcessBinary(const uint8 *binary_image,
                                              VehicleMarkerPose_t *out_pose)
{
    (void)binary_image;
    vehicle_marker_set_position_lost();
    if(out_pose != NULL)
    {
        vehicle_marker_copy_pose(out_pose);
    }
    return 0U;
}

#else
#error "Unsupported VEHICLE_MARKER_MODE"
#endif

const uint8 *VehicleMarkerLocalization_GetBinaryImage(void)
{
    return g_vehicle_marker_binary_image;
}
