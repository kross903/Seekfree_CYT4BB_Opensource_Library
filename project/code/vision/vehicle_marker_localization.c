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

static void vehicle_marker_binarize_image(const uint8 *src, uint8 *dst,
                                          int width, int height, uint8 threshold)
{
    int i = 0;

    for(i = 0; i < width * height; i++)
    {
        dst[i] = (src[i] >= threshold) ? 255 : 0;
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

void VehicleMarkerLocalization_Init(void)
{
    vehicle_marker_set_position_lost();
    g_vehicle_marker_pose_update_seq = 0U;
    memset(g_vehicle_marker_binary_image, 0, sizeof(g_vehicle_marker_binary_image));
    vehicle_marker_reset_loss_debug();
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

static void vehicle_marker_insert_candidate(
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

    IPM_Image2World((int)candidate.img.u, (int)candidate.img.v, &world_x, &world_y);
    candidate.world.x = world_x;
    candidate.world.y = world_y;

    if(*candidate_count < VEHICLE_MARKER_MAX_CANDIDATES)
    {
        candidates[*candidate_count] = candidate;
        (*candidate_count)++;
        return;
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
    }
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
    VehicleMarkerCandidate_t candidates[VEHICLE_MARKER_MAX_CANDIDATES];
    VehicleMarkerBlobGate_t blob_gate;
    static uint8 visited[MT9V03X_H * MT9V03X_W];
    uint8 candidate_count = 0U;
    int y = 0;
    int x = 0;

    vehicle_marker_binarize_image((const uint8 *)image,
                                  g_vehicle_marker_binary_image,
                                  MT9V03X_W,
                                  MT9V03X_H,
                                  VEHICLE_MARKER_THRESHOLD);
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
                    vehicle_marker_insert_candidate(candidates, &candidate_count, &region, &blob_gate);
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

uint8 VehicleMarkerLocalization_Process(const uint8 image[MT9V03X_H][MT9V03X_W],
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

    if(!vehicle_marker_find_points_with_count(image, img_pts, &detected_point_count))
    {
        return vehicle_marker_process_fail(out_pose);
    }

    s_vehicle_marker_debug_raw_detect_valid = 1U;

    for(i = 0; i < detected_point_count; i++)
    {
        float world_x = 0.0f;
        float world_y = 0.0f;

        IPM_Image2World((int)img_pts[i].u, (int)img_pts[i].v, &world_x, &world_y);
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

#elif VEHICLE_MARKER_MODE == VEHICLE_MARKER_MODE_CHEVRON

/* 占位分支：先保留接口，后续再补 > 识别。 */
int VehicleMarkerLocalization_GetScaledMaxBlobArea(void)
{
    return 0;
}

void VehicleMarkerLocalization_BeginBlobFrame(void)
{
}

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

uint8 VehicleMarkerLocalization_AppendBlob(const VisionBlob_t *blob)
{
    (void)blob;
    return 0U;
}

uint8 VehicleMarkerLocalization_EndBlobFrame(VehicleMarkerPose_t *out_pose)
{
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

#else
#error "Unsupported VEHICLE_MARKER_MODE"
#endif

const uint8 *VehicleMarkerLocalization_GetBinaryImage(void)
{
    return g_vehicle_marker_binary_image;
}
