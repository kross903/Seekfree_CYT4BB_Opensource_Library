#include "ir_localization.h"
#include "common/project_headfile.h"
#include <math.h>
#include <string.h>

// Global pose result exported to the rest of the project.
IR_Pose_t ir_pose_estimate = {0};

/*
 * [2026-03-10]
 * Purpose:
 * 1. Expose a monotonic sequence so the height fusion module can tell whether
 *    the visual height sample is new in this cycle.
 * 2. Prevent one old visual sample from being reused across multiple fusion steps.
 */
volatile uint32 ir_pose_update_seq = 0;

volatile uint8 ir_position_lost_flag = 1U;
volatile uint16 ir_position_lost_elapsed_ms = IR_POSITION_LOST_HOLD_MS;

#define IR_THREE_POINT_MIN_SIDE_CM          (2.0f)
#define IR_THREE_POINT_SIDE_RATIO_MAX       (0.45f)
#define IR_THREE_POINT_DIAG_RATIO_MAX       (0.45f)
#define IR_CENTER_KALMAN_Q                  (0.20f)
#define IR_CENTER_KALMAN_R                  (4.0f)
#define IR_CENTER_KALMAN_INITIAL_P          (10.0f)
#define IR_BLOB_MIN_AREA_PX                 (3)
#define IR_BLOB_MAX_AREA_PX                 (260)
#define IR_BLOB_MAX_WIDTH_PX                (32)
#define IR_BLOB_MAX_HEIGHT_PX               (32)
#define IR_BLOB_MAX_ASPECT_RATIO            (2.5f)
#define IR_BLOB_REF_HEIGHT_CM               (100.0f)
#define IR_BLOB_HEIGHT_MIN_CM               (35.0f)
#define IR_BLOB_HEIGHT_MAX_CM               (300.0f)
#define IR_BLOB_MIN_AREA_FLOOR_PX           (1)
#define IR_BLOB_MAX_AREA_CEILING_PX         (1200)
#define IR_BLOB_MAX_WIDTH_CEILING_PX        (80)
#define IR_BLOB_MAX_HEIGHT_CEILING_PX       (80)
#define IR_SQUARE_SIDE_TARGET_CM            (IR_SQUARE_SIDE_M * 100.0f)
#define IR_SQUARE_SIDE_MIN_CM               (5.0f)
#define IR_SQUARE_SIDE_MAX_CM               (18.0f)
#define IR_SQUARE_SIDE_SPREAD_MAX           (0.50f)
#define IR_SQUARE_DIAG_SPREAD_MAX           (0.35f)
#define IR_SQUARE_DIAG_RATIO_MAX            (0.35f)
#define IR_SQUARE_GEOMETRY_MAX_SCORE        (3.5f)
#define IR_SQUARE_CENTER_JUMP_SOFT_CM       (40.0f)
#define IR_SQUARE_CENTER_JUMP_HARD_CM       (100.0f)
#define IR_THREE_POINT_SCORE_PENALTY        (0.60f)
#define IR_SQRT2                            (1.41421356f)

typedef struct
{
    int min_area;
    int max_area;
    int max_width;
    int max_height;
    float expected_area;
} IR_BlobGate_t;

typedef struct
{
    IR_Point2D_t img;
    IR_WorldPoint_t world;
    float score;
} IR_Candidate_t;

typedef struct
{
    float x;
    float p;
    float q;
    float r;
    float k;
    uint8 initialized;
} IR_Kalman1D_t;

static uint8 s_ir_has_trackable_pose = 0U;
static uint16 s_ir_lost_elapsed_ms = IR_POSITION_LOST_HOLD_MS;
static IR_Kalman1D_t s_ir_center_kalman_x = {0};
static IR_Kalman1D_t s_ir_center_kalman_y = {0};
static IR_Candidate_t s_ir_frame_candidates[IR_MAX_CANDIDATES];
static uint8 s_ir_frame_candidate_count = 0U;

static volatile uint8 s_ir_debug_raw_detect_valid = 0U;
static volatile uint32 s_ir_debug_task_5ms_count = 0U;
static volatile uint32 s_ir_debug_process_count = 0U;
static volatile uint32 s_ir_debug_process_success_count = 0U;
static volatile uint32 s_ir_debug_process_fail_count = 0U;
static volatile uint32 s_ir_debug_mark_lost_count = 0U;
static volatile uint32 s_ir_debug_timeout_count = 0U;

static uint8 select_square_points_from_candidates(const IR_Candidate_t candidates[IR_MAX_CANDIDATES],
                                                  uint8 candidate_count,
                                                  IR_Point2D_t out_pts[IR_SQUARE_POINT_COUNT],
                                                  uint8 *out_point_count);
static uint8 IR_Localization_ProcessPoints(IR_Point2D_t img_pts[IR_SQUARE_POINT_COUNT],
                                           uint8 detected_point_count,
                                           IR_Pose_t *out_pose);

static void ir_center_kalman_reset_axis(IR_Kalman1D_t *kf)
{
    if(kf == NULL)
    {
        return;
    }

    kf->x = 0.0f;
    kf->p = IR_CENTER_KALMAN_INITIAL_P;
    kf->q = IR_CENTER_KALMAN_Q;
    kf->r = IR_CENTER_KALMAN_R;
    kf->k = 0.0f;
    kf->initialized = 0U;
}

static void ir_center_kalman_reset(void)
{
    ir_center_kalman_reset_axis(&s_ir_center_kalman_x);
    ir_center_kalman_reset_axis(&s_ir_center_kalman_y);
}

static float ir_center_kalman_update_axis(IR_Kalman1D_t *kf, float measurement)
{
    if(kf == NULL)
    {
        return measurement;
    }

    if(!kf->initialized)
    {
        kf->x = measurement;
        kf->p = IR_CENTER_KALMAN_INITIAL_P;
        kf->q = IR_CENTER_KALMAN_Q;
        kf->r = IR_CENTER_KALMAN_R;
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

static void ir_center_kalman_update(float raw_x, float raw_y, float *filtered_x, float *filtered_y)
{
    if(filtered_x != NULL)
    {
        *filtered_x = ir_center_kalman_update_axis(&s_ir_center_kalman_x, raw_x);
    }
    if(filtered_y != NULL)
    {
        *filtered_y = ir_center_kalman_update_axis(&s_ir_center_kalman_y, raw_y);
    }
}

static float ir_clamp_float(float value, float min_value, float max_value)
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

static int ir_clamp_int(int value, int min_value, int max_value)
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

static int ir_round_to_int(float value)
{
    return (int)(value + 0.5f);
}

static void ir_get_blob_gate(IR_BlobGate_t *gate)
{
    float height_cm = g_ipm.height;
    float linear_scale = 1.0f;
    float area_scale = 1.0f;
    int min_area = IR_BLOB_MIN_AREA_PX;
    int max_area = IR_BLOB_MAX_AREA_PX;
    int max_width = IR_BLOB_MAX_WIDTH_PX;
    int max_height = IR_BLOB_MAX_HEIGHT_PX;

    if(gate == NULL)
    {
        return;
    }

    if(height_cm <= 1.0f)
    {
        height_cm = IR_BLOB_REF_HEIGHT_CM;
    }

    height_cm = ir_clamp_float(height_cm, IR_BLOB_HEIGHT_MIN_CM, IR_BLOB_HEIGHT_MAX_CM);
    linear_scale = IR_BLOB_REF_HEIGHT_CM / height_cm;
    area_scale = linear_scale * linear_scale;

    min_area = ir_round_to_int((float)IR_BLOB_MIN_AREA_PX * area_scale);
    max_area = ir_round_to_int((float)IR_BLOB_MAX_AREA_PX * area_scale);
    max_width = ir_round_to_int((float)IR_BLOB_MAX_WIDTH_PX * linear_scale);
    max_height = ir_round_to_int((float)IR_BLOB_MAX_HEIGHT_PX * linear_scale);

    gate->min_area = ir_clamp_int(min_area, IR_BLOB_MIN_AREA_FLOOR_PX, IR_BLOB_MAX_AREA_PX);
    gate->max_area = ir_clamp_int(max_area, gate->min_area, IR_BLOB_MAX_AREA_CEILING_PX);
    gate->max_width = ir_clamp_int(max_width, 1, IR_BLOB_MAX_WIDTH_CEILING_PX);
    gate->max_height = ir_clamp_int(max_height, 1, IR_BLOB_MAX_HEIGHT_CEILING_PX);
    gate->expected_area = sqrtf((float)gate->min_area * (float)gate->max_area);
    if(gate->expected_area < 1.0f)
    {
        gate->expected_area = 1.0f;
    }
}

static void IR_Localization_InsertCandidate(const VisionBlob_t *blob, const IR_BlobGate_t *blob_gate)
{
    IR_Candidate_t candidate = {0};
    float area_error = 0.0f;
    int replace_index = -1;
    float worst_score = -1.0f;
    int replace_i = 0;

    if(blob == NULL || blob_gate == NULL)
    {
        return;
    }

    candidate.img.u = blob->centroid_col;
    candidate.img.v = blob->centroid_row;
    candidate.img.intensity = (float)blob->area;
    candidate.world.x = blob->world_x;
    candidate.world.y = blob->world_y;
    area_error = fabsf((float)blob->area - blob_gate->expected_area) / blob_gate->expected_area;
    candidate.score = area_error * 20.0f + (blob->aspect_ratio - 1.0f) * 10.0f;

    if(s_ir_frame_candidate_count < IR_MAX_CANDIDATES)
    {
        s_ir_frame_candidates[s_ir_frame_candidate_count] = candidate;
        s_ir_frame_candidate_count++;
        return;
    }

    for(replace_i = 0; replace_i < IR_MAX_CANDIDATES; replace_i++)
    {
        if(s_ir_frame_candidates[replace_i].score > worst_score)
        {
            worst_score = s_ir_frame_candidates[replace_i].score;
            replace_index = replace_i;
        }
    }

    if(replace_index >= 0 && candidate.score < worst_score)
    {
        s_ir_frame_candidates[replace_index] = candidate;
    }
}

static float world_point_distance_cm(const IR_WorldPoint_t *a, const IR_WorldPoint_t *b)
{
    return sqrtf((b->x - a->x) * (b->x - a->x) +
                 (b->y - a->y) * (b->y - a->y));
}

static void sort_float_ascending(float *values, int count)
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

static float calculate_square_side(const IR_WorldPoint_t pts[IR_SQUARE_POINT_COUNT])
{
    float distances[6];
    float side_sum = 0.0f;
    int dist_count = 0;
    int i = 0;
    int j = 0;

    for(i = 0; i < IR_SQUARE_POINT_COUNT - 1; i++)
    {
        for(j = i + 1; j < IR_SQUARE_POINT_COUNT; j++)
        {
            distances[dist_count] = world_point_distance_cm(&pts[i], &pts[j]);
            dist_count++;
        }
    }

    sort_float_ascending(distances, dist_count);

    for(i = 0; i < 4; i++)
    {
        side_sum += distances[i];
    }

    return side_sum / 4.0f;
}

static void calculate_square_center(const IR_WorldPoint_t pts[IR_SQUARE_POINT_COUNT],
                                    float *center_x, float *center_y)
{
    int i = 0;

    *center_x = 0.0f;
    *center_y = 0.0f;

    for(i = 0; i < IR_SQUARE_POINT_COUNT; i++)
    {
        *center_x += pts[i].x;
        *center_y += pts[i].y;
    }

    *center_x /= (float)IR_SQUARE_POINT_COUNT;
    *center_y /= (float)IR_SQUARE_POINT_COUNT;
}

static uint8 reconstruct_square_from_three_points(IR_WorldPoint_t world_pts[IR_SQUARE_POINT_COUNT],
                                                  IR_Point2D_t img_pts[IR_SQUARE_POINT_COUNT])
{
    float d01 = world_point_distance_cm(&world_pts[0], &world_pts[1]);
    float d02 = world_point_distance_cm(&world_pts[0], &world_pts[2]);
    float d12 = world_point_distance_cm(&world_pts[1], &world_pts[2]);
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

    side_a = world_point_distance_cm(&world_pts[right], &world_pts[diag_a]);
    side_b = world_point_distance_cm(&world_pts[right], &world_pts[diag_b]);
    side_avg = 0.5f * (side_a + side_b);

    if(side_avg < IR_THREE_POINT_MIN_SIDE_CM || side_avg > IR_SQUARE_SIDE_MAX_CM)
    {
        return 0U;
    }

    diag_expected = sqrtf(side_a * side_a + side_b * side_b);
    if(diag_expected < IR_THREE_POINT_MIN_SIDE_CM)
    {
        return 0U;
    }

    side_ratio = fabsf(side_a - side_b) / side_avg;
    diag_ratio = fabsf(diag - diag_expected) / diag_expected;

    if(side_ratio > IR_THREE_POINT_SIDE_RATIO_MAX || diag_ratio > IR_THREE_POINT_DIAG_RATIO_MAX)
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

static float square_center_jump_penalty(const IR_WorldPoint_t world_pts[IR_SQUARE_POINT_COUNT])
{
    float center_x = 0.0f;
    float center_y = 0.0f;
    float dx = 0.0f;
    float dy = 0.0f;
    float distance = 0.0f;

    if(!s_ir_has_trackable_pose || ir_position_lost_flag || !ir_pose_estimate.valid)
    {
        return 0.0f;
    }

    calculate_square_center(world_pts, &center_x, &center_y);
    dx = center_x - ir_pose_estimate.raw_x;
    dy = center_y - ir_pose_estimate.raw_y;
    distance = sqrtf(dx * dx + dy * dy);

    if(distance > IR_SQUARE_CENTER_JUMP_HARD_CM)
    {
        return IR_SQUARE_GEOMETRY_MAX_SCORE + 1.0f;
    }

    return distance / IR_SQUARE_CENTER_JUMP_SOFT_CM;
}

static uint8 evaluate_square_geometry(const IR_WorldPoint_t world_pts[IR_SQUARE_POINT_COUNT],
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

    for(i = 0; i < IR_SQUARE_POINT_COUNT - 1; i++)
    {
        for(j = i + 1; j < IR_SQUARE_POINT_COUNT; j++)
        {
            distances[dist_count] = world_point_distance_cm(&world_pts[i], &world_pts[j]);
            dist_count++;
        }
    }

    sort_float_ascending(distances, dist_count);

    for(i = 0; i < 4; i++)
    {
        side_sum += distances[i];
    }

    side_avg = side_sum / 4.0f;
    diag_avg = 0.5f * (distances[4] + distances[5]);

    if(side_avg < IR_SQUARE_SIDE_MIN_CM || side_avg > IR_SQUARE_SIDE_MAX_CM || diag_avg < IR_SQUARE_SIDE_MIN_CM)
    {
        return 0U;
    }

    side_spread = (distances[3] - distances[0]) / side_avg;
    diag_spread = fabsf(distances[5] - distances[4]) / diag_avg;
    diag_expected = side_avg * IR_SQRT2;
    diag_ratio = fabsf(diag_avg - diag_expected) / diag_expected;
    target_ratio = fabsf(side_avg - IR_SQUARE_SIDE_TARGET_CM) / IR_SQUARE_SIDE_TARGET_CM;

    if(side_spread > IR_SQUARE_SIDE_SPREAD_MAX ||
       diag_spread > IR_SQUARE_DIAG_SPREAD_MAX ||
       diag_ratio > IR_SQUARE_DIAG_RATIO_MAX)
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

static uint8 select_square_points_from_candidates(const IR_Candidate_t candidates[IR_MAX_CANDIDATES],
                                                  uint8 candidate_count,
                                                  IR_Point2D_t out_pts[IR_SQUARE_POINT_COUNT],
                                                  uint8 *out_point_count)
{
    IR_Point2D_t best_img_pts[IR_SQUARE_POINT_COUNT];
    float best_score = IR_SQUARE_GEOMETRY_MAX_SCORE;
    uint8 best_count = 0U;
    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;
    int i = 0;

    memset(best_img_pts, 0, sizeof(best_img_pts));
    memset(out_pts, 0, sizeof(IR_Point2D_t) * IR_SQUARE_POINT_COUNT);
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
                    IR_WorldPoint_t world_pts[IR_SQUARE_POINT_COUNT];
                    float score = 0.0f;

                    world_pts[0] = candidates[a].world;
                    world_pts[1] = candidates[b].world;
                    world_pts[2] = candidates[c].world;
                    world_pts[3] = candidates[d].world;

                    if(!evaluate_square_geometry(world_pts, &score))
                    {
                        continue;
                    }

                    score += square_center_jump_penalty(world_pts);

                    if(score < best_score)
                    {
                        best_score = score;
                        best_count = IR_SQUARE_POINT_COUNT;
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
                IR_WorldPoint_t world_pts[IR_SQUARE_POINT_COUNT];
                IR_Point2D_t img_pts[IR_SQUARE_POINT_COUNT];
                float score = 0.0f;

                memset(world_pts, 0, sizeof(world_pts));
                memset(img_pts, 0, sizeof(img_pts));

                world_pts[0] = candidates[a].world;
                world_pts[1] = candidates[b].world;
                world_pts[2] = candidates[c].world;
                img_pts[0] = candidates[a].img;
                img_pts[1] = candidates[b].img;
                img_pts[2] = candidates[c].img;

                if(!reconstruct_square_from_three_points(world_pts, img_pts))
                {
                    continue;
                }

                if(!evaluate_square_geometry(world_pts, &score))
                {
                    continue;
                }

                score += IR_THREE_POINT_SCORE_PENALTY + square_center_jump_penalty(world_pts);

                if(score < best_score)
                {
                    best_score = score;
                    best_count = IR_SQUARE_MIN_DETECT_POINTS;
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

    if(best_count < IR_SQUARE_MIN_DETECT_POINTS)
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

static void IR_Localization_ClearPose(void)
{
    memset(&ir_pose_estimate, 0, sizeof(ir_pose_estimate));
}

static void IR_Localization_ClearTrackedCenter(void)
{
    ir_pose_estimate.x = 0.0f;
    ir_pose_estimate.y = 0.0f;
    ir_pose_estimate.raw_x = 0.0f;
    ir_pose_estimate.raw_y = 0.0f;
    ir_pose_estimate.detected_point_count = 0U;
    ir_pose_estimate.reconstructed_point = 0U;
    ir_pose_estimate.valid = 0U;
}

static void IR_Localization_MarkPositionLost(void)
{
    s_ir_debug_mark_lost_count++;
    s_ir_has_trackable_pose = 0U;
    s_ir_lost_elapsed_ms = IR_POSITION_LOST_HOLD_MS;
    ir_position_lost_elapsed_ms = s_ir_lost_elapsed_ms;
    ir_position_lost_flag = 1U;
    ir_center_kalman_reset();
    IR_Localization_ClearTrackedCenter();
}

static void IR_Localization_SetPositionLost(void)
{
    IR_Localization_MarkPositionLost();
    IR_Localization_ClearPose();
}

static void IR_Localization_CopyPose(IR_Pose_t *out_pose)
{
    if(out_pose)
    {
        *out_pose = ir_pose_estimate;
    }
}

static uint8 IR_Localization_ProcessFail(IR_Pose_t *out_pose)
{
    s_ir_debug_raw_detect_valid = 0U;
    s_ir_debug_process_fail_count++;

    if(s_ir_has_trackable_pose && !ir_position_lost_flag)
    {
        ir_pose_estimate.valid = 1U;
    }
    else
    {
        IR_Localization_SetPositionLost();
    }

    IR_Localization_CopyPose(out_pose);
    return 0U;
}

static void IR_Localization_ResetLossDebug(void)
{
    s_ir_debug_raw_detect_valid = 0U;
    s_ir_debug_task_5ms_count = 0U;
    s_ir_debug_process_count = 0U;
    s_ir_debug_process_success_count = 0U;
    s_ir_debug_process_fail_count = 0U;
    s_ir_debug_mark_lost_count = 0U;
    s_ir_debug_timeout_count = 0U;
}

void IR_Localization_GetLossDebug(IR_LossDebug_t *debug)
{
    if(!debug)
    {
        return;
    }

    debug->raw_detect_valid = s_ir_debug_raw_detect_valid;
    debug->has_trackable_pose = s_ir_has_trackable_pose;
    debug->lost_flag = ir_position_lost_flag;
    debug->pose_valid = ir_pose_estimate.valid;
    debug->lost_elapsed_ms = ir_position_lost_elapsed_ms;
    debug->task_5ms_count = s_ir_debug_task_5ms_count;
    debug->process_count = s_ir_debug_process_count;
    debug->process_success_count = s_ir_debug_process_success_count;
    debug->process_fail_count = s_ir_debug_process_fail_count;
    debug->mark_lost_count = s_ir_debug_mark_lost_count;
    debug->timeout_count = s_ir_debug_timeout_count;
}

void IR_Localization_Init(void)
{
    IR_Localization_SetPositionLost();
    ir_pose_update_seq = 0;
    IR_Localization_ResetLossDebug();
}

void IR_Localization_Task_5ms(void)
{
    uint32 next_elapsed_ms = 0U;

    s_ir_debug_task_5ms_count++;

    if(!s_ir_has_trackable_pose)
    {
        if(!ir_position_lost_flag)
        {
            IR_Localization_SetPositionLost();
        }
        return;
    }

    next_elapsed_ms = (uint32)s_ir_lost_elapsed_ms + IR_POSITION_LOST_TASK_PERIOD_MS;

    if(next_elapsed_ms >= IR_POSITION_LOST_HOLD_MS)
    {
        s_ir_debug_timeout_count++;
        IR_Localization_MarkPositionLost();
        return;
    }

    s_ir_lost_elapsed_ms = (uint16)next_elapsed_ms;
    ir_position_lost_elapsed_ms = s_ir_lost_elapsed_ms;
    ir_position_lost_flag = 0U;
    ir_pose_estimate.valid = 1U;
}

void IR_Localization_ServiceLostProtection(void)
{
    if(ir_position_lost_flag)
    {
        IR_Localization_ClearTrackedCenter();
    }
}

static uint8 IR_Localization_ProcessPoints(IR_Point2D_t img_pts[IR_SQUARE_POINT_COUNT],
                                           uint8 detected_point_count,
                                           IR_Pose_t *out_pose)
{
    IR_WorldPoint_t world_pts[IR_SQUARE_POINT_COUNT];
    float center_x = 0.0f;
    float center_y = 0.0f;
    float filtered_center_x = 0.0f;
    float filtered_center_y = 0.0f;
    float square_side_cm = 0.0f;
    float real_side_cm = IR_SQUARE_SIDE_M * 100.0f;
    float current_height = g_ipm.height;
    float estimated_z = 0.0f;
    uint8 reconstructed_point = 0U;
    int i = 0;

    s_ir_debug_raw_detect_valid = 1U;

    for(i = 0; i < detected_point_count; i++)
    {
        float world_x = 0.0f;
        float world_y = 0.0f;

        IPM_Image2World((int)img_pts[i].u, (int)img_pts[i].v, &world_x, &world_y);

        world_pts[i].x = world_x;
        world_pts[i].y = world_y;
    }

    if(detected_point_count == IR_SQUARE_MIN_DETECT_POINTS)
    {
        if(!reconstruct_square_from_three_points(world_pts, img_pts))
        {
            return IR_Localization_ProcessFail(out_pose);
        }
        reconstructed_point = 1U;
    }

    s_ir_debug_process_success_count++;

    for(i = 0; i < IR_SQUARE_POINT_COUNT; i++)
    {
        ir_pose_estimate.img_pts[i] = img_pts[i];
        ir_pose_estimate.world_pts[i] = world_pts[i];
    }

    calculate_square_center(world_pts, &center_x, &center_y);
    square_side_cm = calculate_square_side(world_pts);
    ir_center_kalman_update(center_x, center_y, &filtered_center_x, &filtered_center_y);

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
        estimated_z = ir_pose_estimate.z;
        if(estimated_z < 5.0f)
        {
            estimated_z = current_height;
        }
    }

    ir_pose_estimate.raw_x = center_x;
    ir_pose_estimate.raw_y = center_y;
    ir_pose_estimate.x = filtered_center_x;
    ir_pose_estimate.y = filtered_center_y;
    ir_pose_estimate.z = estimated_z;
    ir_pose_estimate.square_side_cm = square_side_cm;
    ir_pose_estimate.detected_point_count = detected_point_count;
    ir_pose_estimate.reconstructed_point = reconstructed_point;
    ir_pose_estimate.valid = 1;
    s_ir_has_trackable_pose = 1U;
    s_ir_lost_elapsed_ms = 0U;
    ir_position_lost_elapsed_ms = 0U;
    ir_position_lost_flag = 0U;
    ir_pose_update_seq++;

    IR_Localization_CopyPose(out_pose);

    return 1;
}

void IR_Localization_BeginBlobFrame(void)
{
    memset(s_ir_frame_candidates, 0, sizeof(s_ir_frame_candidates));
    s_ir_frame_candidate_count = 0U;
}

void IR_Localization_AppendBlob(const VisionBlob_t *blob)
{
    IR_BlobGate_t blob_gate;
    int width = 0;
    int height = 0;

    if(blob == NULL)
    {
        return;
    }

    ir_get_blob_gate(&blob_gate);
    width = blob->max_col - blob->min_col + 1;
    height = blob->max_row - blob->min_row + 1;

    if(blob->area < blob_gate.min_area || blob->area > blob_gate.max_area)
    {
        return;
    }
    if(width > blob_gate.max_width || height > blob_gate.max_height)
    {
        return;
    }
    if(blob->aspect_ratio > IR_BLOB_MAX_ASPECT_RATIO)
    {
        return;
    }

    IR_Localization_InsertCandidate(blob, &blob_gate);
}

uint8 IR_Localization_EndBlobFrame(IR_Pose_t *out_pose)
{
    IR_Point2D_t img_pts[IR_SQUARE_POINT_COUNT];
    uint8 detected_point_count = 0U;

    s_ir_debug_process_count++;

    memset(img_pts, 0, sizeof(img_pts));
    if(s_ir_frame_candidate_count < IR_SQUARE_MIN_DETECT_POINTS ||
       !select_square_points_from_candidates(s_ir_frame_candidates,
                                             s_ir_frame_candidate_count,
                                             img_pts,
                                             &detected_point_count))
    {
        return IR_Localization_ProcessFail(out_pose);
    }

    return IR_Localization_ProcessPoints(img_pts, detected_point_count, out_pose);
}
