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

static volatile uint8 s_ir_debug_raw_detect_valid = 0U;
static volatile uint32 s_ir_debug_task_5ms_count = 0U;
static volatile uint32 s_ir_debug_process_count = 0U;
static volatile uint32 s_ir_debug_process_success_count = 0U;
static volatile uint32 s_ir_debug_process_fail_count = 0U;
static volatile uint32 s_ir_debug_mark_lost_count = 0U;
static volatile uint32 s_ir_debug_timeout_count = 0U;

// Binary image buffer used by the IR point detector.
static uint8 g_binary_image[MT9V03X_H * MT9V03X_W];

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

static void binarize_image(const uint8 *src, uint8 *dst, int width, int height, uint8 threshold)
{
    int i = 0;

    for(i = 0; i < width * height; i++)
    {
        dst[i] = (src[i] >= threshold) ? 255 : 0;
    }
}

static uint8 find_ir_points_internal(const uint8 *binary_img, int img_w, int img_h,
                                     IR_Point2D_t out_pts[IR_SQUARE_POINT_COUNT],
                                     uint8 *out_point_count)
{
    typedef struct
    {
        float sum_x;
        float sum_y;
        int count;
        int min_x;
        int max_x;
        int min_y;
        int max_y;
    } Region_t;

    Region_t regions[IR_MAX_CANDIDATES];
    static uint8 visited[MT9V03X_H * MT9V03X_W];
    int region_count = 0;
    int y = 0;
    int x = 0;
    int i = 0;
    int j = 0;
    int selected_count = 0;

    memset(visited, 0, (size_t)(img_w * img_h));
    memset(out_pts, 0, sizeof(IR_Point2D_t) * IR_SQUARE_POINT_COUNT);
    if(out_point_count != NULL)
    {
        *out_point_count = 0U;
    }

    for(y = 0; y < img_h; y++)
    {
        for(x = 0; x < img_w; x++)
        {
            int idx = y * img_w + x;

            if(binary_img[idx] == 0 || visited[idx])
            {
                continue;
            }

            if(region_count >= IR_MAX_CANDIDATES)
            {
                break;
            }

            {
                Region_t *r = &regions[region_count];
                static int stack_x[1000];
                static int stack_y[1000];
                int stack_top = 0;

                r->sum_x = 0.0f;
                r->sum_y = 0.0f;
                r->count = 0;
                r->min_x = img_w;
                r->max_x = 0;
                r->min_y = img_h;
                r->max_y = 0;

                stack_x[stack_top] = x;
                stack_y[stack_top] = y;
                stack_top++;
                visited[idx] = 1;

                while(stack_top > 0)
                {
                    int cx = 0;
                    int cy = 0;
                    int dy = 0;
                    int dx = 0;

                    stack_top--;
                    cx = stack_x[stack_top];
                    cy = stack_y[stack_top];

                    r->sum_x += (float)cx;
                    r->sum_y += (float)cy;
                    r->count++;

                    if(cx < r->min_x) r->min_x = cx;
                    if(cx > r->max_x) r->max_x = cx;
                    if(cy < r->min_y) r->min_y = cy;
                    if(cy > r->max_y) r->max_y = cy;

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

                            if(nx < 0 || nx >= img_w || ny < 0 || ny >= img_h)
                            {
                                continue;
                            }

                            nidx = ny * img_w + nx;
                            if(binary_img[nidx] != 0 && !visited[nidx])
                            {
                                visited[nidx] = 1;
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

                if(r->count >= 3)
                {
                    region_count++;
                }
            }
        }
    }

    if(region_count < IR_SQUARE_MIN_DETECT_POINTS)
    {
        return 0;
    }

    for(i = 0; i < region_count - 1; i++)
    {
        for(j = i + 1; j < region_count; j++)
        {
            if(regions[j].count > regions[i].count)
            {
                Region_t tmp = regions[i];
                regions[i] = regions[j];
                regions[j] = tmp;
            }
        }
    }

    selected_count = region_count;
    if(selected_count > IR_SQUARE_POINT_COUNT)
    {
        selected_count = IR_SQUARE_POINT_COUNT;
    }

    for(i = 0; i < selected_count; i++)
    {
        out_pts[i].u = regions[i].sum_x / (float)regions[i].count;
        out_pts[i].v = regions[i].sum_y / (float)regions[i].count;
        out_pts[i].intensity = (float)regions[i].count;
    }

    if(out_point_count != NULL)
    {
        *out_point_count = (uint8)selected_count;
    }

    return 1;
}

static uint8 IR_FindPointsWithCount(const uint8 image[MT9V03X_H][MT9V03X_W],
                                    IR_Point2D_t out_pts[IR_SQUARE_POINT_COUNT],
                                    uint8 *out_point_count)
{
    binarize_image((const uint8 *)image, g_binary_image, MT9V03X_W, MT9V03X_H, IR_THRESHOLD);
    return find_ir_points_internal(g_binary_image, MT9V03X_W, MT9V03X_H, out_pts, out_point_count);
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

    if(side_avg < IR_THREE_POINT_MIN_SIDE_CM)
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

uint8 IR_FindPoints(const uint8 image[MT9V03X_H][MT9V03X_W],
                    IR_Point2D_t out_pts[IR_SQUARE_POINT_COUNT])
{
    uint8 point_count = 0U;

    return IR_FindPointsWithCount(image, out_pts, &point_count);
}

uint8 IR_Localization_Process(const uint8 image[MT9V03X_H][MT9V03X_W],
                              IR_Pose_t *out_pose)
{
    IR_Point2D_t img_pts[IR_SQUARE_POINT_COUNT];
    IR_WorldPoint_t world_pts[IR_SQUARE_POINT_COUNT];
    float center_x = 0.0f;
    float center_y = 0.0f;
    float filtered_center_x = 0.0f;
    float filtered_center_y = 0.0f;
    float square_side_cm = 0.0f;
    float real_side_cm = IR_SQUARE_SIDE_M * 100.0f;
    float current_height = g_ipm.height;
    float estimated_z = 0.0f;
    uint8 detected_point_count = 0U;
    uint8 reconstructed_point = 0U;
    int i = 0;

    s_ir_debug_process_count++;

    if(!IR_FindPointsWithCount(image, img_pts, &detected_point_count))
    {
        return IR_Localization_ProcessFail(out_pose);
    }

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

const uint8* IR_GetBinaryImage(void)
{
    return g_binary_image;
}
