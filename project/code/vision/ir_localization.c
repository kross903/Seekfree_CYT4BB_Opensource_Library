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

// Binary image buffer used by the IR point detector.
static uint8 g_binary_image[MT9V03X_H * MT9V03X_W];

static void binarize_image(const uint8 *src, uint8 *dst, int width, int height, uint8 threshold)
{
    int i = 0;

    for(i = 0; i < width * height; i++)
    {
        dst[i] = (src[i] >= threshold) ? 255 : 0;
    }
}

static uint8 find_ir_points_internal(const uint8 *binary_img, int img_w, int img_h,
                                     IR_Point2D_t out_pts[3])
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

    memset(visited, 0, (size_t)(img_w * img_h));

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

    if(region_count < 3)
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

    for(i = 0; i < 3; i++)
    {
        out_pts[i].u = regions[i].sum_x / (float)regions[i].count;
        out_pts[i].v = regions[i].sum_y / (float)regions[i].count;
        out_pts[i].intensity = (float)regions[i].count;
    }

    return 1;
}

static float calculate_triangle_side(const IR_WorldPoint_t pts[3])
{
    float d01 = sqrtf((pts[1].x - pts[0].x) * (pts[1].x - pts[0].x) +
                      (pts[1].y - pts[0].y) * (pts[1].y - pts[0].y));
    float d12 = sqrtf((pts[2].x - pts[1].x) * (pts[2].x - pts[1].x) +
                      (pts[2].y - pts[1].y) * (pts[2].y - pts[1].y));
    float d20 = sqrtf((pts[0].x - pts[2].x) * (pts[0].x - pts[2].x) +
                      (pts[0].y - pts[2].y) * (pts[0].y - pts[2].y));

    return (d01 + d12 + d20) / 3.0f;
}

static void calculate_triangle_center(const IR_WorldPoint_t pts[3],
                                      float *center_x, float *center_y)
{
    *center_x = (pts[0].x + pts[1].x + pts[2].x) / 3.0f;
    *center_y = (pts[0].y + pts[1].y + pts[2].y) / 3.0f;
}

void IR_Localization_Init(void)
{
    ir_pose_estimate.x = 0.0f;
    ir_pose_estimate.y = 0.0f;
    ir_pose_estimate.z = 0.0f;
    ir_pose_estimate.triangle_side_cm = 0.0f;
    ir_pose_estimate.valid = 0;
    ir_pose_update_seq = 0;
}

uint8 IR_FindPoints(const uint8 image[MT9V03X_H][MT9V03X_W],
                    IR_Point2D_t out_pts[3])
{
    binarize_image((const uint8 *)image, g_binary_image, MT9V03X_W, MT9V03X_H, IR_THRESHOLD);
    return find_ir_points_internal(g_binary_image, MT9V03X_W, MT9V03X_H, out_pts);
}

static void IR_Localization_CopyOldPose(IR_Pose_t *out_pose)
{
    if(out_pose)
    {
        *out_pose = ir_pose_estimate;
        out_pose->valid = 0;
    }
}

uint8 IR_Localization_Process(const uint8 image[MT9V03X_H][MT9V03X_W],
                              IR_Pose_t *out_pose)
{
    IR_Point2D_t img_pts[3];
    IR_WorldPoint_t world_pts[3];
    float center_x = 0.0f;
    float center_y = 0.0f;
    float triangle_side_cm = 0.0f;
    float real_side_cm = IR_TRIANGLE_SIDE_M * 100.0f;
    float current_height = g_ipm.height;
    float estimated_z = 0.0f;
    int i = 0;

    if(!IR_FindPoints(image, img_pts))
    {
        ir_pose_estimate.valid = 0;
        IR_Localization_CopyOldPose(out_pose);
        return 0;
    }

    for(i = 0; i < 3; i++)
    {
        float world_x = 0.0f;
        float world_y = 0.0f;

        ir_pose_estimate.img_pts[i] = img_pts[i];

        IPM_Image2World((int)img_pts[i].u, (int)img_pts[i].v, &world_x, &world_y);

        world_pts[i].x = world_x;
        world_pts[i].y = world_y;
        ir_pose_estimate.world_pts[i] = world_pts[i];
    }

    calculate_triangle_center(world_pts, &center_x, &center_y);
    triangle_side_cm = calculate_triangle_side(world_pts);

    if(triangle_side_cm > 0.5f && current_height > 1.0f)
    {
        estimated_z = current_height * (real_side_cm / triangle_side_cm);

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

    ir_pose_estimate.x = center_x;
    ir_pose_estimate.y = center_y;
    ir_pose_estimate.z = estimated_z;
    ir_pose_estimate.triangle_side_cm = triangle_side_cm;
    ir_pose_estimate.valid = 1;
    ir_pose_update_seq++;

    if(out_pose)
    {
        *out_pose = ir_pose_estimate;
    }

    return 1;
}

const uint8* IR_GetBinaryImage(void)
{
    return g_binary_image;
}
