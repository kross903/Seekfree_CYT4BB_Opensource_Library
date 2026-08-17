#include "vision/ideal_target_match.h"

#include <math.h>
#include <string.h>

#include "vision/ipm_interface.h"

#define IDEAL_ARROW_TRIANGLE_HEIGHT_CM  (15.5639f)
#define IDEAL_ARROW_HALF_BASE_CM        (10.8999f)
#define IDEAL_ARROW_PLANE_HEIGHT_CM     (17.0f)
#define IDEAL_BEACON_RADIUS_CM          (5.0f)
#define IDEAL_BEACON_PLANE_HEIGHT_CM    (0.0f)

#define IDEAL_SAMPLE_STEP_MIN_PX        (2)
#define IDEAL_SAMPLE_MARGIN_PX          (4)
#define IDEAL_POSITIVE_BAND_SQ_Q2       (9U)   /* 1.5px squared, scaled by 4. */
#define IDEAL_NEGATIVE_GUARD_SQ_Q2      (36U)  /* 3px squared, scaled by 4. */
#define IDEAL_NEAR_NEGATIVE_SQ_Q2       (100U) /* 5px squared, scaled by 4. */
#define IDEAL_FINE_WIDTH                 (MT9V03X_W * 2)
#define IDEAL_FINE_HEIGHT                (MT9V03X_H * 2)

#define IDEAL_ARROW_MIN_GRAY_DIFF       (100U)
#define IDEAL_ARROW_MIN_SEPARATION_Q8   (512U)
#define IDEAL_ARROW_MIN_POSITIVE_Q8     (179U)
#define IDEAL_ARROW_MIN_NEGATIVE_Q8     (192U)
#define IDEAL_ARROW_MIN_BINARY_Q8       (200U)
#define IDEAL_ARROW_MIN_COMPLETE_Q8     (154U)
#define IDEAL_ARROW_MIN_NEAR_DIFF       (90U)

#define IDEAL_BEACON_MIN_GRAY_DIFF      (50U)
#define IDEAL_BEACON_MIN_SEPARATION_Q8  (512U)
#define IDEAL_BEACON_MIN_POSITIVE_Q8    (174U)
#define IDEAL_BEACON_MIN_NEGATIVE_Q8    (192U)
#define IDEAL_BEACON_MIN_BINARY_Q8      (154U)
#define IDEAL_BEACON_MIN_COMPLETE_Q8    (77U)
#define IDEAL_BEACON_MIN_NEAR_DIFF      (80U)

#define IDEAL_MIN_POSITIVE_POINTS       (8U)
#define IDEAL_MIN_NEGATIVE_POINTS       (16U)

typedef struct
{
    uint8 positive_value[VISION_IDEAL_MAX_SAMPLES];
    uint8 positive_group[VISION_IDEAL_MAX_SAMPLES];
    uint8 negative_value[VISION_IDEAL_MAX_SAMPLES];
    uint8 negative_near[VISION_IDEAL_MAX_SAMPLES];
    uint16 positive_count;
    uint16 negative_count;
    uint32 positive_sum;
    uint32 negative_sum;
    uint32 positive_square_sum;
    uint32 negative_square_sum;
    uint32 near_negative_sum;
    uint16 near_negative_count;
    uint16 positive_fixed_hits;
    uint16 negative_fixed_hits;
    uint16 group_count[4];
    uint16 group_fixed_hits[4];
} VisionIdealWorkspace_t;

static VisionIdealWorkspace_t s_workspace;
static float s_arrow_forward_x = 1.0f;
static float s_arrow_forward_y = 0.0f;
static uint8 s_arrow_yaw_valid = 0U;

static int VisionIdeal_ClampInt(int value, int minimum, int maximum)
{
    if(value < minimum) return minimum;
    if(value > maximum) return maximum;
    return value;
}

static uint32 VisionIdeal_ApproxLength(int x, int y)
{
    uint32 ax = (uint32)((x < 0) ? -x : x);
    uint32 ay = (uint32)((y < 0) ? -y : y);
    uint32 maximum = (ax > ay) ? ax : ay;
    uint32 minimum = (ax > ay) ? ay : ax;

    return maximum + ((minimum * 3U) >> 3);
}

static uint32 VisionIdeal_IntegerSqrt(uint32 value)
{
    uint32 result = 0U;
    uint32 bit = 1UL << 30;

    while(bit > value) bit >>= 2;
    while(bit != 0U)
    {
        if(value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

static uint8 VisionIdeal_SampleGray(
    const uint8 image[MT9V03X_H][MT9V03X_W], int x, int y, uint8 *sample)
{
    uint16 source_x_q8 = 0U;
    uint16 source_y_q8 = 0U;

    return IPM_SampleFullUndistortedGrayQ8(
        image, x, y, &source_x_q8, &source_y_q8, sample);
}

static uint8 VisionIdeal_PointNearSegment(
    int px, int py, int x0, int y0, int x1, int y1,
    uint32 limit_sq_q2)
{
    sint32 vx = (sint32)x1 - x0;
    sint32 vy = (sint32)y1 - y0;
    sint32 wx = (sint32)px - x0;
    sint32 wy = (sint32)py - y0;
    sint32 dot = wx * vx + wy * vy;
    uint32 length_sq = (uint32)(vx * vx + vy * vy);

    if(length_sq == 0U)
    {
        sint32 dx = (sint32)px - x0;
        sint32 dy = (sint32)py - y0;
        return (uint8)((uint64)(dx * dx + dy * dy) * 4ULL <=
                       limit_sq_q2);
    }
    if(dot <= 0)
    {
        sint32 dx = (sint32)px - x0;
        sint32 dy = (sint32)py - y0;
        return (uint8)((uint64)(dx * dx + dy * dy) * 4ULL <=
                       limit_sq_q2);
    }
    if((uint32)dot >= length_sq)
    {
        sint32 dx = (sint32)px - x1;
        sint32 dy = (sint32)py - y1;
        return (uint8)((uint64)(dx * dx + dy * dy) * 4ULL <=
                       limit_sq_q2);
    }
    {
        sint32 cross = wx * vy - wy * vx;
        uint64 cross_sq = (uint64)((int64)cross * (int64)cross);
        return (uint8)(cross_sq * 4ULL <=
                       (uint64)limit_sq_q2 * length_sq);
    }
}

static void VisionIdeal_AddPositive(uint8 value, uint8 group,
                                    uint8 binary_threshold)
{
    uint16 index = s_workspace.positive_count;

    if(index >= VISION_IDEAL_MAX_SAMPLES) return;
    s_workspace.positive_value[index] = value;
    s_workspace.positive_group[index] = group;
    s_workspace.positive_count++;
    s_workspace.positive_sum += value;
    s_workspace.positive_square_sum += (uint32)value * value;
    if(value >= binary_threshold) s_workspace.positive_fixed_hits++;
    if(group < 4U)
    {
        s_workspace.group_count[group]++;
        if(value >= binary_threshold) s_workspace.group_fixed_hits[group]++;
    }
}

static void VisionIdeal_AddNegative(uint8 value, uint8 near,
                                    uint8 binary_threshold)
{
    uint16 index = s_workspace.negative_count;

    if(index >= VISION_IDEAL_MAX_SAMPLES) return;
    s_workspace.negative_value[index] = value;
    s_workspace.negative_near[index] = near;
    s_workspace.negative_count++;
    s_workspace.negative_sum += value;
    s_workspace.negative_square_sum += (uint32)value * value;
    if(value < binary_threshold) s_workspace.negative_fixed_hits++;
    if(near)
    {
        s_workspace.near_negative_count++;
        s_workspace.near_negative_sum += value;
    }
}

static uint16 VisionIdeal_RateQ8(uint16 hits, uint16 count)
{
    if(count == 0U) return 0U;
    return (uint16)(((uint32)hits << 8) / count);
}

static uint8 VisionIdeal_RatePass(uint16 hits, uint16 count, uint16 minimum_q8)
{
    if(count == 0U) return 0U;
    return (uint8)(((uint32)hits << 8) >= (uint32)minimum_q8 * count);
}

static uint8 VisionIdeal_Finalize(uint8 target_kind, uint8 binary_threshold,
                                  VisionIdealMatchResult_t *result)
{
    uint16 positive_mean;
    uint16 negative_mean;
    uint16 local_threshold;
    uint16 positive_hits = 0U;
    uint16 negative_hits = 0U;
    uint16 minimum_group_rate = 256U;
    uint16 group_limit = (target_kind == VISION_IDEAL_TARGET_ARROW) ? 2U : 4U;
    uint16 group;
    uint16 index;
    uint32 positive_variance;
    uint32 negative_variance;
    uint32 average_variance;
    uint16 gray_difference;
    uint16 separation_q8;
    uint16 binary_consistency_q8;
    uint16 near_mean = 0U;
    uint16 near_difference = 0U;
    uint16 minimum_gray_difference;
    uint16 minimum_separation_q8;
    uint16 minimum_positive_q8;
    uint16 minimum_negative_q8;
    uint16 minimum_binary_q8;
    uint16 minimum_complete_q8;
    uint16 minimum_near_difference;
    uint8 passed = 1U;

    (void)binary_threshold;
    if(result == NULL ||
       s_workspace.positive_count < IDEAL_MIN_POSITIVE_POINTS ||
       s_workspace.negative_count < IDEAL_MIN_NEGATIVE_POINTS)
    {
        return 0U;
    }
    positive_mean = (uint16)(s_workspace.positive_sum /
                             s_workspace.positive_count);
    negative_mean = (uint16)(s_workspace.negative_sum /
                             s_workspace.negative_count);
    local_threshold = (uint16)((positive_mean + negative_mean) / 2U);
    gray_difference = (positive_mean > negative_mean) ?
        (uint16)(positive_mean - negative_mean) : 0U;

    for(index = 0U; index < s_workspace.positive_count; ++index)
    {
        if(s_workspace.positive_value[index] >= local_threshold) positive_hits++;
    }
    for(index = 0U; index < s_workspace.negative_count; ++index)
    {
        if(s_workspace.negative_value[index] < local_threshold) negative_hits++;
    }
    for(group = 0U; group < group_limit; ++group)
    {
        uint16 rate = VisionIdeal_RateQ8(s_workspace.group_fixed_hits[group],
                                         s_workspace.group_count[group]);
        if(s_workspace.group_count[group] == 0U) rate = 0U;
        if(rate < minimum_group_rate) minimum_group_rate = rate;
    }

    positive_variance = s_workspace.positive_square_sum /
        s_workspace.positive_count - (uint32)positive_mean * positive_mean;
    negative_variance = s_workspace.negative_square_sum /
        s_workspace.negative_count - (uint32)negative_mean * negative_mean;
    average_variance = (positive_variance + negative_variance) / 2U + 1U;
    {
        uint32 deviation = VisionIdeal_IntegerSqrt(average_variance);
        if(deviation < 1U) deviation = 1U;
        separation_q8 = (uint16)(((uint32)gray_difference << 8) / deviation);
    }
    binary_consistency_q8 = (uint16)(
        (VisionIdeal_RateQ8(s_workspace.positive_fixed_hits,
                            s_workspace.positive_count) +
         VisionIdeal_RateQ8(s_workspace.negative_fixed_hits,
                            s_workspace.negative_count)) / 2U);
    if(s_workspace.near_negative_count > 0U)
    {
        near_mean = (uint16)(s_workspace.near_negative_sum /
                             s_workspace.near_negative_count);
        near_difference = (positive_mean > near_mean) ?
            (uint16)(positive_mean - near_mean) : 0U;
    }

    if(target_kind == VISION_IDEAL_TARGET_ARROW)
    {
        minimum_gray_difference = IDEAL_ARROW_MIN_GRAY_DIFF;
        minimum_separation_q8 = IDEAL_ARROW_MIN_SEPARATION_Q8;
        minimum_positive_q8 = IDEAL_ARROW_MIN_POSITIVE_Q8;
        minimum_negative_q8 = IDEAL_ARROW_MIN_NEGATIVE_Q8;
        minimum_binary_q8 = IDEAL_ARROW_MIN_BINARY_Q8;
        minimum_complete_q8 = IDEAL_ARROW_MIN_COMPLETE_Q8;
        minimum_near_difference = IDEAL_ARROW_MIN_NEAR_DIFF;
    }
    else
    {
        minimum_gray_difference = IDEAL_BEACON_MIN_GRAY_DIFF;
        minimum_separation_q8 = IDEAL_BEACON_MIN_SEPARATION_Q8;
        minimum_positive_q8 = IDEAL_BEACON_MIN_POSITIVE_Q8;
        minimum_negative_q8 = IDEAL_BEACON_MIN_NEGATIVE_Q8;
        minimum_binary_q8 = IDEAL_BEACON_MIN_BINARY_Q8;
        minimum_complete_q8 = IDEAL_BEACON_MIN_COMPLETE_Q8;
        minimum_near_difference = IDEAL_BEACON_MIN_NEAR_DIFF;
    }

    if(gray_difference < minimum_gray_difference ||
       separation_q8 < minimum_separation_q8 ||
       !VisionIdeal_RatePass(positive_hits, s_workspace.positive_count,
                             minimum_positive_q8) ||
       !VisionIdeal_RatePass(negative_hits, s_workspace.negative_count,
                             minimum_negative_q8) ||
       binary_consistency_q8 < minimum_binary_q8 ||
       minimum_group_rate < minimum_complete_q8 ||
       s_workspace.near_negative_count == 0U ||
       near_difference < minimum_near_difference)
    {
        passed = 0U;
    }

    result->target_kind = target_kind;
    result->passed = passed;
    result->positive_count = s_workspace.positive_count;
    result->negative_count = s_workspace.negative_count;
    result->near_negative_count = s_workspace.near_negative_count;
    result->positive_mean = positive_mean;
    result->negative_mean = negative_mean;
    result->near_negative_mean = near_mean;
    result->gray_difference = gray_difference;
    result->standardized_separation_q8 = separation_q8;
    result->positive_hit_rate_q8 = VisionIdeal_RateQ8(
        positive_hits, s_workspace.positive_count);
    result->negative_hit_rate_q8 = VisionIdeal_RateQ8(
        negative_hits, s_workspace.negative_count);
    result->binary_consistency_q8 = binary_consistency_q8;
    result->completeness_q8 = minimum_group_rate;
    result->score_q8 = (uint8)VisionIdeal_ClampInt(
        (int)((gray_difference * 255U) / 128U), 0, 255);
    return passed;
}

void VisionIdealMatch_BeginFrame(float car_yaw_rel_uav_deg, uint8 yaw_valid)
{
    float radians;

    s_arrow_yaw_valid = yaw_valid;
    if(!yaw_valid || !isfinite(car_yaw_rel_uav_deg))
    {
        s_arrow_forward_x = 1.0f;
        s_arrow_forward_y = 0.0f;
        return;
    }
    radians = car_yaw_rel_uav_deg * (3.14159265358979323846f / 180.0f);
    s_arrow_forward_x = cosf(radians);
    s_arrow_forward_y = sinf(radians);
}

uint8 VisionIdealMatch_Arrow(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    int center_x, int center_y, uint8 binary_threshold,
    VisionIdealMatchResult_t *result)
{
    float center_world_x;
    float center_world_y;
    float left_x;
    float left_y;
    float apex_x;
    float apex_y;
    float right_x;
    float right_y;
    float side_x = -s_arrow_forward_y;
    float side_y = s_arrow_forward_x;
    int point_x[3];
    int point_y[3];
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int step;
    int x;
    int y;

    if(image == NULL || result == NULL || !s_arrow_yaw_valid) return 0U;
    memset(result, 0, sizeof(*result));
    memset(&s_workspace, 0, sizeof(s_workspace));
    if(!IPM_FullRectifiedImage2PlaneF(
           (float)center_x, (float)center_y, IDEAL_ARROW_PLANE_HEIGHT_CM,
           &center_world_x, &center_world_y))
    {
        return 0U;
    }
    apex_x = center_world_x + (2.0f * IDEAL_ARROW_TRIANGLE_HEIGHT_CM / 3.0f) *
        s_arrow_forward_x;
    apex_y = center_world_y + (2.0f * IDEAL_ARROW_TRIANGLE_HEIGHT_CM / 3.0f) *
        s_arrow_forward_y;
    left_x = center_world_x - (IDEAL_ARROW_TRIANGLE_HEIGHT_CM / 3.0f) *
        s_arrow_forward_x + IDEAL_ARROW_HALF_BASE_CM * side_x;
    left_y = center_world_y - (IDEAL_ARROW_TRIANGLE_HEIGHT_CM / 3.0f) *
        s_arrow_forward_y + IDEAL_ARROW_HALF_BASE_CM * side_y;
    right_x = center_world_x - (IDEAL_ARROW_TRIANGLE_HEIGHT_CM / 3.0f) *
        s_arrow_forward_x - IDEAL_ARROW_HALF_BASE_CM * side_x;
    right_y = center_world_y - (IDEAL_ARROW_TRIANGLE_HEIGHT_CM / 3.0f) *
        s_arrow_forward_y - IDEAL_ARROW_HALF_BASE_CM * side_y;

    if(!IPM_Plane2FullRectifiedImage(left_x, left_y,
                                     IDEAL_ARROW_PLANE_HEIGHT_CM,
                                     &point_x[0], &point_y[0]) ||
       !IPM_Plane2FullRectifiedImage(apex_x, apex_y,
                                     IDEAL_ARROW_PLANE_HEIGHT_CM,
                                     &point_x[1], &point_y[1]) ||
       !IPM_Plane2FullRectifiedImage(right_x, right_y,
                                     IDEAL_ARROW_PLANE_HEIGHT_CM,
                                     &point_x[2], &point_y[2]))
    {
        return 0U;
    }

    min_x = point_x[0];
    max_x = point_x[0];
    min_y = point_y[0];
    max_y = point_y[0];
    for(x = 1; x < 3; ++x)
    {
        if(point_x[x] < min_x) min_x = point_x[x];
        if(point_x[x] > max_x) max_x = point_x[x];
        if(point_y[x] < min_y) min_y = point_y[x];
        if(point_y[x] > max_y) max_y = point_y[x];
    }
    min_x -= IDEAL_SAMPLE_MARGIN_PX;
    max_x += IDEAL_SAMPLE_MARGIN_PX;
    min_y -= IDEAL_SAMPLE_MARGIN_PX;
    max_y += IDEAL_SAMPLE_MARGIN_PX;
    result->box_clipped = (uint8)(min_x < 0 || min_y < 0 ||
        max_x >= IDEAL_FINE_WIDTH || max_y >= IDEAL_FINE_HEIGHT);
    min_x = VisionIdeal_ClampInt(min_x, 0, IDEAL_FINE_WIDTH - 1);
    max_x = VisionIdeal_ClampInt(max_x, 0, IDEAL_FINE_WIDTH - 1);
    min_y = VisionIdeal_ClampInt(min_y, 0, IDEAL_FINE_HEIGHT - 1);
    max_y = VisionIdeal_ClampInt(max_y, 0, IDEAL_FINE_HEIGHT - 1);
    step = IDEAL_SAMPLE_STEP_MIN_PX;
    if((max_x - min_x + 16) / 16 > step) step = (max_x - min_x + 16) / 16;
    if((max_y - min_y + 16) / 16 > step) step = (max_y - min_y + 16) / 16;

    for(y = min_y; y <= max_y; y += step)
    {
        for(x = min_x; x <= max_x; x += step)
        {
            uint8 sample;
            uint8 left_positive;
            uint8 right_positive;
            uint8 within_guard;
            uint8 within_near;

            if(!VisionIdeal_SampleGray(image, x, y, &sample)) continue;
            left_positive = VisionIdeal_PointNearSegment(
                x, y, point_x[0], point_y[0], point_x[1], point_y[1],
                IDEAL_POSITIVE_BAND_SQ_Q2);
            right_positive = VisionIdeal_PointNearSegment(
                x, y, point_x[1], point_y[1], point_x[2], point_y[2],
                IDEAL_POSITIVE_BAND_SQ_Q2);
            if(left_positive || right_positive)
            {
                VisionIdeal_AddPositive(sample,
                    (uint8)((right_positive && !left_positive) ? 1U : 0U),
                    binary_threshold);
                continue;
            }
            within_guard = (uint8)(VisionIdeal_PointNearSegment(
                x, y, point_x[0], point_y[0], point_x[1], point_y[1],
                IDEAL_NEGATIVE_GUARD_SQ_Q2) ||
                VisionIdeal_PointNearSegment(
                x, y, point_x[1], point_y[1], point_x[2], point_y[2],
                IDEAL_NEGATIVE_GUARD_SQ_Q2));
            if(within_guard) continue;
            within_near = (uint8)(VisionIdeal_PointNearSegment(
                x, y, point_x[0], point_y[0], point_x[1], point_y[1],
                IDEAL_NEAR_NEGATIVE_SQ_Q2) ||
                VisionIdeal_PointNearSegment(
                x, y, point_x[1], point_y[1], point_x[2], point_y[2],
                IDEAL_NEAR_NEGATIVE_SQ_Q2));
            VisionIdeal_AddNegative(sample, within_near, binary_threshold);
        }
    }

    result->image_x[0] = (int16)point_x[0];
    result->image_y[0] = (int16)point_y[0];
    result->image_x[1] = (int16)point_x[1];
    result->image_y[1] = (int16)point_y[1];
    result->image_x[2] = (int16)point_x[2];
    result->image_y[2] = (int16)point_y[2];
    result->world_x[0] = left_x;
    result->world_y[0] = left_y;
    result->world_x[1] = apex_x;
    result->world_y[1] = apex_y;
    result->world_x[2] = right_x;
    result->world_y[2] = right_y;
    return VisionIdeal_Finalize(VISION_IDEAL_TARGET_ARROW,
                                binary_threshold, result);
}

uint8 VisionIdealMatch_Beacon(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    int center_x, int center_y, uint8 binary_threshold,
    VisionIdealMatchResult_t *result)
{
    float center_world_x;
    float center_world_y;
    int point_x[5];
    int point_y[5];
    int ex_x;
    int ex_y;
    int ey_x;
    int ey_y;
    sint32 determinant;
    uint64 determinant_sq;
    uint32 min_axis;
    uint32 guard_ratio_q8;
    uint32 near_ratio_q8;
    int extent_x;
    int extent_y;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int step;
    int x;
    int y;

    if(image == NULL || result == NULL) return 0U;
    memset(result, 0, sizeof(*result));
    memset(&s_workspace, 0, sizeof(s_workspace));
    if(!IPM_FullRectifiedImage2PlaneF(
           (float)center_x, (float)center_y, IDEAL_BEACON_PLANE_HEIGHT_CM,
           &center_world_x, &center_world_y))
    {
        return 0U;
    }
    point_x[0] = center_x;
    point_y[0] = center_y;
    if(!IPM_Plane2FullRectifiedImage(center_world_x,
            center_world_y - IDEAL_BEACON_RADIUS_CM,
            IDEAL_BEACON_PLANE_HEIGHT_CM, &point_x[1], &point_y[1]) ||
       !IPM_Plane2FullRectifiedImage(center_world_x,
            center_world_y + IDEAL_BEACON_RADIUS_CM,
            IDEAL_BEACON_PLANE_HEIGHT_CM, &point_x[2], &point_y[2]) ||
       !IPM_Plane2FullRectifiedImage(center_world_x + IDEAL_BEACON_RADIUS_CM,
            center_world_y, IDEAL_BEACON_PLANE_HEIGHT_CM,
            &point_x[3], &point_y[3]) ||
       !IPM_Plane2FullRectifiedImage(center_world_x - IDEAL_BEACON_RADIUS_CM,
            center_world_y, IDEAL_BEACON_PLANE_HEIGHT_CM,
            &point_x[4], &point_y[4]))
    {
        return 0U;
    }
    ex_x = (point_x[2] - point_x[1]) / 2;
    ex_y = (point_y[2] - point_y[1]) / 2;
    ey_x = (point_x[4] - point_x[3]) / 2;
    ey_y = (point_y[4] - point_y[3]) / 2;
    determinant = (sint32)ex_x * ey_y - (sint32)ex_y * ey_x;
    if(determinant == 0) return 0U;
    determinant_sq = (uint64)((int64)determinant * determinant);
    min_axis = VisionIdeal_ApproxLength(ex_x, ex_y);
    {
        uint32 second_axis = VisionIdeal_ApproxLength(ey_x, ey_y);
        if(second_axis < min_axis) min_axis = second_axis;
    }
    if(min_axis < 1U) return 0U;
    guard_ratio_q8 = 256U +
        (VisionIdeal_IntegerSqrt(IDEAL_NEGATIVE_GUARD_SQ_Q2) * 128U +
         min_axis - 1U) / min_axis;
    near_ratio_q8 = 256U +
        (VisionIdeal_IntegerSqrt(IDEAL_NEAR_NEGATIVE_SQ_Q2) * 128U +
         min_axis - 1U) / min_axis;
    extent_x = ((ex_x < 0) ? -ex_x : ex_x) +
               ((ey_x < 0) ? -ey_x : ey_x);
    extent_y = ((ex_y < 0) ? -ex_y : ex_y) +
               ((ey_y < 0) ? -ey_y : ey_y);
    min_x = center_x - extent_x - IDEAL_SAMPLE_MARGIN_PX;
    max_x = center_x + extent_x + IDEAL_SAMPLE_MARGIN_PX;
    min_y = center_y - extent_y - IDEAL_SAMPLE_MARGIN_PX;
    max_y = center_y + extent_y + IDEAL_SAMPLE_MARGIN_PX;
    result->box_clipped = (uint8)(min_x < 0 || min_y < 0 ||
        max_x >= IDEAL_FINE_WIDTH || max_y >= IDEAL_FINE_HEIGHT);
    min_x = VisionIdeal_ClampInt(min_x, 0, IDEAL_FINE_WIDTH - 1);
    max_x = VisionIdeal_ClampInt(max_x, 0, IDEAL_FINE_WIDTH - 1);
    min_y = VisionIdeal_ClampInt(min_y, 0, IDEAL_FINE_HEIGHT - 1);
    max_y = VisionIdeal_ClampInt(max_y, 0, IDEAL_FINE_HEIGHT - 1);
    step = IDEAL_SAMPLE_STEP_MIN_PX;
    if((max_x - min_x + 16) / 16 > step) step = (max_x - min_x + 16) / 16;
    if((max_y - min_y + 16) / 16 > step) step = (max_y - min_y + 16) / 16;

    for(y = min_y; y <= max_y; y += step)
    {
        for(x = min_x; x <= max_x; x += step)
        {
            sint32 dx = (sint32)x - center_x;
            sint32 dy = (sint32)y - center_y;
            sint32 u = dx * ey_y - dy * ey_x;
            sint32 v = ex_x * dy - ex_y * dx;
            uint64 q = (uint64)((int64)u * u) +
                       (uint64)((int64)v * v);
            uint8 sample;

            if(!VisionIdeal_SampleGray(image, x, y, &sample)) continue;
            if(q <= determinant_sq)
            {
                uint8 quadrant = (uint8)((dx >= 0 ? 1U : 0U) +
                                         (dy >= 0 ? 2U : 0U));
                VisionIdeal_AddPositive(sample, quadrant, binary_threshold);
            }
            else if(q * 65536ULL >
                    determinant_sq * guard_ratio_q8 * guard_ratio_q8)
            {
                uint8 near = (uint8)(q * 65536ULL <=
                    determinant_sq * near_ratio_q8 * near_ratio_q8);
                VisionIdeal_AddNegative(sample, near, binary_threshold);
            }
        }
    }

    for(x = 0; x < 5; ++x)
    {
        result->image_x[x] = (int16)point_x[x];
        result->image_y[x] = (int16)point_y[x];
    }
    result->world_x[0] = center_world_x;
    result->world_y[0] = center_world_y;
    return VisionIdeal_Finalize(VISION_IDEAL_TARGET_BEACON,
                                binary_threshold, result);
}
