#include "vision/vision_contour_match.h"
#include "vision/camera_calibration.h"
#include "vision/ipm_interface.h"
#include <math.h>
#include <string.h>

#if CY_CORE_CM7_1

#define CONTOUR_SEARCH_RADIUS       (32)
#define CONTOUR_SOURCE_SEED_REFINE_RADIUS (2)
#define CONTOUR_SEED_TARGET_LIMIT   (2U)
#define CONTOUR_SEED_SEARCH_DIRECTION (4) /* 固定向下寻边，方向与 Moore 初始背景方向一致 */
#define CONTOUR_SCAN_INTERSECTIONS  (64U)
#define CONTOUR_HULL_SAMPLE_POINTS  (96U)
#define CONTOUR_MIN_AREA            (6U)
#define CONTOUR_ARROW_HEIGHT_CM     (14.0f)
#define CONTOUR_ARROW_HALF_BASE_CM  (14.0f)
#define CONTOUR_ARROW_PLANE_CM      (17.0f)

#define BIT_BYTE(x, y) ((uint32)(y) * VISION_RUNTIME_FINE_STRIDE + ((uint32)(x) >> 3))
#define BIT_MASK(x) ((uint8)(0x80U >> ((uint32)(x) & 7U)))
#define BIT_GET(buffer, x, y) (((buffer)[BIT_BYTE((x), (y))] & BIT_MASK(x)) != 0U)
#define BIT_SET(buffer, x, y) ((buffer)[BIT_BYTE((x), (y))] |= BIT_MASK(x))

typedef struct { int16 x; int16 y; } ContourPoint_t;

static const int8 s_dir_x[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const int8 s_dir_y[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
static const uint8 s_atan_lut_deg[33] = {
    0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21, 22, 24, 25, 27,
    28, 29, 31, 32, 33, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45
};

static uint8 s_sampled[VISION_RUNTIME_FINE_BYTES];
static uint8 s_foreground[VISION_RUNTIME_FINE_BYTES];
static uint8 s_processed[VISION_RUNTIME_FINE_BYTES];
static int16 s_arrow_turn[VISION_RUNTIME_CONTOUR_MAX_POINTS];
static int16 s_beacon_turn[VISION_RUNTIME_CONTOUR_MAX_POINTS];
static const uint8 (*s_image)[MT9V03X_W];
static uint8 s_threshold;
static uint32 s_sample_count;
static uint32 s_trace_steps;
static uint32 s_overflow_count;
static float s_forward_x;
static float s_forward_y;

static uint8 contour_in_bounds(int x, int y)
{
    return (uint8)(x >= 0 && y >= 0 &&
                   x < (int)VISION_RUNTIME_FINE_WIDTH &&
                   y < (int)VISION_RUNTIME_FINE_HEIGHT);
}

static uint8 contour_mapping_valid(int x, int y)
{
    uint16 source_x_q8;
    uint16 source_y_q8;
    return CameraCalibration_FullRectifiedPixelToSourceQ8(
        x, y, &source_x_q8, &source_y_q8);
}

static uint8 contour_sample(int x, int y)
{
    uint16 sx_q8;
    uint16 sy_q8;
    uint32 sx;
    uint32 sy;
#if VISION_CONTOUR_SAMPLE_MODE == VISION_CONTOUR_SAMPLE_BILINEAR
    uint32 wx;
    uint32 wy;
    uint32 top;
    uint32 bottom;
#endif
    uint32 value;

    if(!contour_in_bounds(x, y) || s_image == NULL) return 0U;
    if(BIT_GET(s_sampled, x, y)) return (uint8)BIT_GET(s_foreground, x, y);
    BIT_SET(s_sampled, x, y);
    ++s_sample_count;
    if(!CameraCalibration_FullRectifiedPixelToSourceQ8(
           x, y, &sx_q8, &sy_q8))
    {
        return 0U;
    }
#if VISION_CONTOUR_SAMPLE_MODE == VISION_CONTOUR_SAMPLE_NEAREST
    sx = ((uint32)sx_q8 + 128U) >> 8;
    sy = ((uint32)sy_q8 + 128U) >> 8;
    if(sx >= MT9V03X_W || sy >= MT9V03X_H) return 0U;
    value = s_image[sy][sx];
#else
    sx = (uint32)sx_q8 >> 8;
    sy = (uint32)sy_q8 >> 8;
    if(sx + 1U >= MT9V03X_W || sy + 1U >= MT9V03X_H) return 0U;
    wx = (uint32)sx_q8 & 0xFFU;
    wy = (uint32)sy_q8 & 0xFFU;
    top = (uint32)s_image[sy][sx] * (256U - wx) +
          (uint32)s_image[sy][sx + 1U] * wx;
    bottom = (uint32)s_image[sy + 1U][sx] * (256U - wx) +
             (uint32)s_image[sy + 1U][sx + 1U] * wx;
    value = (top * (256U - wy) + bottom * wy + 32768U) >> 16;
#endif
    if(value >= s_threshold)
    {
        BIT_SET(s_foreground, x, y);
        return 1U;
    }
    return 0U;
}

/*
 * 历史种子粗寻点只做去畸变查表和最近邻读取，不写入精确二值缓存。
 * 命中后仍由 contour_sample() 双线性确认，避免粗采样污染轮廓。
 */
static uint8 contour_coarse_rectified_foreground(int x, int y)
{
    uint16 sx_q8;
    uint16 sy_q8;
    uint32 sx;
    uint32 sy;

    if(!contour_in_bounds(x, y) || s_image == NULL) return 0U;
    if(!CameraCalibration_FullRectifiedPixelToSourceQ8(
           x, y, &sx_q8, &sy_q8)) return 0U;
    sx = ((uint32)sx_q8 + 128U) >> 8;
    sy = ((uint32)sy_q8 + 128U) >> 8;
    if(sx >= MT9V03X_W || sy >= MT9V03X_H) return 0U;
    return (uint8)(s_image[sy][sx] >= s_threshold);
}

static int contour_direction(int dx, int dy)
{
    int i;
    for(i = 0; i < 8; ++i)
    {
        if((int)s_dir_x[i] == dx && (int)s_dir_y[i] == dy) return i;
    }
    return 6;
}

static uint16 contour_unsigned_angle_deg(int64 cross_abs, int64 dot)
{
    uint32 ratio;
    uint16 acute;
    uint64 a = (uint64)cross_abs;
    uint64 b = (uint64)((dot < 0) ? -dot : dot);

    if(a == 0U) return (dot < 0) ? 180U : 0U;
    if(b == 0U) return 90U;
    if(a <= b)
    {
        ratio = (uint32)((a * 32U + b / 2U) / b);
        if(ratio > 32U) ratio = 32U;
        acute = s_atan_lut_deg[ratio];
    }
    else
    {
        ratio = (uint32)((b * 32U + a / 2U) / a);
        if(ratio > 32U) ratio = 32U;
        acute = (uint16)(90U - s_atan_lut_deg[ratio]);
    }
    return (dot < 0) ? (uint16)(180U - acute) : acute;
}

static int32 contour_polygon_area2(const VisionContour_t *contour)
{
    int64 area2 = 0;
    uint16 i;
    if(contour == NULL || contour->point_count < 3U) return 0;
    for(i = 0U; i < contour->point_count; ++i)
    {
        uint16 next = (uint16)((i + 1U) % contour->point_count);
        area2 += (int64)contour->point_x[i] * contour->point_y[next] -
                 (int64)contour->point_y[i] * contour->point_x[next];
    }
    if(area2 > 2147483647LL) return 2147483647;
    if(area2 < -2147483647LL) return -2147483647;
    return (int32)area2;
}

static uint32 contour_perimeter_q8(const VisionContour_t *contour)
{
    uint32 perimeter = 0U;
    uint16 i;
    if(contour == NULL || contour->point_count < 2U) return 0U;
    for(i = 0U; i < contour->point_count; ++i)
    {
        uint16 next = (uint16)((i + 1U) % contour->point_count);
        int dx = (int)contour->point_x[next] - (int)contour->point_x[i];
        int dy = (int)contour->point_y[next] - (int)contour->point_y[i];
        if(dx != 0 && dy != 0) perimeter += 362U;
        else if(dx != 0 || dy != 0) perimeter += 256U;
    }
    return perimeter;
}

static uint8 contour_trace(int start_x, int start_y,
                           int initial_back_x, int initial_back_y,
                           VisionContourCandidate_t *candidate)
{
    int current_x = start_x;
    int current_y = start_y;
    int back_x = initial_back_x;
    int back_y = initial_back_y;
    uint32 steps;

    if(candidate == NULL || !contour_sample(start_x, start_y)) return 0U;
    memset(candidate, 0, sizeof(*candidate));
    candidate->vertex_x = VISION_CONTOUR_POINT_INVALID;
    candidate->vertex_y = VISION_CONTOUR_POINT_INVALID;
    candidate->notch_x = VISION_CONTOUR_POINT_INVALID;
    candidate->notch_y = VISION_CONTOUR_POINT_INVALID;
    candidate->endpoint_x[0] = VISION_CONTOUR_POINT_INVALID;
    candidate->endpoint_x[1] = VISION_CONTOUR_POINT_INVALID;
    candidate->endpoint_y[0] = VISION_CONTOUR_POINT_INVALID;
    candidate->endpoint_y[1] = VISION_CONTOUR_POINT_INVALID;
    for(steps = 0U; steps < (uint32)VISION_RUNTIME_CONTOUR_MAX_POINTS * 8U; ++steps)
    {
        int search_start;
        int found_direction = -1;
        int next_x = current_x;
        int next_y = current_y;
        int offset;

        if(candidate->contour.point_count >= VISION_RUNTIME_CONTOUR_MAX_POINTS)
        {
            candidate->contour_overflow = 1U;
            ++s_overflow_count;
            return 1U;
        }
        candidate->contour.point_x[candidate->contour.point_count] = (uint16)current_x;
        candidate->contour.point_y[candidate->contour.point_count] = (uint16)current_y;
        ++candidate->contour.point_count;
        ++s_trace_steps;

        search_start = contour_direction(back_x - current_x, back_y - current_y);
        for(offset = 0; offset < 8; ++offset)
        {
            int test_direction = (search_start + offset) & 7;
            int test_x = current_x + s_dir_x[test_direction];
            int test_y = current_y + s_dir_y[test_direction];
            if(contour_sample(test_x, test_y))
            {
                found_direction = test_direction;
                next_x = test_x;
                next_y = test_y;
                break;
            }
        }
        if(found_direction < 0)
        {
            candidate->contour_closed = 1U;
            break;
        }
        if(current_x != start_x || current_y != start_y)
        {
            if(next_x == start_x && next_y == start_y)
            {
                candidate->contour_closed = 1U;
                break;
            }
        }
        back_x = current_x + s_dir_x[(found_direction + 7) & 7];
        back_y = current_y + s_dir_y[(found_direction + 7) & 7];
        current_x = next_x;
        current_y = next_y;
    }
    if(!candidate->contour_closed && !candidate->contour_overflow)
    {
        candidate->contour_overflow = 1U;
        ++s_overflow_count;
    }
    return 1U;
}

static uint8 contour_touches_image_boundary(const VisionContour_t *contour)
{
    static const int8 neighbor_x[4] = {0, 1, 0, -1};
    static const int8 neighbor_y[4] = {-1, 0, 1, 0};
    uint16 i;
    uint8 direction;
    if(contour == NULL) return 1U;
    for(i = 0U; i < contour->point_count; ++i)
    {
        int x = (int)contour->point_x[i];
        int y = (int)contour->point_y[i];
        if(x <= 0 || y <= 0 ||
           x >= (int)VISION_RUNTIME_FINE_WIDTH - 1 ||
           y >= (int)VISION_RUNTIME_FINE_HEIGHT - 1) return 1U;
        for(direction = 0U; direction < 4U; ++direction)
        {
            if(!contour_mapping_valid(x + neighbor_x[direction],
                                      y + neighbor_y[direction])) return 1U;
        }
    }
    return 0U;
}

static void contour_mark_processed(const VisionContour_t *contour)
{
    int32 intersections[CONTOUR_SCAN_INTERSECTIONS];
    int min_y = (int)VISION_RUNTIME_FINE_HEIGHT - 1;
    int max_y = 0;
    uint16 i;
    int row;

    if(contour == NULL || contour->point_count == 0U) return;
    for(i = 0U; i < contour->point_count; ++i)
    {
        int x = contour->point_x[i];
        int y = contour->point_y[i];
        BIT_SET(s_processed, x, y);
        if(y < min_y) min_y = y;
        if(y > max_y) max_y = y;
    }
    if(contour->point_count < 3U) return;
    for(row = min_y; row <= max_y; ++row)
    {
        uint16 count = 0U;
        int32 scan_y2 = (int32)row * 2 + 1;
        for(i = 0U; i < contour->point_count; ++i)
        {
            uint16 next = (uint16)((i + 1U) % contour->point_count);
            int32 x0 = contour->point_x[i];
            int32 y0 = contour->point_y[i];
            int32 x1 = contour->point_x[next];
            int32 y1 = contour->point_y[next];
            int32 min_y2;
            int32 max_y2;
            int32 x_q8;
            if(y0 == y1) continue;
            min_y2 = 2 * ((y0 < y1) ? y0 : y1);
            max_y2 = 2 * ((y0 > y1) ? y0 : y1);
            if(scan_y2 < min_y2 || scan_y2 >= max_y2) continue;
            if(count >= CONTOUR_SCAN_INTERSECTIONS) break;
            x_q8 = x0 * 256 +
                ((scan_y2 - 2 * y0) * (x1 - x0) * 128) / (y1 - y0);
            intersections[count++] = x_q8;
        }
        for(i = 1U; i < count; ++i)
        {
            int32 value = intersections[i];
            uint16 j = i;
            while(j > 0U && intersections[j - 1U] > value)
            {
                intersections[j] = intersections[j - 1U];
                --j;
            }
            intersections[j] = value;
        }
        for(i = 0U; i + 1U < count; i = (uint16)(i + 2U))
        {
            int start_x = (int)((intersections[i] + 255) >> 8);
            int end_x = (int)(intersections[i + 1U] >> 8);
            int x;
            if(start_x < 0) start_x = 0;
            if(end_x >= (int)VISION_RUNTIME_FINE_WIDTH)
                end_x = (int)VISION_RUNTIME_FINE_WIDTH - 1;
            for(x = start_x; x <= end_x; ++x) BIT_SET(s_processed, x, row);
        }
    }
}

static int32 contour_hull_cross(const ContourPoint_t *a,
                                 const ContourPoint_t *b,
                                 const ContourPoint_t *c)
{
    return (int32)(b->x - a->x) * (c->y - a->y) -
           (int32)(b->y - a->y) * (c->x - a->x);
}

static uint16 contour_solidity_q8(const VisionContourCandidate_t *candidate)
{
    ContourPoint_t points[CONTOUR_HULL_SAMPLE_POINTS];
    ContourPoint_t hull[CONTOUR_HULL_SAMPLE_POINTS * 2U];
    uint16 sample_count;
    uint16 hull_count = 0U;
    uint16 lower_count;
    uint16 i;
    int64 hull_area2 = 0;
    uint32 hull_perimeter_q8 = 0U;
    uint64 hull_cell_area_q8;
    uint64 solidity;

    if(candidate == NULL || candidate->contour.point_count < 3U) return 0U;
    sample_count = candidate->contour.point_count;
    if(sample_count > CONTOUR_HULL_SAMPLE_POINTS)
        sample_count = CONTOUR_HULL_SAMPLE_POINTS;
    for(i = 0U; i < sample_count; ++i)
    {
        uint16 source = (uint16)(((uint32)i * candidate->contour.point_count) /
                                 sample_count);
        points[i].x = (int16)candidate->contour.point_x[source];
        points[i].y = (int16)candidate->contour.point_y[source];
    }
    for(i = 1U; i < sample_count; ++i)
    {
        ContourPoint_t value = points[i];
        uint16 j = i;
        while(j > 0U && (points[j - 1U].x > value.x ||
              (points[j - 1U].x == value.x && points[j - 1U].y > value.y)))
        {
            points[j] = points[j - 1U];
            --j;
        }
        points[j] = value;
    }
    for(i = 0U; i < sample_count; ++i)
    {
        while(hull_count >= 2U &&
              contour_hull_cross(&hull[hull_count - 2U],
                                 &hull[hull_count - 1U], &points[i]) <= 0)
            --hull_count;
        hull[hull_count++] = points[i];
    }
    lower_count = hull_count;
    for(i = sample_count; i > 0U; --i)
    {
        while(hull_count > lower_count &&
              contour_hull_cross(&hull[hull_count - 2U],
                                 &hull[hull_count - 1U], &points[i - 1U]) <= 0)
            --hull_count;
        hull[hull_count++] = points[i - 1U];
    }
    if(hull_count > 1U) --hull_count;
    if(hull_count < 3U) return 0U;
    for(i = 0U; i < hull_count; ++i)
    {
        uint16 next = (uint16)((i + 1U) % hull_count);
        int dx = hull[next].x - hull[i].x;
        int dy = hull[next].y - hull[i].y;
        hull_area2 += (int32)hull[i].x * hull[next].y -
                      (int32)hull[i].y * hull[next].x;
        hull_perimeter_q8 += (uint32)(sqrtf((float)(dx * dx + dy * dy)) * 256.0f + 0.5f);
    }
    if(hull_area2 < 0) hull_area2 = -hull_area2;
    hull_cell_area_q8 = (uint64)hull_area2 * 128U +
                        hull_perimeter_q8 / 2U + 256U;
    if(hull_cell_area_q8 == 0U) return 0U;
    solidity = ((uint64)candidate->blob.area * 65536U +
                hull_cell_area_q8 / 2U) / hull_cell_area_q8;
    return (uint16)((solidity > 256U) ? 256U : solidity);
}

static void contour_fill_blob(VisionContourCandidate_t *candidate)
{
    int32 area2;
    int64 centroid_x_sum = 0;
    int64 centroid_y_sum = 0;
    uint32 perimeter_q8;
    uint32 area;
    uint16 i;
    int min_x = (int)VISION_RUNTIME_FINE_WIDTH;
    int max_x = -1;
    int min_y = (int)VISION_RUNTIME_FINE_HEIGHT;
    int max_y = -1;
    uint64 circularity;

    if(candidate == NULL || candidate->contour.point_count == 0U) return;
    area2 = contour_polygon_area2(&candidate->contour);
    perimeter_q8 = contour_perimeter_q8(&candidate->contour);
    area = (uint32)((area2 < 0) ? -area2 : area2) / 2U +
           perimeter_q8 / 512U + 1U;
    for(i = 0U; i < candidate->contour.point_count; ++i)
    {
        uint16 next = (uint16)((i + 1U) % candidate->contour.point_count);
        int32 cross = (int32)candidate->contour.point_x[i] *
                       candidate->contour.point_y[next] -
                       (int32)candidate->contour.point_y[i] *
                       candidate->contour.point_x[next];
        int x = candidate->contour.point_x[i];
        int y = candidate->contour.point_y[i];
        centroid_x_sum += (int64)(x + candidate->contour.point_x[next]) * cross;
        centroid_y_sum += (int64)(y + candidate->contour.point_y[next]) * cross;
        if(x < min_x) min_x = x;
        if(x > max_x) max_x = x;
        if(y < min_y) min_y = y;
        if(y > max_y) max_y = y;
    }
    candidate->blob.area = (int)area;
    candidate->blob.min_col = min_x;
    candidate->blob.max_col = max_x;
    candidate->blob.min_row = min_y;
    candidate->blob.max_row = max_y;
    if(area2 != 0)
    {
        candidate->blob.centroid_col = (float)centroid_x_sum / (3.0f * (float)area2);
        candidate->blob.centroid_row = (float)centroid_y_sum / (3.0f * (float)area2);
    }
    else
    {
        candidate->blob.centroid_col = 0.5f * (float)(min_x + max_x);
        candidate->blob.centroid_row = 0.5f * (float)(min_y + max_y);
    }
    candidate->blob.raw_min_col = min_x / 2;
    candidate->blob.raw_max_col = max_x / 2;
    candidate->blob.raw_min_row = min_y / 2;
    candidate->blob.raw_max_row = max_y / 2;
    candidate->blob.raw_box_area_pixels =
        (candidate->blob.raw_max_col - candidate->blob.raw_min_col + 1) *
        (candidate->blob.raw_max_row - candidate->blob.raw_min_row + 1);
    candidate->blob.aspect_ratio = (float)((max_x - min_x < max_y - min_y) ?
        (max_x - min_x + 1) : (max_y - min_y + 1)) /
        (float)((max_x - min_x > max_y - min_y) ?
        (max_x - min_x + 1) : (max_y - min_y + 1));
    candidate->blob.occupancy_ratio = (float)area /
        (float)((max_x - min_x + 1) * (max_y - min_y + 1));
    if(perimeter_q8 > 0U)
    {
        circularity = (uint64)3217U * area * 65536U /
                      ((uint64)perimeter_q8 * perimeter_q8);
        if(circularity > 256U) circularity = 256U;
        candidate->circularity_q8 = (uint16)circularity;
        candidate->blob.circularity = (float)circularity / 256.0f;
    }
    candidate->solidity_q8 = contour_solidity_q8(candidate);
    candidate->contour_closed = candidate->contour_closed ? 1U : 0U;
}

static void contour_edge_vector(const VisionContour_t *contour, int edge,
                                int32 *vx, int32 *vy)
{
    int count = contour->point_count;
    int next;
    int dx;
    int dy;
    while(edge < 0) edge += count;
    while(edge >= count) edge -= count;
    next = edge + 1;
    if(next >= count) next = 0;
    dx = (int)contour->point_x[next] - (int)contour->point_x[edge];
    dy = (int)contour->point_y[next] - (int)contour->point_y[edge];
    if(dx != 0 && dy != 0)
    {
        *vx = (dx > 0) ? 181 : -181;
        *vy = (dy > 0) ? 181 : -181;
    }
    else
    {
        *vx = (dx > 0) ? 256 : ((dx < 0) ? -256 : 0);
        *vy = (dy > 0) ? 256 : ((dy < 0) ? -256 : 0);
    }
}

static uint8 contour_turns(const VisionContour_t *contour, uint8 requested_window,
                           int16 *turns)
{
    int count;
    int window;
    int32 before_x = 0;
    int32 before_y = 0;
    int32 after_x = 0;
    int32 after_y = 0;
    int32 area2;
    int orientation;
    int offset;
    int i;

    if(contour == NULL || turns == NULL || contour->point_count < 5U) return 0U;
    count = contour->point_count;
    window = requested_window;
    if(window < 2) window = 2;
    if(window > (count - 1) / 4) window = (count - 1) / 4;
    if(window < 2) return 0U;
    area2 = contour_polygon_area2(contour);
    orientation = (area2 >= 0) ? 1 : -1;
    for(offset = 0; offset < window; ++offset)
    {
        int32 vx;
        int32 vy;
        contour_edge_vector(contour, -window + offset, &vx, &vy);
        before_x += vx;
        before_y += vy;
        contour_edge_vector(contour, offset, &vx, &vy);
        after_x += vx;
        after_y += vy;
    }
    for(i = 0; i < count; ++i)
    {
        int64 dot = (int64)before_x * after_x + (int64)before_y * after_y;
        int64 cross = (int64)before_x * after_y - (int64)before_y * after_x;
        uint16 angle = contour_unsigned_angle_deg((cross < 0) ? -cross : cross, dot);
        turns[i] = (int16)(((int64)orientation * cross >= 0) ? angle : -(int16)angle);
        {
            int32 vx;
            int32 vy;
            contour_edge_vector(contour, i - window, &vx, &vy);
            before_x -= vx;
            before_y -= vy;
            contour_edge_vector(contour, i, &vx, &vy);
            before_x += vx;
            before_y += vy;
            after_x -= vx;
            after_y -= vy;
            contour_edge_vector(contour, i + window, &vx, &vy);
            after_x += vx;
            after_y += vy;
        }
    }
    return (uint8)window;
}

static uint16 contour_collect_positive_regions(
    const VisionContour_t *contour, const int16 *turns,
    uint16 minimum, uint16 maximum,
    uint16 *best_index, uint16 *second_index)
{
    uint16 count = contour->point_count;
    uint16 start = 0U;
    uint16 regions = 0U;
    uint16 best_angle = 0U;
    uint16 second_angle = 0U;
    uint16 step;

    *best_index = VISION_CONTOUR_POINT_INVALID;
    *second_index = VISION_CONTOUR_POINT_INVALID;
    while(start < count && turns[start] >= (int16)minimum &&
          turns[start] <= (int16)maximum) ++start;
    if(start == count) start = 0U;
    for(step = 1U; step <= count; )
    {
        uint16 index = (uint16)((start + step) % count);
        if(turns[index] >= (int16)minimum && turns[index] <= (int16)maximum)
        {
            uint16 peak = index;
            uint16 peak_angle = (uint16)turns[index];
            do
            {
                if((uint16)turns[index] > peak_angle)
                {
                    peak = index;
                    peak_angle = (uint16)turns[index];
                }
                ++step;
                if(step > count) break;
                index = (uint16)((start + step) % count);
            }
            while(turns[index] >= (int16)minimum && turns[index] <= (int16)maximum);
            ++regions;
            if(peak_angle > best_angle)
            {
                second_angle = best_angle;
                *second_index = *best_index;
                best_angle = peak_angle;
                *best_index = peak;
            }
            else if(peak_angle > second_angle)
            {
                second_angle = peak_angle;
                *second_index = peak;
            }
        }
        else
        {
            ++step;
        }
    }
    return regions;
}

static void contour_extract_turn_features(
    VisionContourCandidate_t *candidate,
    const VisionContourThresholds_t *arrow_thresholds,
    const VisionContourThresholds_t *beacon_thresholds)
{
    uint16 endpoint0;
    uint16 endpoint1;
    uint16 vertex0;
    uint16 ignored;
    uint16 i;
    int16 most_negative = 0;
    uint16 notch_index = VISION_CONTOUR_POINT_INVALID;

    if(candidate == NULL || arrow_thresholds == NULL || beacon_thresholds == NULL)
        return;
    if(!contour_turns(&candidate->contour,
                      arrow_thresholds->direction_window_points, s_arrow_turn))
        return;
    contour_collect_positive_regions(
        &candidate->contour, s_arrow_turn,
        arrow_thresholds->min_endpoint_turn_deg, 180U,
        &endpoint0, &endpoint1);
    if(endpoint0 != VISION_CONTOUR_POINT_INVALID)
    {
        candidate->endpoint_x[candidate->endpoint_count] = candidate->contour.point_x[endpoint0];
        candidate->endpoint_y[candidate->endpoint_count] = candidate->contour.point_y[endpoint0];
        ++candidate->endpoint_count;
    }
    if(endpoint1 != VISION_CONTOUR_POINT_INVALID && candidate->endpoint_count < 2U)
    {
        candidate->endpoint_x[candidate->endpoint_count] = candidate->contour.point_x[endpoint1];
        candidate->endpoint_y[candidate->endpoint_count] = candidate->contour.point_y[endpoint1];
        ++candidate->endpoint_count;
    }
    contour_collect_positive_regions(
        &candidate->contour, s_arrow_turn,
        arrow_thresholds->min_vertex_turn_deg,
        arrow_thresholds->max_vertex_turn_deg,
        &vertex0, &ignored);
    if(vertex0 != VISION_CONTOUR_POINT_INVALID)
    {
        candidate->vertex_x = candidate->contour.point_x[vertex0];
        candidate->vertex_y = candidate->contour.point_y[vertex0];
        candidate->vertex_turn_deg = (uint16)s_arrow_turn[vertex0];
        candidate->vertex_angle_deg = (uint16)(180U - candidate->vertex_turn_deg);
    }
    for(i = 0U; i < candidate->contour.point_count; ++i)
    {
        if(s_arrow_turn[i] < most_negative)
        {
            most_negative = s_arrow_turn[i];
            notch_index = i;
        }
    }
    if(notch_index != VISION_CONTOUR_POINT_INVALID)
    {
        candidate->notch_x = candidate->contour.point_x[notch_index];
        candidate->notch_y = candidate->contour.point_y[notch_index];
        candidate->notch_turn_deg = (uint16)(-most_negative);
    }
    if(contour_turns(&candidate->contour,
                     beacon_thresholds->direction_window_points, s_beacon_turn))
    {
        most_negative = 0;
        for(i = 0U; i < candidate->contour.point_count; ++i)
            if(s_beacon_turn[i] < most_negative) most_negative = s_beacon_turn[i];
        candidate->max_negative_turn_deg = (uint16)(-most_negative);
    }
}

static uint8 contour_base_gate(const VisionContourCandidate_t *candidate,
                               const VisionContourThresholds_t *thresholds)
{
    uint32 mask;
    if(candidate == NULL || thresholds == NULL ||
       !candidate->contour_closed || candidate->contour_overflow ||
       candidate->boundary_touched)
        return 0U;
    mask = thresholds->enabled_mask;
    if((mask & VISION_CONTOUR_GATE_MIN_POINTS) &&
       candidate->contour.point_count < thresholds->min_contour_points) return 0U;
    if((mask & VISION_CONTOUR_GATE_MIN_CIRCULARITY) &&
       candidate->circularity_q8 < thresholds->min_circularity_q8) return 0U;
    if((mask & VISION_CONTOUR_GATE_MAX_CIRCULARITY) &&
       candidate->circularity_q8 > thresholds->max_circularity_q8) return 0U;
    if((mask & VISION_CONTOUR_GATE_MIN_SOLIDITY) &&
       candidate->solidity_q8 < thresholds->min_solidity_q8) return 0U;
    if((mask & VISION_CONTOUR_GATE_MAX_SOLIDITY) &&
       candidate->solidity_q8 > thresholds->max_solidity_q8) return 0U;
    if((mask & VISION_CONTOUR_GATE_MIN_ASPECT_RATIO) &&
       candidate->blob.aspect_ratio * 256.0f <
           (float)thresholds->min_aspect_ratio_q8) return 0U;
    return 1U;
}

static uint8 contour_arrow_geometry_gate(
    const VisionContourCandidate_t *candidate,
    const VisionContourThresholds_t *thresholds)
{
    uint32 mask = thresholds->enabled_mask;
    if((mask & VISION_CONTOUR_GATE_MIN_VERTEX_ANGLE) &&
       (candidate->vertex_x == VISION_CONTOUR_POINT_INVALID ||
        candidate->vertex_angle_deg < thresholds->min_vertex_angle_deg)) return 0U;
    if((mask & VISION_CONTOUR_GATE_MAX_VERTEX_ANGLE) &&
       (candidate->vertex_x == VISION_CONTOUR_POINT_INVALID ||
        candidate->vertex_angle_deg > thresholds->max_vertex_angle_deg)) return 0U;
    if((mask & VISION_CONTOUR_GATE_MIN_ENDPOINT_TURN) &&
       candidate->endpoint_count < 2U) return 0U;
    if((mask & VISION_CONTOUR_GATE_MIN_VERTEX_TURN) &&
       (candidate->vertex_x == VISION_CONTOUR_POINT_INVALID ||
        candidate->vertex_turn_deg < thresholds->min_vertex_turn_deg)) return 0U;
    if((mask & VISION_CONTOUR_GATE_MAX_VERTEX_TURN) &&
       (candidate->vertex_x == VISION_CONTOUR_POINT_INVALID ||
        candidate->vertex_turn_deg > thresholds->max_vertex_turn_deg)) return 0U;
    if((mask & VISION_CONTOUR_GATE_MIN_NOTCH_TURN) &&
       (candidate->notch_x == VISION_CONTOUR_POINT_INVALID ||
        candidate->notch_turn_deg < thresholds->min_notch_turn_deg)) return 0U;
    if((mask & VISION_CONTOUR_GATE_MAX_NEGATIVE_TURN) &&
       candidate->max_negative_turn_deg > thresholds->max_negative_turn_deg) return 0U;
    return 1U;
}

static uint16 contour_vector_error_deg(int ax, int ay, int bx, int by)
{
    int64 dot = (int64)ax * bx + (int64)ay * by;
    int64 cross = (int64)ax * by - (int64)ay * bx;
    if((ax == 0 && ay == 0) || (bx == 0 && by == 0)) return 180U;
    return contour_unsigned_angle_deg((cross < 0) ? -cross : cross, dot);
}

static uint8 contour_yaw_signature(const VisionContourCandidate_t *candidate,
                                   uint16 tolerance_deg,
                                   uint16 *best_error)
{
    float center_world_x;
    float center_world_y;
    float side_x = -s_forward_y;
    float side_y = s_forward_x;
    float apex_x;
    float apex_y;
    float left_x;
    float left_y;
    float right_x;
    float right_y;
    int center_img_x;
    int center_img_y;
    int apex_img_x;
    int apex_img_y;
    int left_img_x;
    int left_img_y;
    int right_img_x;
    int right_img_y;
    uint16 error = 180U;

    if(candidate == NULL || best_error == NULL) return 0U;
    center_img_x = (int)(candidate->blob.centroid_col + 0.5f);
    center_img_y = (int)(candidate->blob.centroid_row + 0.5f);
    if(!IPM_FullRectifiedImage2PlaneF(
           candidate->blob.centroid_col, candidate->blob.centroid_row,
           CONTOUR_ARROW_PLANE_CM, &center_world_x, &center_world_y)) return 0U;
    apex_x = center_world_x + (2.0f * CONTOUR_ARROW_HEIGHT_CM / 3.0f) * s_forward_x;
    apex_y = center_world_y + (2.0f * CONTOUR_ARROW_HEIGHT_CM / 3.0f) * s_forward_y;
    left_x = center_world_x - (CONTOUR_ARROW_HEIGHT_CM / 3.0f) * s_forward_x +
             CONTOUR_ARROW_HALF_BASE_CM * side_x;
    left_y = center_world_y - (CONTOUR_ARROW_HEIGHT_CM / 3.0f) * s_forward_y +
             CONTOUR_ARROW_HALF_BASE_CM * side_y;
    right_x = center_world_x - (CONTOUR_ARROW_HEIGHT_CM / 3.0f) * s_forward_x -
              CONTOUR_ARROW_HALF_BASE_CM * side_x;
    right_y = center_world_y - (CONTOUR_ARROW_HEIGHT_CM / 3.0f) * s_forward_y -
              CONTOUR_ARROW_HALF_BASE_CM * side_y;
    if(!IPM_Plane2FullRectifiedImage(apex_x, apex_y, CONTOUR_ARROW_PLANE_CM,
                                     &apex_img_x, &apex_img_y) ||
       !IPM_Plane2FullRectifiedImage(left_x, left_y, CONTOUR_ARROW_PLANE_CM,
                                     &left_img_x, &left_img_y) ||
       !IPM_Plane2FullRectifiedImage(right_x, right_y, CONTOUR_ARROW_PLANE_CM,
                                     &right_img_x, &right_img_y)) return 0U;

    if(candidate->endpoint_count == 2U &&
       (candidate->vertex_x != VISION_CONTOUR_POINT_INVALID ||
        candidate->notch_x != VISION_CONTOUR_POINT_INVALID))
    {
        int mid_x = ((int)candidate->endpoint_x[0] + candidate->endpoint_x[1]) / 2;
        int mid_y = ((int)candidate->endpoint_y[0] + candidate->endpoint_y[1]) / 2;
        if(candidate->vertex_x != VISION_CONTOUR_POINT_INVALID)
            error = contour_vector_error_deg(apex_img_x - center_img_x,
                apex_img_y - center_img_y,
                (int)candidate->vertex_x - mid_x,
                (int)candidate->vertex_y - mid_y);
        if(candidate->notch_x != VISION_CONTOUR_POINT_INVALID)
        {
            uint16 other = contour_vector_error_deg(apex_img_x - center_img_x,
                apex_img_y - center_img_y,
                (int)candidate->notch_x - mid_x,
                (int)candidate->notch_y - mid_y);
            if(other < error) error = other;
        }
    }
    else if(candidate->endpoint_count == 1U)
    {
        int ex = candidate->endpoint_x[0];
        int ey = candidate->endpoint_y[0];
        if(candidate->vertex_x != VISION_CONTOUR_POINT_INVALID)
        {
            uint16 a = contour_vector_error_deg(apex_img_x - left_img_x,
                apex_img_y - left_img_y,
                (int)candidate->vertex_x - ex, (int)candidate->vertex_y - ey);
            uint16 b = contour_vector_error_deg(apex_img_x - right_img_x,
                apex_img_y - right_img_y,
                (int)candidate->vertex_x - ex, (int)candidate->vertex_y - ey);
            error = (a < b) ? a : b;
        }
        if(candidate->notch_x != VISION_CONTOUR_POINT_INVALID)
        {
            uint16 a = contour_vector_error_deg(center_img_x - left_img_x,
                center_img_y - left_img_y,
                (int)candidate->notch_x - ex, (int)candidate->notch_y - ey);
            uint16 b = contour_vector_error_deg(center_img_x - right_img_x,
                center_img_y - right_img_y,
                (int)candidate->notch_x - ex, (int)candidate->notch_y - ey);
            uint16 other = (a < b) ? a : b;
            if(other < error) error = other;
        }
    }
    else if(candidate->vertex_x != VISION_CONTOUR_POINT_INVALID &&
            candidate->notch_x != VISION_CONTOUR_POINT_INVALID)
    {
        error = contour_vector_error_deg(apex_img_x - center_img_x,
            apex_img_y - center_img_y,
            (int)candidate->vertex_x - (int)candidate->notch_x,
            (int)candidate->vertex_y - (int)candidate->notch_y);
    }
    *best_error = error;
    return (uint8)(error <= tolerance_deg);
}

static uint8 contour_centroid_is_foreground(const VisionContourCandidate_t *candidate)
{
    int x = (int)(candidate->blob.centroid_col + 0.5f);
    int y = (int)(candidate->blob.centroid_row + 0.5f);
    return contour_sample(x, y);
}

static void contour_classify(VisionContourCandidate_t *candidate,
                             const VisionContourFrameConfig_t *config,
                             uint8 arrow_already_found)
{
    uint8 arrow_base;
    uint8 arrow_geometry;
    uint8 yaw_signature;
    uint8 beacon_shape;
    uint16 yaw_error = 180U;

    arrow_base = contour_base_gate(candidate, config->arrow_thresholds);
    arrow_geometry = (uint8)(arrow_base &&
        contour_arrow_geometry_gate(candidate, config->arrow_thresholds));
    yaw_signature = (uint8)(arrow_base && contour_yaw_signature(
        candidate, config->yaw_tolerance_deg, &yaw_error));
    beacon_shape = (uint8)(config->beacon_enabled &&
        contour_base_gate(candidate, config->beacon_thresholds) &&
        (!(config->beacon_thresholds->enabled_mask &
           VISION_CONTOUR_GATE_MAX_NEGATIVE_TURN) ||
         candidate->max_negative_turn_deg <=
           config->beacon_thresholds->max_negative_turn_deg));

    candidate->yaw_error_deg = yaw_error;
    if(beacon_shape && contour_centroid_is_foreground(candidate) && !yaw_signature)
    {
        if(config->beacon_physical_gate == NULL ||
           config->beacon_physical_gate(&candidate->blob))
        {
            candidate->target = VISION_CONTOUR_TARGET_BEACON;
            return;
        }
        candidate->beacon_area_rejected = 1U;
    }
    if(!arrow_already_found && arrow_base && (arrow_geometry || yaw_signature))
    {
        candidate->target = VISION_CONTOUR_TARGET_ARROW;
        candidate->yaw_assisted = (uint8)(!arrow_geometry && yaw_signature);
    }
}

/* 任意白色种子沿固定射线寻找边界，并保存相邻黑点作为 Moore 背景点。 */
static uint8 contour_find_boundary(int seed_x, int seed_y,
                                   int *boundary_x, int *boundary_y,
                                   int *back_x, int *back_y)
{
    int x;
    int y;
    int next_x;
    int next_y;
    if(boundary_x == NULL || boundary_y == NULL ||
       back_x == NULL || back_y == NULL ||
       !contour_in_bounds(seed_x, seed_y) ||
       !contour_sample(seed_x, seed_y)) return 0U;
    x = seed_x;
    y = seed_y;
    for(;;)
    {
        next_x = x + s_dir_x[CONTOUR_SEED_SEARCH_DIRECTION];
        next_y = y + s_dir_y[CONTOUR_SEED_SEARCH_DIRECTION];
        if(!contour_in_bounds(next_x, next_y) ||
           !contour_sample(next_x, next_y))
        {
            *boundary_x = x;
            *boundary_y = y;
            *back_x = next_x;
            *back_y = next_y;
            return 1U;
        }
        x = next_x;
        y = next_y;
    }
}

static uint8 contour_try_point(int x, int y,
                               const VisionContourFrameConfig_t *config,
                               VisionContourFrameResult_t *result)
{
    VisionContourCandidate_t *candidate;
    int boundary_x;
    int boundary_y;
    int back_x;
    int back_y;

    if(!contour_in_bounds(x, y) || BIT_GET(s_processed, x, y) ||
       !contour_sample(x, y)) return 0U;
    if(!contour_find_boundary(x, y, &boundary_x, &boundary_y,
                              &back_x, &back_y) ||
       BIT_GET(s_processed, boundary_x, boundary_y)) return 0U;
    if(result->candidate_count >= VISION_RUNTIME_MAX_CANDIDATES) return 0U;
    candidate = &result->candidate[result->candidate_count];
    if(!contour_trace(boundary_x, boundary_y, back_x, back_y, candidate))
        return 0U;
    candidate->boundary_touched =
        contour_touches_image_boundary(&candidate->contour);
    contour_fill_blob(candidate);
    contour_mark_processed(&candidate->contour);
    if(candidate->blob.area < CONTOUR_MIN_AREA) return 0U;
    if(candidate->boundary_touched)
    {
        ++result->candidate_count;
        return 1U;
    }
    contour_extract_turn_features(candidate, config->arrow_thresholds,
                                  config->beacon_thresholds);
    contour_classify(candidate, config, (uint8)(result->arrow_count != 0U));
    if(candidate->target == VISION_CONTOUR_TARGET_BEACON)
    {
        if(result->beacon_count < config->beacon_limit) ++result->beacon_count;
        else candidate->target = VISION_CONTOUR_TARGET_NONE;
    }
    else if(candidate->target == VISION_CONTOUR_TARGET_ARROW)
    {
        result->arrow_count = 1U;
    }
    ++result->candidate_count;
    return 1U;
}

static uint8 contour_try_coarse_rectified_point(
    int x, int y,
    const VisionContourFrameConfig_t *config,
    VisionContourFrameResult_t *result)
{
    if(result->candidate_count >= VISION_RUNTIME_MAX_CANDIDATES ||
       (result->arrow_count != 0U &&
        result->beacon_count >= config->beacon_limit) ||
       !contour_in_bounds(x, y) || BIT_GET(s_processed, x, y) ||
       !contour_coarse_rectified_foreground(x, y)) return 0U;
    return contour_try_point(x, y, config, result);
}

/*
 * 全局回退直接在 188x120 原图找白点。只有粗白点需要映射到完整
 * 去畸变坐标，并在取整位置附近用双线性采样确认。
 */
static uint8 contour_try_source_point(
    int source_x, int source_y,
    const VisionContourFrameConfig_t *config,
    VisionContourFrameResult_t *result)
{
    float rectified_x;
    float rectified_y;
    int fine_x;
    int fine_y;
    int radius;

    if(result->candidate_count >= VISION_RUNTIME_MAX_CANDIDATES ||
       (result->arrow_count != 0U &&
        result->beacon_count >= config->beacon_limit) ||
       source_x < 0 || source_y < 0 ||
       source_x >= MT9V03X_W || source_y >= MT9V03X_H ||
       s_image[source_y][source_x] < s_threshold) return 0U;
    if(!CameraCalibration_SourcePointToRectified(
           (float)source_x, (float)source_y,
           &rectified_x, &rectified_y)) return 0U;
    fine_x = (int)(rectified_x * 2.0f + 0.5f);
    fine_y = (int)(rectified_y * 2.0f + 0.5f);

    for(radius = 0; radius <= CONTOUR_SOURCE_SEED_REFINE_RADIUS; ++radius)
    {
        int x;
        int y;
        for(x = fine_x - radius; x <= fine_x + radius; ++x)
        {
            if(contour_try_point(x, fine_y - radius, config, result)) return 1U;
            if(radius != 0 &&
               contour_try_point(x, fine_y + radius, config, result)) return 1U;
        }
        for(y = fine_y - radius + 1; y < fine_y + radius; ++y)
        {
            if(contour_try_point(fine_x - radius, y, config, result)) return 1U;
            if(radius != 0 &&
               contour_try_point(fine_x + radius, y, config, result)) return 1U;
        }
    }
    return 0U;
}

static void contour_search_around_seed(const VisionContourSeed_t *seed,
                                       const VisionContourFrameConfig_t *config,
                                       VisionContourFrameResult_t *result)
{
    uint8 first_candidate = result->candidate_count;
    int radius;
    int cx = seed->x;
    int cy = seed->y;
    for(radius = 0; radius <= CONTOUR_SEARCH_RADIUS; ++radius)
    {
        int x;
        int y;
        for(x = cx - radius; x <= cx + radius; ++x)
        {
            (void)contour_try_coarse_rectified_point(
                x, cy - radius, config, result);
            if(radius != 0)
                (void)contour_try_coarse_rectified_point(
                    x, cy + radius, config, result);
        }
        for(y = cy - radius + 1; y < cy + radius; ++y)
        {
            (void)contour_try_coarse_rectified_point(
                cx - radius, y, config, result);
            if(radius != 0)
                (void)contour_try_coarse_rectified_point(
                    cx + radius, y, config, result);
        }
        {
            uint8 index;
            uint8 target_count = 0U;
            for(index = first_candidate; index < result->candidate_count; ++index)
            {
                if(result->candidate[index].target != VISION_CONTOUR_TARGET_NONE)
                {
                    ++target_count;
                    if(result->candidate[index].target ==
                           VISION_CONTOUR_TARGET_ARROW ||
                       target_count >= CONTOUR_SEED_TARGET_LIMIT)
                        return;
                }
            }
        }
        if(result->candidate_count >= VISION_RUNTIME_MAX_CANDIDATES) return;
    }
}

static void contour_global_center_search(const VisionContourFrameConfig_t *config,
                                         VisionContourFrameResult_t *result)
{
    int cx = MT9V03X_W / 2;
    int cy = MT9V03X_H / 2;
    int right_radius = MT9V03X_W - 1 - cx;
    int bottom_radius = MT9V03X_H - 1 - cy;
    int max_x_radius = (cx > right_radius) ? cx : right_radius;
    int max_y_radius = (cy > bottom_radius) ? cy : bottom_radius;
    int max_radius = (max_x_radius > max_y_radius) ?
                     max_x_radius : max_y_radius;
    int radius;
    result->used_global_search = 1U;
    for(radius = 0; radius <= max_radius; ++radius)
    {
        int left = cx - radius;
        int right = cx + radius;
        int top = cy - radius;
        int bottom = cy + radius;
        int min_x = (left < 0) ? 0 : left;
        int max_x = (right >= MT9V03X_W) ? MT9V03X_W - 1 : right;
        int min_y = (top < 0) ? 0 : top;
        int max_y = (bottom >= MT9V03X_H) ? MT9V03X_H - 1 : bottom;
        int x;
        int y;

        /* 从原图中心向外逐圈扫描，每个原图像素只比较一次灰度。 */
        if(top >= 0)
        {
            for(x = min_x; x <= max_x; ++x)
            {
                (void)contour_try_source_point(x, top, config, result);
            }
        }
        if(bottom < MT9V03X_H && bottom != top)
        {
            for(x = min_x; x <= max_x; ++x)
            {
                (void)contour_try_source_point(x, bottom, config, result);
            }
        }

        /* 上下边的角点已扫描；超出上下边界后则覆盖整列。 */
        if(top >= 0) ++min_y;
        if(bottom < MT9V03X_H) --max_y;
        if(left >= 0)
        {
            for(y = min_y; y <= max_y; ++y)
            {
                (void)contour_try_source_point(left, y, config, result);
            }
        }
        if(right < MT9V03X_W && right != left)
        {
            for(y = min_y; y <= max_y; ++y)
            {
                (void)contour_try_source_point(right, y, config, result);
            }
        }
        if(result->candidate_count >= VISION_RUNTIME_MAX_CANDIDATES ||
           (result->arrow_count != 0U &&
            result->beacon_count >= config->beacon_limit)) return;
    }
}

void VisionContourMatch_Process(const VisionContourFrameConfig_t *config,
                                VisionContourFrameResult_t *result)
{
    uint8 i;
    float yaw_rad;
    if(config == NULL || result == NULL || config->image == NULL ||
       config->arrow_thresholds == NULL || config->beacon_thresholds == NULL)
        return;
    memset(result, 0, sizeof(*result));
    memset(s_sampled, 0, sizeof(s_sampled));
    memset(s_foreground, 0, sizeof(s_foreground));
    memset(s_processed, 0, sizeof(s_processed));
    s_image = config->image;
    s_threshold = config->threshold ? config->threshold : 1U;
    s_sample_count = 0U;
    s_trace_steps = 0U;
    s_overflow_count = 0U;
    yaw_rad = config->car_yaw_rel_uav_deg * (3.14159265358979323846f / 180.0f);
    s_forward_x = cosf(yaw_rad);
    s_forward_y = sinf(yaw_rad);

    for(i = 0U; i < config->seed_count && i < VISION_CONTOUR_MAX_SEEDS; ++i)
    {
        uint8 before = result->candidate_count;
        contour_search_around_seed(&config->seeds[i], config, result);
        if(result->candidate_count > before) ++result->seed_hits;
    }
    if(config->force_global_search || result->beacon_count < 0U)
    {
        contour_global_center_search(config, result);
    }
    result->sampled_pixels = s_sample_count;
    result->traced_steps = s_trace_steps;
    result->overflow_count = s_overflow_count;
}

const uint8 *VisionContourMatch_GetBinaryImage(void)
{
    return s_foreground;
}

#else

void VisionContourMatch_Process(const VisionContourFrameConfig_t *config,
                                VisionContourFrameResult_t *result)
{
    (void)config;
    if(result != NULL) memset(result, 0, sizeof(*result));
}

const uint8 *VisionContourMatch_GetBinaryImage(void)
{
    return NULL;
}

#endif
