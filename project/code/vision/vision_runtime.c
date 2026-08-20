#include "vision/vision_runtime.h"
#include "vision/vision_contour_match.h"
#include "vision/ipm_interface.h"
#include "vision/vehicle_marker_localization.h"
#include <math.h>
#include <string.h>

#if CY_CORE_CM7_1
#include "zf_device_mt9v03x.h"
#endif

#define VISION_GLOBAL_REFRESH_FRAMES (15U)
#define VISION_YAW_TOLERANCE_DEG     (30U)
/* 19 cm single arm and 70 degree opening, matching the maintained viewer. */
#define VISION_ARROW_HEIGHT_CM       (15.564f)
#define VISION_ARROW_HALF_BASE_CM    (10.898f)
#define VISION_ARROW_POINT_SEARCH_RADIUS_PX (6.0f)
#define VISION_ARROW_CENTER_REFINE_PASSES   (2U)
#define VISION_ARROW_CENTER_CONVERGENCE_CM  (0.05f)
#define VISION_ARROW_MEMBER_CORRIDOR_PX     (5.0f)
#define VISION_ARROW_MEMBER_MAX_SAMPLES     (24U)
#define VISION_ARROW_MEMBER_MIN_RATIO_Q8    (77U)
#define VISION_SPLIT_ARROW_PRIMARY_MIN_POINTS (15U)
#define VISION_SPLIT_ARROW_PARTNER_MIN_POINTS (10U)
#define VISION_SPLIT_ARROW_MIN_DISTANCE_CM    (9.5f)
#define VISION_SPLIT_ARROW_MAX_DISTANCE_CM    (30.0f)
#define VISION_SPLIT_ARROW_OPENING_DEG         (70.0f)
#define VISION_SPLIT_ARROW_OPENING_TOLERANCE_DEG (20.0f)
#define VISION_SPLIT_ARROW_MIN_AXIS_DOMINANCE  (0.45f)
#define VISION_SPLIT_ARROW_MAX_FIT_ERROR_PX    (4.0f)
#define VISION_SPLIT_ARROW_CENTER_SEARCH_PX    (4)
#define VISION_SPLIT_ARROW_MAX_SAMPLES          (32U)

#define ARROW_BASE_MASK (VISION_CONTOUR_GATE_MIN_POINTS | \
    VISION_CONTOUR_GATE_MIN_CIRCULARITY | \
    VISION_CONTOUR_GATE_MAX_CIRCULARITY | \
    VISION_CONTOUR_GATE_MIN_SOLIDITY | \
    VISION_CONTOUR_GATE_MAX_SOLIDITY)
#define ARROW_DIRECTION_MASK (VISION_CONTOUR_GATE_MIN_VERTEX_ANGLE | \
    VISION_CONTOUR_GATE_MAX_VERTEX_ANGLE | \
    VISION_CONTOUR_GATE_MIN_ENDPOINT_TURN | \
    VISION_CONTOUR_GATE_MIN_VERTEX_TURN | \
    VISION_CONTOUR_GATE_MAX_VERTEX_TURN | \
    VISION_CONTOUR_GATE_MIN_NOTCH_TURN | \
    VISION_CONTOUR_GATE_MAX_NEGATIVE_TURN)
#define BEACON_MASK (VISION_CONTOUR_GATE_MIN_POINTS | \
    VISION_CONTOUR_GATE_MIN_CIRCULARITY | \
    VISION_CONTOUR_GATE_MAX_CIRCULARITY | \
    VISION_CONTOUR_GATE_MIN_SOLIDITY | \
    VISION_CONTOUR_GATE_MAX_SOLIDITY | \
    VISION_CONTOUR_GATE_MAX_NEGATIVE_TURN | \
    VISION_CONTOUR_GATE_MIN_ASPECT_RATIO)

static VisionRuntimeDiag_t s_diag;
static VisionRuntimeBeaconSlot_t s_beacon_slots[VISION_RUNTIME_BEACON_SLOT_COUNT];
static VisionRuntimeCandidateBox_t s_candidate_boxes[VISION_RUNTIME_MAX_CANDIDATES];
#if CY_CORE_CM7_1
static VisionContourFrameResult_t s_frame_result;
#endif
static uint8 s_candidate_count;
static uint8 s_threshold = VISION_RUNTIME_FIXED_THRESHOLD;
static uint8 s_beacon_enabled = 1U;
static uint8 s_beacon_limit = VISION_RUNTIME_BEACON_ACCEPT_LIMIT;
static float s_car_yaw_rel_uav_deg = 0.0f;
static uint8 s_car_yaw_valid = 1U;
static VisionContourSeed_t s_previous_seeds[VISION_CONTOUR_MAX_SEEDS];
static uint8 s_previous_seed_count;
static uint32 s_frame_sequence;
static uint32 s_contour_overflow_count;

static VisionContourThresholds_t s_arrow_thresholds = {
    ARROW_BASE_MASK | ARROW_DIRECTION_MASK,
    20U,
    0U,
    230U, /* 0.90 */
    0U,
    236U, /* 0.92 */
    25U,
    130U,
    120U,
    70U,
    100U,
    10U,
    180U,
    0U,
    6U
};

static VisionContourThresholds_t s_beacon_thresholds = {
    BEACON_MASK,
    7U,
    179U, /* 0.70 */
    256U,
    218U, /* 0.85 */
    256U,
    0U,
    180U,
    0U,
    0U,
    180U,
    0U,
    50U,
    102U, /* 0.40 */
    12U
};

static float vision_clamp01(float value)
{
    if(value < 0.0f) return 0.0f;
    if(value > 1.0f) return 1.0f;
    return value;
}

static void vision_insert_area_rank(int area)
{
    uint8 i;
    for(i = 0U; i < VISION_RUNTIME_BLOB_AREA_COUNT; ++i)
    {
        if(area > s_diag.blob_area_rank[i])
        {
            uint8 j = VISION_RUNTIME_BLOB_AREA_COUNT - 1U;
            while(j > i)
            {
                s_diag.blob_area_rank[j] = s_diag.blob_area_rank[j - 1U];
                --j;
            }
            s_diag.blob_area_rank[i] = area;
            break;
        }
    }
}

static void vision_fill_physical_metrics(VisionBlob_t *blob)
{
    float sample_col;
    float sample_row;
    float x0;
    float y0;
    float xx;
    float xy;
    float yx;
    float yy;
    float ux;
    float uy;
    float vx;
    float vy;
    float cell_area;
    float pixel_scale;

    if(blob == NULL || blob->physical_valid) return;
    /* Use the same integer full-resolution map as the runtime sampler. */
    sample_col = floorf(blob->centroid_col + 0.5f);
    sample_row = floorf(blob->centroid_row + 0.5f);
    if(!IPM_FullRectifiedImage2WorldF(sample_col, sample_row,
                                     &x0, &y0))
        return;
    blob->world_x = x0;
    blob->world_y = y0;
    blob->world_valid = 1U;
    if(!IPM_FullRectifiedImage2WorldF(sample_col + 1.0f,
                                     sample_row, &xx, &xy) ||
       !IPM_FullRectifiedImage2WorldF(sample_col,
                                     sample_row + 1.0f, &yx, &yy))
        return;
    ux = xx - x0;
    uy = xy - y0;
    vx = yx - x0;
    vy = yy - y0;
    cell_area = fabsf(ux * vy - uy * vx);
    if(!isfinite(cell_area) || cell_area <= 0.0f) return;
    pixel_scale = sqrtf(cell_area);
    blob->physical_area_cm2 = (float)blob->area * cell_area;
    blob->physical_perimeter_cm =
        2.0f * sqrtf(3.14159265358979323846f * blob->physical_area_cm2 /
                     ((blob->circularity > 0.01f) ? blob->circularity : 0.01f));
    blob->physical_circularity = blob->circularity;
    blob->physical_aspect_ratio = blob->aspect_ratio;
    blob->physical_occupancy_ratio = blob->occupancy_ratio;
    blob->physical_pixel_count = (uint16)((blob->area > 65535) ? 65535 : blob->area);
    blob->physical_valid = (uint8)(isfinite(pixel_scale));
}

static uint8 vision_beacon_physical_gate(VisionBlob_t *blob)
{
    float center_col;
    float center_row;
    float normalized_col;
    float normalized_row;
    float radius2;
    float strict_radius2;
    float disabled_radius2;
    float relaxation;
    float min_scale;
    float max_scale;
    float min_area;
    float max_area;
    if(blob == NULL) return 0U;
    vision_fill_physical_metrics(blob);
    center_col = ((float)VISION_RUNTIME_FINE_WIDTH - 1.0f) * 0.5f;
    center_row = ((float)VISION_RUNTIME_FINE_HEIGHT - 1.0f) * 0.5f;
    normalized_col = (blob->centroid_col - center_col) / center_col;
    normalized_row = (blob->centroid_row - center_row) / center_row;
    radius2 = normalized_col * normalized_col +
              normalized_row * normalized_row;
    strict_radius2 = VISION_RUNTIME_BEACON_AREA_STRICT_RADIUS *
                     VISION_RUNTIME_BEACON_AREA_STRICT_RADIUS;
    disabled_radius2 = VISION_RUNTIME_BEACON_AREA_DISABLED_RADIUS *
                       VISION_RUNTIME_BEACON_AREA_DISABLED_RADIUS;
    /* 外围候选仍需通过其他轮廓条件；这里只是不再用不可信面积拒绝。 */
    if(radius2 >= disabled_radius2) return blob->world_valid;
    if(!blob->physical_valid) return 0U;
    if(radius2 <= strict_radius2)
    {
        relaxation = 0.0f;
    }
    else
    {
        relaxation = (radius2 - strict_radius2) /
                     (disabled_radius2 - strict_radius2);
    }
    min_scale = VISION_RUNTIME_BEACON_AREA_MIN_SCALE + relaxation *
        (VISION_RUNTIME_BEACON_AREA_EDGE_MIN_SCALE -
         VISION_RUNTIME_BEACON_AREA_MIN_SCALE);
    max_scale = VISION_RUNTIME_BEACON_AREA_MAX_SCALE + relaxation *
        (VISION_RUNTIME_BEACON_AREA_EDGE_MAX_SCALE -
         VISION_RUNTIME_BEACON_AREA_MAX_SCALE);
    min_area = VISION_RUNTIME_BEACON_NOMINAL_AREA_CM2 * min_scale;
    max_area = VISION_RUNTIME_BEACON_NOMINAL_AREA_CM2 * max_scale;
    return (uint8)(blob->physical_area_cm2 >= min_area &&
                   blob->physical_area_cm2 <= max_area);
}

static void vision_fill_beacon_confidence(VisionBlob_t *blob)
{
    float center_x;
    float center_y;
    float edge_x;
    float edge_y;
    float edge;
    float box_size;
    float shape;
    float score;

    if(blob == NULL) return;
    center_x = 0.5f * (float)(blob->min_col + blob->max_col);
    center_y = 0.5f * (float)(blob->min_row + blob->max_row);
    edge_x = 1.0f - fabsf(center_x -
        0.5f * (float)(VISION_RUNTIME_FINE_WIDTH - 1U)) /
        (0.5f * (float)VISION_RUNTIME_FINE_WIDTH);
    edge_y = 1.0f - fabsf(center_y -
        0.5f * (float)(VISION_RUNTIME_FINE_HEIGHT - 1U)) /
        (0.5f * (float)VISION_RUNTIME_FINE_HEIGHT);
    edge = vision_clamp01((edge_x < edge_y) ? edge_x : edge_y);
    box_size = vision_clamp01(sqrtf((float)blob->raw_box_area_pixels / 500.0f));
    shape = vision_clamp01(0.5f * blob->circularity +
                           0.5f * blob->occupancy_ratio);
    score = vision_clamp01(0.35f * box_size + 0.45f * shape + 0.20f * edge);
    blob->confidence_box_size = box_size;
    blob->confidence_box_size_min = 0.05f + 0.45f * edge * edge;
    blob->confidence_stability = shape;
    blob->confidence_area = shape;
    blob->confidence_edge = edge;
    blob->detection_confidence = (uint8)(64.0f + 191.0f * score + 0.5f);
    blob->position_sigma_cm = (uint8)(5.0f + 10.0f *
        (1.0f - edge) * (1.0f - edge) + 0.5f);
}

static void vision_fill_candidate_box(uint8 index,
                                      const VisionContourCandidate_t *candidate)
{
    VisionRuntimeCandidateBox_t *box;
    const VisionBlob_t *blob;
    if(index >= VISION_RUNTIME_MAX_CANDIDATES || candidate == NULL) return;
    box = &s_candidate_boxes[index];
    blob = &candidate->blob;
    memset(box, 0, sizeof(*box));
    box->raw_min_row = blob->raw_min_row;
    box->raw_max_row = blob->raw_max_row;
    box->raw_min_col = blob->raw_min_col;
    box->raw_max_col = blob->raw_max_col;
    box->fine_min_row = blob->min_row;
    box->fine_max_row = blob->max_row;
    box->fine_min_col = blob->min_col;
    box->fine_max_col = blob->max_col;
    box->area = blob->area;
    box->raw_box_area_pixels = blob->raw_box_area_pixels;
    box->occupancy_ratio = blob->occupancy_ratio;
    if(candidate->target == VISION_CONTOUR_TARGET_ARROW) box->vehicle_valid = 1U;
    if(candidate->target == VISION_CONTOUR_TARGET_BEACON)
    {
        box->beacon_valid = 1U;
        box->beacon_area_pixels = blob->area;
        box->detection_confidence = blob->detection_confidence;
        box->position_sigma_cm = blob->position_sigma_cm;
        box->beacon_x_cm = blob->world_x;
        box->beacon_y_cm = blob->world_y;
        box->beacon_area_cm2 = blob->physical_area_cm2;
        box->beacon_circularity = blob->circularity;
        box->confidence_box_size = blob->confidence_box_size;
        box->confidence_box_size_min = blob->confidence_box_size_min;
        box->confidence_stability = blob->confidence_stability;
        box->confidence_area = blob->confidence_area;
        box->confidence_edge = blob->confidence_edge;
    }
}

static void vision_fill_beacon_slot(uint8 slot, const VisionBlob_t *blob)
{
    VisionRuntimeBeaconSlot_t *output;
    if(slot >= VISION_RUNTIME_BEACON_SLOT_COUNT || blob == NULL) return;
    output = &s_beacon_slots[slot];
    memset(output, 0, sizeof(*output));
    output->valid = blob->world_valid;
    output->x_cm = blob->world_x;
    output->y_cm = blob->world_y;
    output->min_row = blob->min_row;
    output->max_row = blob->max_row;
    output->min_col = blob->min_col;
    output->max_col = blob->max_col;
    output->raw_min_row = blob->raw_min_row;
    output->raw_max_row = blob->raw_max_row;
    output->raw_min_col = blob->raw_min_col;
    output->raw_max_col = blob->raw_max_col;
    output->area_cm2 = blob->physical_area_cm2;
    output->perimeter_cm = blob->physical_perimeter_cm;
    output->circularity = blob->circularity;
    output->physical_pixel_count = blob->physical_pixel_count;
    output->physical_valid = blob->physical_valid;
    output->raw_box_area_pixels = blob->raw_box_area_pixels;
    output->detection_confidence = blob->detection_confidence;
    output->position_sigma_cm = blob->position_sigma_cm;
    output->confidence_box_size = blob->confidence_box_size;
    output->confidence_box_size_min = blob->confidence_box_size_min;
    output->confidence_stability = blob->confidence_stability;
    output->confidence_area = blob->confidence_area;
    output->confidence_edge = blob->confidence_edge;
}

static float vision_median3(float a, float b, float c)
{
    if(a > b) { float t = a; a = b; b = t; }
    if(b > c) { float t = b; b = c; c = t; }
    if(a > b) { float t = a; a = b; b = t; }
    return b;
}

typedef struct
{
    float axis_x;
    float axis_y;
    float axis_dominance;
    float world_x;
    float world_y;
    uint8 valid;
} VisionSplitArrowAxis_t;

static uint8 vision_split_arrow_axis(
    const VisionContourCandidate_t *candidate,
    VisionSplitArrowAxis_t *axis)
{
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float xx = 0.0f;
    float yy = 0.0f;
    float xy = 0.0f;
    float total;
    float angle;
    float anisotropy;
    uint16 direction_count[4] = {0U, 0U, 0U, 0U};
    uint16 direction_total = 0U;
    uint16 i;

    if(candidate == NULL || axis == NULL ||
       candidate->boundary_touched || candidate->contour_overflow ||
       !candidate->contour_closed ||
       candidate->contour.point_count < VISION_SPLIT_ARROW_PARTNER_MIN_POINTS)
        return 0U;
    memset(axis, 0, sizeof(*axis));
    for(i = 0U; i < candidate->contour.point_count; ++i)
    {
        mean_x += (float)candidate->contour.point_x[i];
        mean_y += (float)candidate->contour.point_y[i];
    }
    mean_x /= (float)candidate->contour.point_count;
    mean_y /= (float)candidate->contour.point_count;
    for(i = 0U; i < candidate->contour.point_count; ++i)
    {
        uint16 next = (uint16)((i + 1U) % candidate->contour.point_count);
        float dx = (float)candidate->contour.point_x[i] - mean_x;
        float dy = (float)candidate->contour.point_y[i] - mean_y;
        int edge_x = (int)candidate->contour.point_x[next] -
                     (int)candidate->contour.point_x[i];
        int edge_y = (int)candidate->contour.point_y[next] -
                     (int)candidate->contour.point_y[i];
        xx += dx * dx;
        yy += dy * dy;
        xy += dx * dy;
        if(edge_x == 0 && edge_y != 0) ++direction_count[0];
        else if(edge_x != 0 && edge_y != 0)
            ++direction_count[(edge_x * edge_y < 0) ? 1 : 3];
        else if(edge_x != 0) ++direction_count[2];
    }
    total = xx + yy;
    if(total <= 0.001f ||
       !IPM_FullRectifiedImage2PlaneF(
           candidate->blob.centroid_col, candidate->blob.centroid_row,
           VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
           &axis->world_x, &axis->world_y)) return 0U;
    angle = 0.5f * atan2f(2.0f * xy, xx - yy);
    axis->axis_x = cosf(angle);
    axis->axis_y = sinf(angle);
    anisotropy = sqrtf((xx - yy) * (xx - yy) + 4.0f * xy * xy) / total;
    for(i = 0U; i < 4U; ++i) direction_total += direction_count[i];
    axis->axis_dominance = anisotropy;
    if(direction_total != 0U)
    {
        uint16 maximum = direction_count[0];
        float direction_dominance;
        for(i = 1U; i < 4U; ++i)
            if(direction_count[i] > maximum) maximum = direction_count[i];
        direction_dominance = (float)maximum / (float)direction_total;
        if(direction_dominance > axis->axis_dominance)
            axis->axis_dominance = direction_dominance;
    }
    axis->valid = 1U;
    return 1U;
}

static float vision_split_axis_error_deg(float first_x, float first_y,
                                         float second_x, float second_y)
{
    float dot = first_x * second_x + first_y * second_y;
    float cross = first_x * second_y - first_y * second_x;
    return atan2f(fabsf(cross), fabsf(dot)) *
           (180.0f / 3.14159265358979323846f);
}

static float vision_split_point_segment_distance(
    float point_x, float point_y,
    const VehicleMarkerPoint2D_t *start,
    const VehicleMarkerPoint2D_t *end)
{
    float dx = end->u - start->u;
    float dy = end->v - start->v;
    float px = point_x - start->u;
    float py = point_y - start->v;
    float length2 = dx * dx + dy * dy;
    float projection;
    if(length2 <= 0.001f) return sqrtf(px * px + py * py);
    projection = (px * dx + py * dy) / length2;
    if(projection < 0.0f) projection = 0.0f;
    else if(projection > 1.0f) projection = 1.0f;
    px -= projection * dx;
    py -= projection * dy;
    return sqrtf(px * px + py * py);
}

static float vision_split_contour_arm_error(
    const VisionContourCandidate_t *candidate,
    const VehicleMarkerPoint2D_t *start,
    const VehicleMarkerPoint2D_t *end,
    uint16 *sample_count)
{
    uint16 step;
    uint16 i;
    float total = 0.0f;
    if(candidate == NULL || start == NULL || end == NULL ||
       sample_count == NULL || candidate->contour.point_count == 0U)
        return 0.0f;
    step = (uint16)(candidate->contour.point_count /
                    VISION_SPLIT_ARROW_MAX_SAMPLES);
    if(step == 0U) step = 1U;
    for(i = 0U; i < candidate->contour.point_count; i = (uint16)(i + step))
    {
        total += vision_split_point_segment_distance(
            (float)candidate->contour.point_x[i],
            (float)candidate->contour.point_y[i], start, end);
        ++(*sample_count);
    }
    return total;
}

static uint8 vision_find_nearest_contour_point(
    const VisionContourCandidate_t *candidate,
    float target_x, float target_y,
    float *point_x, float *point_y)
{
    float best_distance2 = VISION_ARROW_POINT_SEARCH_RADIUS_PX *
                           VISION_ARROW_POINT_SEARCH_RADIUS_PX + 1.0f;
    uint16 i;
    uint8 found = 0U;

    if(candidate == NULL || point_x == NULL || point_y == NULL) return 0U;
    /* A single contour pass is equivalent to expanding search rings, but avoids
       repeatedly scanning the same pixels and cannot attach to another blob. */
    for(i = 0U; i < candidate->contour.point_count; ++i)
    {
        float dx = (float)candidate->contour.point_x[i] - target_x;
        float dy = (float)candidate->contour.point_y[i] - target_y;
        float distance2 = dx * dx + dy * dy;
        if(distance2 <= VISION_ARROW_POINT_SEARCH_RADIUS_PX *
                        VISION_ARROW_POINT_SEARCH_RADIUS_PX &&
           (!found || distance2 < best_distance2))
        {
            best_distance2 = distance2;
            *point_x = (float)candidate->contour.point_x[i];
            *point_y = (float)candidate->contour.point_y[i];
            found = 1U;
        }
    }
    return found;
}

static uint8 vision_project_arrow_points(
    float center_x, float center_y,
    const float offset_x[3], const float offset_y[3],
    VehicleMarkerPoint2D_t points[3])
{
    uint8 i;
    for(i = 0U; i < 3U; ++i)
    {
        int image_x;
        int image_y;
        if(!IPM_Plane2FullRectifiedImage(
               center_x + offset_x[i], center_y + offset_y[i],
               VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
               &image_x, &image_y)) return 0U;
        points[i].u = (float)image_x;
        points[i].v = (float)image_y;
        points[i].intensity = 0.0f;
    }
    return 1U;
}

static uint8 vision_fit_split_arrow_pair(
    const VisionContourCandidate_t *first,
    const VisionContourCandidate_t *second,
    const VisionSplitArrowAxis_t *first_axis,
    const VisionSplitArrowAxis_t *second_axis,
    VehicleMarkerPoint2D_t output[3],
    float *fit_score,
    float *yaw_error_deg)
{
    float total_area;
    float base_center_x;
    float base_center_y;
    float yaw_rad;
    float forward_x;
    float forward_y;
    float side_x;
    float side_y;
    float offset_x[3];
    float offset_y[3];
    float best_score = 1.0e30f;
    float best_yaw_error = 180.0f;
    int offset_col;
    int offset_row;
    uint8 found = 0U;

    if(first == NULL || second == NULL || first_axis == NULL ||
       second_axis == NULL || output == NULL || fit_score == NULL ||
       yaw_error_deg == NULL) return 0U;
    total_area = (float)(first->blob.area + second->blob.area);
    if(total_area <= 0.0f) return 0U;
    base_center_x = (first->blob.centroid_col * (float)first->blob.area +
                     second->blob.centroid_col * (float)second->blob.area) /
                    total_area;
    base_center_y = (first->blob.centroid_row * (float)first->blob.area +
                     second->blob.centroid_row * (float)second->blob.area) /
                    total_area;
    yaw_rad = s_car_yaw_rel_uav_deg *
              (3.14159265358979323846f / 180.0f);
    forward_x = cosf(yaw_rad);
    forward_y = sinf(yaw_rad);
    side_x = -forward_y;
    side_y = forward_x;
    offset_x[0] = (2.0f * VISION_ARROW_HEIGHT_CM / 3.0f) * forward_x;
    offset_y[0] = (2.0f * VISION_ARROW_HEIGHT_CM / 3.0f) * forward_y;
    offset_x[1] = -(VISION_ARROW_HEIGHT_CM / 3.0f) * forward_x +
                  VISION_ARROW_HALF_BASE_CM * side_x;
    offset_y[1] = -(VISION_ARROW_HEIGHT_CM / 3.0f) * forward_y +
                  VISION_ARROW_HALF_BASE_CM * side_y;
    offset_x[2] = -(VISION_ARROW_HEIGHT_CM / 3.0f) * forward_x -
                  VISION_ARROW_HALF_BASE_CM * side_x;
    offset_y[2] = -(VISION_ARROW_HEIGHT_CM / 3.0f) * forward_y -
                  VISION_ARROW_HALF_BASE_CM * side_y;

    for(offset_row = -VISION_SPLIT_ARROW_CENTER_SEARCH_PX;
        offset_row <= VISION_SPLIT_ARROW_CENTER_SEARCH_PX; ++offset_row)
    {
        for(offset_col = -VISION_SPLIT_ARROW_CENTER_SEARCH_PX;
            offset_col <= VISION_SPLIT_ARROW_CENTER_SEARCH_PX; ++offset_col)
        {
            float center_x;
            float center_y;
            VehicleMarkerPoint2D_t projected[3];
            float left_dx;
            float left_dy;
            float right_dx;
            float right_dy;
            uint8 assignment;
            if(!IPM_FullRectifiedImage2PlaneF(
                   base_center_x + (float)offset_col,
                   base_center_y + (float)offset_row,
                   VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
                   &center_x, &center_y) ||
               !vision_project_arrow_points(center_x, center_y,
                                             offset_x, offset_y, projected))
                continue;
            left_dx = projected[1].u - projected[0].u;
            left_dy = projected[1].v - projected[0].v;
            right_dx = projected[2].u - projected[0].u;
            right_dy = projected[2].v - projected[0].v;
            for(assignment = 0U; assignment < 2U; ++assignment)
            {
                float first_error = vision_split_axis_error_deg(
                    first_axis->axis_x, first_axis->axis_y,
                    assignment ? right_dx : left_dx,
                    assignment ? right_dy : left_dy);
                float second_error = vision_split_axis_error_deg(
                    second_axis->axis_x, second_axis->axis_y,
                    assignment ? left_dx : right_dx,
                    assignment ? left_dy : right_dy);
                float error_sum;
                float fit_error;
                float score;
                uint16 samples = 0U;
                if(first_error > (float)VISION_YAW_TOLERANCE_DEG ||
                   second_error > (float)VISION_YAW_TOLERANCE_DEG) continue;
                error_sum = vision_split_contour_arm_error(
                    first, &projected[0],
                    assignment ? &projected[2] : &projected[1], &samples);
                error_sum += vision_split_contour_arm_error(
                    second, &projected[0],
                    assignment ? &projected[1] : &projected[2], &samples);
                if(samples == 0U) continue;
                fit_error = error_sum / (float)samples;
                if(fit_error > VISION_SPLIT_ARROW_MAX_FIT_ERROR_PX) continue;
                score = fit_error + 0.02f * (first_error + second_error);
                if(!found || score < best_score)
                {
                    best_score = score;
                    best_yaw_error = (first_error > second_error) ?
                                     first_error : second_error;
                    output[0] = projected[0];
                    output[1] = projected[1];
                    output[2] = projected[2];
                    found = 1U;
                }
            }
        }
    }
    if(!found) return 0U;
    *fit_score = best_score;
    *yaw_error_deg = best_yaw_error;
    return 1U;
}

static uint8 vision_try_split_arrow(VisionContourFrameResult_t *result,
                                    VehicleMarkerPoint2D_t output[3],
                                    uint8 *primary_index)
{
    VisionSplitArrowAxis_t axes[VISION_RUNTIME_MAX_CANDIDATES];
    VehicleMarkerPoint2D_t best_points[3];
    float best_score = 1.0e30f;
    float best_yaw_error = 180.0f;
    uint8 best_first = 0U;
    uint8 best_second = 0U;
    uint8 first;
    uint8 found = 0U;

    if(result == NULL || output == NULL || primary_index == NULL) return 0U;
    for(first = 0U; first < result->candidate_count; ++first)
    {
        if(result->candidate[first].target == VISION_CONTOUR_TARGET_ARROW &&
           !result->candidate[first].yaw_assisted)
            return 0U;
    }
    memset(axes, 0, sizeof(axes));
    for(first = 0U; first < result->candidate_count; ++first)
        (void)vision_split_arrow_axis(&result->candidate[first], &axes[first]);
    for(first = 0U; first < result->candidate_count; ++first)
    {
        uint8 second;
        if(!axes[first].valid ||
           axes[first].axis_dominance < VISION_SPLIT_ARROW_MIN_AXIS_DOMINANCE)
            continue;
        for(second = (uint8)(first + 1U);
            second < result->candidate_count; ++second)
        {
            float opening_error;
            float dx;
            float dy;
            float distance;
            float score;
            float yaw_error;
            VehicleMarkerPoint2D_t points[3];
            if(!axes[second].valid ||
               axes[second].axis_dominance <
                   VISION_SPLIT_ARROW_MIN_AXIS_DOMINANCE ||
               (result->candidate[first].contour.point_count <
                    VISION_SPLIT_ARROW_PRIMARY_MIN_POINTS &&
                result->candidate[second].contour.point_count <
                    VISION_SPLIT_ARROW_PRIMARY_MIN_POINTS)) continue;
            opening_error = fabsf(vision_split_axis_error_deg(
                axes[first].axis_x, axes[first].axis_y,
                axes[second].axis_x, axes[second].axis_y) -
                VISION_SPLIT_ARROW_OPENING_DEG);
            if(opening_error > VISION_SPLIT_ARROW_OPENING_TOLERANCE_DEG)
                continue;
            dx = axes[first].world_x - axes[second].world_x;
            dy = axes[first].world_y - axes[second].world_y;
            distance = sqrtf(dx * dx + dy * dy);
            if(distance < VISION_SPLIT_ARROW_MIN_DISTANCE_CM ||
               distance > VISION_SPLIT_ARROW_MAX_DISTANCE_CM) continue;
            if(!vision_fit_split_arrow_pair(
                   &result->candidate[first], &result->candidate[second],
                   &axes[first], &axes[second], points, &score, &yaw_error))
                continue;
            if(!found || score < best_score)
            {
                best_score = score;
                best_yaw_error = yaw_error;
                best_first = first;
                best_second = second;
                best_points[0] = points[0];
                best_points[1] = points[1];
                best_points[2] = points[2];
                found = 1U;
            }
        }
    }
    if(found)
    {
        uint8 primary;
        uint8 partner;
        for(first = 0U; first < result->candidate_count; ++first)
        {
            if(result->candidate[first].target == VISION_CONTOUR_TARGET_ARROW &&
               result->candidate[first].yaw_assisted)
            {
                result->candidate[first].target = VISION_CONTOUR_TARGET_NONE;
                result->candidate[first].yaw_assisted = 0U;
            }
        }
        primary =
            (result->candidate[best_first].contour.point_count >=
             result->candidate[best_second].contour.point_count) ?
             best_first : best_second;
        partner = (primary == best_first) ? best_second : best_first;
        if(result->candidate[primary].target == VISION_CONTOUR_TARGET_BEACON &&
           result->beacon_count != 0U) --result->beacon_count;
        if(result->candidate[partner].target == VISION_CONTOUR_TARGET_BEACON &&
           result->beacon_count != 0U) --result->beacon_count;
        result->candidate[primary].target = VISION_CONTOUR_TARGET_ARROW;
        result->candidate[primary].yaw_assisted = 1U;
        result->candidate[primary].yaw_error_deg =
            (uint16)(best_yaw_error + 0.5f);
        result->candidate[partner].target = VISION_CONTOUR_TARGET_NONE;
        result->candidate[partner].yaw_assisted = 0U;
        result->arrow_count = 1U;
        output[0] = best_points[0];
        output[1] = best_points[1];
        output[2] = best_points[2];
        *primary_index = primary;
    }
    return found;
}

static uint8 vision_build_arrow_points(const VisionContourCandidate_t *candidate,
                                       VehicleMarkerPoint2D_t *apex,
                                       VehicleMarkerPoint2D_t *left,
                                       VehicleMarkerPoint2D_t *right)
{
    float center_x;
    float center_y;
    float yaw_rad;
    float forward_x;
    float forward_y;
    float side_x;
    float side_y;
    float offset_x[3];
    float offset_y[3];
    VehicleMarkerPoint2D_t projected[3];
    uint8 pass;

    if(candidate == NULL || apex == NULL || left == NULL || right == NULL) return 0U;
    if(!IPM_FullRectifiedImage2PlaneF(candidate->blob.centroid_col,
            candidate->blob.centroid_row, VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
            &center_x, &center_y)) return 0U;
    yaw_rad = s_car_yaw_rel_uav_deg * (3.14159265358979323846f / 180.0f);
    forward_x = cosf(yaw_rad);
    forward_y = sinf(yaw_rad);
    side_x = -forward_y;
    side_y = forward_x;

    /* Point order is apex, left endpoint, right endpoint. */
    offset_x[0] = (2.0f * VISION_ARROW_HEIGHT_CM / 3.0f) * forward_x;
    offset_y[0] = (2.0f * VISION_ARROW_HEIGHT_CM / 3.0f) * forward_y;
    offset_x[1] = -(VISION_ARROW_HEIGHT_CM / 3.0f) * forward_x +
                  VISION_ARROW_HALF_BASE_CM * side_x;
    offset_y[1] = -(VISION_ARROW_HEIGHT_CM / 3.0f) * forward_y +
                  VISION_ARROW_HALF_BASE_CM * side_y;
    offset_x[2] = -(VISION_ARROW_HEIGHT_CM / 3.0f) * forward_x -
                  VISION_ARROW_HALF_BASE_CM * side_x;
    offset_y[2] = -(VISION_ARROW_HEIGHT_CM / 3.0f) * forward_y -
                  VISION_ARROW_HALF_BASE_CM * side_y;

    /* The centroid of two uniformly bright arms is height/6 toward the apex
       from the triangle centroid used by the theoretical three-point model. */
    center_x -= (VISION_ARROW_HEIGHT_CM / 6.0f) * forward_x;
    center_y -= (VISION_ARROW_HEIGHT_CM / 6.0f) * forward_y;

    for(pass = 0U; pass < VISION_ARROW_CENTER_REFINE_PASSES; ++pass)
    {
        float estimate_x[3];
        float estimate_y[3];
        uint8 estimate_count = 0U;
        uint8 i;
        float refined_x;
        float refined_y;
        float movement_x;
        float movement_y;

        if(!vision_project_arrow_points(center_x, center_y,
                                        offset_x, offset_y, projected)) return 0U;
        for(i = 0U; i < 3U; ++i)
        {
            float contour_x;
            float contour_y;
            float point_world_x;
            float point_world_y;
            if(!vision_find_nearest_contour_point(candidate,
                    projected[i].u, projected[i].v,
                    &contour_x, &contour_y)) continue;
            if(!IPM_FullRectifiedImage2PlaneF(
                    contour_x, contour_y,
                    VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
                    &point_world_x, &point_world_y)) continue;
            estimate_x[estimate_count] = point_world_x - offset_x[i];
            estimate_y[estimate_count] = point_world_y - offset_y[i];
            ++estimate_count;
        }
        if(estimate_count == 0U) break;
        if(estimate_count == 1U)
        {
            refined_x = estimate_x[0];
            refined_y = estimate_y[0];
        }
        else if(estimate_count == 2U)
        {
            refined_x = 0.5f * (estimate_x[0] + estimate_x[1]);
            refined_y = 0.5f * (estimate_y[0] + estimate_y[1]);
        }
        else
        {
            refined_x = vision_median3(estimate_x[0], estimate_x[1], estimate_x[2]);
            refined_y = vision_median3(estimate_y[0], estimate_y[1], estimate_y[2]);
        }
        movement_x = refined_x - center_x;
        movement_y = refined_y - center_y;
        center_x = refined_x;
        center_y = refined_y;
        if(movement_x * movement_x + movement_y * movement_y <=
           VISION_ARROW_CENTER_CONVERGENCE_CM *
           VISION_ARROW_CENTER_CONVERGENCE_CM) break;
    }

    if(!vision_project_arrow_points(center_x, center_y,
                                    offset_x, offset_y, projected)) return 0U;
    *apex = projected[0];
    *left = projected[1];
    *right = projected[2];
    return 1U;
}

static uint8 vision_point_near_arrow_arm(float point_x, float point_y,
                                         const VehicleMarkerPoint2D_t *start,
                                         const VehicleMarkerPoint2D_t *end)
{
    float dx;
    float dy;
    float px;
    float py;
    float length2;
    float projection;
    float distance2;
    float corridor2 = VISION_ARROW_MEMBER_CORRIDOR_PX *
                      VISION_ARROW_MEMBER_CORRIDOR_PX;
    if(start == NULL || end == NULL) return 0U;
    dx = end->u - start->u;
    dy = end->v - start->v;
    px = point_x - start->u;
    py = point_y - start->v;
    length2 = dx * dx + dy * dy;
    if(length2 <= 0.001f) return (uint8)(px * px + py * py <= corridor2);
    projection = px * dx + py * dy;
    if(projection <= 0.0f)
    {
        distance2 = px * px + py * py;
    }
    else if(projection >= length2)
    {
        px = point_x - end->u;
        py = point_y - end->v;
        distance2 = px * px + py * py;
    }
    else
    {
        float cross = px * dy - py * dx;
        return (uint8)(cross * cross <= corridor2 * length2);
    }
    return (uint8)(distance2 <= corridor2);
}

static uint8 vision_candidate_follows_arrow_arms(
    const VisionContourCandidate_t *candidate,
    const VehicleMarkerPoint2D_t *apex,
    const VehicleMarkerPoint2D_t *left,
    const VehicleMarkerPoint2D_t *right)
{
    uint16 step;
    uint16 i;
    uint16 sampled = 0U;
    uint16 hits = 0U;
    if(candidate == NULL || apex == NULL || left == NULL || right == NULL)
        return 0U;
    if(vision_point_near_arrow_arm(candidate->blob.centroid_col,
           candidate->blob.centroid_row, apex, left) ||
       vision_point_near_arrow_arm(candidate->blob.centroid_col,
           candidate->blob.centroid_row, apex, right)) return 1U;
    if(candidate->contour.point_count == 0U) return 0U;
    step = (uint16)((candidate->contour.point_count +
                     VISION_ARROW_MEMBER_MAX_SAMPLES - 1U) /
                    VISION_ARROW_MEMBER_MAX_SAMPLES);
    if(step == 0U) step = 1U;
    for(i = 0U; i < candidate->contour.point_count; i = (uint16)(i + step))
    {
        float point_x = (float)candidate->contour.point_x[i];
        float point_y = (float)candidate->contour.point_y[i];
        ++sampled;
        if(vision_point_near_arrow_arm(point_x, point_y, apex, left) ||
           vision_point_near_arrow_arm(point_x, point_y, apex, right)) ++hits;
    }
    return (uint8)(sampled != 0U &&
        (uint32)hits * 256U >=
        (uint32)sampled * VISION_ARROW_MEMBER_MIN_RATIO_Q8);
}

static void vision_update_nearest_beacon(uint8 beacon_count)
{
    uint8 i;
    uint8 nearest = 0U;
    float nearest_distance = 0.0f;
    s_diag.beacon_has_nearest = 0U;
    if(beacon_count == 0U) return;
    for(i = 0U; i < beacon_count; ++i)
    {
        float distance = sqrtf(s_beacon_slots[i].x_cm * s_beacon_slots[i].x_cm +
                               s_beacon_slots[i].y_cm * s_beacon_slots[i].y_cm);
        if(i == 0U || distance < nearest_distance)
        {
            nearest = i;
            nearest_distance = distance;
        }
    }
    s_diag.beacon_has_nearest = 1U;
    s_diag.beacon_nearest_x_cm = s_beacon_slots[nearest].x_cm;
    s_diag.beacon_nearest_y_cm = s_beacon_slots[nearest].y_cm;
    s_diag.beacon_nearest_distance_cm = nearest_distance;
    s_diag.beacon_nearest_area = (float)s_beacon_slots[nearest].physical_pixel_count;
    s_diag.beacon_nearest_area_cm2 = s_beacon_slots[nearest].area_cm2;
    s_diag.beacon_nearest_perimeter_cm = s_beacon_slots[nearest].perimeter_cm;
    s_diag.beacon_nearest_circularity = s_beacon_slots[nearest].circularity;
    s_diag.beacon_nearest_confidence = s_beacon_slots[nearest].detection_confidence;
}

static void vision_process_result(VisionContourFrameResult_t *result)
{
    uint8 i;
    uint8 beacon_slot = 0U;
    uint8 arrow_committed = 0U;
    uint8 arrow_geometry_valid = 0U;
    uint8 split_arrow_valid;
    uint8 split_arrow_primary = 0U;
    VehicleMarkerPoint2D_t arrow_apex = {0};
    VehicleMarkerPoint2D_t arrow_left = {0};
    VehicleMarkerPoint2D_t arrow_right = {0};
    VehicleMarkerPoint2D_t split_arrow_points[3] = {{0}};
    s_candidate_count = result->candidate_count;
    memset(s_candidate_boxes, 0, sizeof(s_candidate_boxes));
    memset(s_beacon_slots, 0, sizeof(s_beacon_slots));
    memset(s_diag.blob_area_rank, 0, sizeof(s_diag.blob_area_rank));
    s_previous_seed_count = 0U;
    split_arrow_valid = vision_try_split_arrow(
        result, split_arrow_points, &split_arrow_primary);

    for(i = 0U; i < result->candidate_count; ++i)
    {
        VisionContourCandidate_t *candidate = &result->candidate[i];
        VisionBlob_t *blob = &candidate->blob;
        vision_fill_physical_metrics(blob);
        if(candidate->target == VISION_CONTOUR_TARGET_BEACON)
            vision_fill_beacon_confidence(blob);
        vision_insert_area_rank(blob->area);
        if(candidate->target == VISION_CONTOUR_TARGET_ARROW &&
           !arrow_geometry_valid)
        {
            uint8 points_valid;
            if(split_arrow_valid && i == split_arrow_primary)
            {
                arrow_apex = split_arrow_points[0];
                arrow_left = split_arrow_points[1];
                arrow_right = split_arrow_points[2];
                points_valid = 1U;
            }
            else
            {
                points_valid = vision_build_arrow_points(
                    candidate, &arrow_apex, &arrow_left, &arrow_right);
            }
            if(points_valid)
            {
                arrow_geometry_valid = 1U;
                if(VehicleMarkerLocalization_CommitContourArrow(
                   &arrow_apex, &arrow_left, &arrow_right,
                   (float)candidate->yaw_error_deg,
                   candidate->yaw_assisted))
                    arrow_committed = 1U;
            }
        }
    }

    if(arrow_geometry_valid)
    {
        for(i = 0U; i < result->candidate_count; ++i)
        {
            VisionContourCandidate_t *candidate = &result->candidate[i];
            if(candidate->target == VISION_CONTOUR_TARGET_BEACON &&
               vision_candidate_follows_arrow_arms(
                   candidate, &arrow_apex, &arrow_left, &arrow_right))
                candidate->target = VISION_CONTOUR_TARGET_NONE;
        }
    }

    result->beacon_count = 0U;
    for(i = 0U; i < result->candidate_count; ++i)
    {
        VisionContourCandidate_t *candidate = &result->candidate[i];
        VisionBlob_t *blob = &candidate->blob;
        vision_fill_candidate_box(i, candidate);
        if(candidate->target == VISION_CONTOUR_TARGET_BEACON &&
           beacon_slot < s_beacon_limit && blob->world_valid)
        {
            vision_fill_beacon_slot(beacon_slot, blob);
            ++beacon_slot;
            ++result->beacon_count;
        }
        if(candidate->target != VISION_CONTOUR_TARGET_NONE &&
           s_previous_seed_count < VISION_CONTOUR_MAX_SEEDS)
        {
            s_previous_seeds[s_previous_seed_count].x =
                (uint16)(blob->centroid_col + 0.5f);
            s_previous_seeds[s_previous_seed_count].y =
                (uint16)(blob->centroid_row + 0.5f);
            ++s_previous_seed_count;
        }
    }
    s_diag.blob_count = result->candidate_count;
    s_diag.ir_split_count = result->candidate_count;
    s_diag.beacon_split_count = result->candidate_count;
    s_diag.ir_blob_count = arrow_committed ? 1 : 0;
    s_diag.beacon_blob_count = beacon_slot;
    s_diag.beacon_count = beacon_slot;
    s_diag.ir_raw_detect_valid = arrow_committed;
    s_diag.ir_track_valid = (uint8)(g_vehicle_marker_pose.valid &&
                                    !g_vehicle_marker_position_lost_flag);
    s_diag.ir_detected_point_count = g_vehicle_marker_pose.detected_point_count;
    s_diag.ir_x_cm = g_vehicle_marker_pose.x;
    s_diag.ir_y_cm = g_vehicle_marker_pose.y;
    s_diag.ir_square_side_cm = g_vehicle_marker_pose.square_side_cm;
    s_diag.ir_height_cm = g_vehicle_marker_pose.z;
    s_contour_overflow_count = result->overflow_count;
    vision_update_nearest_beacon(beacon_slot);
}

static void vision_process_frame_internal(float roll_deg, float pitch_deg,
                                          float height_cm,
                                          uint8 explicit_pose)
{
#if CY_CORE_CM7_1
    VisionContourFrameConfig_t config;
    uint8 i;

    VehicleMarkerLocalization_ServiceLostProtection();
    if(!mt9v03x_finish_flag) return;
    mt9v03x_finish_flag = 0U;
    if(explicit_pose) IPM_Interface_Update(roll_deg, pitch_deg, height_cm);
    else IPM_Interface_UpdateFromIMU(height_cm);

    VehicleMarkerLocalization_SetOpeningDirectionPrior(
        s_car_yaw_rel_uav_deg, s_car_yaw_valid);
    VehicleMarkerLocalization_BeginFrame();
    memset(&config, 0, sizeof(config));
    config.image = mt9v03x_image;
    config.threshold = s_threshold;
    config.beacon_enabled = s_beacon_enabled;
    config.beacon_limit = s_beacon_limit;
    config.car_yaw_rel_uav_deg = s_car_yaw_rel_uav_deg;
    config.yaw_valid = s_car_yaw_valid;
    config.yaw_tolerance_deg = VISION_YAW_TOLERANCE_DEG;
    config.arrow_thresholds = &s_arrow_thresholds;
    config.beacon_thresholds = &s_beacon_thresholds;
    config.beacon_physical_gate = vision_beacon_physical_gate;
    config.seed_count = s_previous_seed_count;
    for(i = 0U; i < config.seed_count; ++i) config.seeds[i] = s_previous_seeds[i];
    config.force_global_search = (uint8)(
        s_previous_seed_count == 0U ||
        (s_frame_sequence % VISION_GLOBAL_REFRESH_FRAMES) == 0U);
    VisionContourMatch_Process(&config, &s_frame_result);
    vision_process_result(&s_frame_result);
    VehicleMarkerLocalization_EndFrame();
    ++s_frame_sequence;
    ++s_diag.recognition_frame_seq;
#else
    (void)roll_deg;
    (void)pitch_deg;
    (void)height_cm;
    (void)explicit_pose;
#endif
}

void VisionRuntime_ProcessFrame(float height_cm)
{
    vision_process_frame_internal(0.0f, 0.0f, height_cm, 0U);
}

void VisionRuntime_ProcessFrameWithPose(float roll_deg, float pitch_deg,
                                        float height_cm)
{
    vision_process_frame_internal(roll_deg, pitch_deg, height_cm, 1U);
}

void VisionRuntime_SetBeaconRecognitionEnabled(uint8 enabled)
{
    s_beacon_enabled = enabled ? 1U : 0U;
    if(!s_beacon_enabled) memset(s_beacon_slots, 0, sizeof(s_beacon_slots));
}

void VisionRuntime_SetVehicleYawPrior(float yaw_deg, uint16 age_ms, uint8 valid)
{
    if(valid && age_ms <= VISION_RUNTIME_YAW_MAX_AGE_MS && isfinite(yaw_deg))
    {
        s_car_yaw_rel_uav_deg = yaw_deg;
        s_car_yaw_valid = 1U;
    }
    else
    {
        /* Required fallback: invalid vehicle yaw is recognized as 0 degrees. */
        s_car_yaw_rel_uav_deg = 0.0f;
        s_car_yaw_valid = 1U;
    }
}

void VisionRuntime_SetBinaryThreshold(uint8 threshold)
{
    s_threshold = threshold ? threshold : 1U;
}

void VisionRuntime_SetBinaryThresholds(uint8 inner, uint8 middle, uint8 outer)
{
    (void)inner;
    (void)outer;
    VisionRuntime_SetBinaryThreshold(middle);
}

void VisionRuntime_GetBinaryThresholds(uint8 thresholds[VISION_BINARY_ZONE_COUNT])
{
    if(thresholds == NULL) return;
    thresholds[0] = s_threshold;
    thresholds[1] = s_threshold;
    thresholds[2] = s_threshold;
}

void VisionRuntime_SetRecognitionThresholds(uint8 vehicle, uint8 beacon)
{
    (void)vehicle;
    VisionRuntime_SetBinaryThreshold(beacon);
}

void VisionRuntime_GetRecognitionThresholds(uint8 *vehicle, uint8 *beacon)
{
    if(vehicle != NULL) *vehicle = s_threshold;
    if(beacon != NULL) *beacon = s_threshold;
}

void VisionRuntime_SetCoarseThreshold(uint8 threshold)
{
    VisionRuntime_SetBinaryThreshold(threshold);
}

uint8 VisionRuntime_GetCoarseThreshold(void) { return s_threshold; }

void VisionRuntime_SetCoarseThresholdRange(uint8 center_threshold,
                                           uint8 edge_threshold)
{
    (void)edge_threshold;
    VisionRuntime_SetBinaryThreshold(center_threshold);
}

void VisionRuntime_GetCoarseThresholdRange(uint8 *center_threshold,
                                           uint8 *edge_threshold)
{
    if(center_threshold != NULL) *center_threshold = s_threshold;
    if(edge_threshold != NULL) *edge_threshold = s_threshold;
}

void VisionRuntime_SetCoarseZoneThresholds(
    const uint8 thresholds[VISION_RUNTIME_COARSE_ZONE_COUNT])
{
    if(thresholds != NULL) VisionRuntime_SetBinaryThreshold(thresholds[2]);
}

void VisionRuntime_GetCoarseZoneThresholds(
    uint8 thresholds[VISION_RUNTIME_COARSE_ZONE_COUNT])
{
    uint8 i;
    if(thresholds == NULL) return;
    for(i = 0U; i < VISION_RUNTIME_COARSE_ZONE_COUNT; ++i)
        thresholds[i] = s_threshold;
}

void VisionRuntime_SetContourThresholds(uint8 arrow,
                                        const VisionContourThresholds_t *thresholds)
{
    if(thresholds == NULL || thresholds->direction_window_points < 2U ||
       thresholds->direction_window_points > 32U) return;
    if(arrow) s_arrow_thresholds = *thresholds;
    else s_beacon_thresholds = *thresholds;
}

void VisionRuntime_GetContourThresholds(uint8 arrow,
                                        VisionContourThresholds_t *thresholds)
{
    if(thresholds == NULL) return;
    *thresholds = arrow ? s_arrow_thresholds : s_beacon_thresholds;
}

void VisionRuntime_SetBeaconAcceptLimit(uint8 limit)
{
    if(limit < 1U) limit = 1U;
    if(limit > VISION_RUNTIME_BEACON_SLOT_COUNT)
        limit = VISION_RUNTIME_BEACON_SLOT_COUNT;
    s_beacon_limit = limit;
}

uint8 VisionRuntime_GetBeaconAcceptLimit(void) { return s_beacon_limit; }

void VisionRuntime_GetDiag(VisionRuntimeDiag_t *diag)
{
    if(diag != NULL) *diag = s_diag;
}

void VisionRuntime_SetDiag(const VisionRuntimeDiag_t *diag)
{
    if(diag != NULL) s_diag = *diag;
}

void VisionRuntime_GetBeaconSlots(VisionRuntimeBeaconSlot_t *slots,
                                  uint8 max_slots)
{
    uint8 count = (max_slots < VISION_RUNTIME_BEACON_SLOT_COUNT) ?
                  max_slots : VISION_RUNTIME_BEACON_SLOT_COUNT;
    if(slots != NULL) memcpy(slots, s_beacon_slots,
                             (size_t)count * sizeof(*slots));
}

void VisionRuntime_SetBeaconSlots(const VisionRuntimeBeaconSlot_t *slots,
                                  uint8 max_slots)
{
    uint8 count = (max_slots < VISION_RUNTIME_BEACON_SLOT_COUNT) ?
                  max_slots : VISION_RUNTIME_BEACON_SLOT_COUNT;
    memset(s_beacon_slots, 0, sizeof(s_beacon_slots));
    if(slots != NULL) memcpy(s_beacon_slots, slots,
                             (size_t)count * sizeof(*slots));
}

uint8 VisionRuntime_GetCandidateBoxes(VisionRuntimeCandidateBox_t *boxes,
                                      uint8 max_boxes)
{
    uint8 count = (max_boxes < s_candidate_count) ? max_boxes : s_candidate_count;
    if(boxes != NULL) memcpy(boxes, s_candidate_boxes,
                             (size_t)count * sizeof(*boxes));
    return count;
}

const uint8 *VisionRuntime_GetBinaryImage(void)
{
    return VisionContourMatch_GetBinaryImage();
}

const uint8 *VisionRuntime_GetBinaryDisplayImage(void)
{
    return VisionContourMatch_GetBinaryImage();
}

const uint8 *VisionRuntime_GetUndistortedImage(void) { return NULL; }
uint32 VisionRuntime_GetFineRleOverflowCount(void) { return s_contour_overflow_count; }

#if defined(VISION_RUNTIME_PC_DIAGNOSTICS)
void VisionRuntime_SetVehicleFallbackEnabled(uint8 enabled) { (void)enabled; }
void VisionRuntime_SetVehicleFallbackThreshold(uint8 threshold) { (void)threshold; }
uint8 VisionRuntime_GetVehicleFallbackThreshold(void) { return s_threshold; }
const uint8 *VisionRuntime_GetVehicleBinaryImage(void)
{
    return VisionContourMatch_GetBinaryImage();
}
#endif
