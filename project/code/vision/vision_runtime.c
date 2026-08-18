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
#define VISION_ARROW_HEIGHT_CM       (14.0f)
#define VISION_ARROW_HALF_BASE_CM    (14.0f)

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
    VISION_CONTOUR_GATE_MAX_NEGATIVE_TURN)

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
    8U,
    0U,
    210U, /* 0.82 */
    0U,
    236U, /* 0.92 */
    25U,
    130U,
    120U,
    70U,
    100U,
    10U,
    180U,
    6U
};

static VisionContourThresholds_t s_beacon_thresholds = {
    BEACON_MASK,
    8U,
    90U,  /* 0.35 */
    256U,
    166U, /* 0.65 */
    256U,
    0U,
    180U,
    0U,
    0U,
    180U,
    0U,
    50U,
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

    if(blob == NULL ||
       !IPM_FullRectifiedImage2WorldF(blob->centroid_col, blob->centroid_row,
                                     &x0, &y0))
        return;
    blob->world_x = x0;
    blob->world_y = y0;
    blob->world_valid = 1U;
    if(!IPM_FullRectifiedImage2WorldF(blob->centroid_col + 1.0f,
                                     blob->centroid_row, &xx, &xy) ||
       !IPM_FullRectifiedImage2WorldF(blob->centroid_col,
                                     blob->centroid_row + 1.0f, &yx, &yy))
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
    float apex_x;
    float apex_y;
    float left_x;
    float left_y;
    float right_x;
    float right_y;
    int image_x;
    int image_y;

    if(candidate == NULL || apex == NULL || left == NULL || right == NULL) return 0U;
    if(candidate->vertex_x != VISION_CONTOUR_POINT_INVALID &&
       candidate->endpoint_count == 2U)
    {
        apex->u = candidate->vertex_x;
        apex->v = candidate->vertex_y;
        left->u = candidate->endpoint_x[0];
        left->v = candidate->endpoint_y[0];
        right->u = candidate->endpoint_x[1];
        right->v = candidate->endpoint_y[1];
        return 1U;
    }
    if(!IPM_FullRectifiedImage2PlaneF(candidate->blob.centroid_col,
            candidate->blob.centroid_row, VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM,
            &center_x, &center_y)) return 0U;
    yaw_rad = s_car_yaw_rel_uav_deg * (3.14159265358979323846f / 180.0f);
    forward_x = cosf(yaw_rad);
    forward_y = sinf(yaw_rad);
    side_x = -forward_y;
    side_y = forward_x;
    apex_x = center_x + (2.0f * VISION_ARROW_HEIGHT_CM / 3.0f) * forward_x;
    apex_y = center_y + (2.0f * VISION_ARROW_HEIGHT_CM / 3.0f) * forward_y;
    left_x = center_x - (VISION_ARROW_HEIGHT_CM / 3.0f) * forward_x +
             VISION_ARROW_HALF_BASE_CM * side_x;
    left_y = center_y - (VISION_ARROW_HEIGHT_CM / 3.0f) * forward_y +
             VISION_ARROW_HALF_BASE_CM * side_y;
    right_x = center_x - (VISION_ARROW_HEIGHT_CM / 3.0f) * forward_x -
              VISION_ARROW_HALF_BASE_CM * side_x;
    right_y = center_y - (VISION_ARROW_HEIGHT_CM / 3.0f) * forward_y -
              VISION_ARROW_HALF_BASE_CM * side_y;
    if(!IPM_Plane2FullRectifiedImage(apex_x, apex_y,
            VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM, &image_x, &image_y)) return 0U;
    apex->u = (float)image_x;
    apex->v = (float)image_y;
    if(!IPM_Plane2FullRectifiedImage(left_x, left_y,
            VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM, &image_x, &image_y)) return 0U;
    left->u = (float)image_x;
    left->v = (float)image_y;
    if(!IPM_Plane2FullRectifiedImage(right_x, right_y,
            VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM, &image_x, &image_y)) return 0U;
    right->u = (float)image_x;
    right->v = (float)image_y;
    return 1U;
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
    s_candidate_count = result->candidate_count;
    memset(s_candidate_boxes, 0, sizeof(s_candidate_boxes));
    memset(s_beacon_slots, 0, sizeof(s_beacon_slots));
    memset(s_diag.blob_area_rank, 0, sizeof(s_diag.blob_area_rank));
    s_previous_seed_count = 0U;

    for(i = 0U; i < result->candidate_count; ++i)
    {
        VisionContourCandidate_t *candidate = &result->candidate[i];
        VisionBlob_t *blob = &candidate->blob;
        vision_fill_physical_metrics(blob);
        if(candidate->target == VISION_CONTOUR_TARGET_BEACON)
            vision_fill_beacon_confidence(blob);
        vision_fill_candidate_box(i, candidate);
        vision_insert_area_rank(blob->area);
        if(candidate->target == VISION_CONTOUR_TARGET_BEACON &&
           beacon_slot < s_beacon_limit && blob->world_valid)
        {
            vision_fill_beacon_slot(beacon_slot, blob);
            ++beacon_slot;
        }
        else if(candidate->target == VISION_CONTOUR_TARGET_ARROW &&
                !arrow_committed)
        {
            VehicleMarkerPoint2D_t apex = {0};
            VehicleMarkerPoint2D_t left = {0};
            VehicleMarkerPoint2D_t right = {0};
            if(vision_build_arrow_points(candidate, &apex, &left, &right) &&
               VehicleMarkerLocalization_CommitContourArrow(
                   &apex, &left, &right,
                   (float)candidate->yaw_error_deg,
                   candidate->yaw_assisted))
            {
                arrow_committed = 1U;
            }
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
