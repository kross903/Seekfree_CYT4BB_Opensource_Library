#include "beacon_detect.h"
#include "common/project_headfile.h"
#include "vision/beacon_adaptive_threshold.h"
#include "vision/camera_calibration.h"
#include "vision/ipm_interface.h"
#include <math.h>

BeaconConfig_t g_beacon_config = {
    .binary_threshold = 10,
    .area_threshold   = 1,
    .min_aspect       = 0.15f,
    .max_aspect       = 3.0f,
    .min_occupancy    = 0.10f,
    .min_circularity  = 0.15f,
    .min_physical_area_cm2 = BEACON_PHYSICAL_AREA_MIN_CM2,
    .max_physical_area_cm2 = BEACON_PHYSICAL_AREA_MAX_CM2
};

static BeaconResult_t *s_beacon_frame_results = NULL;
static int s_beacon_frame_max_results = 0;
static int s_beacon_frame_count = 0;

static float BeaconDetect_Clamp01(float value)
{
    if(value < 0.0f) return 0.0f;
    if(value > 1.0f) return 1.0f;
    return value;
}

#if !VISION_RUNTIME_PHYSICAL_GRID_ENABLE
static uint8 BeaconDetect_IsEdgeRescueCandidate(
    const VisionBlob_t *blob, const BeaconAdaptiveGate_t *gate,
    float circularity)
{
    if(blob == NULL || gate == NULL) return 0U;
    if(gate->normalized_radius_sq < BEACON_EDGE_RESCUE_MIN_RADIUS_SQ ||
       gate->normalized_area_px < BEACON_EDGE_RESCUE_MIN_AREA_PX ||
       gate->normalized_area_px > gate->max_area_px ||
       circularity < BEACON_EDGE_RESCUE_MIN_CIRCULARITY ||
       blob->confidence_stability < BEACON_EDGE_RESCUE_MIN_STABILITY ||
       blob->confidence_box_size <
           blob->confidence_box_size_min * BEACON_EDGE_RESCUE_BOX_MIN_SCALE)
    {
        return 0U;
    }
    return 1U;
}
#endif

static float BeaconDetect_GetBoxSizeMinimum(const VisionBlob_t *blob)
{
    float center_u;
    float center_v;
    float radius;
    float dx;
    float dy;
    float radius_sq;
    float strictness;

    if(blob == NULL)
    {
        return BEACON_BOX_SIZE_MIN_EDGE;
    }
    center_u = 0.5f * (float)(blob->raw_min_col + blob->raw_max_col);
    center_v = 0.5f * (float)(blob->raw_min_row + blob->raw_max_row);
    /* Preserve circular distance through the longer horizontal field. */
    radius = (CAMERA_CALIBRATION_CX >
              ((float)BEACON_IMG_COLS - 1.0f - CAMERA_CALIBRATION_CX)) ?
             CAMERA_CALIBRATION_CX :
             ((float)BEACON_IMG_COLS - 1.0f - CAMERA_CALIBRATION_CX);
    dx = (center_u - CAMERA_CALIBRATION_CX) / radius;
    dy = (center_v - CAMERA_CALIBRATION_CY) / radius;
    radius_sq = BeaconDetect_Clamp01(dx * dx + dy * dy);
    strictness = 1.0f - radius_sq;
    strictness *= strictness;
    strictness = BEACON_BOX_SIZE_MIN_EDGE +
                 (BEACON_BOX_SIZE_MIN_CENTER - BEACON_BOX_SIZE_MIN_EDGE) *
                 strictness;
#if BEACON_ADAPTIVE_ENABLE
    strictness *= 1.0f - 0.25f * blob->adaptive_relaxation;
#endif
    return strictness;
}

static void BeaconDetect_UpdateConfidence(VisionBlob_t *blob)
{
    float score;
    float position_quality;
    float sigma_cm;
    float area_score;
    float box_size = 0.0f;
    float stability = 0.0f;
    float area = 0.0f;
    float edge = 0.0f;

    if(blob == NULL)
    {
        return;
    }

    box_size = BeaconDetect_Clamp01(blob->confidence_box_size);
    stability = BeaconDetect_Clamp01(blob->confidence_stability);
    edge = BeaconDetect_Clamp01(blob->confidence_edge);
    blob->confidence_box_size_min = BeaconDetect_GetBoxSizeMinimum(blob);

#if VISION_RUNTIME_PHYSICAL_GRID_ENABLE
    area = blob->physical_area_cm2;
    if(area <= BEACON_PHYSICAL_AREA_MIN_CM2 ||
       area >= BEACON_PHYSICAL_AREA_MAX_CM2)
    {
        area_score = 0.0f;
    }
    else if(area <= BEACON_PHYSICAL_AREA_IDEAL_CM2)
    {
        area_score = (area - BEACON_PHYSICAL_AREA_MIN_CM2) /
                     (BEACON_PHYSICAL_AREA_IDEAL_CM2 -
                      BEACON_PHYSICAL_AREA_MIN_CM2);
    }
    else
    {
        area_score = (BEACON_PHYSICAL_AREA_MAX_CM2 - area) /
                     (BEACON_PHYSICAL_AREA_MAX_CM2 -
                      BEACON_PHYSICAL_AREA_IDEAL_CM2);
    }
#else
    area = blob->adaptive_normalized_area_px;
    if(area <= blob->adaptive_min_area_px ||
       area >= blob->adaptive_max_area_px)
    {
        area_score = 0.0f;
    }
    else if(area <= BEACON_UNDISTORTED_AREA_IDEAL_PX)
    {
        area_score = (area - blob->adaptive_min_area_px) /
                     (BEACON_UNDISTORTED_AREA_IDEAL_PX -
                      blob->adaptive_min_area_px);
    }
    else
    {
        area_score = (blob->adaptive_max_area_px - area) /
                     (blob->adaptive_max_area_px -
                      BEACON_UNDISTORTED_AREA_IDEAL_PX);
    }
#endif

    blob->confidence_box_size = box_size;
    blob->confidence_stability = stability;
    blob->confidence_area = BeaconDetect_Clamp01(area_score);
    blob->confidence_edge = edge;
    score = BEACON_CONFIDENCE_BOX_SIZE_WEIGHT * box_size +
            BEACON_CONFIDENCE_STABILITY_WEIGHT * stability +
            BEACON_CONFIDENCE_AREA_WEIGHT * blob->confidence_area +
            BEACON_CONFIDENCE_EDGE_WEIGHT * edge;
    score = BeaconDetect_Clamp01(score);
    /* Keep a usable floor for distant lamps while preserving high-score headroom. */
    score = 64.0f / 255.0f + score * (191.0f / 255.0f);
    score = BeaconDetect_Clamp01(score);
    blob->detection_confidence = (uint8)(score * 255.0f + 0.5f);

    position_quality = 0.35f * box_size +
                       0.25f * stability +
                       0.20f * blob->confidence_area +
                       0.20f * edge;
    position_quality = BeaconDetect_Clamp01(position_quality);
    sigma_cm = (float)BEACON_POSITION_SIGMA_MIN_CM +
               (1.0f - position_quality) * (1.0f - position_quality) *
               (float)(BEACON_POSITION_SIGMA_MAX_CM -
                       BEACON_POSITION_SIGMA_MIN_CM);
    if(sigma_cm < (float)BEACON_POSITION_SIGMA_MIN_CM)
    {
        sigma_cm = (float)BEACON_POSITION_SIGMA_MIN_CM;
    }
    else if(sigma_cm > (float)BEACON_POSITION_SIGMA_MAX_CM)
    {
        sigma_cm = (float)BEACON_POSITION_SIGMA_MAX_CM;
    }
    blob->position_sigma_cm = (uint8)(sigma_cm + 0.5f);
}

static void BeaconDetect_FillResultFromBlob(const VisionBlob_t *blob, BeaconResult_t *result)
{
    float dist = 0.0f;
    float angle = 0.0f;

    if(blob == NULL || result == NULL)
    {
        return;
    }

    dist = sqrtf(blob->world_x * blob->world_x + blob->world_y * blob->world_y);
    angle = atan2f(blob->world_y, blob->world_x) * (180.0f / 3.14159265f);

    result->centroid_row = blob->centroid_row;
    result->centroid_col = blob->centroid_col;
    result->area = blob->area;
    result->min_row = blob->min_row;
    result->max_row = blob->max_row;
    result->min_col = blob->min_col;
    result->max_col = blob->max_col;
    result->raw_min_row = blob->raw_min_row;
    result->raw_max_row = blob->raw_max_row;
    result->raw_min_col = blob->raw_min_col;
    result->raw_max_col = blob->raw_max_col;
    result->average_brightness = blob->average_brightness;
    result->aspect_ratio = blob->aspect_ratio;
    result->occupancy_ratio = blob->occupancy_ratio;
    result->physical_area_cm2 = blob->physical_area_cm2;
    result->physical_perimeter_cm = blob->physical_perimeter_cm;
    result->physical_circularity = blob->physical_circularity;
    result->physical_aspect_ratio = blob->physical_aspect_ratio;
    result->physical_occupancy_ratio = blob->physical_occupancy_ratio;
    result->physical_pixel_count = blob->physical_pixel_count;
    result->physical_valid = blob->physical_valid;
    result->raw_box_area_pixels = blob->raw_box_area_pixels;
    result->detection_confidence = blob->detection_confidence;
    result->position_sigma_cm = blob->position_sigma_cm;
    result->confidence_box_size = blob->confidence_box_size;
    result->confidence_box_size_min = blob->confidence_box_size_min;
    result->confidence_stability = blob->confidence_stability;
    result->confidence_area = blob->confidence_area;
    result->confidence_edge = blob->confidence_edge;
    result->world_x = blob->world_x;
    result->world_y = blob->world_y;
    result->distance_cm = dist;
    result->angle_deg = angle;
}

void BeaconDetect_Init(void)
{
    BeaconAdaptiveThreshold_Init();
}

void BeaconDetect_BeginBlobFrame(BeaconResult_t *results, int max_results)
{
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float height_cm = BEACON_ADAPTIVE_REFERENCE_HEIGHT_CM;

    s_beacon_frame_results = results;
    s_beacon_frame_max_results = max_results;
    s_beacon_frame_count = 0;
    IPM_GetStatus(&roll_deg, &pitch_deg, &height_cm, NULL, NULL);
    BeaconAdaptiveThreshold_UpdateFrame(roll_deg, pitch_deg, height_cm);
}

uint8 BeaconDetect_AppendBlob(VisionBlob_t *blob)
{
    float aspect_ratio;
    float occupancy_ratio;
    float circularity;
#if !VISION_RUNTIME_PHYSICAL_GRID_ENABLE
    BeaconAdaptiveBaseGate_t base_gate;
    BeaconAdaptiveGate_t adaptive_gate;
    uint8 normal_area_valid;
#endif

    if(blob == NULL || s_beacon_frame_results == NULL || s_beacon_frame_max_results <= 0)
    {
        return 0U;
    }
    if(!blob->world_valid)
    {
        return 0U;
    }

    if(blob->area < g_beacon_config.area_threshold)
    {
        return 0U;
    }

    aspect_ratio = blob->aspect_ratio;
    occupancy_ratio = blob->occupancy_ratio;
    circularity = blob->circularity;
#if VISION_RUNTIME_PHYSICAL_GRID_ENABLE
    if(!blob->physical_valid) return 0U;
    if(blob->physical_area_cm2 < g_beacon_config.min_physical_area_cm2 ||
       blob->physical_area_cm2 > g_beacon_config.max_physical_area_cm2)
    {
        return 0U;
    }
    aspect_ratio = blob->physical_aspect_ratio;
    occupancy_ratio = blob->physical_occupancy_ratio;
    circularity = blob->physical_circularity;
#else
    base_gate.min_area_px = BEACON_UNDISTORTED_AREA_MIN_PX;
    base_gate.ideal_area_px = BEACON_UNDISTORTED_AREA_IDEAL_PX;
    base_gate.max_area_px = BEACON_UNDISTORTED_AREA_MAX_PX;
    base_gate.min_aspect = g_beacon_config.min_aspect;
    base_gate.max_aspect = g_beacon_config.max_aspect;
    base_gate.min_occupancy = g_beacon_config.min_occupancy;
    base_gate.min_circularity = g_beacon_config.min_circularity;
    BeaconAdaptiveThreshold_Evaluate(
        0.5f * (float)(blob->raw_min_col + blob->raw_max_col),
        0.5f * (float)(blob->raw_min_row + blob->raw_max_row),
        (float)blob->area, &base_gate, &adaptive_gate);
    blob->adaptive_normalized_area_px = adaptive_gate.normalized_area_px;
    blob->adaptive_relaxation = adaptive_gate.relaxation;
    blob->adaptive_min_area_px = adaptive_gate.min_area_px;
    blob->adaptive_max_area_px = adaptive_gate.max_area_px;
    blob->adaptive_min_circularity = adaptive_gate.min_circularity;

    normal_area_valid = (uint8)(
        adaptive_gate.normalized_area_px >= adaptive_gate.min_area_px &&
        adaptive_gate.normalized_area_px <= adaptive_gate.max_area_px);
    if(adaptive_gate.normalized_area_px > adaptive_gate.max_area_px ||
       (adaptive_gate.normalized_area_px < adaptive_gate.min_area_px &&
        (adaptive_gate.normalized_radius_sq < BEACON_EDGE_RESCUE_MIN_RADIUS_SQ ||
         adaptive_gate.normalized_area_px < BEACON_EDGE_RESCUE_MIN_AREA_PX)))
    {
        return 0U;
    }
#endif

#if VISION_RUNTIME_PHYSICAL_GRID_ENABLE
    if(aspect_ratio < g_beacon_config.min_aspect ||
       aspect_ratio > g_beacon_config.max_aspect)
    {
        return 0U;
    }
    if(occupancy_ratio < g_beacon_config.min_occupancy)
    {
        return 0U;
    }
    if(circularity < g_beacon_config.min_circularity)
    {
        return 0U;
    }
#else
    if(aspect_ratio < adaptive_gate.min_aspect ||
       aspect_ratio > adaptive_gate.max_aspect)
    {
        return 0U;
    }
    if(occupancy_ratio < adaptive_gate.min_occupancy)
    {
        return 0U;
    }
    if(circularity < adaptive_gate.min_circularity)
    {
        return 0U;
    }
#endif
    if(blob->centroid_row > (BEACON_IMG_ROWS / 2) && blob->area < 10)
    {
        return 0U;
    }

    BeaconDetect_UpdateConfidence(blob);
#if VISION_RUNTIME_PHYSICAL_GRID_ENABLE
    if(blob->confidence_box_size <= blob->confidence_box_size_min)
    {
        return 0U;
    }
#else
    if((!normal_area_valid ||
        blob->confidence_box_size <= blob->confidence_box_size_min) &&
       !BeaconDetect_IsEdgeRescueCandidate(blob, &adaptive_gate, circularity))
    {
        return 0U;
    }
#endif
    if(s_beacon_frame_count >= s_beacon_frame_max_results)
    {
        return 0U;
    }

    BeaconDetect_FillResultFromBlob(blob, &s_beacon_frame_results[s_beacon_frame_count]);
    s_beacon_frame_count++;
    return 1U;
}

int BeaconDetect_EndBlobFrame(void)
{
    return s_beacon_frame_count;
}

int BeaconDetect_GetNearestIndex(const BeaconResult_t *results, int count)
{
    int best = 0;
    int i = 0;
    float best_dist = 0.0f;

    if(results == NULL || count <= 0)
    {
        return -1;
    }

    best_dist = results[0].distance_cm;
    for(i = 1; i < count; i++)
    {
        if(results[i].distance_cm < best_dist)
        {
            best_dist = results[i].distance_cm;
            best = i;
        }
    }

    return best;
}
