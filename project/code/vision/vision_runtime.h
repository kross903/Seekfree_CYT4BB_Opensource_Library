#ifndef VISION_RUNTIME_H_
#define VISION_RUNTIME_H_

#include "zf_common_headfile.h"

/* Single-threshold, on-demand rectified contour pipeline. */
#define VISION_RUNTIME_FIXED_THRESHOLD        (120U)
#define VISION_BINARY_THRESHOLD               VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_BINARY_THRESHOLD_VEHICLE       VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_BINARY_THRESHOLD_BEACON        VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_BINARY_THRESHOLD_VEHICLE_FALLBACK VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_BINARY_THRESHOLD_INNER         VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_BINARY_THRESHOLD_MIDDLE        VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_BINARY_THRESHOLD_OUTER         VISION_RUNTIME_FIXED_THRESHOLD

/* Compatibility constants for the existing cross-core parameter page. */
#define VISION_BINARY_ZONE_INNER               (0U)
#define VISION_BINARY_ZONE_MIDDLE              (1U)
#define VISION_BINARY_ZONE_OUTER               (2U)
#define VISION_BINARY_ZONE_COUNT               (3U)
#define VISION_RUNTIME_COARSE_ZONE_COUNT       (5U)
#define VISION_RUNTIME_COARSE_THRESHOLD_CENTER VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_RUNTIME_COARSE_THRESHOLD_NEAR   VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_RUNTIME_COARSE_THRESHOLD_MIDDLE VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_RUNTIME_COARSE_THRESHOLD_FAR    VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_RUNTIME_COARSE_THRESHOLD_EDGE   VISION_RUNTIME_FIXED_THRESHOLD
#define VISION_RUNTIME_COARSE_THRESHOLD        VISION_RUNTIME_FIXED_THRESHOLD

#define VISION_RUNTIME_FINE_WIDTH              (MT9V03X_W * 2U)
#define VISION_RUNTIME_FINE_HEIGHT             (MT9V03X_H * 2U)
#define VISION_RUNTIME_FINE_STRIDE             ((VISION_RUNTIME_FINE_WIDTH + 7U) / 8U)
#define VISION_RUNTIME_FINE_BYTES              (VISION_RUNTIME_FINE_STRIDE * VISION_RUNTIME_FINE_HEIGHT)
#define VISION_RUNTIME_BINARY_STRIDE           VISION_RUNTIME_FINE_STRIDE
#define VISION_RUNTIME_BINARY_BYTES            VISION_RUNTIME_FINE_BYTES
#define VISION_RUNTIME_CONTOUR_MAX_POINTS      (640U)
#define VISION_RUNTIME_MAX_CANDIDATES          (8U)
#define VISION_RUNTIME_BEACON_ACCEPT_LIMIT     (6U)
#define VISION_RUNTIME_BEACON_SLOT_COUNT       VISION_RUNTIME_BEACON_ACCEPT_LIMIT
#define VISION_RUNTIME_BLOB_AREA_COUNT         (16U)
#define VISION_RUNTIME_YAW_MAX_AGE_MS          (500U)

/* Runtime-selectable contour gates. Ratios use unsigned Q8 (256 == 1.0). */
#define VISION_CONTOUR_GATE_MIN_POINTS         (1UL << 0)
#define VISION_CONTOUR_GATE_MIN_CIRCULARITY    (1UL << 1)
#define VISION_CONTOUR_GATE_MAX_CIRCULARITY    (1UL << 2)
#define VISION_CONTOUR_GATE_MIN_SOLIDITY       (1UL << 3)
#define VISION_CONTOUR_GATE_MAX_SOLIDITY       (1UL << 4)
#define VISION_CONTOUR_GATE_MIN_VERTEX_ANGLE   (1UL << 5)
#define VISION_CONTOUR_GATE_MAX_VERTEX_ANGLE   (1UL << 6)
#define VISION_CONTOUR_GATE_MIN_ENDPOINT_TURN  (1UL << 7)
#define VISION_CONTOUR_GATE_MIN_VERTEX_TURN    (1UL << 8)
#define VISION_CONTOUR_GATE_MAX_VERTEX_TURN    (1UL << 9)
#define VISION_CONTOUR_GATE_MIN_NOTCH_TURN     (1UL << 10)
#define VISION_CONTOUR_GATE_MAX_NEGATIVE_TURN  (1UL << 11)

typedef struct
{
    uint32 enabled_mask;
    uint16 min_contour_points;
    uint16 min_circularity_q8;
    uint16 max_circularity_q8;
    uint16 min_solidity_q8;
    uint16 max_solidity_q8;
    uint16 min_vertex_angle_deg;
    uint16 max_vertex_angle_deg;
    uint16 min_endpoint_turn_deg;
    uint16 min_vertex_turn_deg;
    uint16 max_vertex_turn_deg;
    uint16 min_notch_turn_deg;
    uint16 max_negative_turn_deg;
    uint8 direction_window_points;
} VisionContourThresholds_t;

typedef struct
{
    float centroid_row;
    float centroid_col;
    int area;
    int min_row;
    int max_row;
    int min_col;
    int max_col;
    float average_brightness;
    float aspect_ratio;
    float occupancy_ratio;
    float circularity;
    float physical_area_cm2;
    float physical_perimeter_cm;
    float physical_circularity;
    float physical_aspect_ratio;
    float physical_occupancy_ratio;
    uint16 physical_pixel_count;
    uint8 physical_valid;
    int raw_min_row;
    int raw_max_row;
    int raw_min_col;
    int raw_max_col;
    int raw_box_area_pixels;
    float confidence_box_size;
    float confidence_box_size_min;
    float confidence_stability;
    float confidence_area;
    float confidence_edge;
    float adaptive_normalized_area_px;
    float adaptive_relaxation;
    float adaptive_min_area_px;
    float adaptive_max_area_px;
    float adaptive_min_circularity;
    uint8 detection_confidence;
    uint8 position_sigma_cm;
    float world_x;
    float world_y;
    uint8 world_valid;
} VisionBlob_t;

typedef struct
{
    uint16 point_count;
    uint16 point_x[VISION_RUNTIME_CONTOUR_MAX_POINTS];
    uint16 point_y[VISION_RUNTIME_CONTOUR_MAX_POINTS];
} VisionContour_t;

typedef struct
{
    int blob_count;
    int ir_split_count;
    int beacon_split_count;
    int ir_blob_count;
    int beacon_blob_count;
    int blob_area_rank[VISION_RUNTIME_BLOB_AREA_COUNT];
    int ir_reject_area_low_count;
    int ir_reject_area_high_count;
    int ir_reject_size_count;
    int ir_reject_aspect_count;
    int ir_reject_insert_count;
    uint8 ir_raw_detect_valid;
    uint8 ir_track_valid;
    uint8 ir_detected_point_count;
    float ir_x_cm;
    float ir_y_cm;
    float ir_square_side_cm;
    float ir_height_cm;
    int beacon_count;
    uint8 beacon_has_nearest;
    float beacon_nearest_x_cm;
    float beacon_nearest_y_cm;
    float beacon_nearest_distance_cm;
    float beacon_nearest_area;
    float beacon_nearest_area_cm2;
    float beacon_nearest_perimeter_cm;
    float beacon_nearest_circularity;
    float beacon_nearest_brightness;
    float beacon_nearest_occupancy;
    float beacon_nearest_aspect;
    uint8 beacon_nearest_confidence;
    float recognition_time_us;
    uint32 recognition_frame_seq;
} VisionRuntimeDiag_t;

typedef struct
{
    uint8 valid;
    float x_cm;
    float y_cm;
    int min_row;
    int max_row;
    int min_col;
    int max_col;
    int raw_min_row;
    int raw_max_row;
    int raw_min_col;
    int raw_max_col;
    float area_cm2;
    float perimeter_cm;
    float circularity;
    uint16 physical_pixel_count;
    uint8 physical_valid;
    int raw_box_area_pixels;
    uint8 detection_confidence;
    uint8 position_sigma_cm;
    float confidence_box_size;
    float confidence_box_size_min;
    float confidence_stability;
    float confidence_area;
    float confidence_edge;
} VisionRuntimeBeaconSlot_t;

typedef struct
{
    int raw_min_row;
    int raw_max_row;
    int raw_min_col;
    int raw_max_col;
    int fine_min_row;
    int fine_max_row;
    int fine_min_col;
    int fine_max_col;
    int area;
    int raw_box_area_pixels;
    int beacon_area_pixels;
    uint8 vehicle_valid;
    uint8 vehicle_skipped_small_box;
    uint8 fine_roi_oversize;
    uint8 beacon_valid;
    uint8 detection_confidence;
    uint8 position_sigma_cm;
    float beacon_x_cm;
    float beacon_y_cm;
    float beacon_area_cm2;
    float beacon_circularity;
    float confidence_box_size;
    float confidence_box_size_min;
    float confidence_stability;
    float confidence_area;
    float confidence_edge;
    float adaptive_normalized_area_px;
    float adaptive_relaxation;
    float adaptive_min_area_px;
    float adaptive_max_area_px;
    float adaptive_min_circularity;
    float occupancy_ratio;
    float horizontal_run_ratio;
} VisionRuntimeCandidateBox_t;

void VisionRuntime_ProcessFrame(float height_cm);
void VisionRuntime_ProcessFrameWithPose(float roll_deg, float pitch_deg, float height_cm);
void VisionRuntime_SetBeaconRecognitionEnabled(uint8 enabled);
void VisionRuntime_SetVehicleYawPrior(float car_yaw_rel_uav_deg,
                                      uint16 age_ms, uint8 valid);
void VisionRuntime_SetBinaryThreshold(uint8 threshold);
void VisionRuntime_SetBinaryThresholds(uint8 inner, uint8 middle, uint8 outer);
void VisionRuntime_GetBinaryThresholds(uint8 thresholds[VISION_BINARY_ZONE_COUNT]);
void VisionRuntime_SetRecognitionThresholds(uint8 vehicle, uint8 beacon);
void VisionRuntime_GetRecognitionThresholds(uint8 *vehicle, uint8 *beacon);
void VisionRuntime_SetCoarseThreshold(uint8 threshold);
uint8 VisionRuntime_GetCoarseThreshold(void);
void VisionRuntime_SetCoarseThresholdRange(uint8 center_threshold, uint8 edge_threshold);
void VisionRuntime_GetCoarseThresholdRange(uint8 *center_threshold, uint8 *edge_threshold);
void VisionRuntime_SetCoarseZoneThresholds(
    const uint8 thresholds[VISION_RUNTIME_COARSE_ZONE_COUNT]);
void VisionRuntime_GetCoarseZoneThresholds(
    uint8 thresholds[VISION_RUNTIME_COARSE_ZONE_COUNT]);
void VisionRuntime_SetContourThresholds(uint8 arrow,
                                        const VisionContourThresholds_t *thresholds);
void VisionRuntime_GetContourThresholds(uint8 arrow,
                                        VisionContourThresholds_t *thresholds);
void VisionRuntime_SetBeaconAcceptLimit(uint8 limit);
uint8 VisionRuntime_GetBeaconAcceptLimit(void);
void VisionRuntime_GetDiag(VisionRuntimeDiag_t *diag);
void VisionRuntime_SetDiag(const VisionRuntimeDiag_t *diag);
void VisionRuntime_GetBeaconSlots(VisionRuntimeBeaconSlot_t *slots, uint8 max_slots);
void VisionRuntime_SetBeaconSlots(const VisionRuntimeBeaconSlot_t *slots, uint8 max_slots);
uint8 VisionRuntime_GetCandidateBoxes(VisionRuntimeCandidateBox_t *boxes, uint8 max_boxes);
const uint8 *VisionRuntime_GetBinaryImage(void);
const uint8 *VisionRuntime_GetBinaryDisplayImage(void);
const uint8 *VisionRuntime_GetUndistortedImage(void);
uint32 VisionRuntime_GetFineRleOverflowCount(void);

#if defined(VISION_RUNTIME_PC_DIAGNOSTICS)
void VisionRuntime_SetVehicleFallbackEnabled(uint8 enabled);
void VisionRuntime_SetVehicleFallbackThreshold(uint8 threshold);
uint8 VisionRuntime_GetVehicleFallbackThreshold(void);
const uint8 *VisionRuntime_GetVehicleBinaryImage(void);
#endif

#endif /* VISION_RUNTIME_H_ */
