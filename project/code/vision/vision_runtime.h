#ifndef VISION_RUNTIME_H_
#define VISION_RUNTIME_H_

#include "zf_common_headfile.h"

/* 候选区完整分辨率精筛阈值：箭头较亮，信标允许保留较暗像素。 */
#define VISION_BINARY_THRESHOLD_VEHICLE          (145U) /* 箭头首轮精筛阈值 */
#define VISION_BINARY_THRESHOLD_VEHICLE_FALLBACK (60U)  /* 首轮失败后的箭头回退阈值 */
#define VISION_BINARY_THRESHOLD_BEACON           (10U)  /* 信标精筛二值化阈值 */
#if VISION_BINARY_THRESHOLD_VEHICLE_FALLBACK == 0U || \
    VISION_BINARY_THRESHOLD_VEHICLE_FALLBACK >= VISION_BINARY_THRESHOLD_VEHICLE
#error "Vehicle fallback threshold must be lower than the primary threshold"
#endif
/* Legacy names keep the shared-memory interface source compatible. */
#define VISION_BINARY_THRESHOLD_INNER  VISION_BINARY_THRESHOLD_VEHICLE
#define VISION_BINARY_THRESHOLD_MIDDLE VISION_BINARY_THRESHOLD_BEACON
#define VISION_BINARY_THRESHOLD_OUTER  VISION_BINARY_THRESHOLD_BEACON
#define VISION_BINARY_THRESHOLD        VISION_BINARY_THRESHOLD_BEACON
#define VISION_RUNTIME_COARSE_ZONE_COUNT       (5U)  /* 中心到边缘共五个同心分区 */
/* 粗筛阈值从中心向边缘递减：近处要求高，远处弱信标使用较低阈值。 */
#define VISION_RUNTIME_COARSE_THRESHOLD_CENTER (150U) /* 光轴中心圈 */
#define VISION_RUNTIME_COARSE_THRESHOLD_NEAR   (60U) /* 近圈 */
#define VISION_RUNTIME_COARSE_THRESHOLD_MIDDLE (50U) /* 中圈 */
#define VISION_RUNTIME_COARSE_THRESHOLD_FAR    (50U) /* 远圈 */
#define VISION_RUNTIME_COARSE_THRESHOLD_EDGE   (50U) /* 图像边缘圈 */
/* Legacy single-threshold name keeps existing callers source compatible. */
#define VISION_RUNTIME_COARSE_THRESHOLD VISION_RUNTIME_COARSE_THRESHOLD_CENTER
#define VISION_BINARY_ZONE_INNER       (0U)
#define VISION_BINARY_ZONE_MIDDLE      (1U)
#define VISION_BINARY_ZONE_OUTER       (2U)
#define VISION_BINARY_ZONE_COUNT       (3U)
#define VISION_RUNTIME_BEACON_SLOT_COUNT (6U)
#define VISION_RUNTIME_BLOB_AREA_COUNT (16U)
#define VISION_RUNTIME_BEACON_SPLIT_AREA_THRESHOLD (4)
#define VISION_RUNTIME_CONTOUR_MAX_POINTS (640U)
#define VISION_RUNTIME_LOCAL_BIRDVIEW_SCALE_CM_PER_PIXEL (0.5f)
#define VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH (128U)
#define VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_HEIGHT (128U)
#define VISION_RUNTIME_BEACON_BOX_AREA_MIN_PIXELS (8.0f)
#define VISION_RUNTIME_BEACON_BOX_AREA_MAX_PIXELS (100.0f)
#define VISION_RUNTIME_VEHICLE_MIN_RAW_BOX_AREA       (100U) /* 常规箭头候选框门槛 */
#define VISION_RUNTIME_VEHICLE_SMALL_BOX_MIN_AREA     (90U)  /* 稀疏小箭头候选框门槛 */
#define VISION_RUNTIME_VEHICLE_SMALL_BOX_MAX_FILL_PCT (70U)  /* 小框粗连通域最大占空比 */
#if VISION_RUNTIME_VEHICLE_SMALL_BOX_MIN_AREA > \
        VISION_RUNTIME_VEHICLE_MIN_RAW_BOX_AREA || \
    VISION_RUNTIME_VEHICLE_SMALL_BOX_MAX_FILL_PCT > 100U
#error "Invalid vehicle small-box gate"
#endif
#define VISION_RUNTIME_FINE_MIN_FOREGROUND_PIXELS (8U)
#ifndef VISION_RUNTIME_PHYSICAL_GRID_ENABLE
#define VISION_RUNTIME_PHYSICAL_GRID_ENABLE (0U)
#endif
#if VISION_RUNTIME_PHYSICAL_GRID_ENABLE != 0U && \
    VISION_RUNTIME_PHYSICAL_GRID_ENABLE != 1U
#error "VISION_RUNTIME_PHYSICAL_GRID_ENABLE must be 0 or 1"
#endif
#ifndef VISION_RUNTIME_DEBUG_UNDISTORT_IMAGE_ENABLE
#define VISION_RUNTIME_DEBUG_UNDISTORT_IMAGE_ENABLE (0U)
#endif

/* Candidate refinement pipeline selection. */
#define VISION_RUNTIME_PIPELINE_UNDISTORT_THEN_IPM (0U)
#define VISION_RUNTIME_PIPELINE_IPM_THEN_BINARY   (1U)
#ifndef VISION_RUNTIME_PIPELINE_MODE
#define VISION_RUNTIME_PIPELINE_MODE VISION_RUNTIME_PIPELINE_UNDISTORT_THEN_IPM
#endif
#if VISION_RUNTIME_PIPELINE_MODE != VISION_RUNTIME_PIPELINE_UNDISTORT_THEN_IPM && \
    VISION_RUNTIME_PIPELINE_MODE != VISION_RUNTIME_PIPELINE_IPM_THEN_BINARY
#error "Unsupported VISION_RUNTIME_PIPELINE_MODE"
#endif

#define VISION_RUNTIME_FINE_WIDTH   (MT9V03X_W * 2U)
#define VISION_RUNTIME_FINE_HEIGHT  (MT9V03X_H * 2U)
#define VISION_RUNTIME_FINE_STRIDE  ((VISION_RUNTIME_FINE_WIDTH + 7U) / 8U)
#define VISION_RUNTIME_FINE_BYTES   (VISION_RUNTIME_FINE_STRIDE * VISION_RUNTIME_FINE_HEIGHT)
#define VISION_RUNTIME_BINARY_STRIDE VISION_RUNTIME_FINE_STRIDE
#define VISION_RUNTIME_BINARY_BYTES  VISION_RUNTIME_FINE_BYTES
#define VISION_RUNTIME_RLE_MAX_RUNS  (4096U)

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
} VisionRuntimeCandidateBox_t;

void VisionRuntime_ProcessFrame(float height_cm);
void VisionRuntime_ProcessFrameWithPose(float roll_deg, float pitch_deg, float height_cm);
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
void VisionRuntime_GetDiag(VisionRuntimeDiag_t *diag);
void VisionRuntime_SetDiag(const VisionRuntimeDiag_t *diag);
void VisionRuntime_GetBeaconSlots(VisionRuntimeBeaconSlot_t *slots, uint8 max_slots);
void VisionRuntime_SetBeaconSlots(const VisionRuntimeBeaconSlot_t *slots, uint8 max_slots);
uint8 VisionRuntime_GetCandidateBoxes(VisionRuntimeCandidateBox_t *boxes, uint8 max_boxes);
/* Returns the row-aligned MSB-first 1-bit 376x240 binary bitmap. */
const uint8 *VisionRuntime_GetBinaryImage(void);
const uint8 *VisionRuntime_GetBinaryDisplayImage(void);
const uint8 *VisionRuntime_GetUndistortedImage(void);
uint32 VisionRuntime_GetFineRleOverflowCount(void);

#endif /* VISION_RUNTIME_H_ */
