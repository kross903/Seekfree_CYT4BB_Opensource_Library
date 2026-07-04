#ifndef VISION_RUNTIME_H_
#define VISION_RUNTIME_H_

#include "zf_common_headfile.h"

#define VISION_BINARY_THRESHOLD (150)
#define VISION_RUNTIME_BEACON_SLOT_COUNT (5U)
#define VISION_RUNTIME_BLOB_AREA_COUNT (16U)
#define VISION_RUNTIME_BEACON_SPLIT_AREA_THRESHOLD (17)
#define VISION_RUNTIME_CONTOUR_MAX_POINTS (640U)

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
    float world_x;
    float world_y;
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
    float beacon_nearest_brightness;
    float beacon_nearest_occupancy;
    float beacon_nearest_aspect;
} VisionRuntimeDiag_t;

typedef struct
{
    uint8 valid;
    float x_cm;
    float y_cm;
} VisionRuntimeBeaconSlot_t;

void VisionRuntime_ProcessFrame(float height_cm);
void VisionRuntime_ProcessFrameWithPose(float roll_deg, float pitch_deg, float height_cm);
void VisionRuntime_GetDiag(VisionRuntimeDiag_t *diag);
void VisionRuntime_SetDiag(const VisionRuntimeDiag_t *diag);
void VisionRuntime_GetBeaconSlots(VisionRuntimeBeaconSlot_t *slots, uint8 max_slots);
void VisionRuntime_SetBeaconSlots(const VisionRuntimeBeaconSlot_t *slots, uint8 max_slots);

#endif /* VISION_RUNTIME_H_ */
