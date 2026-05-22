#ifndef VISION_RUNTIME_H_
#define VISION_RUNTIME_H_

#include "zf_common_headfile.h"

typedef struct
{
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

void VisionRuntime_ProcessImage(const uint8 image[MT9V03X_H][MT9V03X_W],
                                float roll_deg,
                                float pitch_deg,
                                float height_cm);
void VisionRuntime_ProcessFrame(float height_cm);
void VisionRuntime_GetDiag(VisionRuntimeDiag_t *diag);
void VisionRuntime_SetDiag(const VisionRuntimeDiag_t *diag);
void VisionRuntime_UpdateDiagFromCurrentState(void);

#endif /* VISION_RUNTIME_H_ */
