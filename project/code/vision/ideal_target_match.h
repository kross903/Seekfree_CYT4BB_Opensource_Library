#ifndef IDEAL_TARGET_MATCH_H_
#define IDEAL_TARGET_MATCH_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_IDEAL_TARGET_ARROW  (1U)
#define VISION_IDEAL_TARGET_BEACON (2U)

#define VISION_IDEAL_MAX_SAMPLES (256U)

typedef struct
{
    uint8 target_kind;
    uint8 passed;
    uint8 score_q8;
    uint8 box_clipped;
    uint16 positive_count;
    uint16 negative_count;
    uint16 near_negative_count;
    uint16 positive_mean;
    uint16 negative_mean;
    uint16 near_negative_mean;
    uint16 gray_difference;
    uint16 standardized_separation_q8;
    uint16 positive_hit_rate_q8;
    uint16 negative_hit_rate_q8;
    uint16 binary_consistency_q8;
    uint16 completeness_q8;
    int16 image_x[5];
    int16 image_y[5];
    float world_x[5];
    float world_y[5];
} VisionIdealMatchResult_t;

void VisionIdealMatch_BeginFrame(float car_yaw_rel_uav_deg, uint8 yaw_valid);

uint8 VisionIdealMatch_Arrow(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    int center_x, int center_y, uint8 binary_threshold,
    VisionIdealMatchResult_t *result);

uint8 VisionIdealMatch_Beacon(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    int center_x, int center_y, uint8 binary_threshold,
    VisionIdealMatchResult_t *result);

#ifdef __cplusplus
}
#endif

#endif /* IDEAL_TARGET_MATCH_H_ */
