#ifndef VISION_CONTOUR_MATCH_H_
#define VISION_CONTOUR_MATCH_H_

#include "vision/vision_runtime.h"

#define VISION_CONTOUR_POINT_INVALID (0xFFFFU)
#define VISION_CONTOUR_TARGET_NONE   (0U)
#define VISION_CONTOUR_TARGET_BEACON (1U)
#define VISION_CONTOUR_TARGET_ARROW  (2U)
#define VISION_CONTOUR_MAX_SEEDS     (VISION_RUNTIME_BEACON_SLOT_COUNT + 1U)

#define VISION_CONTOUR_SAMPLE_BILINEAR (0U)
#define VISION_CONTOUR_SAMPLE_NEAREST  (1U)
#ifndef VISION_CONTOUR_SAMPLE_MODE
#define VISION_CONTOUR_SAMPLE_MODE VISION_CONTOUR_SAMPLE_BILINEAR
#endif
#if VISION_CONTOUR_SAMPLE_MODE != VISION_CONTOUR_SAMPLE_BILINEAR && \
    VISION_CONTOUR_SAMPLE_MODE != VISION_CONTOUR_SAMPLE_NEAREST
#error "Unsupported on-demand contour sampling mode"
#endif

typedef struct { uint16 x; uint16 y; } VisionContourSeed_t;

typedef struct
{
    VisionBlob_t blob;
    VisionContour_t contour;
    uint16 circularity_q8;
    uint16 solidity_q8;
    uint16 endpoint_x[2];
    uint16 endpoint_y[2];
    uint16 vertex_x;
    uint16 vertex_y;
    uint16 notch_x;
    uint16 notch_y;
    uint16 vertex_angle_deg;
    uint16 vertex_turn_deg;
    uint16 notch_turn_deg;
    uint16 max_negative_turn_deg;
    uint8 endpoint_count;
    uint8 contour_closed;
    uint8 contour_overflow;
    uint8 target;
    uint8 yaw_assisted;
    uint8 beacon_area_rejected;
    uint16 yaw_error_deg;
} VisionContourCandidate_t;

typedef struct
{
    const uint8 (*image)[MT9V03X_W];
    uint8 threshold;
    uint8 beacon_enabled;
    uint8 beacon_limit;
    float car_yaw_rel_uav_deg;
    uint8 yaw_valid;
    uint16 yaw_tolerance_deg;
    const VisionContourThresholds_t *arrow_thresholds;
    const VisionContourThresholds_t *beacon_thresholds;
    uint8 (*beacon_physical_gate)(VisionBlob_t *blob);
    VisionContourSeed_t seeds[VISION_CONTOUR_MAX_SEEDS];
    uint8 seed_count;
    uint8 force_global_search;
} VisionContourFrameConfig_t;

typedef struct
{
    VisionContourCandidate_t candidate[VISION_RUNTIME_MAX_CANDIDATES];
    uint8 candidate_count;
    uint8 beacon_count;
    uint8 arrow_count;
    uint8 seed_hits;
    uint8 used_global_search;
    uint32 sampled_pixels;
    uint32 traced_steps;
    uint32 overflow_count;
} VisionContourFrameResult_t;

void VisionContourMatch_Process(const VisionContourFrameConfig_t *config,
                                VisionContourFrameResult_t *result);
const uint8 *VisionContourMatch_GetBinaryImage(void);

#endif /* VISION_CONTOUR_MATCH_H_ */
