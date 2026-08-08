#ifndef DUAL_CAMERA_FUSION_H_
#define DUAL_CAMERA_FUSION_H_

#include "zf_common_headfile.h"
#include "communication/camera_board_protocol.h"
#include "communication/cross_core_camera.h"

#define VISION_FUSION_OUTPUT_SHADOW          (0U)
#define VISION_FUSION_OUTPUT_LIVE            (1U)

#ifndef VISION_FUSION_OUTPUT_MODE
#define VISION_FUSION_OUTPUT_MODE            VISION_FUSION_OUTPUT_SHADOW
#endif

#if (VISION_FUSION_OUTPUT_MODE != VISION_FUSION_OUTPUT_SHADOW) && \
    (VISION_FUSION_OUTPUT_MODE != VISION_FUSION_OUTPUT_LIVE)
#error "Invalid VISION_FUSION_OUTPUT_MODE"
#endif

#define DUAL_CAMERA_FUSION_MAX_TARGETS       (6U)

typedef enum
{
    DUAL_CAMERA_FUSION_SOURCE_NONE = 0,
    DUAL_CAMERA_FUSION_SOURCE_FRONT,
    DUAL_CAMERA_FUSION_SOURCE_REAR,
    DUAL_CAMERA_FUSION_SOURCE_BOTH_FRONT,
    DUAL_CAMERA_FUSION_SOURCE_BOTH_REAR,
} DualCameraFusionSource_t;

typedef struct
{
    uint8 valid;
    uint8 track_id;
    uint8 source;
    uint8 detection_confidence;
    uint8 position_sigma_cm;
    float x_cm;
    float y_cm;
    float dx_cm;
    float dy_cm;
} DualCameraFusionTarget_t;

typedef struct
{
    uint32 reference_time_us;
    uint32 reference_frame_seq;
    uint32 tracker_reset_count;
    float car_x_cm;
    float car_y_cm;
    float yaw_deg;
    uint8 valid;
    uint8 paired;
    uint8 reference_source;
    uint8 car_valid;
    uint8 car_raw_valid;
    uint8 yaw_valid;
    uint8 target_count;
    DualCameraFusionTarget_t target[DUAL_CAMERA_FUSION_MAX_TARGETS];
} DualCameraFusionSnapshot_t;

typedef struct
{
    uint8 output_mode;
    uint8 extrinsic_nominal;
    uint8 extrinsic_valid;
    uint8 front_healthy;
    uint8 rear_healthy;
    uint8 front_car_valid;
    uint8 rear_car_valid;
    uint8 front_beacon_count;
    uint8 rear_beacon_count;
    uint8 pair_valid;
    uint8 output_count;
    uint8 duplicate_count;
    uint16 front_age_ms;
    uint16 rear_age_ms;
    int16 pair_dt_ms;
    float pair_yaw_delta_deg;
    float duplicate_residual_cm;
    float duplicate_residual_median_cm;
    float duplicate_residual_p95_cm;
    float duplicate_residual_max_cm;
    uint16 pair_rate_permille;
    uint32 build_count;
    uint32 dual_healthy_build_count;
    uint32 paired_build_count;
    uint32 front_only_build_count;
    uint32 rear_only_build_count;
    uint32 empty_build_count;
    uint32 front_stale_build_count;
    uint32 rear_stale_build_count;
    uint32 state_lookup_miss_count;
    uint32 cross_source_reject_count;
    uint32 duplicate_total_count;
    uint32 output_overflow_count;
    uint32 tracker_reset_count;
    uint32 link_crc_error_count;
    uint32 link_rx_overflow_count;
    uint32 link_tx_busy_drop_count;
    uint32 rear_dropped_frame_count;
    uint16 rear_crc_error_count;
    uint16 rear_rx_overflow_count;
    uint16 rear_tx_busy_drop_count;
} DualCameraFusionDiag_t;

void DualCameraFusion_Init(void);
void DualCameraFusion_ServiceInputs(void);
void DualCameraFusion_PushFrontFrame(const CrossCoreCameraFrontFrame_t *frame);
void DualCameraFusion_PushRearFrame(const CameraBoardVisionFrame_t *frame);
void DualCameraFusion_BuildSnapshot(uint32 now_us);
uint8 DualCameraFusion_GetSnapshot(DualCameraFusionSnapshot_t *snapshot);
void DualCameraFusion_GetDiag(DualCameraFusionDiag_t *diag);

#endif /* DUAL_CAMERA_FUSION_H_ */
