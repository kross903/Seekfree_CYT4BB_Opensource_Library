#ifndef CROSS_CORE_VISION_H_
#define CROSS_CORE_VISION_H_

#include "zf_common_headfile.h"
#include "vision/ir_localization.h"
#include "vision/vision_runtime.h"

typedef struct
{
    uint32 magic;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float height_cm;
    uint32 seq;
} CrossCoreVisionInput_t;

typedef struct
{
    float x_cm;
    float y_cm;
    float z_cm;
    float square_side_cm;
    uint8 raw_detect_valid;
    uint8 track_valid;
    uint8 position_lost;
    uint8 detected_point_count;
} CrossCoreVisionControlResult_t;

typedef struct
{
    uint32 magic;
    CrossCoreVisionControlResult_t control;
    VisionRuntimeDiag_t diag;
    uint32 seq;
} CrossCoreVisionOutput_t;

void CrossCoreVision_InitShared(void);

void CrossCoreVision_ControlCorePublishInput(float roll_deg,
                                             float pitch_deg,
                                             float yaw_deg,
                                             float height_cm);
uint8 CrossCoreVision_CameraCoreFetchInput(CrossCoreVisionInput_t *out_input);

void CrossCoreVision_BuildControlResult(const IR_Pose_t *ir_pose,
                                        uint8 position_lost,
                                        const VisionRuntimeDiag_t *diag,
                                        CrossCoreVisionControlResult_t *out_control);
void CrossCoreVision_CameraCorePublishOutput(const CrossCoreVisionControlResult_t *control,
                                             const VisionRuntimeDiag_t *diag);
uint8 CrossCoreVision_ControlCoreFetchOutput(CrossCoreVisionOutput_t *out_output);
void CrossCoreVision_ControlCoreApplyOutput(const CrossCoreVisionOutput_t *output);
void CrossCoreVision_ControlCoreService(float roll_deg,
                                        float pitch_deg,
                                        float yaw_deg,
                                        float height_cm);

#endif
