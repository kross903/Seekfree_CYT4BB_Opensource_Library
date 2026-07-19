#ifndef CROSS_CORE_CAMERA_H
#define CROSS_CORE_CAMERA_H

#include "zf_common_headfile.h"

#define CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY  0U
#define CROSS_CORE_CAMERA_VISION_DISPLAY_RAW     1U
#define CROSS_CORE_CAMERA_VISION_DISPLAY_UNDISTORT 2U

#define CROSS_CORE_CAMERA_VISION_STATUS_IDLE                 0U
#define CROSS_CORE_CAMERA_VISION_STATUS_SENDING              1U
#define CROSS_CORE_CAMERA_VISION_STATUS_SENT                 2U
#define CROSS_CORE_CAMERA_VISION_STATUS_TIMEOUT              3U
#define CROSS_CORE_CAMERA_VISION_STATUS_REQUEST_FAILED       4U
#define CROSS_CORE_CAMERA_VISION_STATUS_ASSISTANT_REQUIRED   5U

void CrossCoreCamera_CameraCoreInit(void);
void CrossCoreCamera_CameraCoreTask(void);
uint8 CrossCoreCamera_ControlCoreFetchFrame(void);
#if CY_CORE_CM7_0
void CrossCoreCamera_ControlCoreAssistantInit(void);
void CrossCoreCamera_ControlCoreTask(void);
uint8 CrossCoreCamera_ControlCoreService(float roll_deg,
                                         float pitch_deg,
                                         float yaw_deg,
                                         float tof_height_cm,
                                         float fused_height_cm);
void CrossCoreCamera_ControlCorePublishInput(float roll_deg,
                                              float pitch_deg,
                                              float yaw_deg,
                                              float tof_height_cm,
                                              float fused_height_cm);
void CrossCoreCamera_ControlCoreSetBinaryThreshold(uint8 zone, uint8 threshold);
uint8 CrossCoreCamera_ControlCoreGetBinaryThreshold(uint8 zone);
uint32 CrossCoreCamera_ControlCoreResultPending(void);
uint8 CrossCoreCamera_ControlCoreRequestAssistantSnapshot(void);
uint8 CrossCoreCamera_ControlCoreSendAssistantFrame(void);
void CrossCoreCamera_ControlCoreRequestVisionScreen(uint8 enable);
void CrossCoreCamera_ControlCoreSetVisionDisplayMode(uint8 mode);
uint8 CrossCoreCamera_ControlCoreGetVisionDisplayMode(void);
void CrossCoreCamera_ControlCoreSetVisionStatus(uint8 status);
void CrossCoreCamera_ControlCoreSetVisionAssistantStream(uint8 enable);
uint8 CrossCoreCamera_ControlCoreGetVisionAssistantStream(void);
uint8 CrossCoreCamera_ControlCoreIsVisionScreenActive(void);
#endif

#endif
