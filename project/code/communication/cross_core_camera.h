#ifndef CROSS_CORE_CAMERA_H
#define CROSS_CORE_CAMERA_H

#include "zf_common_headfile.h"

#define CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY  0U
#define CROSS_CORE_CAMERA_VISION_DISPLAY_RAW     1U

void CrossCoreCamera_CameraCoreInit(void);
void CrossCoreCamera_CameraCoreTask(void);
uint8 CrossCoreCamera_ControlCoreFetchFrame(void);
#if CY_CORE_CM7_0
void CrossCoreCamera_ControlCoreAssistantInit(void);
void CrossCoreCamera_ControlCoreTask(void);
uint8 CrossCoreCamera_ControlCoreService(float roll_deg, float pitch_deg, float height_cm);
void CrossCoreCamera_ControlCorePublishInput(float roll_deg, float pitch_deg, float height_cm);
uint32 CrossCoreCamera_ControlCoreResultPending(void);
uint8 CrossCoreCamera_ControlCoreSendAssistantFrame(void);
void CrossCoreCamera_ControlCoreRequestVisionScreen(uint8 enable);
void CrossCoreCamera_ControlCoreSetVisionDisplayMode(uint8 mode);
uint8 CrossCoreCamera_ControlCoreGetVisionDisplayMode(void);
void CrossCoreCamera_ControlCoreSetVisionAssistantStream(uint8 enable);
uint8 CrossCoreCamera_ControlCoreGetVisionAssistantStream(void);
uint8 CrossCoreCamera_ControlCoreIsVisionScreenActive(void);
#endif

#endif
