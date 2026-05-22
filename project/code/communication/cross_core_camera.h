#ifndef CROSS_CORE_CAMERA_H
#define CROSS_CORE_CAMERA_H

#include "zf_common_headfile.h"

void CrossCoreCamera_CameraCoreInit(void);
uint8 CrossCoreCamera_CameraCoreTask(void);
uint8 CrossCoreCamera_ControlCoreFetchFrame(void);
#if CY_CORE_CM7_0
void CrossCoreCamera_ControlCoreAssistantInit(void);
uint8 CrossCoreCamera_ControlCoreSendAssistantFrame(void);
#endif

#endif
