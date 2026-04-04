#ifndef CROSS_CORE_CAMERA_H
#define CROSS_CORE_CAMERA_H

#include "zf_common_headfile.h"

void CrossCoreCamera_CameraCoreInit(void);
void CrossCoreCamera_CameraCoreTask(void);
uint8 CrossCoreCamera_ControlCoreFetchFrame(void);

#endif
