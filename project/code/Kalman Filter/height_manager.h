#ifndef HEIGHT_MANAGER_H_
#define HEIGHT_MANAGER_H_

#include "zf_common_typedef.h"
#include "pid_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float height_cm;
    float velocity_cms;
} HeightFusionResult_t;

extern HeightFusionResult_t g_height_result;

extern PID_Controller HeightPID;

extern float HeightSpeedPID_output;
extern float tof_height_true_filt;

void HeightManager_Init(float initial_height_cm);
void HeightManager_Task_25ms(void);

#ifdef __cplusplus
}
#endif

#endif
