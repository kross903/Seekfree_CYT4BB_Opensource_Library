#ifndef HEIGHT_MANAGER_H_
#define HEIGHT_MANAGER_H_

#include "zf_common_typedef.h"
#include "control/pid_controller.h"

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
extern float g_height_target_cm;

void HeightManager_Init(float initial_height_cm);
void HeightManager_Task_10ms(void);
void HeightManager_Set_Target(float target_cm);
float HeightManager_Get_Target(void);
float HeightManager_Get_Active_Target(void);
float HeightManager_GetCenterHeightFromTofCm(float tof_los_cm, float roll_deg, float pitch_deg);
float HeightManager_GetFlowHeightFromTofCm(float tof_los_cm, float roll_deg, float pitch_deg);

#ifdef __cplusplus
}
#endif

#endif
