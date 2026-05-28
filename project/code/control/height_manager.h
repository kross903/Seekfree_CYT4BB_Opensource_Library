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

typedef struct
{
    float tof_raw_cm;
    float tof_comp_cm;
    float tof_median_cm;
    float tof_height_filt_cm;
    float tof_velocity_cms;
    float acc_z_level_mg;
    float gravity_bias_mg;
    float acc_z_cms2;
    float acc_velocity_cms;
    float fused_velocity_cms;
    float target_velocity_cms;
    float velocity_error_cms;
    float height_pid_output_cms;
    float velocity_pid_output;
    uint8 tof_valid;
    uint8 acc_used;
    uint8 cascade_enabled;
} HeightManagerDiag_t;

extern HeightFusionResult_t g_height_result;

extern PID_Controller HeightPID;
extern PID_Controller HeightVelPID;

extern float HeightSpeedPID_output;
extern float tof_height_true_filt;
extern float g_height_target_cm;

void HeightManager_Init(float initial_height_cm);
void HeightManager_Task_10ms(void);
void HeightManager_Set_Target(float target_cm);
float HeightManager_Get_Target(void);
float HeightManager_Get_Active_Target(void);
void HeightManager_Get_Diag(HeightManagerDiag_t *diag);
float HeightManager_GetCenterHeightFromTofCm(float tof_los_cm, float roll_deg, float pitch_deg);
float HeightManager_GetFlowHeightFromTofCm(float tof_los_cm, float roll_deg, float pitch_deg);

#ifdef __cplusplus
}
#endif

#endif
