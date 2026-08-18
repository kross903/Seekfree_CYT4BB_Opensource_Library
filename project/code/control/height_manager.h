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

typedef enum
{
    HEIGHT_TOF_STATE_INVALID = 0,
    HEIGHT_TOF_STATE_LIVE,
    HEIGHT_TOF_STATE_HELD_REJECTED,
    HEIGHT_TOF_STATE_REACQUIRING
} HeightTofState_t;

typedef struct
{
    float raw_los_cm;
    float tof1_raw_cm;
    float tof2_raw_cm;
    float tof_selected_cm;
    float tof_disagreement_cm;
    float compensated_center_cm;
    float flow_height_cm;
    float filtered_height_cm;
    float vertical_velocity_cms;
    float fused_velocity_cms;
    float predicted_height_cm;
    float innovation_cm;
    float gate_cm;
    float candidate_mean_cm;
    uint32 sample_sequence;
    uint16 raw_sample_age_ms;
    uint16 accepted_age_ms;
    uint8 candidate_count;
    uint8 reject_count;
    uint8 new_sample;
    uint8 valid;
    uint8 state;
    uint8 tof1_valid;
    uint8 tof2_valid;
    uint8 tof_source;
} HeightTofSnapshot_t;

typedef struct
{
    float tof_raw_cm;
    float tof1_raw_cm;
    float tof2_raw_cm;
    float tof_selected_cm;
    float tof_disagreement_cm;
    float tof_comp_cm;
    float tof_median_cm;
    float tof_height_filt_cm;
    float tof_velocity_cms;
    float acc_z_level_mg;
    float gravity_bias_mg;
    float acc_z_cms2;
    float acc_velocity_cms;
    float fused_velocity_cms;
    float velocity_feedback_cms;
    float velocity_feedback_view_cms;
    float target_velocity_cms;
    float velocity_error_cms;
    float height_pid_output_cms;
    float velocity_pid_output;
    uint8 height_pid_limit_flags;
    uint8 velocity_pid_limit_flags;
    uint8 height_integrator_held;
    uint8 velocity_integrator_held;
    float tof_predicted_cm;
    float tof_innovation_cm;
    float tof_gate_cm;
    float tof_candidate_cm;
    uint32 tof_sample_sequence;
    uint16 tof_raw_age_ms;
    uint16 tof_accepted_age_ms;
    uint8 tof_valid;
    uint8 tof_candidate_count;
    uint8 tof_reject_count;
    uint8 tof_state;
    uint8 tof1_valid;
    uint8 tof2_valid;
    uint8 tof_source;
    uint8 tof1_initialized;
    uint8 tof2_initialized;
    uint16 tof1_age_ms;
    uint16 tof2_age_ms;
    uint32 tof1_invalid_count;
    uint32 tof2_invalid_count;
    uint32 tof2_bus_error_count;
    uint8 acc_used;
    uint8 cascade_enabled;
} HeightManagerDiag_t;

extern HeightFusionResult_t g_height_result;

extern PID_Controller HeightPID;
extern PID_Controller HeightVelPID;

extern float HeightSpeedPID_output;
extern float tof_height_true_filt;
extern float g_height_target_cm;

void HeightManager_Init(void);
void HeightManager_Task_10ms(void);
void HeightManager_Set_Target(float target_cm);
float HeightManager_Get_Target(void);
float HeightManager_Get_Active_Target(void);
void HeightManager_Get_Diag(HeightManagerDiag_t *diag);
void HeightManager_GetTofSnapshot(HeightTofSnapshot_t *snapshot);
uint8 HeightManager_IsTofValid(void);
uint8 HeightManager_IsReadyForTakeoff(void);
void HeightManager_PrepareSoftTakeoff(void);
void HeightManager_BeginSoftTakeoff(void);
void HeightManager_SetOutputBlend(float blend);
void HeightManager_AbortSoftTakeoff(void);
uint8 HeightManager_BeginAutoLanding(void);
void HeightManager_EndAutoLanding(void);
uint8 HeightManager_IsAutoLandingActive(void);
uint8 HeightManager_AutoLandingTouchdownConfirmed(void);
float HeightManager_GetAutoLandingTarget(void);
float HeightManager_GetCenterHeightFromTofCm(float tof_los_cm, float roll_deg, float pitch_deg);
float HeightManager_GetFlowHeightFromTofCm(float tof_los_cm, float roll_deg, float pitch_deg);

#ifdef __cplusplus
}
#endif

#endif
