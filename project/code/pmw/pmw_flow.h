#ifndef _PMW_FLOW_H_
#define _PMW_FLOW_H_

#include "zf_common_headfile.h"
#include "flow_sensor_config.h"

#define PMW_FLOW_TOF_SOURCE_NONE       (0U)
#define PMW_FLOW_TOF_SOURCE_LIVE       (1U)
#define PMW_FLOW_TOF_SOURCE_HOLD       (2U)

#define PMW_FLOW_AXIS_X                (0U)
#define PMW_FLOW_AXIS_Y                (1U)

#define PMW_FLOW_REJECT_NONE           (0U)
#define PMW_FLOW_REJECT_FLOW_INVALID   (1U << 0)
#define PMW_FLOW_REJECT_TOF_NOT_LIVE   (1U << 1)
#define PMW_FLOW_REJECT_TILT           (1U << 2)
#define PMW_FLOW_REJECT_RAW_LIMIT      (1U << 3)
#define PMW_FLOW_REJECT_INNOVATION     (1U << 4)
#define PMW_FLOW_REJECT_RAW_JUMP       (1U << 5)
#define PMW_FLOW_REJECT_REACQUIRING    (1U << 6)
#define PMW_FLOW_REJECT_UNINITIALIZED  (1U << 7)

typedef enum
{
    PMW_FLOW_AXIS_INVALID = 0,
    PMW_FLOW_AXIS_LIVE,
    PMW_FLOW_AXIS_REACQUIRING
} PMWFlowAxisState_t;

typedef struct
{
    uint8 valid;
    uint8 flow_used;
    uint8 acc_used;
    /* Legacy aggregate: true only when both X and Y fused feedback are valid. */
    uint8 fused_velocity_valid;
    uint8 flow_x_used;
    uint8 flow_y_used;
    uint8 fused_velocity_x_valid;
    uint8 fused_velocity_y_valid;
    uint8 flow_x_accepted;
    uint8 flow_y_accepted;
    uint8 flow_reject_reason_x;
    uint8 flow_reject_reason_y;
    uint8 flow_x_state;
    uint8 flow_y_state;
    int16 flow_delta_x_count;
    int16 flow_delta_y_count;
    float dt_s;
    float tof_los_cm;
    float range_cm;
    float roll_deg;
    float pitch_deg;
    float flow_x_rad;
    float flow_y_rad;
    float gyro_flow_x_delta_rad;
    float gyro_flow_y_delta_rad;
    float flow_comp_x_rad;
    float flow_comp_y_rad;
    float velocity_x_raw_cms;
    float velocity_y_raw_cms;
    float velocity_x_slew_cms;
    float velocity_y_slew_cms;
    float velocity_x_lpf_cms;
    float velocity_y_lpf_cms;
    float velocity_x_output_slew_cms;
    float velocity_y_output_slew_cms;
    float velocity_x_cms;
    float velocity_y_cms;
    /* ICM-predicted + optical-flow-corrected velocity used by the speed loop. */
    float velocity_x_fused_cms;
    float velocity_y_fused_cms;
    float velocity_x_fused_residual_cms;
    float velocity_y_fused_residual_cms;
    float flow_confidence;
    float flow_fusion_alpha;
    float kalman_k_x;
    float kalman_k_y;
    float kalman_p_x;
    float kalman_p_y;
    float kalman_r_x;
    float kalman_r_y;
    float flow_prediction_x_cms;
    float flow_prediction_y_cms;
    float flow_innovation_x_cms;
    float flow_innovation_y_cms;
    float flow_gate_x_cms;
    float flow_gate_y_cms;
    float flow_raw_jump_x_cms;
    float flow_raw_jump_y_cms;
    float flow_prediction_age_x_ms;
    float flow_prediction_age_y_ms;
    uint8 flow_reacquire_count_x;
    uint8 flow_reacquire_count_y;
    uint32 flow_reject_count_x;
    uint32 flow_reject_count_y;
    float flow_residual_norm_cms;
    float velocity_x_world_cms;
    float velocity_y_world_cms;
    float position_x_world_cm;
    float position_y_world_cm;
    float flow_delta_x_world_cm;
    float flow_delta_y_world_cm;
    float flow_position_x_world_cm;
    float flow_position_y_world_cm;
    uint8 flow_position_valid;
    float yaw_deg;
    float speed_cms;
} PMWFlowState_t;

void PMW_Flow_Init(void);
void PMW_Flow_Reset(void);
void PMW_Flow_AccumulateGyro(float gyro_x_rate_dps, float gyro_y_rate_dps, float dt_s);
void PMW_Flow_PredictVelocity(float imu_velocity_x_cms, float imu_velocity_y_cms, float dt_s);
void PMW_Flow_UpdateSample(int16 flow_delta_x,
                           int16 flow_delta_y,
                           float roll_deg,
                           float pitch_deg,
                           float tof_height_cm,
                           uint8 tof_source,
                           uint8 flow_sample_valid,
                           float dt_s,
                           float imu_velocity_x_cms,
                           float imu_velocity_y_cms);
void PMW_Flow_SyncMainLoopBuffer(void);
void PMW_Flow_GetState(PMWFlowState_t *state);
void PMW_Flow_ResetWorldPosition(void);

/* 陀螺去旋转增益（按轴独立）。kx 或 ky 传负值表示该轴保持不变。 */
void PMW_Flow_SetGyroCompGain(float kx, float ky);
void PMW_Flow_GetGyroCompGain(float *kx, float *ky);

#endif
