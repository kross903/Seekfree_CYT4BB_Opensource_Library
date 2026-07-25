#ifndef _PMW_FLOW_H_
#define _PMW_FLOW_H_

#include "zf_common_headfile.h"
#include "flow_sensor_config.h"

#define PMW_FLOW_TOF_SOURCE_NONE       (0U)
#define PMW_FLOW_TOF_SOURCE_LIVE       (1U)
#define PMW_FLOW_TOF_SOURCE_HOLD       (2U)

typedef struct
{
    uint8 valid;
    uint8 flow_used;
    uint8 acc_used;
    uint8 fused_velocity_valid;
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
    float gyro_flow_z_delta_rad;
    float flow_comp_x_rad;
    float flow_comp_y_rad;
    float gyro_rate_x_rad_s;
    float gyro_rate_y_rad_s;
    float gyro_rate_z_rad_s;
    float gyro_accel_max_rad_s2;
    float velocity_x_camera_cms;
    float velocity_y_camera_cms;
    float mount_comp_velocity_x_cms;
    float mount_comp_velocity_y_cms;
    float velocity_x_center_body_cms;
    float velocity_y_center_body_cms;
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
    float flow_rotation_confidence;
    float flow_fusion_alpha;
    float fusion_alpha_x;
    float fusion_alpha_y;
    float fusion_bias_ki_x;
    float fusion_bias_ki_y;
    float fusion_accel_bias_x_cms2;
    float fusion_accel_bias_y_cms2;
    float fusion_innovation_evidence_x;
    float fusion_innovation_evidence_y;
    /* 最近一次光流反馈更新时的融合分解快照。 */
    float fusion_imu_velocity_x_cms;
    float fusion_imu_velocity_y_cms;
    float fusion_imu_delta_x_cms;
    float fusion_imu_delta_y_cms;
    float fusion_prediction_x_cms;
    float fusion_prediction_y_cms;
    float fusion_residual_x_cms;
    float fusion_residual_y_cms;
    float fusion_correction_x_cms;
    float fusion_correction_y_cms;
    float fusion_output_x_cms;
    float fusion_output_y_cms;
    float kalman_k_x;
    float kalman_k_y;
    float kalman_p_x;
    float kalman_p_y;
    float kalman_r_x;
    float kalman_r_y;
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
    uint32 flow_frame_seq;
    float flow_rx_dt_ms;
    float flow_sensor_dt_ms;
    float gyro_window_dt_ms;
    float flow_consumer_delay_ms;
    uint8 flow_sync_valid;
    uint16 flow_dropped_frame_count;
    uint16 flow_sync_error_count;
} PMWFlowState_t;

void PMW_Flow_Init(void);
void PMW_Flow_Reset(void);
void PMW_Flow_AccumulateGyro(float gyro_x_rate_dps,
                             float gyro_y_rate_dps,
                             float gyro_z_rate_dps,
                             float dt_s);
void PMW_Flow_PredictVelocity(float imu_velocity_x_cms, float imu_velocity_y_cms, float dt_s);
uint8 PMW_Flow_GetGyroSnapshotAtCycle(uint32 target_cycle,
                                      float *total_x_rad,
                                      float *total_y_rad,
                                      float *total_z_rad);
void PMW_Flow_GetAccumulatedGyroDelta(float *gyro_x_delta_rad,
                                      float *gyro_y_delta_rad,
                                      float *gyro_z_delta_rad);
void PMW_Flow_SetSyncDiagnostics(uint32 frame_seq,
                                 float rx_dt_ms,
                                 float sensor_dt_ms,
                                 float gyro_window_dt_ms,
                                 float consumer_delay_ms,
                                 uint8 sync_valid,
                                 uint16 dropped_frame_count,
                                 uint16 sync_error_count);
void PMW_Flow_UpdateSample(int16 flow_delta_x,
                           int16 flow_delta_y,
                           float roll_deg,
                           float pitch_deg,
                           float tof_height_cm,
                           uint8 tof_source,
                           float dt_s,
                           float imu_velocity_x_cms,
                           float imu_velocity_y_cms,
                           float gyro_flow_x_delta_rad,
                           float gyro_flow_y_delta_rad,
                           float gyro_flow_z_delta_rad);
void PMW_Flow_SyncMainLoopBuffer(void);
void PMW_Flow_GetState(PMWFlowState_t *state);
void PMW_Flow_ResetWorldPosition(void);

/* 陀螺去旋转增益（按轴独立）。kx 或 ky 传负值表示该轴保持不变。 */
void PMW_Flow_SetGyroCompGain(float kx, float ky);
void PMW_Flow_GetGyroCompGain(float *kx, float *ky);

#endif
