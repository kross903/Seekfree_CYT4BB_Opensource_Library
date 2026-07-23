#ifndef VELOCITY_LOOP_H_
#define VELOCITY_LOOP_H_

#include "zf_common_headfile.h"
#include "control/pid_controller.h"

extern PID_Controller vel_pid_x;
extern PID_Controller vel_pid_y;

#define VELOCITY_LOOP_AXIS_X    (0U)
#define VELOCITY_LOOP_AXIS_Y    (1U)

typedef struct
{
    float roll_target_rate_raw_dps;
    float pitch_target_rate_raw_dps;
    float roll_target_rate_filt_dps;
    float pitch_target_rate_filt_dps;
    float roll_rate_ff_dps;
    float pitch_rate_ff_dps;
} VelocityLoopRateFeedforwardDiag_t;

/* 保存同一次速度 PID 计算的分项，Y 轴分项已转换为实际 Roll 修正方向。 */
typedef struct
{
    float target_velocity_cms;
    float actual_velocity_cms;
    float error_velocity_cms;
    float p_output_deg;
    float i_output_deg;
    float d_output_deg;
    float raw_output_deg;
    float output_deg;
    uint8 saturated;
    uint8 active;
} VelocityLoopAxisDiag_t;

void Velocity_Loop_Set_Enabled(uint8 enabled);
uint8 Velocity_Loop_Is_Enabled(void);
void Velocity_Loop_Set_Target_Velocity(float vx_cms, float vy_cms);
void Velocity_Loop_Get_Target_Velocity(float *vx_cms, float *vy_cms);
void Velocity_Loop_Reset_Runtime(void);
uint8 Velocity_Loop_Set_Rate_Feedforward_Parameters(uint8 axis,
                                                    float gain,
                                                    float limit_dps);
void Velocity_Loop_Get_Rate_Feedforward_Parameters(uint8 axis,
                                                   float *gain,
                                                   float *limit_dps);
void Velocity_Loop_Get_Rate_Feedforward_Diag(VelocityLoopRateFeedforwardDiag_t *diag);
uint8 Velocity_Loop_Get_Axis_Diag(uint8 axis, VelocityLoopAxisDiag_t *diag);
void Velocity_Loop_Update_Attitude_Target(uint8 velocity_ready,
                                          float measured_vx_cms,
                                          float measured_vy_cms,
                                          float captured_roll,
                                          float captured_pitch,
                                          float *target_roll,
                                          float *target_pitch);
void Velocity_Loop_Get_Diag(float *pitch_corr_deg,
                            float *roll_corr_deg,
                            float *fb_vx_cms,
                            float *fb_vy_cms,
                            float *x_i_out,
                            float *y_i_out,
                            uint8 *active);

#endif
