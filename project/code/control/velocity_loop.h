#ifndef VELOCITY_LOOP_H_
#define VELOCITY_LOOP_H_

#include "zf_common_headfile.h"
#include "control/pid_controller.h"

extern PID_Controller vel_pid_x;
extern PID_Controller vel_pid_y;

void Velocity_Loop_Set_Enabled(uint8 enabled);
uint8 Velocity_Loop_Is_Enabled(void);
void Velocity_Loop_Set_Target_Velocity(float vx_cms, float vy_cms);
void Velocity_Loop_Get_Target_Velocity(float *vx_cms, float *vy_cms);
void Velocity_Loop_Reset_Runtime(void);
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
