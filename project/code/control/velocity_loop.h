#ifndef VELOCITY_LOOP_H_
#define VELOCITY_LOOP_H_

#include "zf_common_headfile.h"
#include "control/pid_controller.h"

extern PID_Controller vel_pid_x;
extern PID_Controller vel_pid_y;

#define VELOCITY_LOOP_AXIS_X    (0U)
#define VELOCITY_LOOP_AXIS_Y    (1U)
#define VELOCITY_LOOP_D_FILTER_DEFAULT_HZ (8.0f)
#define VELOCITY_LOOP_D_FILTER_MIN_HZ     (1.0f)
#define VELOCITY_LOOP_D_FILTER_MAX_HZ     (50.0f)

#define VELOCITY_LOOP_STATUS_ACCEL_LIMITED  (0x01U)
#define VELOCITY_LOOP_STATUS_MOTOR_LIMITED  (0x02U)
#define VELOCITY_LOOP_STATUS_INTEGRATOR_HELD (0x04U)

typedef struct
{
    float roll_target_rate_raw_dps;
    float pitch_target_rate_raw_dps;
    float roll_target_rate_filt_dps;
    float pitch_target_rate_filt_dps;
    float roll_rate_ff_dps;
    float pitch_rate_ff_dps;
} VelocityLoopRateFeedforwardDiag_t;

typedef struct
{
    float target_velocity_cms;
    float actual_velocity_cms;
    float error_velocity_cms;
    float p_output_cms2;
    float i_output_cms2;
    float d_output_cms2;
    float acceleration_ff_cms2;
    float raw_output_cms2;
    float output_cms2;
    uint8 saturated;
    uint8 active;
} VelocityLoopAxisDiag_t;

typedef struct
{
    float target_vx_earth_cms;
    float target_vy_earth_cms;
    float actual_vx_earth_cms;
    float actual_vy_earth_cms;
    float acceleration_ff_x_earth_cms2;
    float acceleration_ff_y_earth_cms2;
    float acceleration_correction_x_earth_cms2;
    float acceleration_correction_y_earth_cms2;
    float acceleration_target_x_earth_cms2;
    float acceleration_target_y_earth_cms2;
    float target_roll_deg;
    float target_pitch_deg;
    float dt_s;
    uint8 status_flags;
    uint8 timing_valid;
} VelocityLoopHorizontalDiag_t;

void Velocity_Loop_Set_Enabled(uint8 enabled);
uint8 Velocity_Loop_Is_Enabled(void);
void Velocity_Loop_Set_Target_Velocity(float vx_earth_cms, float vy_earth_cms);
void Velocity_Loop_Set_Target_State(float vx_earth_cms,
                                    float vy_earth_cms,
                                    float ax_earth_cms2,
                                    float ay_earth_cms2);
void Velocity_Loop_Get_Target_Velocity(float *vx_earth_cms, float *vy_earth_cms);
void Velocity_Loop_Reset_Runtime(void);
void Velocity_Loop_Reset_D_Filter_State(void);
uint8 Velocity_Loop_Set_D_Filter_Hz(uint8 axis, float cutoff_hz);
float Velocity_Loop_Get_D_Filter_Hz(uint8 axis);
uint8 Velocity_Loop_Set_Rate_Feedforward_Parameters(uint8 axis,
                                                    float gain,
                                                    float limit_dps);
void Velocity_Loop_Get_Rate_Feedforward_Parameters(uint8 axis,
                                                   float *gain,
                                                   float *limit_dps);
void Velocity_Loop_Get_Rate_Feedforward_Diag(VelocityLoopRateFeedforwardDiag_t *diag);
uint8 Velocity_Loop_Get_Axis_Diag(uint8 axis, VelocityLoopAxisDiag_t *diag);
void Velocity_Loop_Get_Horizontal_Diag(VelocityLoopHorizontalDiag_t *diag);
void Velocity_Loop_Update_Attitude_Target(uint8 velocity_ready,
                                          float measured_vx_body_cms,
                                          float measured_vy_body_cms,
                                          float yaw_deg,
                                          float dt_s,
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
