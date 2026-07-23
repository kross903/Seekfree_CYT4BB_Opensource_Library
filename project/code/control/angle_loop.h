#ifndef ANGLE_LOOP_H_
#define ANGLE_LOOP_H_

#include "zf_common_headfile.h"
#include "control/pid_controller.h"

extern PID_Controller angle_pid_roll;
extern PID_Controller angle_pid_pitch;
extern PID_Controller angle_pid_yaw;

extern float target_roll_rate;
extern float target_pitch_rate;
extern float target_yaw_rate;
extern float yaw_angle_error_deg;

typedef struct
{
    float roll_pid_rate_dps;
    float pitch_pid_rate_dps;
    float roll_rate_ff_dps;
    float pitch_rate_ff_dps;
    float roll_rate_total_raw_dps;
    float pitch_rate_total_raw_dps;
    float roll_rate_final_dps;
    float pitch_rate_final_dps;
} AngleLoopRateFeedforwardDiag_t;

float Angle_WrapDeg180(float angle_deg);
void Angle_Loop_InitTiming(void);
void Angle_Loop_Set_Rate_Feedforward(float roll_dps, float pitch_dps);
void Angle_Loop_Reset_Rate_Feedforward(void);
void Angle_Loop_Get_Rate_Feedforward_Diag(AngleLoopRateFeedforwardDiag_t *diag);
void Angle_Loop_Reset_Rate_Cmd_Lpf(void);
void Angle_Loop_Update(float target_roll,
                       float target_pitch,
                       float target_yaw,
                       float current_roll,
                       float current_pitch,
                       float current_yaw);

#endif
