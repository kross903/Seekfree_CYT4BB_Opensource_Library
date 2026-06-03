#ifndef RATE_LOOP_H_
#define RATE_LOOP_H_

#include "zf_common_headfile.h"
#include "control/pid_controller.h"

extern PID_Controller gyro_pid_roll;
extern PID_Controller gyro_pid_pitch;
extern PID_Controller gyro_pid_yaw;

extern float roll_output;
extern float pitch_output;
extern float yaw_output;

typedef struct
{
    float roll_raw_gyro;
    float roll_filtered_gyro;
    float roll_raw_derivative;
    float roll_d_output;
    float pitch_raw_gyro;
    float pitch_filtered_gyro;
    float pitch_raw_derivative;
    float pitch_d_output;
} RateLoopDTermDiag_t;

void Rate_Loop_Reset_Output(void);
void Rate_Loop_Reset_DTerm(void);
void Rate_Loop_Reset_DTerm_Axis(uint8 axis);
void Rate_Loop_Get_DTerm_Diag(RateLoopDTermDiag_t *diag);
void Rate_Loop_Update(float target_roll_rate,
                      float target_pitch_rate,
                      float target_yaw_rate,
                      float current_gyro_roll,
                      float current_gyro_pitch,
                      float current_gyro_yaw);

#endif
