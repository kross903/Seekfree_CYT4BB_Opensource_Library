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

void Angle_Loop_Reset_Rate_Cmd_Lpf(void);
void Angle_Loop_Update(float target_roll,
                       float target_pitch,
                       float target_yaw,
                       float current_roll,
                       float current_pitch,
                       float current_yaw);

#endif
