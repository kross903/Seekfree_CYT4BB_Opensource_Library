#include "control/rate_loop.h"

#define RATE_LOOP_DT_S                  0.001f

PID_Controller gyro_pid_roll;
PID_Controller gyro_pid_pitch;
PID_Controller gyro_pid_yaw;

float roll_output = 0.0f;
float pitch_output = 0.0f;
float yaw_output = 0.0f;

void Rate_Loop_Reset_Output(void)
{
    roll_output = 0.0f;
    pitch_output = 0.0f;
    yaw_output = 0.0f;
}

void Rate_Loop_Update(float target_roll_rate,
                      float target_pitch_rate,
                      float target_yaw_rate,
                      float current_gyro_roll,
                      float current_gyro_pitch,
                      float current_gyro_yaw)
{
    roll_output = PID_Calculate(&gyro_pid_roll, target_roll_rate, current_gyro_roll, RATE_LOOP_DT_S);
    pitch_output = PID_Calculate(&gyro_pid_pitch, target_pitch_rate, current_gyro_pitch, RATE_LOOP_DT_S);
    yaw_output = -PID_Calculate(&gyro_pid_yaw, target_yaw_rate, current_gyro_yaw, RATE_LOOP_DT_S);
}
