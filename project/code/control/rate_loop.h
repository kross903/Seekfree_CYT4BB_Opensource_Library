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

/* 保存同一次角速度 PID 计算的分项，Yaw 分项已转换为实际混控方向。 */
typedef struct
{
    float target_rate;
    float actual_rate;
    float error;
    float p_output;
    float i_output;
    float d_output;
    float raw_output;
    float output;
    uint8 limit_flags;
    uint8 integrator_held;
    uint8 saturated;
} RateLoopDiag_t;

void Rate_Loop_Reset_Output(void);
void Rate_Loop_Update(float target_roll_rate,
                      float target_pitch_rate,
                      float target_yaw_rate,
                      float current_gyro_roll,
                      float current_gyro_pitch,
                      float current_gyro_yaw);
/* axis: 0=Roll，1=Pitch，2=Yaw。 */
uint8 Rate_Loop_Get_Diag(uint8 axis, RateLoopDiag_t *diag);

#endif
