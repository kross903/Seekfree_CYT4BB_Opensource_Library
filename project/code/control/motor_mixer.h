#ifndef MOTOR_MIXER_H_
#define MOTOR_MIXER_H_

#include "zf_common_headfile.h"

extern float motor_output[4];

typedef struct
{
    uint8 high;
    uint8 low;
    uint8 roll_limited;
    uint8 pitch_limited;
} MotorMixerSaturation_t;

uint16 Motor_Mixer_Limit_Output(float output);
void Motor_Mixer_Update(volatile uint16 motor_throttle_out[4],
                        uint8 armed_state,
                        float base_throttle_input,
                        float roll_deg,
                        float pitch_deg);
void Motor_Mixer_Update_BaseOnly(volatile uint16 motor_throttle_out[4],
                                 uint8 armed_state,
                                 float base_throttle_input,
                                 float roll_deg,
                                 float pitch_deg);
uint8 Motor_Mixer_HasInvalidOutput(void);
void Motor_Mixer_Get_Saturation(MotorMixerSaturation_t *saturation);

#endif
