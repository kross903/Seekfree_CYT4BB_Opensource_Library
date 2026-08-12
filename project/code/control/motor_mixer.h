#ifndef MOTOR_MIXER_H_
#define MOTOR_MIXER_H_

#include "zf_common_headfile.h"

/*
 * 电机布局（从机体上方向下俯视，机头朝 +X）：
 *
 *                         机头 (+X)
 *                            ^
 *          左前 M3 (CW)             右前 M2 (CCW)
 *                    \             /
 *                     \   机身    /
 *                     /           \    机臂
 *          左后 M4 (CCW)            右后 M1 (CW)
 *
 * M1~M4 分别对应 motor_throttle[0]~motor_throttle[3]。
 * CW/CCW 表示俯视时桨叶的顺/逆时针旋转方向，实际旋向需拆桨点动确认。
 */
extern float motor_output[4];

typedef struct
{
    uint8 high;
    uint8 low;
    uint8 roll_limited;
    uint8 pitch_limited;
} MotorMixerSaturation_t;

#define MOTOR_MIXER_LIMIT_POSITIVE  (0x01U)
#define MOTOR_MIXER_LIMIT_NEGATIVE  (0x02U)

typedef struct
{
    float raw_motor[4];
    uint16 clamped_motor[4];
    uint8 high_mask;
    uint8 low_mask;
    uint8 roll_limit_flags;
    uint8 pitch_limit_flags;
    uint8 yaw_limit_flags;
    uint8 throttle_limit_flags;
    uint8 invalid_output;
} MotorMixerLimitDiag_t;

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
void Motor_Mixer_Get_Limit_Diag(MotorMixerLimitDiag_t *diag);

#endif
