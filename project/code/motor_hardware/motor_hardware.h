#ifndef MOTOR_HARDWARE_H_
#define MOTOR_HARDWARE_H_

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
typedef struct
{
    float raw_output[4];
    float compensated_output[4];
    float compensation_gain;
    float hard_output_limit;
    uint8 saturation_mask;
    uint8 bypassed;
    uint8 invalid_output;
    uint8 battery_pack;
    uint8 battery_fallback_active;
} MotorHardwareVoltageCompDiag;

void MotorHardware_InitOnce(void);
void MotorHardware_WriteOutput(const volatile uint16 motor_throttle[4],
                               uint8 armed,
                               uint8 output_enabled,
                               uint8 bypass_motor_trim);
void MotorHardware_ForceEmergencyStop(void);
void MotorHardware_ServiceEmergencyStop(void);
void MotorHardware_GetVoltageCompDiag(MotorHardwareVoltageCompDiag *diag);
uint8 MotorHardware_IsInitialized(void);

#endif
