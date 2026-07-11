#ifndef MOTOR_HARDWARE_H_
#define MOTOR_HARDWARE_H_

#include "zf_common_headfile.h"

typedef struct
{
    float raw_output[4];
    float compensated_output[4];
    float compensation_gain;
    uint8 saturation_mask;
    uint8 bypassed;
    uint8 invalid_output;
} MotorHardwareVoltageCompDiag;

void MotorHardware_InitOnce(void);
void MotorHardware_WriteOutput(const volatile uint16 motor_throttle[4],
                               uint8 armed,
                               uint8 output_enabled,
                               uint8 bypass_compensation);
void MotorHardware_GetVoltageCompDiag(MotorHardwareVoltageCompDiag *diag);
uint8 MotorHardware_IsInitialized(void);

#endif
