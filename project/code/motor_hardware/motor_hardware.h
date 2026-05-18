#ifndef MOTOR_HARDWARE_H_
#define MOTOR_HARDWARE_H_

#include "zf_common_headfile.h"

void MotorHardware_InitOnce(void);
void MotorHardware_WriteOutput(const volatile uint16 motor_throttle[4],
                               uint8 armed,
                               uint8 output_enabled,
                               uint8 bypass_compensation);

#endif
