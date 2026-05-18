#ifndef BATTERY_MONITOR_H_
#define BATTERY_MONITOR_H_

#include "zf_common_headfile.h"

#define BATTERY_NOMINAL_VOLTAGE        12.4f

extern float voltage;
extern uint16 adc_value;
extern volatile float car_bat_voltage;

void BatteryMonitor_Task_5ms(void);

#endif
