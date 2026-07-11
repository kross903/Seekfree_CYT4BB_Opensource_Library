#ifndef BATTERY_MONITOR_H_
#define BATTERY_MONITOR_H_

#include "zf_common_headfile.h"

#define BATTERY_ADC_TO_VOLTAGE_SCALE       0.00820715f
#define BATTERY_ADC_CHANNEL                ADC1_CH24_P14_4

#define BATTERY_SERIES_CELLS               4.0f
#define BATTERY_NOMINAL_VOLTAGE            14.8f
#define BATTERY_COMP_REFERENCE_V           15.2f
#define BATTERY_COMP_GAIN_MIN              0.90f
#define BATTERY_COMP_GAIN_MAX              1.15f
#define BATTERY_LOW_CELL_WARN_V            3.70f
#define BATTERY_LOW_CELL_RECOVER_V         3.80f
#define BATTERY_LOW_WARN_THRESHOLD         (BATTERY_SERIES_CELLS * BATTERY_LOW_CELL_WARN_V)
#define BATTERY_LOW_RECOVER_THRESHOLD      (BATTERY_SERIES_CELLS * BATTERY_LOW_CELL_RECOVER_V)

typedef struct
{
    uint16 adc_raw;
    float local_raw_voltage_v;
    float local_filtered_voltage_v;
    float remote_voltage_v;
    float compensation_gain;
    uint16 remote_age_ms;
    uint8 local_valid;
    uint8 remote_valid;
    uint8 low_warning;
} BatteryStatus;

extern float voltage;
extern uint16 adc_value;
extern volatile float car_bat_voltage;

void BatteryMonitor_Task_5ms(void);
void BatteryMonitor_SetRemoteVoltage(float voltage_v);
void BatteryMonitor_GetStatus(BatteryStatus *status);
float BatteryMonitor_GetCompensationGain(void);

#endif
