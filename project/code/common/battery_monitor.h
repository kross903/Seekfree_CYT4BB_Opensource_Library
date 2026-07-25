#ifndef BATTERY_MONITOR_H_
#define BATTERY_MONITOR_H_

#include "zf_common_headfile.h"

/* E1 母板官方电池分压检测：P07_5，12 位 ADC 满量程对应约 36.3 V。 */
#define BATTERY_ADC_TO_VOLTAGE_SCALE       (36.3f / 4096.0f)
#define BATTERY_ADC_CHANNEL                ADC0_CH21_P07_5

#define BATTERY_COMP_REFERENCE_V           15.2f
#define BATTERY_COMP_GAIN_SCALE            0.94f
#define BATTERY_COMP_GAIN_MIN              0.55f
#define BATTERY_LOW_CELL_WARN_V            3.70f
#define BATTERY_LOW_CELL_RECOVER_V         3.80f

#define BATTERY_4S_DETECT_MIN_V             12.0f
#define BATTERY_4S_DETECT_MAX_V             17.2f
#define BATTERY_6S_DETECT_MIN_V             18.0f
#define BATTERY_6S_DETECT_MAX_V             25.8f

typedef enum
{
    BATTERY_PACK_UNKNOWN = 0,
    BATTERY_PACK_4S = 4,
    BATTERY_PACK_6S = 6
} BatteryPackType;

typedef struct
{
    uint16 adc_raw;
    float local_raw_voltage_v;
    float local_filtered_voltage_v;
    float remote_voltage_v;
    float compensation_gain;
    float output_voltage_v;
    float low_warn_voltage_v;
    float low_recover_voltage_v;
    uint16 remote_age_ms;
    BatteryPackType pack_type;
    uint8 local_valid;
    uint8 remote_valid;
    uint8 low_warning;
    uint8 invalid_warning;
} BatteryStatus;

extern float voltage;
extern uint16 adc_value;
extern volatile float car_bat_voltage;

void BatteryMonitor_Task_5ms(void);
void BatteryMonitor_SetRemoteVoltage(float voltage_v);
void BatteryMonitor_GetStatus(BatteryStatus *status);
float BatteryMonitor_GetCompensationGain(void);

#endif
