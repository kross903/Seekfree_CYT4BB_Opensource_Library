#include "common/battery_monitor.h"

#include "common/flight_safety.h"

float voltage = BATTERY_NOMINAL_VOLTAGE;
uint16 adc_value = 0;
volatile float car_bat_voltage = 0.0f;

void BatteryMonitor_Task_5ms(void)
{
    adc_value = adc_mean_filter_convert(BATTERY_ADC_CHANNEL, 10);
    voltage = adc_value * BATTERY_ADC_TO_VOLTAGE_SCALE;

    Flight_Safety_Update_Low_Voltage(car_bat_voltage);
}
