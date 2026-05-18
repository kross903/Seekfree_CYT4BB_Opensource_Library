#ifndef FLIGHT_SAFETY_H_
#define FLIGHT_SAFETY_H_

#include "zf_common_headfile.h"

#define SAFE_MAX_ANGLE_ROLL     30.0f
#define SAFE_MAX_ANGLE_PITCH    30.0f
#define SAFE_MAX_ANGLE_YAW      180.0f

#define BATTERY_ADC_TO_VOLTAGE_SCALE   0.00820715f
#define BATTERY_ADC_CHANNEL            ADC1_CH24_P14_4
#define BATTERY_SERIES_CELLS           3.0f
#define BATTERY_LOW_CELL_WARN_V        3.767f
#define BATTERY_LOW_CELL_RECOVER_V     3.833f
#define BATTERY_LOW_WARN_THRESHOLD     (BATTERY_SERIES_CELLS * BATTERY_LOW_CELL_WARN_V)
#define BATTERY_LOW_RECOVER_THRESHOLD  (BATTERY_SERIES_CELLS * BATTERY_LOW_CELL_RECOVER_V)

#define EMERGENCY_STOP_REASON_NONE            0
#define EMERGENCY_STOP_REASON_ATTITUDE_ROLL   1
#define EMERGENCY_STOP_REASON_ATTITUDE_PITCH  2
#define EMERGENCY_STOP_REASON_MANUAL_BUTTON   3
#define EMERGENCY_STOP_REASON_LINK_TIMEOUT    4

extern volatile uint8 emergency_stop_triggered;
extern volatile uint8 emergency_stop_reason;
extern int low_voltage_protect_flag;

uint8 Flight_Safety_Check_Attitude(void);
void Flight_Safety_Update_Low_Voltage(float car_voltage_v);
void low_voltage_check(void);

void Flight_Runtime_Emergency_Stop(uint8 reason);
void Flight_Runtime_Clear_Emergency_Stop_Latch(void);

void PIT_Demo_Safety_Reset_Angle_Rate_Cmd_Lpf(void);
void PIT_Demo_Safety_Clear_Motor_Throttle_State(void);

#endif
