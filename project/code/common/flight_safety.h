#ifndef FLIGHT_SAFETY_H_
#define FLIGHT_SAFETY_H_

#include "zf_common_headfile.h"

#define SAFE_MAX_ANGLE_ROLL     35.0f
#define SAFE_MAX_ANGLE_PITCH    35.0f
#define SAFE_MAX_ANGLE_YAW      180.0f

#define EMERGENCY_STOP_REASON_NONE            0
#define EMERGENCY_STOP_REASON_ATTITUDE_ROLL   1
#define EMERGENCY_STOP_REASON_ATTITUDE_PITCH  2
#define EMERGENCY_STOP_REASON_MANUAL_BUTTON   3
#define EMERGENCY_STOP_REASON_LINK_TIMEOUT    4
#define EMERGENCY_STOP_REASON_INVALID_CONTROL 5
#define EMERGENCY_STOP_REASON_TAKEOFF_TOF     6
#define EMERGENCY_STOP_REASON_YAW_SPIN        7
#define EMERGENCY_STOP_REASON_ESC_FAULT       8
#define EMERGENCY_STOP_REASON_BATTERY_INVALID 9
#define EMERGENCY_STOP_REASON_FLOW_TIMEOUT    10

#define FLIGHT_ESTOP_REARM_INHIBIT_MS          2000U

typedef enum
{
    FLIGHT_ARM_REJECT_NONE = 0,
    FLIGHT_ARM_REJECT_ESTOP,
    FLIGHT_ARM_REJECT_IMU_CALIBRATING,
    FLIGHT_ARM_REJECT_INVALID_SENSOR,
    FLIGHT_ARM_REJECT_ATTITUDE,
    FLIGHT_ARM_REJECT_MOTOR_NOT_READY,
    FLIGHT_ARM_REJECT_BATTERY_INVALID,
    FLIGHT_ARM_REJECT_REARM_INHIBIT,
    FLIGHT_ARM_REJECT_BATTERY_LOW
} FlightArmRejectReason;

extern volatile uint8 emergency_stop_triggered;
extern volatile uint8 emergency_stop_reason;
extern int low_voltage_protect_flag;

uint8 Flight_Safety_Check_Attitude(void);
uint8 Flight_Safety_Check_Esc(void);
uint8 Flight_Safety_Check_Battery(void);
uint8 Flight_Safety_Can_Arm(FlightArmRejectReason *reason);
void Flight_Safety_Update_Battery_Warning(void);
void Flight_Safety_Tick_5ms(void);
uint16 Flight_Safety_GetRearmInhibitRemainingMs(void);
void Flight_Safety_ResetYawSpinMonitor(void);
const char *Flight_Safety_GetEscFaultText(void);
const char *Flight_Safety_GetEscReportText(void);
void Flight_Safety_ClearEscReportLatch(void);

void Flight_Runtime_Emergency_Stop(uint8 reason);
void Flight_Runtime_Emergency_StopAt(uint8 reason,
                                     const char *detail,
                                     const char *file,
                                     uint32 line);
uint8 Flight_Runtime_Clear_Emergency_Stop_Latch(void);

void PIT_Demo_Safety_Reset_Angle_Rate_Cmd_Lpf(void);
void PIT_Demo_Safety_Clear_Motor_Throttle_State(void);

#endif
