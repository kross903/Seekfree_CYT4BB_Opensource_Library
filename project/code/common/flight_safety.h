#ifndef FLIGHT_SAFETY_H_
#define FLIGHT_SAFETY_H_

#include "zf_common_headfile.h"

#define SAFE_MAX_ANGLE_ROLL     30.0f
#define SAFE_MAX_ANGLE_PITCH    30.0f
#define SAFE_MAX_ANGLE_YAW      180.0f

#define EMERGENCY_STOP_REASON_NONE            0
#define EMERGENCY_STOP_REASON_ATTITUDE_ROLL   1
#define EMERGENCY_STOP_REASON_ATTITUDE_PITCH  2
#define EMERGENCY_STOP_REASON_MANUAL_BUTTON   3
#define EMERGENCY_STOP_REASON_LINK_TIMEOUT    4
#define EMERGENCY_STOP_REASON_INVALID_CONTROL 5

typedef enum
{
    FLIGHT_ARM_REJECT_NONE = 0,
    FLIGHT_ARM_REJECT_ESTOP,
    FLIGHT_ARM_REJECT_IMU_CALIBRATING,
    FLIGHT_ARM_REJECT_INVALID_SENSOR,
    FLIGHT_ARM_REJECT_ATTITUDE,
    FLIGHT_ARM_REJECT_MOTOR_NOT_READY
} FlightArmRejectReason;

extern volatile uint8 emergency_stop_triggered;
extern volatile uint8 emergency_stop_reason;
extern int low_voltage_protect_flag;

uint8 Flight_Safety_Check_Attitude(void);
uint8 Flight_Safety_Can_Arm(FlightArmRejectReason *reason);
void Flight_Safety_Update_Battery_Warning(void);

void Flight_Runtime_Emergency_Stop(uint8 reason);
void Flight_Runtime_Clear_Emergency_Stop_Latch(void);

void PIT_Demo_Safety_Reset_Angle_Rate_Cmd_Lpf(void);
void PIT_Demo_Safety_Clear_Motor_Throttle_State(void);

#endif
