#include "common/flight_safety.h"
#include "common/PIT_Demo.h"
#include "control/flight_control.h"
#include "menu/menu_fly_test.h"
#include "menu/menu_motor_test.h"

volatile uint8 emergency_stop_triggered = 0U;
volatile uint8 emergency_stop_reason = EMERGENCY_STOP_REASON_NONE;
int low_voltage_protect_flag = 0;

uint8 Flight_Safety_Check_Attitude(void)
{
    if(!armed)
    {
        return 0U;
    }

    if(current_roll > SAFE_MAX_ANGLE_ROLL || current_roll < -SAFE_MAX_ANGLE_ROLL)
    {
        Flight_Runtime_Emergency_Stop(EMERGENCY_STOP_REASON_ATTITUDE_ROLL);
        return 1U;
    }
    if(current_pitch > SAFE_MAX_ANGLE_PITCH || current_pitch < -SAFE_MAX_ANGLE_PITCH)
    {
        Flight_Runtime_Emergency_Stop(EMERGENCY_STOP_REASON_ATTITUDE_PITCH);
        return 1U;
    }
    return 0U;
}

void Flight_Safety_Update_Low_Voltage(float car_voltage_v)
{
    if(car_voltage_v > 0.0f)
    {
        if(car_voltage_v < BATTERY_LOW_WARN_THRESHOLD)
        {
            low_voltage_protect_flag = 1;
        }
        else if(car_voltage_v >= BATTERY_LOW_RECOVER_THRESHOLD)
        {
            low_voltage_protect_flag = 0;
        }
    }
}

void low_voltage_check(void)
{
    Flight_Safety_Update_Low_Voltage(car_bat_voltage);
}

void Flight_Runtime_Emergency_Stop(uint8 reason)
{
    armed = 0U;
    emergency_stop_triggered = 1U;
    emergency_stop_reason = reason;
    MenuMotorTest_Stop();
    MenuFlyTest_Stop();
    target_throttle = 0.0f;
    current_throttle = 0.0f;
    base_throttle = 0.0f;
    Flight_Takeoff_Abort();
    Velocity_Loop_Set_Enabled(0U);
    Position_Loop_Set_Enabled(0U);
    PIT_Demo_Safety_Reset_Angle_Rate_Cmd_Lpf();
    PIT_Demo_Safety_Clear_Motor_Throttle_State();
}

void Flight_Runtime_Clear_Emergency_Stop_Latch(void)
{
    emergency_stop_triggered = 0U;
    emergency_stop_reason = EMERGENCY_STOP_REASON_NONE;
}
