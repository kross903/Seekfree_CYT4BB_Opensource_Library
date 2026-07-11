#include "common/flight_safety.h"
#include "common/battery_monitor.h"
#include "common/PIT_Demo.h"
#include "control/flight_control.h"
#include "icm/icm42688.h"
#include "menu/menu_fly_test.h"
#include "menu/menu_motor_test.h"
#include "motor_hardware/motor_hardware.h"
#include "motor_hardware/small_driver_uart_control.h"
#include <math.h>

volatile uint8 emergency_stop_triggered = 0U;
volatile uint8 emergency_stop_reason = EMERGENCY_STOP_REASON_NONE;
int low_voltage_protect_flag = 0;

uint8 Flight_Safety_Check_Attitude(void)
{
    if(!armed)
    {
        return 0U;
    }

    if(!isfinite(current_roll) || !isfinite(current_pitch) || !isfinite(current_yaw) ||
       !isfinite(current_gyro_roll) || !isfinite(current_gyro_pitch) || !isfinite(current_gyro_yaw))
    {
        Flight_Runtime_Emergency_Stop(EMERGENCY_STOP_REASON_INVALID_CONTROL);
        return 1U;
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

uint8 Flight_Safety_Can_Arm(FlightArmRejectReason *reason)
{
    FlightArmRejectReason reject_reason = FLIGHT_ARM_REJECT_NONE;

    if(emergency_stop_triggered)
    {
        reject_reason = FLIGHT_ARM_REJECT_ESTOP;
    }
    else if(icm_calibration_flag)
    {
        reject_reason = FLIGHT_ARM_REJECT_IMU_CALIBRATING;
    }
    else if(!isfinite(current_roll) || !isfinite(current_pitch) || !isfinite(current_yaw) ||
            !isfinite(current_gyro_roll) || !isfinite(current_gyro_pitch) || !isfinite(current_gyro_yaw))
    {
        reject_reason = FLIGHT_ARM_REJECT_INVALID_SENSOR;
    }
    else if((current_roll > SAFE_MAX_ANGLE_ROLL) || (current_roll < -SAFE_MAX_ANGLE_ROLL) ||
            (current_pitch > SAFE_MAX_ANGLE_PITCH) || (current_pitch < -SAFE_MAX_ANGLE_PITCH))
    {
        reject_reason = FLIGHT_ARM_REJECT_ATTITUDE;
    }
    else if(!MotorHardware_IsInitialized())
    {
        reject_reason = FLIGHT_ARM_REJECT_MOTOR_NOT_READY;
    }

    if(reason != NULL)
    {
        *reason = reject_reason;
    }
    return (uint8)(reject_reason == FLIGHT_ARM_REJECT_NONE);
}

void Flight_Safety_Update_Battery_Warning(void)
{
    BatteryStatus status;

    BatteryMonitor_GetStatus(&status);
    low_voltage_protect_flag = status.low_warning ? 1 : 0;
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
    small_driver_force_zero();
}

void Flight_Runtime_Clear_Emergency_Stop_Latch(void)
{
    emergency_stop_triggered = 0U;
    emergency_stop_reason = EMERGENCY_STOP_REASON_NONE;
}
