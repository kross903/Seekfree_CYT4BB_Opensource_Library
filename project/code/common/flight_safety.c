#include "common/flight_safety.h"
#include "common/battery_monitor.h"
#include "common/PIT_Demo.h"
#include "control/flight_control.h"
#include "icm/icm.h"
#include "menu/menu_fly_test.h"
#include "menu/menu_motor_test.h"
#include "motor_hardware/motor_hardware.h"
#include "motor_hardware/small_driver_uart_control.h"
#include <math.h>
#include <stdio.h>

volatile uint8 emergency_stop_triggered = 0U;
volatile uint8 emergency_stop_reason = EMERGENCY_STOP_REASON_NONE;
int low_voltage_protect_flag = 0;

#define FLIGHT_SAFETY_TICK_MS                 5U
#define FLIGHT_ESTOP_REARM_INHIBIT_TICKS      (FLIGHT_ESTOP_REARM_INHIBIT_MS / FLIGHT_SAFETY_TICK_MS)
#define FLIGHT_CONTROL_TICK_US                 250U
#define YAW_SPIN_RATE_LIMIT_DPS                200.0f
#define YAW_SPIN_DURATION_MS                   150U
#define YAW_SPIN_LIMIT_TICKS                   ((YAW_SPIN_DURATION_MS * 1000U) / FLIGHT_CONTROL_TICK_US)
#define ESC_REPORT_HOLD_MS                     5000U
#define ESC_REPORT_HOLD_TICKS                  (ESC_REPORT_HOLD_MS / FLIGHT_SAFETY_TICK_MS)

static volatile uint16 s_rearm_inhibit_ticks = 0U;
static uint16 s_yaw_spin_ticks = 0U;
static char s_esc_active_text[32] = "ESC FAULT";
static char s_esc_report_text[32] = "ESC FAULT";
static volatile uint16 s_esc_report_ticks = 0U;

static const char *flight_safety_esc_fault_name(uint8 fault_code)
{
    switch((SmallDriverEscFaultCode)fault_code)
    {
        case SMALL_DRIVER_ESC_FAULT_VOLTAGE:          return "VOLTAGE";
        case SMALL_DRIVER_ESC_FAULT_LOCKED:           return "LOCKED";
        case SMALL_DRIVER_ESC_FAULT_DISTURB:          return "DISTURB";
        case SMALL_DRIVER_ESC_FAULT_START_FAIL:       return "START FAIL";
        case SMALL_DRIVER_ESC_FAULT_TEMPERATURE_LOW:  return "TEMP LOW";
        case SMALL_DRIVER_ESC_FAULT_TEMPERATURE_HIGH: return "TEMP HIGH";
        case SMALL_DRIVER_ESC_FAULT_OVER_CURRENT:     return "OVERCURRENT";
        case SMALL_DRIVER_ESC_FAULT_LOW_SPEED_RESTART:return "LOW SPEED";
        case SMALL_DRIVER_ESC_FAULT_NONE:
        default:                                      return "UNKNOWN";
    }
}

static uint8 flight_safety_first_motor(uint8 motor_mask)
{
    uint8 i;

    for(i = 0U; i < 4U; i++)
    {
        if((motor_mask & (uint8)(1U << i)) != 0U)
        {
            return i;
        }
    }
    return 0U;
}

static void flight_safety_format_esc_fault(char *text,
                                           uint32 text_size,
                                           uint8 fault_code,
                                           uint8 motor_mask)
{
    const char *name = flight_safety_esc_fault_name(fault_code);
    uint8 board_fault = (uint8)(fault_code == SMALL_DRIVER_ESC_FAULT_VOLTAGE ||
                                fault_code == SMALL_DRIVER_ESC_FAULT_TEMPERATURE_LOW ||
                                fault_code == SMALL_DRIVER_ESC_FAULT_TEMPERATURE_HIGH ||
                                fault_code == SMALL_DRIVER_ESC_FAULT_OVER_CURRENT);

    if(board_fault)
    {
        snprintf(text, text_size, "ESC BOARD %s", name);
    }
    else if((motor_mask & (uint8)(motor_mask - 1U)) == 0U && motor_mask != 0U)
    {
        snprintf(text,
                 text_size,
                 "ESC M%u %s",
                 (unsigned int)(flight_safety_first_motor(motor_mask) + 1U),
                 name);
    }
    else
    {
        snprintf(text, text_size, "ESC MULTI %s", name);
    }
}

const char *Flight_Safety_GetEscFaultText(void)
{
    SmallDriverEscStatus status;
    uint8 first_code = 0U;
    uint8 motor_mask = 0U;
    uint8 i;

    small_driver_get_esc_status(&status);
    for(i = 0U; i < 4U; i++)
    {
        if(status.active_fault[i] != 0U)
        {
            first_code = status.active_fault[i];
            break;
        }
    }
    if(first_code != 0U)
    {
        for(i = 0U; i < 4U; i++)
        {
            if(status.active_fault[i] == first_code)
            {
                motor_mask |= (uint8)(1U << i);
            }
        }
        flight_safety_format_esc_fault(s_esc_active_text,
                                       sizeof(s_esc_active_text),
                                       first_code,
                                       motor_mask);
        return s_esc_active_text;
    }
    return NULL;
}

const char *Flight_Safety_GetEscReportText(void)
{
    const char *active_fault = Flight_Safety_GetEscFaultText();

    if(active_fault != NULL)
    {
        return active_fault;
    }
    return (s_esc_report_ticks > 0U) ? s_esc_report_text : NULL;
}

uint8 Flight_Safety_Check_Esc(void)
{
    SmallDriverEscFaultEvent event;

    if(!small_driver_take_fault_event(&event))
    {
        return 0U;
    }

    /* 独立锁存报告文本，急停诊断刷新不能覆盖瞬时电调故障。 */
    flight_safety_format_esc_fault(s_esc_report_text,
                                   sizeof(s_esc_report_text),
                                   event.fault_code,
                                   event.motor_mask);
    s_esc_report_ticks = ESC_REPORT_HOLD_TICKS;
    return 0U;
}

void Flight_Safety_ResetYawSpinMonitor(void)
{
    s_yaw_spin_ticks = 0U;
}

uint8 Flight_Safety_Check_Attitude(void)
{
    if(!armed)
    {
        Flight_Safety_ResetYawSpinMonitor();
        return 0U;
    }

    if(!isfinite(current_roll) || !isfinite(current_pitch) || !isfinite(current_yaw))
    {
        Flight_Runtime_Emergency_StopAt(EMERGENCY_STOP_REASON_INVALID_CONTROL,
                                        "ATTITUDE NAN/INF",
                                        __FILE__,
                                        __LINE__);
        return 1U;
    }
    if(!isfinite(current_gyro_roll) || !isfinite(current_gyro_pitch) || !isfinite(current_gyro_yaw))
    {
        Flight_Runtime_Emergency_StopAt(EMERGENCY_STOP_REASON_INVALID_CONTROL,
                                        "GYRO NAN/INF",
                                        __FILE__,
                                        __LINE__);
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
    if(fabsf(current_gyro_yaw) > YAW_SPIN_RATE_LIMIT_DPS)
    {
        if(s_yaw_spin_ticks < YAW_SPIN_LIMIT_TICKS)
        {
            s_yaw_spin_ticks++;
        }
        if(s_yaw_spin_ticks >= YAW_SPIN_LIMIT_TICKS)
        {
            Flight_Runtime_Emergency_Stop(EMERGENCY_STOP_REASON_YAW_SPIN);
            return 1U;
        }
    }
    else
    {
        Flight_Safety_ResetYawSpinMonitor();
    }
    return 0U;
}

uint8 Flight_Safety_Can_Arm(FlightArmRejectReason *reason)
{
    FlightArmRejectReason reject_reason = FLIGHT_ARM_REJECT_NONE;

    if(s_rearm_inhibit_ticks > 0U)
    {
        reject_reason = FLIGHT_ARM_REJECT_REARM_INHIBIT;
    }
    else if(emergency_stop_triggered)
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
    else if(!ICM_Health_IsArmable())
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
    else if(small_driver_esc_has_active_fault())
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

void Flight_Safety_Tick_5ms(void)
{
    small_driver_tick_5ms();
    if(s_rearm_inhibit_ticks > 0U)
    {
        s_rearm_inhibit_ticks--;
    }
    if(s_esc_report_ticks > 0U)
    {
        s_esc_report_ticks--;
    }
}

uint16 Flight_Safety_GetRearmInhibitRemainingMs(void)
{
    return (uint16)(s_rearm_inhibit_ticks * FLIGHT_SAFETY_TICK_MS);
}

static const char *flight_safety_estop_reason_text(uint8 reason)
{
    switch(reason)
    {
        case EMERGENCY_STOP_REASON_ATTITUDE_ROLL:   return "ESTOP ROLL LIMIT";
        case EMERGENCY_STOP_REASON_ATTITUDE_PITCH:  return "ESTOP PITCH LIMIT";
        case EMERGENCY_STOP_REASON_MANUAL_BUTTON:   return "ESTOP BUTTON";
        case EMERGENCY_STOP_REASON_LINK_TIMEOUT:    return "ESTOP LINK TIMEOUT";
        case EMERGENCY_STOP_REASON_INVALID_CONTROL: return "ESTOP INVALID CTRL";
        case EMERGENCY_STOP_REASON_TAKEOFF_TOF:     return "ESTOP TAKEOFF TOF";
        case EMERGENCY_STOP_REASON_YAW_SPIN:        return "ESTOP YAW SPIN";
        case EMERGENCY_STOP_REASON_ESC_FAULT:       return "ESTOP ESC FAULT";
        case EMERGENCY_STOP_REASON_NONE:
        default:                                    return "ESTOP UNKNOWN";
    }
}

void Flight_Runtime_Emergency_StopAt(uint8 reason,
                                     const char *detail,
                                     const char *file,
                                     uint32 line)
{
    const char *stop_text = (detail != NULL) ? detail : flight_safety_estop_reason_text(reason);

    armed = 0U;
    emergency_stop_triggered = 1U;
    emergency_stop_reason = reason;
    if(reason == EMERGENCY_STOP_REASON_MANUAL_BUTTON ||
       reason == EMERGENCY_STOP_REASON_YAW_SPIN ||
       reason == EMERGENCY_STOP_REASON_ESC_FAULT)
    {
        s_rearm_inhibit_ticks = FLIGHT_ESTOP_REARM_INHIBIT_TICKS;
    }
    Flight_Safety_ResetYawSpinMonitor();
    MenuMotorTest_Stop();
    MenuFlyTest_Stop();
    target_throttle = 0.0f;
    current_throttle = 0.0f;
    base_throttle = 0.0f;
    Flight_Takeoff_RecordStopAt(stop_text, file, line);
    Flight_Takeoff_AbortAt(stop_text, file, line);
    Velocity_Loop_Set_Enabled(0U);
    Position_Loop_Set_Enabled(0U);
    PIT_Demo_Safety_Reset_Angle_Rate_Cmd_Lpf();
    PIT_Demo_Safety_Clear_Motor_Throttle_State();
    MotorHardware_ForceEmergencyStop();
}

void Flight_Runtime_Emergency_Stop(uint8 reason)
{
    Flight_Runtime_Emergency_StopAt(reason, NULL, __FILE__, __LINE__);
}

uint8 Flight_Runtime_Clear_Emergency_Stop_Latch(void)
{
    if(s_rearm_inhibit_ticks > 0U)
    {
        return 0U;
    }
    if(emergency_stop_reason == EMERGENCY_STOP_REASON_ESC_FAULT &&
       small_driver_esc_has_active_fault())
    {
        return 0U;
    }

    emergency_stop_triggered = 0U;
    emergency_stop_reason = EMERGENCY_STOP_REASON_NONE;
    Flight_Safety_ResetYawSpinMonitor();
    return 1U;
}
