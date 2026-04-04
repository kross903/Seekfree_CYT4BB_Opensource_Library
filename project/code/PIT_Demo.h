#ifndef _PIT_Demo_h_
#define _PIT_Demo_h_

#include "zf_common_headfile.h"
#include "pid_controller.h"

/*
 * PIT_Demo 对外主要暴露两类内容:
 * 1. ????????????????????
 * 2. ??????????????????????
 */

// ==================== 安全限制参数 ====================
#define SAFE_MAX_THROTTLE       1600.0f
#define SAFE_MIN_THROTTLE       100.0f
#define SAFE_MAX_MOTOR_OUTPUT   1600

#define SAFE_MAX_ANGLE_ROLL     20.0f
#define SAFE_MAX_ANGLE_PITCH    20.0f
#define SAFE_MAX_ANGLE_YAW      180.0f

/*
 * [2026-03-10]
 * 修改目的:
 * 1. ??????????????????????????????????????????
 * 2. ??????? 6S ??????????????????????????????????
 *    ??????? BATTERY_ADC_TO_VOLTAGE_SCALE ??????
 */
#define BATTERY_ADC_TO_VOLTAGE_SCALE   0.008753698f
#define BATTERY_ADC_CHANNEL            ADC1_CH24_P14_4
#define BATTERY_SERIES_CELLS           6.0f
#define BATTERY_LOW_CELL_WARN_V        3.55f
#define BATTERY_LOW_CELL_RECOVER_V     3.65f
#define BATTERY_LOW_WARN_THRESHOLD     (BATTERY_SERIES_CELLS * BATTERY_LOW_CELL_WARN_V)
#define BATTERY_LOW_RECOVER_THRESHOLD  (BATTERY_SERIES_CELLS * BATTERY_LOW_CELL_RECOVER_V)

/*
 * [2026-03-10]
 * 修改目的:
 * 1. ??????????????????????????????????? `armed = 0`?
 * 2. ????????????????? VOFA ??????????
 */
#define EMERGENCY_STOP_REASON_NONE            0
#define EMERGENCY_STOP_REASON_ATTITUDE_ROLL   1
#define EMERGENCY_STOP_REASON_ATTITUDE_PITCH  2
#define EMERGENCY_STOP_REASON_MANUAL_BUTTON   3
#define EMERGENCY_STOP_REASON_LINK_TIMEOUT    4

// ==================== ??????? ====================
extern volatile uint8 armed;
extern volatile uint8 control_mode;
extern volatile uint8 emergency_stop_triggered;
extern volatile uint8 emergency_stop_reason;

/*
 * [2026-03-10]
 * Purpose:
 * 1. Keep these three variables as an active-mode snapshot for existing monitor/UI code.
 * 2. The real runtime throttle state is now stored per control mode and mirrored here.
 */
extern volatile float target_throttle;
extern volatile float current_throttle;
extern volatile float base_throttle;

extern uint16 throttle_buffer[4];

// ==================== ???? ====================
extern float current_roll;
extern float current_pitch;
extern float current_yaw;

extern float current_gyro_roll;
extern float current_gyro_pitch;
extern float current_gyro_yaw;

// ==================== 姿态目标量 ====================
extern volatile float target_angle_roll;
extern volatile float target_angle_pitch;
extern volatile float target_angle_yaw;

extern volatile float target_angle_roll_add;
extern volatile float target_angle_pitch_add;
extern volatile float target_angle_yaw_add;

extern float target_roll_rate;
extern float target_pitch_rate;
extern float target_yaw_rate;

// ==================== 控制输出缓存 ====================
extern float roll_output;
extern float pitch_output;

// ==================== ?????? ====================
extern int low_voltage_protect_flag;
extern float voltage;

// ==================== ??ADC?? ====================
extern volatile int adc_x;
extern volatile int adc_y;
extern volatile float adc_x_filtered;
extern volatile float adc_y_filtered;
extern volatile float adc_x_nomalize ;
extern volatile float adc_y_nomalize ;
/*
 * [2026-03-10]
 * 修改目的:
 * 1. ???????????????????????????
 * 2. ??????? `low_voltage_check()` ?????????????
 */
void low_voltage_check(void);

/*
 * [2026-03-10]
 * 修改目的:
 * 1. ??????????????????????? ISR ??????????
 * 2. ????????????????????????????????
 */
void Flight_Runtime_Emergency_Stop(uint8 reason);
void Flight_Runtime_Clear_Emergency_Stop_Latch(void);

// ==================== PID ?? ====================
/*
 * [2026-03-10]
 * Purpose:
 * 1. Keep the existing Flight_Control_* call sites stable while using one shared throttle chain again.
 * 2. The `mode` parameter is retained only for compatibility with existing wrappers.
 */
void Flight_Runtime_Reset_All_Throttle_States(void);
void Flight_Runtime_Set_Throttle_For_Mode(uint8 mode, float throttle);
float Flight_Runtime_Get_Target_Throttle_For_Mode(uint8 mode);
float Flight_Runtime_Get_Base_Throttle_For_Mode(uint8 mode);
void Flight_Runtime_Sync_Public_Throttle_State(void);

void PIT_KeyVoltage_Task_5ms(void);
void PIT_Adc_Task_5ms(void);
void PIT_PMW3901_Task_20ms(void);
void PIT_Control_Task_250us(void);
void PIT_Motor_Task_500us(void);

extern PID_Controller gyro_pid_roll;
extern PID_Controller gyro_pid_pitch;
extern PID_Controller gyro_pid_yaw;

extern PID_Controller angle_pid_roll;
extern PID_Controller angle_pid_pitch;
extern PID_Controller angle_pid_yaw;
extern PID_Controller joystick_pid_roll;
extern PID_Controller joystick_pid_pitch;


#endif
