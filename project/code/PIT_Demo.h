#ifndef _PIT_Demo_h_
#define _PIT_Demo_h_

#include "zf_common_headfile.h"
#include "pid_controller.h"

/*
 * PIT_Demo 对外主要暴露两类内容:
 * 1. 控制主循环中会被其他模块读取的实时状态�?
 * 2. 需要被初始化或菜单逻辑直接修改的控制目标量�?
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
 * 1. 把电池换算系数和低压阈值统一收口，避免启动检测、运行期检测和监视页各自写一套硬编码�?
 * 2. 当前工程默认�?6S 电池处理，低压提示按“每节电压”推导，后续如果显示值与万用表不一致，
 *    只需要优先校�?BATTERY_ADC_TO_VOLTAGE_SCALE 这一处�?
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
 * 1. 给“姿态超限急停”和“人工按键急停”统一一个原因码，避免后续继续散落着�?armed = 0�?
 * 2. 保持原因码简单稳定，方便后续扩展�?VOFA 或其它外部急停入口�?
 */
#define EMERGENCY_STOP_REASON_NONE            0
#define EMERGENCY_STOP_REASON_ATTITUDE_ROLL   1
#define EMERGENCY_STOP_REASON_ATTITUDE_PITCH  2
#define EMERGENCY_STOP_REASON_MANUAL_BUTTON   3
#define EMERGENCY_STOP_REASON_LINK_TIMEOUT    4

// ==================== 解锁与控制模�?====================
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

// ==================== 姿态状�?====================
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

// ==================== 电源保护状�?====================
extern int low_voltage_protect_flag;
extern float voltage;

// ==================== 摇杆ADC状�?====================
extern volatile int adc_x;
extern volatile int adc_y;
extern volatile float adc_x_filtered;
extern volatile float adc_y_filtered;
extern volatile float adc_x_nomalize ;
extern volatile float adc_y_nomalize ;
/*
 * [2026-03-10]
 * 修改目的:
 * 1. 低压检测改为“提示型”逻辑，只刷新告警，不阻塞主流程�?
 * 2. 保持现有调用�?`low_voltage_check()` 不变，降低对主流程的影响�?
 */
void low_voltage_check(void);

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 对外暴露统一急停入口，让菜单�?监视页按键和主控�?ISR 共用同一套停机动作�?
 * 2. 单独保留“清除急停锁存”接口，避免重新解锁时还带着旧的告警状态�?
 */
void Flight_Runtime_Emergency_Stop(uint8 reason);
void Flight_Runtime_Clear_Emergency_Stop_Latch(void);

// ==================== PID 控制�?====================
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
