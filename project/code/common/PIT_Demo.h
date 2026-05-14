#ifndef _PIT_Demo_h_
#define _PIT_Demo_h_

#include "zf_common_headfile.h"
#include "control/pid_controller.h"

/*
 * PIT_Demo 对外主要暴露两类内容:
 * 1. ????????????????????
 * 2. ??????????????????????
 */

// ==================== 电机输出协议选择 ====================
// 0 = 标准 PWM（旧电调，200Hz，计数器值 2000~4000）
// 1 = DShot600（新电调，GPIO 模拟时序，值域 0 / 300~2047）
#define MOTOR_OUTPUT_PROTOCOL       1

// ==================== 安全限制参数 ====================
#define MOTOR_OUTPUT_SCALE_MAX      2000U
#define SAFE_MAX_MOTOR_OUTPUT_LIMIT MOTOR_OUTPUT_SCALE_MAX
#define SAFE_MAX_THROTTLE           ((float)SAFE_MAX_MOTOR_OUTPUT_LIMIT)
#define SAFE_MIN_THROTTLE           300.0f
#define MOTOR_TEST_MAX_THROTTLE     450U
#define MOTOR_TEST_STEP             50U

#define SAFE_MAX_ANGLE_ROLL     30.0f
#define SAFE_MAX_ANGLE_PITCH    30.0f
#define SAFE_MAX_ANGLE_YAW      180.0f

/*
 * [2026-03-10]
 * 修改目的:
 * 1. ??????????????????????????????????????????
 * 2. ??????? 6S ??????????????????????????????????
 *    ??????? BATTERY_ADC_TO_VOLTAGE_SCALE ??????
 */
#define BATTERY_ADC_TO_VOLTAGE_SCALE   0.00820715f
#define BATTERY_ADC_CHANNEL            ADC1_CH24_P14_4
#define BATTERY_SERIES_CELLS           3.0f
#define BATTERY_LOW_CELL_WARN_V        3.767f   /* 11.3V / 3，低于此值需要充电。 */
#define BATTERY_LOW_CELL_RECOVER_V     3.833f   /* 11.5V / 3，超过此值才解除警报（过溶放电）。 */
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

extern volatile uint16 motor_throttle[4];
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

extern volatile float angle_roll_compensation;
extern volatile float angle_pitch_compensation;
extern volatile float angle_yaw_compensation;

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
extern volatile int adc_x_filtered;
extern volatile int adc_y_filtered;
extern volatile float adc_x_nomalize ;
extern volatile float adc_y_nomalize ;
extern volatile float car_bat_voltage;  /* 小车通过 core_link 发送的电池电压，单位 V。 */

/* 位置闭环 adc/ToF 双数据校验观测量。 */
extern volatile float pos_fusion_r_adc;
extern volatile float pos_fusion_r_tof;
extern volatile float pos_fusion_r_fused;
extern volatile float pos_fusion_residual;
extern volatile uint8 pos_fusion_fault;
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

void Flight_Motor_Test_Start(void);
void Flight_Motor_Test_Stop(void);
uint8 Flight_Motor_Test_Is_Active(void);
void Flight_Motor_Test_Set_Selected(uint8 motor_index);
uint8 Flight_Motor_Test_Get_Selected(void);
void Flight_Motor_Test_Set_Throttle(uint16 throttle);
uint16 Flight_Motor_Test_Get_Throttle(void);
void Flight_Motor_Test_Set_Output_Enable(uint8 enable);
uint8 Flight_Motor_Test_Get_Output_Enable(void);
uint16 Flight_Motor_Test_Get_Output_Limit(void);

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

/* 光流速度闭环独立 PID:
 *   vel_pid_x: 前向速度 (vx_cms) -> ax_cmd_cms2 -> pitch 修正
 *   vel_pid_y: 右向速度 (vy_cms) -> ay_cmd_cms2 -> roll  修正
 * 默认参数在 init_default_pid_parameters; VOFA 用 V/W 命令实时调。 */
extern PID_Controller vel_pid_x;
extern PID_Controller vel_pid_y;

/*
 * 光流速度闭环 (Velocity Loop) 对外接口。
 *
 * Feedback source: fused body horizontal velocity (velocity_x/y_fused_cms, +x forward, +y right).
 * 控制器: 独立 PID vel_pid_x / vel_pid_y。
 * 输出  : 先生成水平加速度指令, 再换算成姿态修正角并写入 target_angle_*。
 * Current takeoff base is fixed at 0deg/0deg.
 *
 * 默认关闭, 起飞与降落阶段 PID 保持 reset; 启用后若光流 invalid 自动退回锁基准角。
 */
void  Velocity_Loop_Set_Enabled(uint8 enabled);
uint8 Velocity_Loop_Is_Enabled(void);
void  Velocity_Loop_Set_Target_Velocity(float vx_cms, float vy_cms);
void  Velocity_Loop_Get_Target_Velocity(float *vx_cms, float *vy_cms);
void  Velocity_Loop_Get_Diag(float *pitch_corr_deg,
                             float *roll_corr_deg,
                             float *fb_vx_cms,
                             float *fb_vy_cms,
                             float *x_i_out,
                             float *y_i_out,
                             uint8 *active);

/* 光流 ToF 容错状态诊断 (PIT_PMW3901_Task_20ms 内部维护).
 * source: 0=NONE 1=LIVE(本帧 dl1b) 2=HOLD(上一帧缓存兜底). */
void  PMW_Flow_Get_TOF_Status(float *tof_los_cm, uint16 *stale_ticks, uint8 *source);


#endif
