#ifndef ICM_H_
#define ICM_H_

#include "zf_common_typedef.h"

// 统一数学常量与类型到 ControlMath 模块
#include "../control/ControlMath.h"
#include "icm42688.h"

// ==================== 常量定义 ====================
#define dAimSpeed_MAX       (4.0f/4000.0f)

// 角度转换宏已在 ControlMath.h 中定义：DEG2RAD、RAD2DEG、Ang2Rad、Rad2Ang

// 物理常量
#define ROAD_WIDTH          (0.45f)
#define G_VALUE             (9.81f)         // 重力加速度 m/s^2

// EKF 参数
#define STATE_DIM           7               // 状态维度：4 元数 + 3 陀螺零偏
#define MEAS_DIM            3               // 测量维度

#define ICM_SAMPLE_RATE_HZ  (4000.0f)
#define ICM_SAMPLE_DT_S     (0.00025f)

// 校准参数
#define IMU_calibration_seconds  3          // 校准时长(秒)

// ==================== 数据结构定义 ====================

/**
 * @brief PID控制器参数结构体
 */
typedef struct {
    float Kp;  // 比例增益
    float Ki;  // 积分增益
    float Kd;  // 微分增益
} PID_t;

/**
 * @brief 二阶 Biquad 滤波器类型
 */
typedef enum {
    BIQUAD_LOWPASS = 0,     // 低通滤波器
    BIQUAD_HIGHPASS,        // 高通滤波器
    BIQUAD_BANDPASS_PEAK,   // 带通滤波器(峰值型)
    BIQUAD_BANDSTOP_NOTCH   // 带阻/陷波滤波器
} biquad_type;

/**
 * @brief 二阶Biquad滤波器状态结构体
 */
typedef struct {
    float a0, a1, a2, a3, a4;  // 滤波器系数
    float x1, x2;               // 输入历史
    float y1, y2;               // 输出历史
} biquad_state;

// Quaternion_t 已在 ControlMath.h 中统一定义

/**
 * @brief EKF状态结构体
 */
typedef struct {
    float x[STATE_DIM];                // 状态向量：[q0,q1,q2,q3, bgx, bgy, bgz]
    float P[STATE_DIM][STATE_DIM];     // 状态协方差
    float Q[STATE_DIM][STATE_DIM];     // 过程噪声协方差
    float R[MEAS_DIM][MEAS_DIM];       // 测量噪声协方差
} EKF_State;

#define ICM_HEALTH_FAULT_NONE          (0U)
#define ICM_HEALTH_FAULT_NONFINITE     (1U << 0)
#define ICM_HEALTH_FAULT_ACC_NORM      (1U << 1)
#define ICM_HEALTH_FAULT_GYRO_SAT      (1U << 2)
#define ICM_HEALTH_FAULT_QUATERNION    (1U << 3)
#define ICM_HEALTH_FAULT_SAMPLE_STALE  (1U << 4)
#define ICM_HEALTH_CRITICAL_FAULT_MASK (ICM_HEALTH_FAULT_NONFINITE | \
                                        ICM_HEALTH_FAULT_GYRO_SAT | \
                                        ICM_HEALTH_FAULT_QUATERNION | \
                                        ICM_HEALTH_FAULT_SAMPLE_STALE)

typedef struct
{
    uint32 sample_sequence;
    uint32 stale_count;
    uint32 consecutive_invalid_count;
    float sample_interval_us;
    float acc_norm_mg;
    float quaternion_norm;
    uint32 fault_mask;
    uint8 gyro_saturated;
    uint8 sample_stale;
    uint8 attitude_valid;
} ICM_Health_t;

// ==================== 全局变量声明 ====================

// Mahony算法参数
extern volatile float twoKp;           // 比例增益
extern volatile float twoKi;           // 积分增益
extern volatile float q0, q1, q2, q3;  // Mahony 四元数
extern volatile float integralFBx, integralFBy, integralFBz;  // 积分反馈项

// Madgwick算法参数
extern volatile float q01, q11, q21, q31;  // Madgwick 四元数
extern volatile float beta;                 // Madgwick增益

// 四元数结构体(用于外部调用)
extern Quaternion_t q_imu;

// 姿态角变量(弧度)
extern float Pitch_a_Pi, Roll_a_Pi, Yaw_a_Pi;     // Mahony姿态角(弧度)
extern float Yaw_a_Pi_genrl;                      // 相对初始 Yaw 角

// 姿态角变量(度)
extern float Pitch_a, Roll_a, Yaw_a;              // Mahony 姿态角(度)
extern float Pitch_b, Roll_b, Yaw_b;              // Madgwick 姿态角(度)
extern float Pitch_c, Roll_c, Yaw_c;              // 备用姿态角
extern float Pitch, Roll, Yaw;                    // 融合姿态角
extern float Pitch_b_use, Roll_b_use, Yaw_b_use;  // 滤波后Madgwick姿态角

// 初始姿态角
extern float Roll_A0, Pitch_A0, Yaw_A0;

// 历史姿态角
extern float Pitch_a_last, Roll_a_last, Yaw_a_last;

// 目标姿态角
extern float aim_roll_a, aim_pitch_a, aim_yaw_a;
extern float d_aim_yaw_a;  // Yaw角指令调整速度

// 锁定姿态角
extern float Yaw_Lock_a;

// 角速度(度/秒)
extern float Pitch_g, Roll_g, Yaw_g;              // 原始角速度
extern float Pitch_g_F, Roll_g_F, Yaw_g_F;        // 滤波后角速度
extern float Pitch_g_last, Roll_g_last, Yaw_g_last;  // 历史角速度

// 目标角速度
extern float aim_x_v, aim_y_v, aim_y_v_last, aim_yaw_g;

// 角加速度(度/秒^2)
extern float Pitch_angular_acc, Yaw_angular_acc;

// 加速度(mg)
extern float Pitch_acc, Roll_acc, Yaw_acc;        // 原始加速度
extern float Pitch_acc_F, Roll_acc_F, Yaw_acc_F;  // 滤波后加速度
extern float acc_norm_filtered;                   // 姿态解算实际使用的滤波后加速度模值
extern float acc_level_x_bias_mg, acc_level_y_bias_mg;
extern float acc_level_x_scale, acc_level_y_scale, acc_level_z_scale;

// 线速度变量
extern float X_V, Y_V;                            // 估计线速度
extern float X_V_F, Y_V_F;                        // 滤波后线速度
extern float X_V_last, Y_V_last;                  // 历史线速度
extern float X_V_R_last, Y_V_R_last;              // 相对速度
extern float Horizontal_Accel_X_Cms2;             // 机头水平系X加速度，cm/s²
extern float Horizontal_Accel_Y_Cms2;             // 机头水平系Y加速度，cm/s²
extern uint8 Horizontal_Accel_Valid;               // 当前水平加速度数值有效

// 陀螺仪辅助变量
extern float Yaw_angle_counter;
extern float Yaw_angle;   // Yaw角度(用于外部调用)
extern float Yaw_gyro;    // Yaw角速度(用于外部调用)

// EKF 状态
extern EKF_State ekf;
extern int ekf_initialized;

// 滤波器状态
extern biquad_state X_V_biquad, Y_V_biquad;
extern biquad_state Pitch_g_biquad, Roll_g_biquad, Yaw_g_biquad;
extern biquad_state Pitch_acc_biquad, Roll_acc_biquad, Yaw_acc_biquad;

// PID参数
extern volatile PID_t PIDK_AA, PIDK_AB, PIDK_AC;  // X平移控制
extern volatile PID_t PIDK_BA, PIDK_BB, PIDK_BC;  // Y平移控制
extern volatile PID_t PIDK_YG, PIDK_YA, PIDK_YV;  // Yaw控制
extern volatile PID_t PIDK_CA, PIDK_CB, PIDK_CC;  // 备用控制

// ==================== 函数声明 ====================

#ifdef __cplusplus
extern "C" {
#endif

// ========== 初始化接口 ==========
void ICM_Calculate_Init(void);
void Filter_Init(void);
void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value);
void IMU_calibration(void);
void Acc_Level_Calibration(void);
void calculate_origin_point(void);

// ========== 数据处理接口 ==========
void get_ICM_data(void);
float biquad(biquad_state *state, float data);

// ========== 姿态解算接口 ==========
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float acc_norm);

void updateMadgwickEulerIMU(float gx, float gy, float gz,
                            float ax, float ay, float az, float dt);

// ========== 辅助计算函数 ==========
void Calculate_Angular_Acceleration(void);
void Angle_Limit_Process(void);
void Update_Velocity_Estimate(void);
void Body_To_RollPitch_Level_Frame(float bx, float by, float bz,
                                   float *lx, float *ly, float *lz);
void Body_To_World_Frame(float bx, float by, float bz,
                         float *wx, float *wy, float *wz);
float invSqrt(float x);
void normalize_vector(float *v, int n);
int invert_3x3(const float A[3][3], float Ainv[3][3]);

// ========== 姿态控制接口 ==========
void Yaw_Lock(void);
float Get_Yaw_Error(void);
void ICM_Attitude_Reset(void);
void Get_Attitude_Angle(float *roll, float *pitch, float *yaw);
void Get_Angular_Rate(float *roll_rate, float *pitch_rate, float *yaw_rate);
void Get_Acceleration(float *ax, float *ay, float *az);
void ICM_GetHealth(ICM_Health_t *health);
uint8 ICM_Health_IsArmable(void);
uint8 ICM_IsStill(void);
void ICM_VisionYaw_Configure(uint8 enabled,
                             float marker_world_heading_deg,
                             float camera_yaw_offset_deg);
void ICM_VisionYaw_Update(float marker_heading_deg, uint8 valid, uint32 update_sequence);

// ========== Yaw 数据接口 ==========
float get_Yaw_data(void);
float get_yaw(void);
float get_yaw_g(void);

// ========== 调试诊断接口 ==========
int Check_IMU_Data_Quality(void);
void Display_IMU_Info(void);
void Adaptive_Parameter_Tuning(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM_H_ */
