#ifndef ICM_H_
#define ICM_H_

// 统一数学常量与类型到 ControlMath 模块
#include "../control/ControlMath.h"

// ==================== 宏定义 ====================
#define dAimSpeed_MAX       (4.0f/4000.0f)

// 角度弧度转换宏已在 ControlMath.h 中统一定义（DEG2RAD, RAD2DEG, Ang2Rad, Rad2Ang）

// 物理常量
#define ROAD_WIDTH          (0.45f)
#define G_VALUE             (9.81f)         // 重力加速度 m/s?

// EKF 参数
#define STATE_DIM           7               // 状态维度：4个四元数 + 3个陀螺偏置
#define MEAS_DIM            3               // 测量维度

// 注意：DT已在其他地方定义，这里不再重复定义
// 如果需要使用，请使用 ICM_DT
#ifndef ICM_DT
#define ICM_DT              (0.00025f)      // 采样周期：1/4000 s
#endif

// 校准参数
#define IMU_calibration_seconds  3          // 校准时间(秒)

// ==================== 结构体定义 ====================

/**
 * @brief PID控制器参数结构体
 */
typedef struct {
    float Kp;  // 比例增益
    float Ki;  // 积分增益
    float Kd;  // 微分增益
} PID_t;

/**
 * @brief 二阶Biquad滤波器类型枚举
 */
typedef enum {
    BIQUAD_LOWPASS = 0,     // 低通滤波器
    BIQUAD_HIGHPASS,        // 高通滤波器
    BIQUAD_BANDPASS_PEAK,   // 带通滤波器(峰值型)
    BIQUAD_BANDSTOP_NOTCH   // 带阻滤波器(陷波型)
} biquad_type;

/**
 * @brief 二阶Biquad滤波器状态结构体
 */
typedef struct {
    float a0, a1, a2, a3, a4;  // 滤波器系数
    float x1, x2;               // 输入历史值
    float y1, y2;               // 输出历史值
} biquad_state;

// Quaternion_t 已在 ControlMath.h 中统一定义

/**
 * @brief EKF状态结构体
 */
typedef struct {
    float x[STATE_DIM];                // 状态向量：[q0,q1,q2,q3, bgx, bgy, bgz]
    float P[STATE_DIM][STATE_DIM];     // 协方差矩阵
    float Q[STATE_DIM][STATE_DIM];     // 过程噪声协方差
    float R[MEAS_DIM][MEAS_DIM];       // 测量噪声协方差
} EKF_State;

// ==================== 全局变量声明 ====================

// Mahony算法参数
extern volatile float twoKp;           // 比例增益
extern volatile float twoKi;           // 积分增益
extern volatile float q0, q1, q2, q3;  // Mahony四元数
extern volatile float integralFBx, integralFBy, integralFBz;  // 积分反馈项

// Madgwick算法参数
extern volatile float q01, q11, q21, q31;  // Madgwick四元数
extern volatile float beta;                 // Madgwick增益

// 四元数结构体(用于外部调用)
extern Quaternion_t q_imu;

// 姿态角变量(弧度)
extern float Pitch_a_Pi, Roll_a_Pi, Yaw_a_Pi;     // Mahony姿态角(弧度)
extern float Yaw_a_Pi_genrl;                      // 相对初始Yaw角(弧度)

// 姿态角变量(度)
extern float Pitch_a, Roll_a, Yaw_a;              // Mahony姿态角(度)
extern float Pitch_b, Roll_b, Yaw_b;              // Madgwick姿态角(度)
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

// 角速度变量(度/秒)
extern float Pitch_g, Roll_g, Yaw_g;              // 原始角速度
extern float Pitch_g_F, Roll_g_F, Yaw_g_F;        // 滤波后角速度
extern float Pitch_g_last, Roll_g_last, Yaw_g_last;  // 历史角速度

// 目标角速度
extern float aim_x_v, aim_y_v, aim_y_v_last, aim_yaw_g;

// 角加速度变量(度/秒?)
extern float Pitch_angular_acc, Yaw_angular_acc;

// 加速度计变量(mg)
extern float Pitch_acc, Roll_acc, Yaw_acc;        // 原始加速度
extern float Pitch_acc_F, Roll_acc_F, Yaw_acc_F;  // 滤波后加速度

// 线速度变量
extern float X_V, Y_V;                            // 估计线速度
extern float X_V_F, Y_V_F;                        // 滤波后线速度
extern float X_V_last, Y_V_last;                  // 历史线速度
extern float X_V_R_last, Y_V_R_last;              // 相对速度

// 陀螺仪辅助变量
extern float Yaw_angle_counter;
extern float Yaw_angle;   // Yaw角度(用于外部调用)
extern float Yaw_gyro;    // Yaw角速度(用于外部调用)

// EKF全局状态
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

// ========== 初始化函数 ==========
void ICM_Calculate_Init(void);
void Filter_Init(void);
void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value);
void IMU_calibration(void);
void calculate_origin_point(void);

// ========== 数据获取与处理 ==========
void get_ICM_data(void);
float biquad(biquad_state *state, float data);

// ========== 姿态解算算法 ==========
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float acc_norm);

void updateMadgwickEulerIMU(float gx, float gy, float gz,
                            float ax, float ay, float az, float dt);

// ========== 辅助计算函数 ==========
void Calculate_Angular_Acceleration(void);
void Angle_Limit_Process(void);
void Update_Velocity_Estimate(void);
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

// ========== 兼容旧接口 ==========
float get_Yaw_data(void);
float get_yaw(void);
float get_yaw_g(void);

// ========== 调试与监控 ==========
int Check_IMU_Data_Quality(void);
void Display_IMU_Info(void);
void Adaptive_Parameter_Tuning(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM_H_ */
