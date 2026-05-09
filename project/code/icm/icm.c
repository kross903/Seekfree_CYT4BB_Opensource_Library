#include "icm.h"
#include "common/project_headfile.h"
#include <math.h>

#define ACC_MG_TO_MPS2              (9.81f / 1000.0f)  // mg -> m/s^2, 1000mg = 1g = 9.81m/s^2
#define ACC_VEL_MPS_TO_CMS          (100.0f)           // m/s -> cm/s, velocity output X_V/Y_V uses cm/s
#define ACC_VEL_LEAK                (0.99995f)         // velocity leak per 4kHz update, suppresses long-term drift but also attenuates real low speed
#define ACC_VEL_DEADBAND_MG         (0.0f)             // horizontal acceleration deadband, |acc| below this value is treated as 0mg
#define ACC_VEL_GYRO_GATE_DPS       (100.0f)           // skip velocity integration when any filtered gyro axis exceeds this rate
#define ACC_VEL_NORM_GATE_MG        (1000.0f)          // skip integration when |acc_norm - 1000mg| exceeds this value
#define ACC_VEL_MAX_INTEGRATE_MG    (300.0f)           // clamp horizontal acceleration before integration to limit velocity spikes
#define ACC_LEVEL_BIAS_FLASH_SECTOR (0U)               // flash sector used to store leveled horizontal acceleration bias
#define ACC_LEVEL_BIAS_FLASH_PAGE   (12U)              // flash page used to store leveled horizontal acceleration bias
#define ACC_LEVEL_BIAS_FLASH_WORDS  (3U)               // stored words: magic + x bias + y bias
#define ACC_LEVEL_BIAS_LIMIT_MG     (200.0f)           // reject stored/calibrated horizontal bias if magnitude is larger than this
#define ACC_LEVEL_BIAS_SAMPLE_HZ    (4000U)            // acceleration bias calibration sample rate
#define ACC_LEVEL_BIAS_SETTLE_MS    (1000U)            // wait time before sampling bias, lets attitude quaternion settle after reset

// ==================== 全局变量定义 ====================
#define sampleFreq  4000.0f         // 采样频率(赫兹)
#define twoKpDef    (2.0f * 0.5f)   // Mahony比例增益
#define twoKiDef    (2.0f * 0.0f)   // Mahony积分增益

// 注意：这里使用 DT_ICM 避免与其他地方的 DT 冲突
#ifndef DT
#define DT 0.00025f                 // 采样周期(8000Hz = 0.125ms)
#endif

#define G 9.81f                     // 重力加速度(m/s?)


// Mahony算法参数
volatile float invsampleFreq = 1.0f / sampleFreq;
volatile float twoKp = twoKpDef;
volatile float twoKi = twoKiDef;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;      // Mahony四元数
volatile float q01 = 1.0f, q11 = 0.0f, q21 = 0.0f, q31 = 0.0f;  // Madgwick四元数
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
volatile float beta = 0.11f;  // Madgwick算法参数

// 姿态角变量
float Roll_A0, Pitch_A0, Yaw_A0;           // 初始姿态角
float Yaw_Lock_a;                          // 锁定的Yaw角
float Pitch_a, Roll_a, Yaw_a;              // 实时姿态角(度)
float Pitch_b, Roll_b, Yaw_b;              // Madgwick姿态角(度)
float Pitch, Roll, Yaw;                    // 融合后的姿态角
float Pitch_a_Pi, Roll_a_Pi, Yaw_a_Pi;     // 姿态角(弧度)
float Yaw_a_Pi_genrl;                      // 相对初始Yaw角
float Pitch_a_last, Roll_a_last, Yaw_a_last;

// 角速度变量
float Pitch_g, Roll_g, Yaw_g;              // 原始角速度
float Pitch_g_F, Roll_g_F, Yaw_g_F;        // 滤波后角速度
float Pitch_g_last, Roll_g_last, Yaw_g_last;
float Yaw_angle, Yaw_gyro;                 // 用于外部调用

// 角加速度变量
float Pitch_acc, Roll_acc, Yaw_acc;        // 加速度计数据
float Pitch_acc_F, Roll_acc_F, Yaw_acc_F;  // 滤波后加速度
float Pitch_angular_acc, Yaw_angular_acc;  // 角加速度
float acc_level_x_bias_mg = 0.0f;          // Roll/pitch leveled X acceleration bias
float acc_level_y_bias_mg = 0.0f;          // Roll/pitch leveled Y acceleration bias
float acc_level_x_scale = 0.9940f;
float acc_level_y_scale = 0.9940f;
float acc_level_z_scale = 1.0000f;


// 线速度变量
float X_V, Y_V;                            // 估计的线速度
float X_V_F, Y_V_F;                        // 滤波后线速度
float X_V_last, Y_V_last;

// 滤波器状态
biquad_state X_V_biquad, Y_V_biquad;
biquad_state Pitch_g_biquad, Roll_g_biquad, Yaw_g_biquad;
biquad_state Pitch_acc_biquad, Roll_acc_biquad, Yaw_acc_biquad;

// PID参数(根据你的应用调整)
volatile PID_t PIDK_AA = {700, 0.15, 7};   // X平移位置
volatile PID_t PIDK_AB = {1, 0, 0};        // X平移速度
volatile PID_t PIDK_AC = {1, 0, 10};       // X平移加速度

volatile PID_t PIDK_BA = {700, 0.15, 7};   // Y平移位置
volatile PID_t PIDK_BB = {0, 0, 0};        // Y平移速度
volatile PID_t PIDK_BC = {5, 0, 20};       // Y平移加速度

volatile PID_t PIDK_YG = {10, 0.005, 1};   // Yaw角速度
volatile PID_t PIDK_YA = {10, 0, 0};       // Yaw角度
volatile PID_t PIDK_YV = {0, 0, 15};       // Yaw速度

// ==================== 函数声明 ====================
float invSqrt(float x);
void normalize_vector(float *v, int n);
int invert_3x3(const float A[3][3], float Ainv[3][3]);

typedef enum
{
    ORIGIN_AXIS_ROLL = 0,
    ORIGIN_AXIS_PITCH,
    ORIGIN_AXIS_YAW,
    ORIGIN_AXIS_COUNT
} origin_axis_enum;


#define ORIGIN_GROUP_SAMPLE_COUNT 10U

static uint8 icm_is_valid_gyro_bias(float value)
{
    return (value == value) && (fabsf(value) < 200.0f);
}

static uint8 icm_is_valid_acc_level_bias(float value)
{
    return (value == value) && (fabsf(value) < ACC_LEVEL_BIAS_LIMIT_MG);
}

static void icm_acc_level_bias_load_or_calibrate(uint8 force_calibration);

static void origin_sample_live_attitude(float history_sum[ORIGIN_AXIS_COUNT])
{
    history_sum[ORIGIN_AXIS_ROLL] += current_roll;
    history_sum[ORIGIN_AXIS_PITCH] += current_pitch;
    history_sum[ORIGIN_AXIS_YAW] += current_yaw;
}

static void origin_clear_accumulator(float history_sum[ORIGIN_AXIS_COUNT])
{
    history_sum[ORIGIN_AXIS_ROLL] = 0.0f;
    history_sum[ORIGIN_AXIS_PITCH] = 0.0f;
    history_sum[ORIGIN_AXIS_YAW] = 0.0f;
}

static void origin_commit_targets(const float sum[ORIGIN_AXIS_COUNT])
{
    target_angle_roll = sum[ORIGIN_AXIS_ROLL] / (float)IMU_calibration_seconds;
    target_angle_pitch = sum[ORIGIN_AXIS_PITCH] / (float)IMU_calibration_seconds;
    target_angle_yaw = sum[ORIGIN_AXIS_YAW] / (float)IMU_calibration_seconds;
}

void calculate_origin_point(void)
{
    uint32 sample_group_index = 0U;
    uint32 sample_count_in_group = 0U;
    uint8 i = 0U;
    float history[ORIGIN_AXIS_COUNT][IMU_calibration_seconds];
    float history_sum[ORIGIN_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
    float final_sum[ORIGIN_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};

    ui_origin_draw_calibration_layout();

    system_delay_ms(1000);
    ui_origin_show_phase("Start in 1s");
    system_delay_ms(1000);
    ui_origin_show_phase("Sampling 3s");

    do
    {
        system_delay_ms(100);

        if(sample_count_in_group < ORIGIN_GROUP_SAMPLE_COUNT)
        {
            origin_sample_live_attitude(history_sum);
            ui_origin_show_live_attitude(current_roll, current_pitch, current_yaw);
            sample_count_in_group++;
        }
        else
        {
            history[ORIGIN_AXIS_ROLL][sample_group_index] = history_sum[ORIGIN_AXIS_ROLL] / (float)ORIGIN_GROUP_SAMPLE_COUNT;
            history[ORIGIN_AXIS_PITCH][sample_group_index] = history_sum[ORIGIN_AXIS_PITCH] / (float)ORIGIN_GROUP_SAMPLE_COUNT;
            history[ORIGIN_AXIS_YAW][sample_group_index] = history_sum[ORIGIN_AXIS_YAW] / (float)ORIGIN_GROUP_SAMPLE_COUNT;

            ui_origin_show_average_attitude(history[ORIGIN_AXIS_ROLL][sample_group_index],
                                            history[ORIGIN_AXIS_PITCH][sample_group_index],
                                            history[ORIGIN_AXIS_YAW][sample_group_index]);

            origin_clear_accumulator(history_sum);
            sample_count_in_group = 0U;
            sample_group_index++;
        }
    } while(sample_group_index < IMU_calibration_seconds);

    for(i = 0U; i < IMU_calibration_seconds; i++)
    {
        final_sum[ORIGIN_AXIS_ROLL] += history[ORIGIN_AXIS_ROLL][i];
        final_sum[ORIGIN_AXIS_PITCH] += history[ORIGIN_AXIS_PITCH][i];
        final_sum[ORIGIN_AXIS_YAW] += history[ORIGIN_AXIS_YAW][i];
    }

    origin_commit_targets(final_sum);

    ui_origin_show_saved();
    system_delay_ms(500);
}

// ==================== 初始化函数 ====================
/**
 * @brief ICM姿态解算系统完整初始化
 * @note 在主程序开始时调用一次
 */
void ICM_Calculate_Init(void)
{
    uint8 force_acc_level_calibration = 0U;

    icm_calibration_flag = 1;

    // 1. 硬件初始化
    Init_ICM42688();
    system_delay_ms(100);
    ips200_show_string(0, 16*0, "ICM Init...");

    // 2. 滤波器初始化
    Filter_Init();

    // 3. 四元数初始化
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    q01 = 1.0f; q11 = 0.0f; q21 = 0.0f; q31 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;

    // 4. 陀螺仪零飘校准
    if(IMU_calibrate_flag == 0) {
        IMU_calibration();  // 自动校准
        icm_calibration_over_flag = 1;
        force_acc_level_calibration = 1U;
    } else {
        flash_read_page_to_buffer(0, 11, 4); 

        gyro_x_correction = gyro_x_correction_experience=flash_union_buffer[1].float_type;
        gyro_y_correction = gyro_y_correction_experience=flash_union_buffer[2].float_type;
        gyro_z_correction = gyro_z_correction_experience=flash_union_buffer[3].float_type;
        ips200_show_string(0, 16*2, "Using Gyro Exp Val");
        ips200_show_float(96, 56,  gyro_x_correction, 3, 3);
        ips200_show_float(96, 80,  gyro_y_correction, 3, 3);
        ips200_show_float(96, 104, gyro_z_correction, 3, 3);
        if(!(icm_is_valid_gyro_bias(gyro_x_correction) &&
             icm_is_valid_gyro_bias(gyro_y_correction) &&
             icm_is_valid_gyro_bias(gyro_z_correction)))
        {
            gyro_x_correction = gyro_x_correction_experience = 0.0f;
            gyro_y_correction = gyro_y_correction_experience = 0.0f;
            gyro_z_correction = gyro_z_correction_experience = 0.0f;
            ips200_show_string(0, 16*2, "Gyro Bias Invalid ");
            IMU_calibration();
            icm_calibration_over_flag = 1;
            force_acc_level_calibration = 1U;
        }
    }      
    

    // 5. 预热姿态解算(让四元数收敛)
    ips200_show_string(0, 16*3, "Warming up...");
    for(int i = 0; i < 200; i++) {
        get_ICM_data();
        system_delay_ms(5);
    }

    icm_acc_level_bias_load_or_calibrate(force_acc_level_calibration);

    // 6. 记录初始姿态
    Yaw_A0 = Yaw_a_Pi;
    Roll_A0 = Roll_a;
    Pitch_A0 = Pitch_a;
    Yaw_Lock_a = Yaw_a;

    // 7. 清零速度估计
    X_V = 0.0f; Y_V = 0.0f;
    X_V_F = 0.0f; Y_V_F = 0.0f;

    ips200_show_string(0, 16*3, "ICM Ready!        ");
    system_delay_ms(500);
    icm_calibration_flag = 0;

}

// ==================== 滤波器初始化 ====================
/**
 * @brief 初始化所有二阶Biquad滤波器
 */
void Filter_Init(void)
{
    // 速度滤波器(1000Hz采样, 截止频率根据PID.Kd)
    biquad_filter_init(&X_V_biquad, BIQUAD_LOWPASS, 1000, PIDK_AC.Kd, 0.7071);
    biquad_filter_init(&Y_V_biquad, BIQUAD_LOWPASS, 1000, PIDK_BC.Kd, 0.7071);

    // 角速度滤波器(8000Hz采样, 90Hz截止频率)
    biquad_filter_init(&Pitch_g_biquad, BIQUAD_LOWPASS, 8000, 90, 0.7071);
    biquad_filter_init(&Roll_g_biquad, BIQUAD_LOWPASS, 8000, 90, 0.7071);
    biquad_filter_init(&Yaw_g_biquad, BIQUAD_LOWPASS, 8000, 90, 0.7071);

    // 【修改1】加速度滤波器 - 提升截止频率到20Hz，平衡响应和抗振
    biquad_filter_init(&Pitch_acc_biquad, BIQUAD_LOWPASS, 8000, 20, 0.7071);
    biquad_filter_init(&Roll_acc_biquad, BIQUAD_LOWPASS, 8000, 20, 0.7071);
    biquad_filter_init(&Yaw_acc_biquad, BIQUAD_LOWPASS, 8000, 20, 0.7071);
}

/**
 * @brief 配置二阶Biquad滤波器
 * @param state 滤波器状态结构体
 * @param type 滤波器类型
 * @param fs 采样频率(Hz)
 * @param fc 截止频率(Hz)
 * @param q_value 品质因数(Butterworth: 0.7071)
 */
void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value)
{
    float w0, sin_w0, cos_w0, alpha;
    float b0, b1, b2, a0, a1, a2;

    w0 = 2.0f * PI * fc / fs;  // 归一化角频率
    sin_w0 = sinf(w0);
    cos_w0 = cosf(w0);
    alpha = sin_w0 / (2.0f * q_value);

    switch(type)
    {
    case BIQUAD_LOWPASS:
        b0 = (1.0f - cos_w0) / 2.0f;
        b1 = b0 * 2.0f;
        b2 = b0;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    case BIQUAD_HIGHPASS:
        b0 = (1.0f + cos_w0) / 2.0f;
        b1 = -b0 * 2.0f;
        b2 = b0;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    case BIQUAD_BANDPASS_PEAK:
        b0 = alpha;
        b1 = 0.0f;
        b2 = -alpha;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    case BIQUAD_BANDSTOP_NOTCH:
        b0 = 1.0f;
        b1 = -2.0f * cos_w0;
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;
    default:
        b0 = 1.0f; b1 = 0.0f; b2 = 0.0f;
        a0 = 1.0f; a1 = 0.0f; a2 = 0.0f;
        break;
    }

    state->a0 = b0 / a0;
    state->a1 = b1 / a0;
    state->a2 = b2 / a0;
    state->a3 = a1 / a0;
    state->a4 = a2 / a0;
    state->x1 = state->x2 = 0.0f;
    state->y1 = state->y2 = 0.0f;
}

/**
 * @brief 二阶Biquad滤波器处理
 * @param state 滤波器状态
 * @param data 输入数据
 * @return 滤波后的数据
 */
float biquad(biquad_state *state, float data)
{
    float result = state->a0 * data + state->a1 * state->x1 + state->a2 * state->x2
                 - state->a3 * state->y1 - state->a4 * state->y2;

    state->x2 = state->x1;
    state->x1 = data;
    state->y2 = state->y1;
    state->y1 = result;

    return result;
}

// ==================== 主数据获取与处理 ====================
/**
 * @brief 获取ICM数据并进行姿态解算
 * @note 在8KHz定时器中断中调用
 */
void get_ICM_data(void)
{
    // 1. 读取原始数据
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();

    // ?? 保持原有的角度取反逻辑（用户要求不改）
//    icm42688_gyro_x = -icm42688_gyro_x;  // Roll角速度取反
//    icm42688_gyro_y = -icm42688_gyro_y;  // Pitch角速度取反
//    // icm42688_gyro_z 保持不变（Yaw方向正确）
//
//    icm42688_acc_x = -icm42688_acc_x;    // Roll加速度取反
//    icm42688_acc_y = -icm42688_acc_y;    // Pitch加速度取反
    // icm42688_acc_z 保持不变

    // 2. 计算角加速度(用于高级控制)
    float gx = icm42688_gyro_x * DEG2RAD;
    float gy = icm42688_gyro_y * DEG2RAD;
    float gz = icm42688_gyro_z * DEG2RAD;

    alpha_gx = (gx - prev_gx) * sampleFreq;  // rad/s?
    alpha_gy = (gy - prev_gy) * sampleFreq;
    alpha_gz = (gz - prev_gz) * sampleFreq;

    prev_gx = gx;
    prev_gy = gy;
    prev_gz = gz;

    // 3. 加速度数据处理
    Pitch_acc = icm42688_acc_y;
    Roll_acc  = icm42688_acc_x;
    Yaw_acc   = icm42688_acc_z;

    // 【修改2】先进行 Biquad 滤波 - 这是关键！
    Pitch_acc_F = biquad(&Pitch_acc_biquad, Pitch_acc);
    Roll_acc_F  = biquad(&Roll_acc_biquad, Roll_acc);
    Yaw_acc_F   = biquad(&Yaw_acc_biquad, Yaw_acc);

    // 【修改3】使用滤波后的加速度计算模值，避免振动导致模值乱跳
    float acc_filtered = sqrtf(Roll_acc_F * Roll_acc_F +
                               Pitch_acc_F * Pitch_acc_F +
                               Yaw_acc_F * Yaw_acc_F);

    // 保留原始模值用于对比调试
    acc = sqrtf(icm42688_acc_x * icm42688_acc_x +
                icm42688_acc_y * icm42688_acc_y +
                icm42688_acc_z * icm42688_acc_z);

    // 保留一阶滤波用于兼容性
    icm42688_acc_x_use = 0.992f * icm42688_acc_x_use + 0.008f * icm42688_acc_x;
    icm42688_acc_y_use = 0.992f * icm42688_acc_y_use + 0.008f * icm42688_acc_y;
    icm42688_acc_z_use = 0.992f * icm42688_acc_z_use + 0.008f * icm42688_acc_z;

    acc_use = sqrtf(icm42688_acc_x_use * icm42688_acc_x_use +
                    icm42688_acc_y_use * icm42688_acc_y_use +
                    icm42688_acc_z_use * icm42688_acc_z_use);

    // 4. 角速度数据处理
    Pitch_g = icm42688_gyro_y;
    Roll_g  = icm42688_gyro_x;
    Yaw_g   = icm42688_gyro_z;

    // 二阶滤波器处理角速度
    Pitch_g_F = biquad(&Pitch_g_biquad, Pitch_g);
    Roll_g_F  = biquad(&Roll_g_biquad, Roll_g);
    Yaw_g_F   = biquad(&Yaw_g_biquad, Yaw_g);

    // 【修改4】Mahony姿态解算 - 传入滤波后的加速度和模值
    MahonyAHRSupdateIMU(icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z,
                        Roll_acc_F, Pitch_acc_F, Yaw_acc_F, acc_filtered);

    // 6. 转换为欧拉角(度)
    Pitch_a = Rad2Ang * Pitch_a_Pi;
    Roll_a  = Rad2Ang * Roll_a_Pi;
    Yaw_a   = Rad2Ang * Yaw_a_Pi;

    // 可选: 使用Madgwick算法作为对比
    // updateMadgwickEulerIMU(icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z,
    //                        Roll_acc_F, Pitch_acc_F, Yaw_acc_F, DT);

    // 7. 双算法融合(可选)
    // Pitch = 0.5f * Pitch_a + 0.5f * Pitch_b;
    // Roll  = 0.5f * Roll_a + 0.5f * Roll_b;
    // Yaw   = 0.5f * Yaw_a + 0.5f * Yaw_b;

    // 8. 计算相对Yaw角
    Yaw_a_Pi_genrl = Yaw_a_Pi - Yaw_A0;

    // 9. 更新外部接口变量
    Yaw_angle = Yaw_a;
    Yaw_gyro = Yaw_g_F;

    // 10. 计算角加速度(用于前馈控制)
    Calculate_Angular_Acceleration();

    // 11. 更新只经 roll / pitch 水平化后的机体系水平速度估计
    Update_Velocity_Estimate();

    // 12. 姿态角限幅
    Angle_Limit_Process();
}

// ==================== Mahony姿态解算 ====================
/**
 * @brief Mahony互补滤波算法(IMU版本) - 改进版
 * @param gx, gy, gz 角速度(°/s)
 * @param ax, ay, az 加速度(mg) - 已经过 Biquad 滤波
 * @param acc_norm 加速度模值 - 用于运动检测
 */
void MahonyAHRSupdateIMU(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float acc_norm)
{
    static float recipNorm;
    static float halfvx, halfvy, halfvz;
    static float halfex, halfey, halfez;
    static float qa, qb, qc;

    // 判断加速度数据是否有效
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // 角速度转换为弧度
        gx *= DEG2RAD;
        gy *= DEG2RAD;
        gz *= DEG2RAD;

        // 归一化加速度
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向(从四元数推导)
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 计算误差(重力方向估计 × 加速度测量)
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 积分反馈(如果Ki > 0)
        if(twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * invsampleFreq;
            integralFBy += twoKi * halfey * invsampleFreq;
            integralFBz += twoKi * halfez * invsampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 【修改5】自适应调整Kp - 核心修改点
        // 使用滤波后的模值，放宽阈值，防止完全归零
        // 假设静止时重力约为 1000mg (根据你的传感器量程调整)
        if(fabsf(acc_norm - 1000.0f) < 50.0f) {
            // 静止或微小运动: 高增益，快速收敛
            twoKp = 40.6f;
        }
        else if(fabsf(acc_norm - 1000.0f) < 200.0f) {
            // 中等运动: 中等增益，平衡响应和稳定性
            twoKp = 0.2f;
        }
        else {
            // 【关键修改】剧烈运动: 保留极小增益，防止长时间漂移
            // 不要设为 0，这样即使在振动环境下也能保持微弱的修正
            twoKp = 0.01f;
        }

        // 比例反馈
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 四元数微分方程
    gx *= (0.5f * invsampleFreq);
    gy *= (0.5f * invsampleFreq);
    gz *= (0.5f * invsampleFreq);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // 四元数转欧拉角(弧度)
    static float r11, r12, r21, r31, r32;
    r11 = 2.0f * (q0 * q1 + q2 * q3);
    r12 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    r21 = 2.0f * (q0 * q2 - q3 * q1);
    r31 = 2.0f * (q0 * q3 + q1 * q2);
    r32 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);

    Yaw_a_Pi   = atan2f(r31, r32);
    Pitch_a_Pi = asinf(r21);
    Roll_a_Pi  = atan2f(r11, r12);
}

// ==================== Madgwick姿态解算(可选) ====================
/**
 * @brief Madgwick梯度下降算法
 * @param gx, gy, gz 角速度(°/s)
 * @param ax, ay, az 加速度(mg)
 * @param dt 采样周期(s)
 */
void updateMadgwickEulerIMU(float gx, float gy, float gz,
                            float ax, float ay, float az, float dt)
{
    gx *= Ang2Rad;
    gy *= Ang2Rad;
    gz *= Ang2Rad;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // 陀螺仪积分
    qDot0 = 0.5f * (-q11 * gx - q21 * gy - q31 * gz);
    qDot1 = 0.5f * ( q01 * gx + q21 * gz - q31 * gy);
    qDot2 = 0.5f * ( q01 * gy - q11 * gz + q31 * gx);
    qDot3 = 0.5f * ( q01 * gz + q11 * gy - q21 * gx);

    // 加速度计修正
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        _2q0 = 2.0f * q01; _2q1 = 2.0f * q11;
        _2q2 = 2.0f * q21; _2q3 = 2.0f * q31;
        _4q0 = 4.0f * q01; _4q1 = 4.0f * q11; _4q2 = 4.0f * q21;
        _8q1 = 8.0f * q11; _8q2 = 8.0f * q21;
        q0q0 = q01 * q01; q1q1 = q11 * q11;
        q2q2 = q21 * q21; q3q3 = q31 * q31;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q11 - _2q0 * ay - _4q1
           + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q21 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
           + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q31 - _2q1 * ax + 4.0f * q2q2 * q31 - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm;
        s2 *= recipNorm; s3 *= recipNorm;

        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
    }

    // 积分更新四元数
    q01 += qDot0 * dt;
    q11 += qDot1 * dt;
    q21 += qDot2 * dt;
    q31 += qDot3 * dt;

    // 归一化
    recipNorm = invSqrt(q01 * q01 + q11 * q11 + q21 * q21 + q31 * q31);
    q01 *= recipNorm; q11 *= recipNorm;
    q21 *= recipNorm; q31 *= recipNorm;

    // 转欧拉角
    Roll_b  = atan2f(2.0f * (q01 * q11 + q21 * q31),
                     1.0f - 2.0f * (q11 * q11 + q21 * q21));
    Pitch_b = asinf(2.0f * (q01 * q21 - q31 * q11));
    Yaw_b   = atan2f(2.0f * (q01 * q31 + q11 * q21),
                     1.0f - 2.0f * (q21 * q21 + q31 * q31));

    Roll_b  *= Rad2Ang;
    Pitch_b *= Rad2Ang;
    Yaw_b   *= Rad2Ang;
}

// ==================== 辅助计算函数 ====================
/**
 * @brief 计算角加速度
 */
void Calculate_Angular_Acceleration(void)
{
    Pitch_angular_acc = (Pitch_g_F - Pitch_g_last) * sampleFreq;
    Yaw_angular_acc   = (Yaw_g_F - Yaw_g_last) * sampleFreq;

    Pitch_g_last = Pitch_g_F;
    Roll_g_last  = Roll_g_F;
    Yaw_g_last   = Yaw_g_F;
}

/**
 * @brief 姿态角限幅处理
 */
void Angle_Limit_Process(void)
{
    // Yaw角周期处理(-180° ~ 180°)
    while(Yaw_a > 180.0f) Yaw_a -= 360.0f;
    while(Yaw_a < -180.0f) Yaw_a += 360.0f;

    // Pitch和Roll限幅(-90° ~ 90°)
    if(Pitch_a > 90.0f) Pitch_a = 90.0f;
    if(Pitch_a < -90.0f) Pitch_a = -90.0f;
    if(Roll_a > 90.0f) Roll_a = 90.0f;
    if(Roll_a < -90.0f) Roll_a = -90.0f;

    // 更新历史数据
    Pitch_a_last = Pitch_a;
    Roll_a_last = Roll_a;
    Yaw_a_last = Yaw_a;
}
/**
 * @brief 速度估计(通过加速度积分)
 * @note 需要配合其他传感器融合以避免漂移
 */
void Update_Velocity_Estimate(void)
{
    float ax_level_mg;
    float ay_level_mg;
    float az_level_mg;
    float acc_norm_mg;

    /*
     * Velocity integration uses the raw accelerometer path here.
     * Keep the filtered accelerometer only for attitude estimation.
     */
    Body_To_RollPitch_Level_Frame(Roll_acc, Pitch_acc, Yaw_acc,
                                  &ax_level_mg, &ay_level_mg, &az_level_mg);
    (void)az_level_mg;

    ax_level_mg -= acc_level_x_bias_mg;
    ay_level_mg -= acc_level_y_bias_mg;

    if(fabsf(Roll_g_F) > ACC_VEL_GYRO_GATE_DPS ||
       fabsf(Pitch_g_F) > ACC_VEL_GYRO_GATE_DPS ||
       fabsf(Yaw_g_F) > ACC_VEL_GYRO_GATE_DPS)
    {
        return;
    }

    acc_norm_mg = sqrtf(Roll_acc * Roll_acc +
                        Pitch_acc * Pitch_acc +
                        Yaw_acc * Yaw_acc);
    if(fabsf(acc_norm_mg - 1000.0f) > ACC_VEL_NORM_GATE_MG)
    {
        return;
    }

    if(fabsf(ax_level_mg) < ACC_VEL_DEADBAND_MG)
    {
        ax_level_mg = 0.0f;
    }

    if(fabsf(ay_level_mg) < ACC_VEL_DEADBAND_MG)
    {
        ay_level_mg = 0.0f;
    }

    if(ax_level_mg > ACC_VEL_MAX_INTEGRATE_MG)
    {
        ax_level_mg = ACC_VEL_MAX_INTEGRATE_MG;
    }
    else if(ax_level_mg < -ACC_VEL_MAX_INTEGRATE_MG)
    {
        ax_level_mg = -ACC_VEL_MAX_INTEGRATE_MG;
    }

    if(ay_level_mg > ACC_VEL_MAX_INTEGRATE_MG)
    {
        ay_level_mg = ACC_VEL_MAX_INTEGRATE_MG;
    }
    else if(ay_level_mg < -ACC_VEL_MAX_INTEGRATE_MG)
    {
        ay_level_mg = -ACC_VEL_MAX_INTEGRATE_MG;
    }

    // Acc input is mg. Convert to m/s^2, then integrate to cm/s.
    X_V += ax_level_mg * ACC_MG_TO_MPS2 * ACC_VEL_MPS_TO_CMS * DT;
    Y_V += ay_level_mg * ACC_MG_TO_MPS2 * ACC_VEL_MPS_TO_CMS * DT;

    X_V_F = biquad(&X_V_biquad, X_V);
    Y_V_F = biquad(&Y_V_biquad, Y_V);

    X_V *= ACC_VEL_LEAK;
    Y_V *= ACC_VEL_LEAK;
}

/*
 * Quaternion-based roll/pitch leveling helper. Yaw is removed deliberately so
 * the output x/y axes remain tied to the aircraft heading.
 */
void Body_To_RollPitch_Level_Frame(float bx, float by, float bz,
                                   float *lx, float *ly, float *lz)
{
    float bx_scaled = bx * acc_level_x_scale;
    float by_scaled = by * acc_level_y_scale;
    float bz_scaled = bz * acc_level_z_scale;
    float qw = q0;
    float qx = q1;
    float qy = q2;
    float qz = q3;
    float yaw_rad = atan2f(2.0f * (qw * qz + qx * qy),
                           1.0f - 2.0f * (qy * qy + qz * qz));
    float half_yaw = 0.5f * yaw_rad;
    float cy = cosf(half_yaw);
    float sy = sinf(half_yaw);
    float rw;
    float rx;
    float ry;
    float rz;
    float norm_inv;
    float rw2;
    float rx2;
    float ry2;
    float rz2;

    // q_rp = q_yaw_conj * q_full, q_yaw_conj = [cy, 0, 0, -sy].
    rw = cy * qw + sy * qz;
    rx = cy * qx + sy * qy;
    ry = cy * qy - sy * qx;
    rz = cy * qz - sy * qw;

    norm_inv = invSqrt(rw * rw + rx * rx + ry * ry + rz * rz);
    rw *= norm_inv;
    rx *= norm_inv;
    ry *= norm_inv;
    rz *= norm_inv;

    rw2 = rw * rw;
    rx2 = rx * rx;
    ry2 = ry * ry;
    rz2 = rz * rz;

    *lx = (rw2 + rx2 - ry2 - rz2) * bx_scaled + 2.0f * (rx * ry - rw * rz) * by_scaled
        + 2.0f * (rx * rz + rw * ry) * bz_scaled;
    *ly = 2.0f * (rx * ry + rw * rz) * bx_scaled + (rw2 - rx2 + ry2 - rz2) * by_scaled
        + 2.0f * (ry * rz - rw * rx) * bz_scaled;
    *lz = 2.0f * (rx * rz - rw * ry) * bx_scaled + 2.0f * (ry * rz + rw * rx) * by_scaled
        + (rw2 - rx2 - ry2 + rz2) * bz_scaled;
}

/**
 * @brief 机体坐标系到世界坐标系转换
 */
void Body_To_World_Frame(float bx, float by, float bz,
                         float *wx, float *wy, float *wz)
{
    float q0q0 = q0 * q0, q1q1 = q1 * q1;
    float q2q2 = q2 * q2, q3q3 = q3 * q3;

    *wx = (q0q0 + q1q1 - q2q2 - q3q3) * bx + 2.0f * (q1*q2 - q0*q3) * by
        + 2.0f * (q1*q3 + q0*q2) * bz;
    *wy = 2.0f * (q1*q2 + q0*q3) * bx + (q0q0 - q1q1 + q2q2 - q3q3) * by
        + 2.0f * (q2*q3 - q0*q1) * bz;
    *wz = 2.0f * (q1*q3 - q0*q2) * bx + 2.0f * (q2*q3 + q0*q1) * by
        + (q0q0 - q1q1 - q2q2 + q3q3) * bz;
}

// ==================== 陀螺仪校准 ====================
/**
 * @brief IMU零飘校准
 * @note 设备必须保持静止3秒
 */
static uint8 icm_acc_level_bias_load_from_flash(void)
{
    flash_read_page_to_buffer(ACC_LEVEL_BIAS_FLASH_SECTOR,
                              ACC_LEVEL_BIAS_FLASH_PAGE,
                              ACC_LEVEL_BIAS_FLASH_WORDS);

    acc_level_x_bias_mg = flash_union_buffer[1].float_type;
    acc_level_y_bias_mg = flash_union_buffer[2].float_type;

    return (uint8)(icm_is_valid_acc_level_bias(acc_level_x_bias_mg) &&
                   icm_is_valid_acc_level_bias(acc_level_y_bias_mg));
}

static void icm_acc_level_bias_save_to_flash(void)
{
    flash_buffer_clear();
    flash_union_buffer[1].float_type = acc_level_x_bias_mg;
    flash_union_buffer[2].float_type = acc_level_y_bias_mg;
    flash_write_page_from_buffer(ACC_LEVEL_BIAS_FLASH_SECTOR,
                                 ACC_LEVEL_BIAS_FLASH_PAGE,
                                 ACC_LEVEL_BIAS_FLASH_WORDS);
}

static void icm_acc_level_bias_reset_velocity(void)
{
    X_V = 0.0f;
    Y_V = 0.0f;
    X_V_F = 0.0f;
    Y_V_F = 0.0f;
    X_V_biquad.x1 = 0.0f;
    X_V_biquad.x2 = 0.0f;
    X_V_biquad.y1 = 0.0f;
    X_V_biquad.y2 = 0.0f;
    Y_V_biquad.x1 = 0.0f;
    Y_V_biquad.x2 = 0.0f;
    Y_V_biquad.y1 = 0.0f;
    Y_V_biquad.y2 = 0.0f;
}

static void icm_acc_level_bias_settle_attitude(void)
{
    uint32 settle_count = (ACC_LEVEL_BIAS_SETTLE_MS * ACC_LEVEL_BIAS_SAMPLE_HZ) / 1000U;
    uint32 i;

    ips200_show_string(0, 16*3, "Acc bias settle  ");

    for(i = 0U; i < settle_count; i++)
    {
        get_ICM_data();
        system_delay_us(1000000U / ACC_LEVEL_BIAS_SAMPLE_HZ);
    }

    icm_acc_level_bias_reset_velocity();
}

void Acc_Level_Calibration(void)
{
    uint32 sample_count = (uint32)IMU_calibration_seconds * ACC_LEVEL_BIAS_SAMPLE_HZ;
    uint32 i;
    float ax_level_mg;
    float ay_level_mg;
    float az_level_mg;
    float sum_x_mg = 0.0f;
    float sum_y_mg = 0.0f;

    ips200_show_string(0, 16*3, "Acc bias calib... ");
    icm_acc_level_bias_settle_attitude();
    ips200_show_string(0, 16*3, "Acc bias calib... ");

    for(i = 0U; i < sample_count; i++)
    {
        get_ICM_data();
        Body_To_RollPitch_Level_Frame(Roll_acc_F, Pitch_acc_F, Yaw_acc_F,
                                      &ax_level_mg, &ay_level_mg, &az_level_mg);
        (void)az_level_mg;

        sum_x_mg += ax_level_mg;
        sum_y_mg += ay_level_mg;
        system_delay_us(1000000U / ACC_LEVEL_BIAS_SAMPLE_HZ);
    }

    acc_level_x_bias_mg = sum_x_mg / (float)sample_count;
    acc_level_y_bias_mg = sum_y_mg / (float)sample_count;

    icm_acc_level_bias_save_to_flash();
    icm_acc_level_bias_reset_velocity();

    ips200_show_string(0, 16*3, "Acc bias saved   ");
    ips200_show_float(96, 80, acc_level_x_bias_mg, 3, 3);
    ips200_show_float(96, 104, acc_level_y_bias_mg, 3, 3);
    system_delay_ms(500);
}

static void icm_acc_level_bias_load_or_calibrate(uint8 force_calibration)
{
    if(force_calibration || !icm_acc_level_bias_load_from_flash())
    {
        acc_level_x_bias_mg = 0.0f;
        acc_level_y_bias_mg = 0.0f;
        Acc_Level_Calibration();
        return;
    }

    ips200_show_string(0, 16*3, "Using Acc Bias   ");
    ips200_show_float(96, 80, acc_level_x_bias_mg, 3, 3);
    ips200_show_float(96, 104, acc_level_y_bias_mg, 3, 3);
    icm_acc_level_bias_reset_velocity();
    system_delay_ms(300);
}

void IMU_calibration(void)
{
    uint32 s_cnt = 0, g_cnt = 0;
    float gyro_history[3][IMU_calibration_seconds];
    int32 gyro_history_sum[3] = {0, 0, 0};

    // 校准提示
    ips200_show_string(0, 16*0, "IMU calibration");
    ips200_show_string(0, 16*2, "Don't move me!(2)");
    system_delay_ms(1000);
    ips200_show_string(0, 16*2, "Don't move me!(1)");
    system_delay_ms(1000);

    // 采集数据(8KHz * 3秒)
    do {
        system_delay_us(125);
        Get_RAW_Gyro_ICM42688();

        if(g_cnt < 8000) {
            gyro_history_sum[0] += icm42688_gyro_x_r;
            gyro_history_sum[1] += icm42688_gyro_y_r;
            gyro_history_sum[2] += icm42688_gyro_z_r;
            g_cnt++;
        }
        else {
            gyro_history[0][s_cnt] = (float)gyro_history_sum[0] / 8000.0f;
            gyro_history[1][s_cnt] = (float)gyro_history_sum[1] / 8000.0f;
            gyro_history[2][s_cnt] = (float)gyro_history_sum[2] / 8000.0f;
            gyro_history_sum[0] = 0;
            gyro_history_sum[1] = 0;
            gyro_history_sum[2] = 0;
            g_cnt = 0;
            s_cnt++;
        }
    } while(s_cnt < IMU_calibration_seconds);
    // 计算平均零飘
    float sum[3] = {0, 0, 0};
    float k = icm42688_gyro_inv / IMU_calibration_seconds;

    for(uint8 i = 0; i < IMU_calibration_seconds; i++) {
        sum[0] += gyro_history[0][i];
        sum[1] += gyro_history[1][i];
        sum[2] += gyro_history[2][i];
 
    }
    flash_buffer_clear();
    flash_union_buffer[1].float_type=  gyro_x_correction = k * sum[0];
    flash_union_buffer[2].float_type=gyro_y_correction = k * sum[1];
    flash_union_buffer[3].float_type=gyro_z_correction = k * sum[2];

    ips200_show_float(96, 56, flash_union_buffer[1].float_type, 3, 3);
    ips200_show_float(96, 80, flash_union_buffer[2].float_type, 3, 3);
    ips200_show_float(96, 104, flash_union_buffer[3].float_type, 3, 3);
    ips200_show_string(0, 16*2, "IMU Calibration OK      ");
    flash_write_page_from_buffer(0, 11, 4);
    system_delay_ms(500);

}

// ==================== 姿态控制接口 ====================
/**
 * @brief 锁定当前Yaw角
 */
void Yaw_Lock(void)
{
    Yaw_Lock_a = Yaw_a;
}

/**
 * @brief 获取Yaw角偏差
 * @return Yaw角偏差(°)
 */
float Get_Yaw_Error(void)
{
    float error = Yaw_a - Yaw_Lock_a;

    // 处理角度跳变
    if(error > 180.0f) error -= 360.0f;
    if(error < -180.0f) error += 360.0f;

    return error;
}

/**
 * @brief 复位姿态解算系统
 */
void ICM_Attitude_Reset(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    q01 = 1.0f; q11 = 0.0f; q21 = 0.0f; q31 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    X_V = 0.0f; Y_V = 0.0f;
    X_V_F = 0.0f; Y_V_F = 0.0f;

    system_delay_ms(100);
    Yaw_A0 = Yaw_a_Pi;
}

/**
 * @brief 获取当前姿态角
 */
void Get_Attitude_Angle(float *roll, float *pitch, float *yaw)
{
    *roll = Roll_a;
    *pitch = Pitch_a;
    *yaw = Yaw_a;
}

/**
 * @brief 获取当前角速度
 */
void Get_Angular_Rate(float *roll_rate, float *pitch_rate, float *yaw_rate)
{
    *roll_rate = Roll_g_F;
    *pitch_rate = Pitch_g_F;
    *yaw_rate = Yaw_g_F;
}

/**
 * @brief 数据质量检测
 * @return 0:正常, -1:异常
 */
int Check_IMU_Data_Quality(void)
{
    // 检查加速度范围
    if(acc < 500.0f || acc > 1500.0f) return -1;

    // 检查角速度范围
    if(fabsf(Pitch_g) > 2000.0f || fabsf(Roll_g) > 2000.0f || fabsf(Yaw_g) > 2000.0f)
        return -1;

    // 检查四元数归一化
    float q_norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(fabsf(q_norm - 1.0f) > 0.1f) return -1;

    return 0;
}

/**
 * @brief 显示IMU调试信息
 */
void Display_IMU_Info(void)
{
    ips200_show_float(0, 16*4, Roll_a, 2, 2);
    ips200_show_float(100, 16*4, Pitch_a, 2, 2);
    ips200_show_float(200, 16*4, Yaw_a, 2, 2);

    ips200_show_float(0, 16*5, Roll_g_F, 2, 2);
    ips200_show_float(100, 16*5, Pitch_g_F, 2, 2);
    ips200_show_float(200, 16*5, Yaw_g_F, 2, 2);

    ips200_show_float(0, 16*6, acc, 2, 2);
    ips200_show_float(100, 16*6, twoKp, 2, 3);
}

// ==================== 数学工具函数 ====================
/**
 * @brief 快速平方根倒数(Quake III算法)
 */
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 向量归一化
 */
void normalize_vector(float *v, int n)
{
    float norm = 0.0f;
    for (int i = 0; i < n; i++) {
        norm += v[i] * v[i];
    }
    norm = sqrtf(norm);
    if (norm < 1e-6f) return;
    for (int i = 0; i < n; i++) {
        v[i] /= norm;
    }
}

/**
 * @brief 3x3矩阵求逆
 */
int invert_3x3(const float A[3][3], float Ainv[3][3])
{
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
              - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
              + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (fabsf(det) < 1e-6f) return -1;

    float invdet = 1.0f / det;
    Ainv[0][0] =  (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invdet;
    Ainv[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) * invdet;
    Ainv[0][2] =  (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invdet;
    Ainv[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) * invdet;
    Ainv[1][1] =  (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invdet;
    Ainv[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) * invdet;
    Ainv[2][0] =  (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invdet;
    Ainv[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) * invdet;
    Ainv[2][2] =  (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invdet;

    return 0;
}
