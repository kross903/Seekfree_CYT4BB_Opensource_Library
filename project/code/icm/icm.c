#include "icm.h"
#include "common/project_headfile.h"
#include <math.h>
#include <string.h>

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
#define ICM_DWT_LAR_UNLOCK          (0xC5ACCE55UL)
#define ICM_STALE_IDENTICAL_LIMIT   (16U)
#define ICM_GYRO_SATURATION_DPS     (1950.0f)
#define ICM_HEALTH_ACC_MIN_MG       (400.0f)
#define ICM_HEALTH_ACC_MAX_MG       (1600.0f)
#define ICM_STILL_GYRO_MAX_DPS      (2.0f)
#define ICM_STILL_ACC_MIN_MG        (950.0f)
#define ICM_STILL_ACC_MAX_MG        (1050.0f)
#define MAHONY_KP_MIN               (0.05f)
#define MAHONY_KP_MAX               (2.0f)
#define MAHONY_ACC_FULL_TRUST_MG    (50.0f)
#define MAHONY_ACC_ZERO_TRUST_MG    (300.0f)
#define MAHONY_GYRO_FULL_TRUST_DPS  (8.0f)
#define MAHONY_GYRO_ZERO_TRUST_DPS  (100.0f)
#define MAHONY_NORM_EPSILON         (1.0e-9f)
#define ICM_VISION_YAW_RESIDUAL_GATE_DEG (30.0f)
#define ICM_VISION_YAW_CONFIRM_FRAMES (3U)
#define ICM_VISION_YAW_CORRECTION_ALPHA (0.02f)
#define ICM_VISION_YAW_MAX_STEP_DEG (0.20f)

// ==================== 全局变量定义 ====================
#define twoKpDef    (2.0f * 0.5f)   // Mahony比例增益
#define twoKiDef    (2.0f * 0.0f)   // Mahony积分增益

#define G 9.81f                     // 重力加速度(m/s?)


// Mahony算法参数
volatile float invsampleFreq = ICM_SAMPLE_DT_S;
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
float acc_norm_filtered = 0.0f;            // 姿态解算实际使用的滤波后加速度模值
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

static ICM_Health_t s_icm_health = {0};
static uint8 s_icm_dwt_ready = 0U;
static uint32 s_icm_last_cycle = 0U;
static int16 s_icm_last_raw[6] = {0};
static uint32 s_icm_identical_raw_count = 0U;
static uint32 s_icm_transient_fault_mask = 0U;
static float s_last_valid_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

static uint8 s_vision_yaw_enabled = 0U;
static float s_vision_marker_world_heading_deg = 0.0f;
static float s_vision_camera_yaw_offset_deg = 0.0f;
static uint32 s_vision_yaw_last_sequence = 0U;
static uint8 s_vision_yaw_confirm_count = 0U;

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

static float icm_clamp_float(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

static float icm_wrap_angle_deg(float angle_deg)
{
    while(angle_deg > 180.0f) angle_deg -= 360.0f;
    while(angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static void icm_dwt_init(void)
{
    if(s_icm_dwt_ready)
    {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
    {
        return;
    }

    DWT->LAR = ICM_DWT_LAR_UNLOCK;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_icm_dwt_ready = 1U;
}

static void icm_update_sample_timing(void)
{
    uint32 now_cycle;

    icm_dwt_init();
    if(!s_icm_dwt_ready || SystemCoreClock == 0U)
    {
        return;
    }

    now_cycle = DWT->CYCCNT;
    if(s_icm_last_cycle != 0U)
    {
        s_icm_health.sample_interval_us = ((float)(now_cycle - s_icm_last_cycle) * 1000000.0f) /
                                          (float)SystemCoreClock;
    }
    s_icm_last_cycle = now_cycle;
}

static void icm_update_euler_from_quaternion(void)
{
    float roll_sin = 2.0f * (q0 * q1 + q2 * q3);
    float roll_cos = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    float pitch_sin = 2.0f * (q0 * q2 - q3 * q1);
    float yaw_sin = 2.0f * (q0 * q3 + q1 * q2);
    float yaw_cos = 1.0f - 2.0f * (q2 * q2 + q3 * q3);

    pitch_sin = icm_clamp_float(pitch_sin, -1.0f, 1.0f);
    Roll_a_Pi = atan2f(roll_sin, roll_cos);
    Pitch_a_Pi = asinf(pitch_sin);
    Yaw_a_Pi = atan2f(yaw_sin, yaw_cos);
}

static void icm_set_quaternion_from_level_accel(float ax, float ay, float az)
{
    float roll_rad;
    float pitch_rad;
    float half_roll;
    float half_pitch;
    float cr;
    float sr;
    float cp;
    float sp;
    float norm;

    roll_rad = atan2f(ay, az);
    pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));
    half_roll = 0.5f * roll_rad;
    half_pitch = 0.5f * pitch_rad;
    cr = cosf(half_roll);
    sr = sinf(half_roll);
    cp = cosf(half_pitch);
    sp = sinf(half_pitch);

    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;

    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if(isfinite(norm) && norm > MAHONY_NORM_EPSILON)
    {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
        s_last_valid_q[0] = q0;
        s_last_valid_q[1] = q1;
        s_last_valid_q[2] = q2;
        s_last_valid_q[3] = q3;
        icm_update_euler_from_quaternion();
    }
}

static void icm_refresh_health(void)
{
    uint32 fault_mask = s_icm_transient_fault_mask;
    float quaternion_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    uint8 gyro_saturated;

    s_icm_transient_fault_mask = 0U;
    gyro_saturated = (uint8)((fabsf(Roll_g) >= ICM_GYRO_SATURATION_DPS) ||
                             (fabsf(Pitch_g) >= ICM_GYRO_SATURATION_DPS) ||
                             (fabsf(Yaw_g) >= ICM_GYRO_SATURATION_DPS));

    if(!isfinite(Roll_acc) || !isfinite(Pitch_acc) || !isfinite(Yaw_acc) ||
       !isfinite(Roll_g) || !isfinite(Pitch_g) || !isfinite(Yaw_g) ||
       !isfinite(Roll_a) || !isfinite(Pitch_a) || !isfinite(Yaw_a))
    {
        fault_mask |= ICM_HEALTH_FAULT_NONFINITE;
    }
    if(!isfinite(acc_norm_filtered) ||
       acc_norm_filtered < ICM_HEALTH_ACC_MIN_MG ||
       acc_norm_filtered > ICM_HEALTH_ACC_MAX_MG)
    {
        fault_mask |= ICM_HEALTH_FAULT_ACC_NORM;
    }
    if(gyro_saturated)
    {
        fault_mask |= ICM_HEALTH_FAULT_GYRO_SAT;
    }
    if(!isfinite(quaternion_norm) || quaternion_norm < 0.90f || quaternion_norm > 1.10f)
    {
        fault_mask |= ICM_HEALTH_FAULT_QUATERNION;
    }
    if(s_icm_identical_raw_count >= ICM_STALE_IDENTICAL_LIMIT)
    {
        fault_mask |= ICM_HEALTH_FAULT_SAMPLE_STALE;
    }
    s_icm_health.sample_sequence++;
    s_icm_health.acc_norm_mg = acc_norm_filtered;
    s_icm_health.quaternion_norm = quaternion_norm;
    s_icm_health.gyro_saturated = gyro_saturated;
    s_icm_health.sample_stale = (uint8)((fault_mask & ICM_HEALTH_FAULT_SAMPLE_STALE) != 0U);
    s_icm_health.attitude_valid = (uint8)(fault_mask == 0U);
    s_icm_health.fault_mask = fault_mask;

    if(fault_mask != 0U)
    {
        if(s_icm_health.consecutive_invalid_count < 0xFFFFFFFFU)
        {
            s_icm_health.consecutive_invalid_count++;
        }
    }
    else
    {
        s_icm_health.consecutive_invalid_count = 0U;
    }
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
    target_angle_yaw = Angle_WrapDeg180(sum[ORIGIN_AXIS_YAW] / (float)IMU_calibration_seconds);
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
    s_last_valid_q[0] = 1.0f;
    s_last_valid_q[1] = 0.0f;
    s_last_valid_q[2] = 0.0f;
    s_last_valid_q[3] = 0.0f;
    memset(&s_icm_health, 0, sizeof(s_icm_health));
    s_icm_last_cycle = 0U;
    s_icm_identical_raw_count = 0U;
    ICM_VisionYaw_Configure(0U, 0.0f, 0.0f);

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
    

    // 5. 预热传感器与数字滤波器，并直接使用当前重力方向初始化 Roll/Pitch 四元数。
    ips200_show_string(0, 16*3, "Warming up...");
    for(int i = 0; i < 400; i++) {
        get_ICM_data();
        system_delay_us((uint32)(ICM_SAMPLE_DT_S * 1000000.0f));
    }

    if(isfinite(acc_norm_filtered) &&
       acc_norm_filtered >= ICM_HEALTH_ACC_MIN_MG &&
       acc_norm_filtered <= ICM_HEALTH_ACC_MAX_MG)
    {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        icm_set_quaternion_from_level_accel(Roll_acc_F, Pitch_acc_F, Yaw_acc_F);
        Roll_a = Roll_a_Pi * Rad2Ang;
        Pitch_a = Pitch_a_Pi * Rad2Ang;
        Yaw_a = Yaw_a_Pi * Rad2Ang;
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
    // 速度滤波器(4kHz采样, 10Hz截止频率)
    biquad_filter_init(&X_V_biquad, BIQUAD_LOWPASS, (int)ICM_SAMPLE_RATE_HZ, 10, 0.7071f);
    biquad_filter_init(&Y_V_biquad, BIQUAD_LOWPASS, (int)ICM_SAMPLE_RATE_HZ, 10, 0.7071f);

    // 角速度滤波器(4000Hz采样, 120Hz截止频率)
    biquad_filter_init(&Pitch_g_biquad, BIQUAD_LOWPASS, (int)ICM_SAMPLE_RATE_HZ, 120, 0.7071f);
    biquad_filter_init(&Roll_g_biquad, BIQUAD_LOWPASS, (int)ICM_SAMPLE_RATE_HZ, 120, 0.7071f);
    biquad_filter_init(&Yaw_g_biquad, BIQUAD_LOWPASS, (int)ICM_SAMPLE_RATE_HZ, 120, 0.7071f);

    // 加速度滤波器(4000Hz采样, 30Hz截止频率)
    biquad_filter_init(&Pitch_acc_biquad, BIQUAD_LOWPASS, (int)ICM_SAMPLE_RATE_HZ, 30, 0.7071f);
    biquad_filter_init(&Roll_acc_biquad, BIQUAD_LOWPASS, (int)ICM_SAMPLE_RATE_HZ, 30, 0.7071f);
    biquad_filter_init(&Yaw_acc_biquad, BIQUAD_LOWPASS, (int)ICM_SAMPLE_RATE_HZ, 30, 0.7071f);
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
 * @note 在 4 kHz 控制定时器中断中调用
 */
void get_ICM_data(void)
{
    int16 current_raw[6];
    uint8 raw_identical = 1U;

    icm_update_sample_timing();

    // 1. 读取原始数据
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();

    current_raw[0] = icm42688_acc_x_r;
    current_raw[1] = icm42688_acc_y_r;
    current_raw[2] = icm42688_acc_z_r;
    current_raw[3] = icm42688_gyro_x_r;
    current_raw[4] = icm42688_gyro_y_r;
    current_raw[5] = icm42688_gyro_z_r;
    for(uint8 raw_index = 0U; raw_index < 6U; raw_index++)
    {
        if(current_raw[raw_index] != s_icm_last_raw[raw_index])
        {
            raw_identical = 0U;
        }
        s_icm_last_raw[raw_index] = current_raw[raw_index];
    }
    if(raw_identical)
    {
        if(s_icm_identical_raw_count < 0xFFFFFFFFU)
        {
            s_icm_identical_raw_count++;
        }
    }
    else
    {
        s_icm_identical_raw_count = 0U;
    }
    s_icm_health.stale_count = s_icm_identical_raw_count;

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

    alpha_gx = (gx - prev_gx) * ICM_SAMPLE_RATE_HZ;
    alpha_gy = (gy - prev_gy) * ICM_SAMPLE_RATE_HZ;
    alpha_gz = (gz - prev_gz) * ICM_SAMPLE_RATE_HZ;

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
    acc_norm_filtered = sqrtf(Roll_acc_F * Roll_acc_F +
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
                        Roll_acc_F, Pitch_acc_F, Yaw_acc_F, acc_norm_filtered);

    // 6. 转换为欧拉角(度)
    Pitch_a = Rad2Ang * Pitch_a_Pi;
    Roll_a  = Rad2Ang * Roll_a_Pi;
    Yaw_a   = Rad2Ang * Yaw_a_Pi;

    // 可选: 使用Madgwick算法作为对比
    // updateMadgwickEulerIMU(icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z,
    //                        Roll_acc_F, Pitch_acc_F, Yaw_acc_F, ICM_SAMPLE_DT_S);

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
    icm_refresh_health();
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
    float gyro_abs_max_dps;
    float acc_norm_sq;
    float acc_norm_error_mg;
    float acc_confidence;
    float gyro_confidence;
    float correction_confidence;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfex;
    float halfey;
    float halfez;
    float qa;
    float qb;
    float qc;
    float norm;
    uint8 accel_valid;

    if(!isfinite(gx) || !isfinite(gy) || !isfinite(gz))
    {
        s_icm_transient_fault_mask |= ICM_HEALTH_FAULT_NONFINITE;
        return;
    }

    gyro_abs_max_dps = fabsf(gx);
    if(fabsf(gy) > gyro_abs_max_dps) gyro_abs_max_dps = fabsf(gy);
    if(fabsf(gz) > gyro_abs_max_dps) gyro_abs_max_dps = fabsf(gz);

    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;

    acc_norm_sq = ax * ax + ay * ay + az * az;
    accel_valid = (uint8)(isfinite(ax) && isfinite(ay) && isfinite(az) &&
                          isfinite(acc_norm) && acc_norm_sq > MAHONY_NORM_EPSILON);

    if(accel_valid)
    {
        norm = sqrtf(acc_norm_sq);
        ax /= norm;
        ay /= norm;
        az /= norm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;

        acc_norm_error_mg = fabsf(acc_norm - 1000.0f);
        acc_confidence = 1.0f - (acc_norm_error_mg - MAHONY_ACC_FULL_TRUST_MG) /
                                  (MAHONY_ACC_ZERO_TRUST_MG - MAHONY_ACC_FULL_TRUST_MG);
        gyro_confidence = 1.0f - (gyro_abs_max_dps - MAHONY_GYRO_FULL_TRUST_DPS) /
                                   (MAHONY_GYRO_ZERO_TRUST_DPS - MAHONY_GYRO_FULL_TRUST_DPS);
        acc_confidence = icm_clamp_float(acc_confidence, 0.0f, 1.0f);
        gyro_confidence = icm_clamp_float(gyro_confidence, 0.0f, 1.0f);
        correction_confidence = acc_confidence * gyro_confidence;
        twoKp = MAHONY_KP_MIN + correction_confidence * (MAHONY_KP_MAX - MAHONY_KP_MIN);

        if(twoKi > 0.0f && correction_confidence > 0.0f)
        {
            integralFBx += twoKi * halfex * invsampleFreq * correction_confidence;
            integralFBy += twoKi * halfey * invsampleFreq * correction_confidence;
            integralFBz += twoKi * halfez * invsampleFreq * correction_confidence;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    else
    {
        twoKp = MAHONY_KP_MIN;
        s_icm_transient_fault_mask |= ICM_HEALTH_FAULT_ACC_NORM;
    }

    gx *= 0.5f * ICM_SAMPLE_DT_S;
    gy *= 0.5f * ICM_SAMPLE_DT_S;
    gz *= 0.5f * ICM_SAMPLE_DT_S;
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += -qb * gx - qc * gy - q3 * gz;
    q1 +=  qa * gx + qc * gz - q3 * gy;
    q2 +=  qa * gy - qb * gz + q3 * gx;
    q3 +=  qa * gz + qb * gy - qc * gx;

    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if(!isfinite(norm) || norm <= MAHONY_NORM_EPSILON)
    {
        q0 = s_last_valid_q[0];
        q1 = s_last_valid_q[1];
        q2 = s_last_valid_q[2];
        q3 = s_last_valid_q[3];
        s_icm_transient_fault_mask |= ICM_HEALTH_FAULT_QUATERNION;
    }
    else
    {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
        s_last_valid_q[0] = q0;
        s_last_valid_q[1] = q1;
        s_last_valid_q[2] = q2;
        s_last_valid_q[3] = q3;
    }

    icm_update_euler_from_quaternion();
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
    Pitch_angular_acc = (Pitch_g_F - Pitch_g_last) * ICM_SAMPLE_RATE_HZ;
    Yaw_angular_acc   = (Yaw_g_F - Yaw_g_last) * ICM_SAMPLE_RATE_HZ;

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
    X_V += ax_level_mg * ACC_MG_TO_MPS2 * ACC_VEL_MPS_TO_CMS * ICM_SAMPLE_DT_S;
    Y_V += ay_level_mg * ACC_MG_TO_MPS2 * ACC_VEL_MPS_TO_CMS * ICM_SAMPLE_DT_S;

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

    // 按当前 4 kHz ODR 采集，每组累积 1 秒。
    do {
        system_delay_us((uint32)(ICM_SAMPLE_DT_S * 1000000.0f));
        Get_RAW_Gyro_ICM42688();

        gyro_history_sum[0] += icm42688_gyro_x_r;
        gyro_history_sum[1] += icm42688_gyro_y_r;
        gyro_history_sum[2] += icm42688_gyro_z_r;
        g_cnt++;
        if(g_cnt >= (uint32)ICM_SAMPLE_RATE_HZ) {
            gyro_history[0][s_cnt] = (float)gyro_history_sum[0] / ICM_SAMPLE_RATE_HZ;
            gyro_history[1][s_cnt] = (float)gyro_history_sum[1] / ICM_SAMPLE_RATE_HZ;
            gyro_history[2][s_cnt] = (float)gyro_history_sum[2] / ICM_SAMPLE_RATE_HZ;
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

uint8 ICM_IsStill(void)
{
    if(!isfinite(Roll_g_F) || !isfinite(Pitch_g_F) || !isfinite(Yaw_g_F) ||
       !isfinite(acc_norm_filtered))
    {
        return 0U;
    }

    return (uint8)(fabsf(Roll_g_F) < ICM_STILL_GYRO_MAX_DPS &&
                   fabsf(Pitch_g_F) < ICM_STILL_GYRO_MAX_DPS &&
                   fabsf(Yaw_g_F) < ICM_STILL_GYRO_MAX_DPS &&
                   acc_norm_filtered >= ICM_STILL_ACC_MIN_MG &&
                   acc_norm_filtered <= ICM_STILL_ACC_MAX_MG &&
                   !s_icm_health.sample_stale);
}

void ICM_GetHealth(ICM_Health_t *health)
{
    uint32 irq_state;

    if(health == NULL)
    {
        return;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    *health = s_icm_health;
    if(irq_state == 0U)
    {
        __enable_irq();
    }
}

uint8 ICM_Health_IsArmable(void)
{
    return (uint8)(s_icm_health.attitude_valid &&
                   s_icm_health.fault_mask == 0U &&
                   s_icm_health.consecutive_invalid_count == 0U);
}

void ICM_VisionYaw_Configure(uint8 enabled,
                             float marker_world_heading_deg,
                             float camera_yaw_offset_deg)
{
    s_vision_yaw_enabled = enabled ? 1U : 0U;
    s_vision_marker_world_heading_deg = icm_wrap_angle_deg(marker_world_heading_deg);
    s_vision_camera_yaw_offset_deg = icm_wrap_angle_deg(camera_yaw_offset_deg);
    s_vision_yaw_last_sequence = 0U;
    s_vision_yaw_confirm_count = 0U;
}

void ICM_VisionYaw_Update(float marker_heading_deg, uint8 valid, uint32 update_sequence)
{
    float observed_yaw_deg;
    float residual_deg;
    float correction_deg;
    float half_correction_rad;
    float cw;
    float sw;
    float old_q0;
    float old_q1;
    float old_q2;
    float old_q3;
    float norm;

    if(!s_vision_yaw_enabled || update_sequence == s_vision_yaw_last_sequence)
    {
        return;
    }
    s_vision_yaw_last_sequence = update_sequence;

    if(!valid || !isfinite(marker_heading_deg) || !s_icm_health.attitude_valid)
    {
        s_vision_yaw_confirm_count = 0U;
        return;
    }

    observed_yaw_deg = icm_wrap_angle_deg(s_vision_marker_world_heading_deg -
                                          marker_heading_deg +
                                          s_vision_camera_yaw_offset_deg);
    residual_deg = icm_wrap_angle_deg(observed_yaw_deg - Yaw_a);
    if(fabsf(residual_deg) > ICM_VISION_YAW_RESIDUAL_GATE_DEG)
    {
        s_vision_yaw_confirm_count = 0U;
        return;
    }

    if(s_vision_yaw_confirm_count < ICM_VISION_YAW_CONFIRM_FRAMES)
    {
        s_vision_yaw_confirm_count++;
        return;
    }

    correction_deg = icm_clamp_float(residual_deg * ICM_VISION_YAW_CORRECTION_ALPHA,
                                     -ICM_VISION_YAW_MAX_STEP_DEG,
                                     ICM_VISION_YAW_MAX_STEP_DEG);
    half_correction_rad = 0.5f * correction_deg * DEG2RAD;
    cw = cosf(half_correction_rad);
    sw = sinf(half_correction_rad);
    old_q0 = q0;
    old_q1 = q1;
    old_q2 = q2;
    old_q3 = q3;

    q0 = cw * old_q0 - sw * old_q3;
    q1 = cw * old_q1 - sw * old_q2;
    q2 = cw * old_q2 + sw * old_q1;
    q3 = cw * old_q3 + sw * old_q0;
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if(!isfinite(norm) || norm <= MAHONY_NORM_EPSILON)
    {
        q0 = old_q0;
        q1 = old_q1;
        q2 = old_q2;
        q3 = old_q3;
        s_icm_transient_fault_mask |= ICM_HEALTH_FAULT_QUATERNION;
        return;
    }

    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
    s_last_valid_q[0] = q0;
    s_last_valid_q[1] = q1;
    s_last_valid_q[2] = q2;
    s_last_valid_q[3] = q3;
    icm_update_euler_from_quaternion();
    Roll_a = Roll_a_Pi * Rad2Ang;
    Pitch_a = Pitch_a_Pi * Rad2Ang;
    Yaw_a = Yaw_a_Pi * Rad2Ang;
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
    s_last_valid_q[0] = 1.0f;
    s_last_valid_q[1] = 0.0f;
    s_last_valid_q[2] = 0.0f;
    s_last_valid_q[3] = 0.0f;
}

/**
 * @brief 获取当前姿态角
 */
void Get_Attitude_Angle(float *roll, float *pitch, float *yaw)
{
    if(roll != NULL) *roll = Roll_a;
    if(pitch != NULL) *pitch = Pitch_a;
    if(yaw != NULL) *yaw = Yaw_a;
}

/**
 * @brief 获取当前角速度
 */
void Get_Angular_Rate(float *roll_rate, float *pitch_rate, float *yaw_rate)
{
    if(roll_rate != NULL) *roll_rate = Roll_g_F;
    if(pitch_rate != NULL) *pitch_rate = Pitch_g_F;
    if(yaw_rate != NULL) *yaw_rate = Yaw_g_F;
}

/**
 * @brief 数据质量检测
 * @return 0:正常, -1:异常
 */
int Check_IMU_Data_Quality(void)
{
    return (s_icm_health.fault_mask == ICM_HEALTH_FAULT_NONE) ? 0 : -1;
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
    if(!isfinite(x) || x <= MAHONY_NORM_EPSILON)
    {
        return 0.0f;
    }
    return 1.0f / sqrtf(x);
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
