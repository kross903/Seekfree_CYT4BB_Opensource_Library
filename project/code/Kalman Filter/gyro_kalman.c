/**
 * @file gyro_kalman.c
 * @brief 陀螺仪角速度卡尔曼滤波实现
 *
 * 当前主流程中，这个模块只负责三轴角速度的一维卡尔曼滤波。
 * 主流程调用入口：
 * 1. ICM 初始化阶段调用 GyroKalman_Init()
 * 2. 姿态控制循环中通过 GyroKalman_Update() 获取滤波后的角速度
 */

#include "gyro_kalman.h"

// 单轴角速度卡尔曼滤波器状态。
typedef struct {
    float x;  // 当前估计值
    float p;  // 估计协方差
    float q;  // 过程噪声
    float r;  // 测量噪声
    float k;  // 当前卡尔曼增益
} GyroKalman1D_t;

// 三轴滤波器实例，仅供本模块内部使用。
static GyroKalman1D_t roll_gyro_kalman;
static GyroKalman1D_t pitch_gyro_kalman;
static GyroKalman1D_t yaw_gyro_kalman;

// 初始化单轴一维卡尔曼滤波器。
static void gyro_kalman_1d_init(GyroKalman1D_t *kf, float q, float r)
{
    kf->x = 0.0f;
    kf->p = 1.0f;
    kf->q = q;
    kf->r = r;
    kf->k = 0.0f;
}

// 使用一次测量值更新单轴角速度估计。
static float gyro_kalman_1d_update(GyroKalman1D_t *kf, float measurement)
{
    // 预测步骤：当前主流程采用恒定角速度模型。
    kf->p = kf->p + kf->q;

    // 更新步骤：根据测量噪声和预测协方差计算增益。
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1.0f - kf->k) * kf->p;

    return kf->x;
}

void GyroKalman_Init(void)
{
    // 当前参数适用于 8kHz 左右的角速度采样场景。
    gyro_kalman_1d_init(&roll_gyro_kalman, 0.01f, 0.5f);
    gyro_kalman_1d_init(&pitch_gyro_kalman, 0.01f, 0.5f);
    gyro_kalman_1d_init(&yaw_gyro_kalman, 0.01f, 0.5f);
}

void GyroKalman_Update(float roll_gyro_raw, float pitch_gyro_raw, float yaw_gyro_raw,
                       float *roll_gyro_out, float *pitch_gyro_out, float *yaw_gyro_out)
{
    float roll_filtered = gyro_kalman_1d_update(&roll_gyro_kalman, roll_gyro_raw);
    float pitch_filtered = gyro_kalman_1d_update(&pitch_gyro_kalman, pitch_gyro_raw);
    float yaw_filtered = gyro_kalman_1d_update(&yaw_gyro_kalman, yaw_gyro_raw);

    if (roll_gyro_out != 0)
    {
        *roll_gyro_out = roll_filtered;
    }

    if (pitch_gyro_out != 0)
    {
        *pitch_gyro_out = pitch_filtered;
    }

    if (yaw_gyro_out != 0)
    {
        *yaw_gyro_out = yaw_filtered;
    }
}