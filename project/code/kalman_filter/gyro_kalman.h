/**
 * @file gyro_kalman.h
 * @brief 陀螺仪角速度卡尔曼滤波接口
 *
 * 当前主流程只使用一维卡尔曼滤波，对 Roll、Pitch、Yaw 三轴角速度分别降噪。
 */

#ifndef GYRO_KALMAN_H_
#define GYRO_KALMAN_H_

#ifdef __cplusplus
extern "C" {
#endif

// 初始化三轴角速度卡尔曼滤波器。
void GyroKalman_Init(void);

// 对三轴角速度执行滤波，并把结果写回输出指针。
void GyroKalman_Update(float roll_gyro_raw, float pitch_gyro_raw, float yaw_gyro_raw,
                       float *roll_gyro_out, float *pitch_gyro_out, float *yaw_gyro_out);

#ifdef __cplusplus
}
#endif

#endif /* GYRO_KALMAN_H_ */