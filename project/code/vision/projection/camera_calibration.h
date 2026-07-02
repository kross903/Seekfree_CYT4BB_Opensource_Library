/**
 * @file camera_calibration.h
 * @brief 相机标定参数与点去畸变入口
 *
 * 本模块集中管理 MT9V03X 摄像头的内参、畸变参数和安装外参。
 * 当前默认参数保持直通行为，后续完成实际标定后只需要更新这里，
 * IR_Localization 和 BeaconDetect 不直接关心标定细节。
 */

#ifndef CAMERA_CALIBRATION_H_
#define CAMERA_CALIBRATION_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 内参。默认按当前 188x120 图像中心给出占位值。 */
#define CAMERA_FX    94.0f
#define CAMERA_FY    60.0f
#define CAMERA_CX    ((float)(MT9V03X_W - 1) * 0.5f)
#define CAMERA_CY    ((float)(MT9V03X_H - 1) * 0.5f)

/* 畸变参数。默认全 0，表示不做畸变补偿。 */
#define CAMERA_K1    0.0f
#define CAMERA_K2    0.0f
#define CAMERA_P1    0.0f
#define CAMERA_P2    0.0f
#define CAMERA_K3    0.0f

/* 相机相对机体的安装角偏差。 */
#define CAMERA_MOUNT_ROLL_OFFSET_DEG     0.0f
#define CAMERA_MOUNT_PITCH_OFFSET_DEG    0.0f
#define CAMERA_MOUNT_YAW_OFFSET_DEG      0.0f

/* 相机光心相对机体参考点的偏移，单位 cm。 */
#define CAMERA_OFFSET_X_CM               0.0f
#define CAMERA_OFFSET_Y_CM               0.0f
#define CAMERA_OFFSET_Z_CM               0.0f

typedef struct
{
    float fx;
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;
    float p1;
    float p2;
    float k3;
    float mount_roll_offset_deg;
    float mount_pitch_offset_deg;
    float mount_yaw_offset_deg;
    float offset_x_cm;
    float offset_y_cm;
    float offset_z_cm;
} CameraCalibration_t;

extern const CameraCalibration_t g_camera_calibration;

/**
 * @brief 对单个图像点做去畸变
 * @param distorted_x 原始图像列坐标
 * @param distorted_y 原始图像行坐标
 * @param undistorted_x 输出去畸变后的列坐标
 * @param undistorted_y 输出去畸变后的行坐标
 *
 * 当前默认畸变参数为 0，因此该函数保持直通。后续可替换为查表或
 * Brown-Conrady 点去畸变实现，不需要修改 IR / Beacon 调用方。
 */
void CameraCalibration_UndistortPoint(float distorted_x,
                                      float distorted_y,
                                      float *undistorted_x,
                                      float *undistorted_y);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_CALIBRATION_H_ */
