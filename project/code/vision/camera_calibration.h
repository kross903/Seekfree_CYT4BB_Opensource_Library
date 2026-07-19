/**
 * @file camera_calibration.h
 * @brief 相机内参、畸变、安装外参与点级去畸变接口。
 */

#ifndef CAMERA_CALIBRATION_H_
#define CAMERA_CALIBRATION_H_

#include "zf_common_headfile.h"
#include "vision/camera_calibration_params.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CAMERA_CALIBRATION_ENABLE
#define CAMERA_CALIBRATION_ENABLE (1)
#endif

typedef struct
{
    float fx;
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;
    float k3;
    float k4;
    float valid_radius_px;
    float mount_roll_offset_deg;
    float mount_pitch_offset_deg;
    float mount_yaw_offset_deg;
    float offset_x_cm;
    float offset_y_cm;
    float offset_z_cm;
    uint8 has_generated_params;
    uint8 has_ray_table;
    uint8 has_rect_map;
} CameraCalibration_Params_t;

void CameraCalibration_Init(void);
void CameraCalibration_GetParams(CameraCalibration_Params_t *params);

uint8 CameraCalibration_IsPixelInValidRegion(float img_x, float img_y);
uint8 CameraCalibration_PixelToBodyRay(float img_x, float img_y, float ray_body[3]);
uint8 CameraCalibration_BodyRayToPixel(const float ray_body[3], float *img_x, float *img_y);

uint8 CameraCalibration_UndistortPoint(float distorted_x,
                                       float distorted_y,
                                       float *undistorted_x,
                                       float *undistorted_y);

/* 将去畸变坐标反查为原始畸变图中的浮点坐标。 */
uint8 CameraCalibration_RectifiedPointToSource(float rectified_x,
                                               float rectified_y,
                                               float *source_x,
                                               float *source_y);

/* 使用标定模型反查任意去畸变坐标，不受188x120离线映射表边界限制。 */
uint8 CameraCalibration_RectifiedModelPointToSource(float rectified_x,
                                                    float rectified_y,
                                                    float *source_x,
                                                    float *source_y);

uint8 CameraCalibration_RectifiedPixelToSource(int dst_x,
                                               int dst_y,
                                               int *src_x,
                                               int *src_y);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_CALIBRATION_H_ */
