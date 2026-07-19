/**
 * @file camera_calibration_params.h
 * @brief 由 scripts/generate_firmware_headers.py 自动生成。
 *
 * 初版标定：10 张有效图片，RMS 0.198817 px。
 * 相机安装角和相对机体位置不是棋盘格内参标定的输出，当前为 0，需实机测量。
 */

#ifndef CAMERA_CALIBRATION_PARAMS_H_
#define CAMERA_CALIBRATION_PARAMS_H_

#define CAMERA_CALIBRATION_HAS_GENERATED_PARAMS     (1)
#define CAMERA_CALIBRATION_IMAGE_WIDTH              (188)
#define CAMERA_CALIBRATION_IMAGE_HEIGHT             (120)
#define CAMERA_CALIBRATION_FALLBACK_DIAGONAL_FOV_DEG (170.0f)
#define CAMERA_CALIBRATION_FX                       (79.9405212f)
#define CAMERA_CALIBRATION_FY                       (80.064407f)
#define CAMERA_CALIBRATION_CX                       (96.0476949f)
#define CAMERA_CALIBRATION_CY                       (59.6924987f)
#define CAMERA_CALIBRATION_K1                       (-0.00971921042f)
#define CAMERA_CALIBRATION_K2                       (-0.0145698917f)
#define CAMERA_CALIBRATION_K3                       (0.00376024812f)
#define CAMERA_CALIBRATION_K4                       (0.0013695079f)
#define CAMERA_CALIBRATION_VALID_RADIUS_PX          (0.0f)
#define CAMERA_MOUNT_ROLL_OFFSET_DEG                (0.0f)
#define CAMERA_MOUNT_PITCH_OFFSET_DEG               (0.0f)
#define CAMERA_MOUNT_YAW_OFFSET_DEG                 (0.0f)
#define CAMERA_OFFSET_X_CM                          (0.0f)
#define CAMERA_OFFSET_Y_CM                          (0.0f)
#define CAMERA_OFFSET_Z_CM                          (0.0f)

#endif /* CAMERA_CALIBRATION_PARAMS_H_ */
