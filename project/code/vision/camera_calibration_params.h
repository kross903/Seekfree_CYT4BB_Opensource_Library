/**
 * @file camera_calibration_params.h
 * @brief 相机标定参数占位文件，由电脑端标定工具生成后覆盖。
 *
 * 当前值只用于保持固件可编译和保留原有近似效果，不代表真实 210 度镜头标定结果。
 */

#ifndef CAMERA_CALIBRATION_PARAMS_H_
#define CAMERA_CALIBRATION_PARAMS_H_

#define CAMERA_CALIBRATION_HAS_GENERATED_PARAMS     (0)

#define CAMERA_CALIBRATION_IMAGE_WIDTH              (188)
#define CAMERA_CALIBRATION_IMAGE_HEIGHT             (120)

/* fallback: 沿用当前工程近似鱼眼对角视场，真实值由离线标定生成。 */
#define CAMERA_CALIBRATION_FALLBACK_DIAGONAL_FOV_DEG (170.0f)

#define CAMERA_CALIBRATION_FX                       (90.0f)
#define CAMERA_CALIBRATION_FY                       (90.0f)
#define CAMERA_CALIBRATION_CX                       (94.0f)
#define CAMERA_CALIBRATION_CY                       (60.0f)

/* OpenCV fisheye / Kannala-Brandt k1-k4。 */
#define CAMERA_CALIBRATION_K1                       (0.0f)
#define CAMERA_CALIBRATION_K2                       (0.0f)
#define CAMERA_CALIBRATION_K3                       (0.0f)
#define CAMERA_CALIBRATION_K4                       (0.0f)

#define CAMERA_CALIBRATION_VALID_RADIUS_PX          (111.5f)

/* 相机相对机体系安装偏差，先保持 0，外参标定后由工具写入。 */
#define CAMERA_MOUNT_ROLL_OFFSET_DEG                (0.0f)
#define CAMERA_MOUNT_PITCH_OFFSET_DEG               (0.0f)
#define CAMERA_MOUNT_YAW_OFFSET_DEG                 (0.0f)

#define CAMERA_OFFSET_X_CM                          (0.0f)
#define CAMERA_OFFSET_Y_CM                          (0.0f)
#define CAMERA_OFFSET_Z_CM                          (0.0f)

#endif /* CAMERA_CALIBRATION_PARAMS_H_ */
