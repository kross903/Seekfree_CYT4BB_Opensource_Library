#include "vision/projection/camera_calibration.h"
#include "common/project_headfile.h"

const CameraCalibration_t g_camera_calibration = {
    .fx = CAMERA_FX,
    .fy = CAMERA_FY,
    .cx = CAMERA_CX,
    .cy = CAMERA_CY,
    .k1 = CAMERA_K1,
    .k2 = CAMERA_K2,
    .p1 = CAMERA_P1,
    .p2 = CAMERA_P2,
    .k3 = CAMERA_K3,
    .mount_roll_offset_deg = CAMERA_MOUNT_ROLL_OFFSET_DEG,
    .mount_pitch_offset_deg = CAMERA_MOUNT_PITCH_OFFSET_DEG,
    .mount_yaw_offset_deg = CAMERA_MOUNT_YAW_OFFSET_DEG,
    .offset_x_cm = CAMERA_OFFSET_X_CM,
    .offset_y_cm = CAMERA_OFFSET_Y_CM,
    .offset_z_cm = CAMERA_OFFSET_Z_CM
};

void CameraCalibration_UndistortPoint(float distorted_x,
                                      float distorted_y,
                                      float *undistorted_x,
                                      float *undistorted_y)
{
    if(undistorted_x != NULL)
    {
        *undistorted_x = distorted_x;
    }
    if(undistorted_y != NULL)
    {
        *undistorted_y = distorted_y;
    }
}
