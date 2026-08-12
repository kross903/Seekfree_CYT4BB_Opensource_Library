#include "vision/camera_calibration.h"
#include "vision/camera_calibration_params.h"
#include "vision/camera_ray_table.h"
#include "vision/camera_undistort_rect_map.h"
#include <math.h>
#include <string.h>

#ifndef PI_F
#define PI_F (3.14159265f)
#endif

#define CAMERA_CALIBRATION_DEG2RAD (PI_F / 180.0f)

#define CAMERA_CALIBRATION_IMG_CENTER_X (((float)MT9V03X_W) * 0.5f)
#define CAMERA_CALIBRATION_IMG_CENTER_Y (((float)MT9V03X_H) * 0.5f)
#define CAMERA_CALIBRATION_IMG_HALF_DIAG \
    (0.5f * sqrtf(((float)MT9V03X_W * (float)MT9V03X_W) + \
                  ((float)MT9V03X_H * (float)MT9V03X_H)))

/* 主摄绕光轴旋转180度安装：X/Y反向，Z方向保持不变。 */
static const float s_camera_mount_rotation[3][3] =
{
    {-1.0f, 0.0f, 0.0f},
    {0.0f, -1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static void CameraCalibration_ApplyMountOffset(float ray[3])
{
    float x = ray[0];
    float y = ray[1];
    float z = ray[2];

    ray[0] = s_camera_mount_rotation[0][0] * x +
             s_camera_mount_rotation[0][1] * y +
             s_camera_mount_rotation[0][2] * z;
    ray[1] = s_camera_mount_rotation[1][0] * x +
             s_camera_mount_rotation[1][1] * y +
             s_camera_mount_rotation[1][2] * z;
    ray[2] = s_camera_mount_rotation[2][0] * x +
             s_camera_mount_rotation[2][1] * y +
             s_camera_mount_rotation[2][2] * z;
}

static void CameraCalibration_RemoveMountOffset(float ray[3])
{
    float x = ray[0];
    float y = ray[1];
    float z = ray[2];

    ray[0] = s_camera_mount_rotation[0][0] * x +
             s_camera_mount_rotation[1][0] * y +
             s_camera_mount_rotation[2][0] * z;
    ray[1] = s_camera_mount_rotation[0][1] * x +
             s_camera_mount_rotation[1][1] * y +
             s_camera_mount_rotation[2][1] * z;
    ray[2] = s_camera_mount_rotation[0][2] * x +
             s_camera_mount_rotation[1][2] * y +
             s_camera_mount_rotation[2][2] * z;
}

static uint8 CameraCalibration_SolveFisheyeTheta(float theta_d, float *theta)
{
    float t = theta_d;
    uint8 i = 0U;

    if(theta == NULL)
    {
        return 0U;
    }

    for(i = 0U; i < 8U; i++)
    {
        float t2 = t * t;
        float t4 = t2 * t2;
        float t6 = t4 * t2;
        float t8 = t4 * t4;
        float f = t * (1.0f +
                       CAMERA_CALIBRATION_K1 * t2 +
                       CAMERA_CALIBRATION_K2 * t4 +
                       CAMERA_CALIBRATION_K3 * t6 +
                       CAMERA_CALIBRATION_K4 * t8) - theta_d;
        float df = 1.0f +
                   3.0f * CAMERA_CALIBRATION_K1 * t2 +
                   5.0f * CAMERA_CALIBRATION_K2 * t4 +
                   7.0f * CAMERA_CALIBRATION_K3 * t6 +
                   9.0f * CAMERA_CALIBRATION_K4 * t8;

        if(fabsf(df) < 1e-6f)
        {
            break;
        }
        t -= f / df;
        if(t < 0.0f)
        {
            t = 0.0f;
        }
    }

    *theta = t;
    return 1U;
}

static uint8 CameraCalibration_ParamsPixelToRay(float img_x, float img_y, float ray_body[3])
{
    float mx = 0.0f;
    float my = 0.0f;
    float theta_d = 0.0f;
    float theta = 0.0f;
    float sin_theta = 0.0f;
    float cos_theta = 0.0f;
    float scale = 0.0f;

    if(ray_body == NULL || !CameraCalibration_IsPixelInValidRegion(img_x, img_y))
    {
        return 0U;
    }

    mx = (img_x - CAMERA_CALIBRATION_CX) / CAMERA_CALIBRATION_FX;
    my = (img_y - CAMERA_CALIBRATION_CY) / CAMERA_CALIBRATION_FY;
    theta_d = sqrtf((mx * mx) + (my * my));

    if(theta_d < 1e-6f)
    {
        ray_body[0] = 0.0f;
        ray_body[1] = 0.0f;
        ray_body[2] = 1.0f;
        CameraCalibration_ApplyMountOffset(ray_body);
        return 1U;
    }

    if(!CameraCalibration_SolveFisheyeTheta(theta_d, &theta))
    {
        return 0U;
    }

    sin_theta = sinf(theta);
    cos_theta = cosf(theta);
    scale = sin_theta / theta_d;

    ray_body[0] = my * scale;
    ray_body[1] = mx * scale;
    ray_body[2] = cos_theta;
    CameraCalibration_ApplyMountOffset(ray_body);
    return 1U;
}

static uint8 CameraCalibration_FallbackPixelToRay(float img_x, float img_y, float ray_body[3])
{
    float du = img_x - CAMERA_CALIBRATION_IMG_CENTER_X;
    float dv = img_y - CAMERA_CALIBRATION_IMG_CENTER_Y;
    float radius = sqrtf((du * du) + (dv * dv));
    float half_fov_rad = 0.5f * CAMERA_CALIBRATION_FALLBACK_DIAGONAL_FOV_DEG * CAMERA_CALIBRATION_DEG2RAD;
    float theta = 0.0f;
    float sin_theta = 0.0f;
    float cos_theta = 0.0f;
    float scale = 0.0f;

    if(ray_body == NULL || !CameraCalibration_IsPixelInValidRegion(img_x, img_y))
    {
        return 0U;
    }

    if(radius < 1e-6f)
    {
        ray_body[0] = 0.0f;
        ray_body[1] = 0.0f;
        ray_body[2] = 1.0f;
        CameraCalibration_ApplyMountOffset(ray_body);
        return 1U;
    }

    theta = radius * half_fov_rad / CAMERA_CALIBRATION_IMG_HALF_DIAG;
    sin_theta = sinf(theta);
    cos_theta = cosf(theta);
    scale = sin_theta / radius;

    ray_body[0] = dv * scale;
    ray_body[1] = du * scale;
    ray_body[2] = cos_theta;
    CameraCalibration_ApplyMountOffset(ray_body);
    return 1U;
}

#if CAMERA_RAY_TABLE_HAS_DATA
static uint8 CameraCalibration_TablePixelToRay(float img_x, float img_y, float ray_body[3])
{
    int x0 = (int)floorf(img_x);
    int y0 = (int)floorf(img_y);
    int x1 = 0;
    int y1 = 0;
    float tx = img_x - (float)x0;
    float ty = img_y - (float)y0;
    float inv_q = 1.0f / (float)(1L << CAMERA_RAY_TABLE_Q);
    float rx = 0.0f;
    float ry = 0.0f;
    float rz = 0.0f;
    float len = 0.0f;

    if(ray_body == NULL ||
       img_x < 0.0f || img_x > ((float)MT9V03X_W - 1.0f) ||
       img_y < 0.0f || img_y > ((float)MT9V03X_H - 1.0f))
    {
        return 0U;
    }

    x1 = (x0 + 1 < MT9V03X_W) ? (x0 + 1) : x0;
    y1 = (y0 + 1 < MT9V03X_H) ? (y0 + 1) : y0;

    if(camera_ray_table_valid[y0][x0] == 0U ||
       camera_ray_table_valid[y0][x1] == 0U ||
       camera_ray_table_valid[y1][x0] == 0U ||
       camera_ray_table_valid[y1][x1] == 0U)
    {
        return 0U;
    }

    rx = ((1.0f - tx) * (1.0f - ty) * (float)camera_ray_table_x[y0][x0]) +
         (tx * (1.0f - ty) * (float)camera_ray_table_x[y0][x1]) +
         ((1.0f - tx) * ty * (float)camera_ray_table_x[y1][x0]) +
         (tx * ty * (float)camera_ray_table_x[y1][x1]);
    ry = ((1.0f - tx) * (1.0f - ty) * (float)camera_ray_table_y[y0][x0]) +
         (tx * (1.0f - ty) * (float)camera_ray_table_y[y0][x1]) +
         ((1.0f - tx) * ty * (float)camera_ray_table_y[y1][x0]) +
         (tx * ty * (float)camera_ray_table_y[y1][x1]);
    rz = ((1.0f - tx) * (1.0f - ty) * (float)camera_ray_table_z[y0][x0]) +
         (tx * (1.0f - ty) * (float)camera_ray_table_z[y0][x1]) +
         ((1.0f - tx) * ty * (float)camera_ray_table_z[y1][x0]) +
         (tx * ty * (float)camera_ray_table_z[y1][x1]);

    ray_body[0] = rx * inv_q;
    ray_body[1] = ry * inv_q;
    ray_body[2] = rz * inv_q;
    len = sqrtf((ray_body[0] * ray_body[0]) +
                (ray_body[1] * ray_body[1]) +
                (ray_body[2] * ray_body[2]));
    if(len < 1e-6f)
    {
        return 0U;
    }

    ray_body[0] /= len;
    ray_body[1] /= len;
    ray_body[2] /= len;
    CameraCalibration_ApplyMountOffset(ray_body);
    return 1U;
}
#endif

void CameraCalibration_Init(void)
{
}

void CameraCalibration_GetParams(CameraCalibration_Params_t *params)
{
    if(params == NULL)
    {
        return;
    }

    memset(params, 0, sizeof(*params));
    params->fx = CAMERA_CALIBRATION_FX;
    params->fy = CAMERA_CALIBRATION_FY;
    params->cx = CAMERA_CALIBRATION_CX;
    params->cy = CAMERA_CALIBRATION_CY;
    params->k1 = CAMERA_CALIBRATION_K1;
    params->k2 = CAMERA_CALIBRATION_K2;
    params->k3 = CAMERA_CALIBRATION_K3;
    params->k4 = CAMERA_CALIBRATION_K4;
    params->valid_radius_px = CAMERA_CALIBRATION_VALID_RADIUS_PX;
    params->mount_roll_offset_deg = CAMERA_MOUNT_ROLL_OFFSET_DEG;
    params->mount_pitch_offset_deg = CAMERA_MOUNT_PITCH_OFFSET_DEG;
    params->mount_yaw_offset_deg = CAMERA_MOUNT_YAW_OFFSET_DEG;
    params->offset_x_cm = CAMERA_OFFSET_X_CM;
    params->offset_y_cm = CAMERA_OFFSET_Y_CM;
    params->offset_z_cm = CAMERA_OFFSET_Z_CM;
    params->has_generated_params = (uint8)CAMERA_CALIBRATION_HAS_GENERATED_PARAMS;
    params->has_ray_table = (uint8)CAMERA_RAY_TABLE_HAS_DATA;
    params->has_rect_map = (uint8)CAMERA_UNDISTORT_RECT_MAP_HAS_DATA;
}

uint8 CameraCalibration_IsPixelInValidRegion(float img_x, float img_y)
{
    float dx = img_x - CAMERA_CALIBRATION_CX;
    float dy = img_y - CAMERA_CALIBRATION_CY;
    float radius = sqrtf((dx * dx) + (dy * dy));

    if(img_x < 0.0f || img_x > ((float)MT9V03X_W - 1.0f) ||
       img_y < 0.0f || img_y > ((float)MT9V03X_H - 1.0f))
    {
        return 0U;
    }

    if(CAMERA_CALIBRATION_VALID_RADIUS_PX > 0.0f &&
       radius > CAMERA_CALIBRATION_VALID_RADIUS_PX)
    {
        return 0U;
    }

    return 1U;
}

uint8 CameraCalibration_PixelToBodyRay(float img_x, float img_y, float ray_body[3])
{
#if CAMERA_RAY_TABLE_HAS_DATA
    return CameraCalibration_TablePixelToRay(img_x, img_y, ray_body);
#else
#if CAMERA_CALIBRATION_HAS_GENERATED_PARAMS
    return CameraCalibration_ParamsPixelToRay(img_x, img_y, ray_body);
#else
    return CameraCalibration_FallbackPixelToRay(img_x, img_y, ray_body);
#endif
#endif
}

uint8 CameraCalibration_BodyRayToPixel(const float ray_body[3], float *img_x, float *img_y)
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float lateral = 0.0f;
    float theta = 0.0f;
    float theta2 = 0.0f;
    float theta4 = 0.0f;
    float theta6 = 0.0f;
    float theta8 = 0.0f;
    float theta_d = 0.0f;
    float scale = 0.0f;
    float camera_ray[3];

    if(ray_body == NULL || img_x == NULL || img_y == NULL || ray_body[2] <= 0.0f)
    {
        return 0U;
    }

    camera_ray[0] = ray_body[0];
    camera_ray[1] = ray_body[1];
    camera_ray[2] = ray_body[2];
    CameraCalibration_RemoveMountOffset(camera_ray);

    x = camera_ray[1];
    y = camera_ray[0];
    z = camera_ray[2];
    lateral = sqrtf((x * x) + (y * y));

    if(lateral < 1e-6f)
    {
        *img_x = CAMERA_CALIBRATION_CX;
        *img_y = CAMERA_CALIBRATION_CY;
        return CameraCalibration_IsPixelInValidRegion(*img_x, *img_y);
    }

    theta = atan2f(lateral, z);

#if CAMERA_CALIBRATION_HAS_GENERATED_PARAMS
    theta2 = theta * theta;
    theta4 = theta2 * theta2;
    theta6 = theta4 * theta2;
    theta8 = theta4 * theta4;
    theta_d = theta * (1.0f +
                       CAMERA_CALIBRATION_K1 * theta2 +
                       CAMERA_CALIBRATION_K2 * theta4 +
                       CAMERA_CALIBRATION_K3 * theta6 +
                       CAMERA_CALIBRATION_K4 * theta8);
    scale = theta_d / lateral;
    *img_x = (CAMERA_CALIBRATION_FX * x * scale) + CAMERA_CALIBRATION_CX;
    *img_y = (CAMERA_CALIBRATION_FY * y * scale) + CAMERA_CALIBRATION_CY;
#else
    scale = (theta / (0.5f * CAMERA_CALIBRATION_FALLBACK_DIAGONAL_FOV_DEG * CAMERA_CALIBRATION_DEG2RAD)) *
            CAMERA_CALIBRATION_IMG_HALF_DIAG / lateral;
    *img_x = (x * scale) + CAMERA_CALIBRATION_IMG_CENTER_X;
    *img_y = (y * scale) + CAMERA_CALIBRATION_IMG_CENTER_Y;
#endif

    return CameraCalibration_IsPixelInValidRegion(*img_x, *img_y);
}

uint8 CameraCalibration_UndistortPoint(float distorted_x,
                                       float distorted_y,
                                       float *undistorted_x,
                                       float *undistorted_y)
{
    float ray[3];

    if(undistorted_x == NULL || undistorted_y == NULL)
    {
        return 0U;
    }

    if(!CameraCalibration_PixelToBodyRay(distorted_x, distorted_y, ray))
    {
        return 0U;
    }

    if(fabsf(ray[2]) < 1e-6f)
    {
        return 0U;
    }

    *undistorted_x = CAMERA_CALIBRATION_CX + CAMERA_CALIBRATION_FX * (ray[1] / ray[2]);
    *undistorted_y = CAMERA_CALIBRATION_CY + CAMERA_CALIBRATION_FY * (ray[0] / ray[2]);
    return 1U;
}

uint8 CameraCalibration_RectifiedModelPointToSource(float rectified_x,
                                                    float rectified_y,
                                                    float *source_x,
                                                    float *source_y)
{
    float ray_body[3];
    float length;

    if(source_x == NULL || source_y == NULL)
    {
        return 0U;
    }

    ray_body[0] = (rectified_y - CAMERA_CALIBRATION_CY) / CAMERA_CALIBRATION_FY;
    ray_body[1] = (rectified_x - CAMERA_CALIBRATION_CX) / CAMERA_CALIBRATION_FX;
    ray_body[2] = 1.0f;
    length = sqrtf((ray_body[0] * ray_body[0]) +
                   (ray_body[1] * ray_body[1]) +
                   (ray_body[2] * ray_body[2]));
    if(length < 1e-6f)
    {
        return 0U;
    }
    ray_body[0] /= length;
    ray_body[1] /= length;
    ray_body[2] /= length;

    return CameraCalibration_BodyRayToPixel(ray_body, source_x, source_y);
}

uint8 CameraCalibration_RectifiedPointToSource(float rectified_x,
                                               float rectified_y,
                                               float *source_x,
                                               float *source_y)
{
#if CAMERA_UNDISTORT_RECT_MAP_HAS_DATA
    int x0;
    int y0;
    int x1;
    int y1;
    float tx;
    float ty;

    if(source_x == NULL || source_y == NULL ||
       rectified_x < 0.0f || rectified_x > ((float)MT9V03X_W - 1.0f) ||
       rectified_y < 0.0f || rectified_y > ((float)MT9V03X_H - 1.0f))
    {
        return 0U;
    }

    x0 = (int)floorf(rectified_x);
    y0 = (int)floorf(rectified_y);
    x1 = (x0 + 1 < MT9V03X_W) ? (x0 + 1) : x0;
    y1 = (y0 + 1 < MT9V03X_H) ? (y0 + 1) : y0;
    tx = rectified_x - (float)x0;
    ty = rectified_y - (float)y0;
    if(camera_undistort_rect_src_x_q8[y0][x0] == CAMERA_UNDISTORT_RECT_MAP_INVALID ||
       camera_undistort_rect_src_x_q8[y0][x1] == CAMERA_UNDISTORT_RECT_MAP_INVALID ||
       camera_undistort_rect_src_x_q8[y1][x0] == CAMERA_UNDISTORT_RECT_MAP_INVALID ||
       camera_undistort_rect_src_x_q8[y1][x1] == CAMERA_UNDISTORT_RECT_MAP_INVALID ||
       camera_undistort_rect_src_y_q8[y0][x0] == CAMERA_UNDISTORT_RECT_MAP_INVALID ||
       camera_undistort_rect_src_y_q8[y0][x1] == CAMERA_UNDISTORT_RECT_MAP_INVALID ||
       camera_undistort_rect_src_y_q8[y1][x0] == CAMERA_UNDISTORT_RECT_MAP_INVALID ||
       camera_undistort_rect_src_y_q8[y1][x1] == CAMERA_UNDISTORT_RECT_MAP_INVALID)
    {
        return 0U;
    }

    *source_x = (
        ((1.0f - tx) * (1.0f - ty) * (float)camera_undistort_rect_src_x_q8[y0][x0]) +
        (tx * (1.0f - ty) * (float)camera_undistort_rect_src_x_q8[y0][x1]) +
        ((1.0f - tx) * ty * (float)camera_undistort_rect_src_x_q8[y1][x0]) +
        (tx * ty * (float)camera_undistort_rect_src_x_q8[y1][x1])) /
        (float)(1U << CAMERA_UNDISTORT_RECT_MAP_Q);
    *source_y = (
        ((1.0f - tx) * (1.0f - ty) * (float)camera_undistort_rect_src_y_q8[y0][x0]) +
        (tx * (1.0f - ty) * (float)camera_undistort_rect_src_y_q8[y0][x1]) +
        ((1.0f - tx) * ty * (float)camera_undistort_rect_src_y_q8[y1][x0]) +
        (tx * ty * (float)camera_undistort_rect_src_y_q8[y1][x1])) /
        (float)(1U << CAMERA_UNDISTORT_RECT_MAP_Q);
    return 1U;
#else
    float model_x = CAMERA_CALIBRATION_CX +
        (rectified_x - 0.5f * ((float)MT9V03X_W - 1.0f)) *
        CAMERA_UNDISTORT_RECT_SCALE_X;
    float model_y = CAMERA_CALIBRATION_CY +
        (rectified_y - 0.5f * ((float)MT9V03X_H - 1.0f)) *
        CAMERA_UNDISTORT_RECT_SCALE_Y;
    return CameraCalibration_RectifiedModelPointToSource(model_x, model_y,
                                                         source_x, source_y);
#endif
}

uint8 CameraCalibration_SourcePointToRectified(float source_x,
                                               float source_y,
                                               float *rectified_x,
                                               float *rectified_y)
{
    float model_x;
    float model_y;

    if(rectified_x == NULL || rectified_y == NULL ||
       !CameraCalibration_UndistortPoint(source_x, source_y, &model_x, &model_y))
    {
        return 0U;
    }

    *rectified_x = ((model_x - CAMERA_CALIBRATION_CX) /
                    CAMERA_UNDISTORT_RECT_SCALE_X) +
                   0.5f * ((float)MT9V03X_W - 1.0f);
    *rectified_y = ((model_y - CAMERA_CALIBRATION_CY) /
                    CAMERA_UNDISTORT_RECT_SCALE_Y) +
                   0.5f * ((float)MT9V03X_H - 1.0f);
    return (uint8)(*rectified_x >= 0.0f &&
                   *rectified_x <= ((float)MT9V03X_W - 1.0f) &&
                   *rectified_y >= 0.0f &&
                   *rectified_y <= ((float)MT9V03X_H - 1.0f));
}

uint8 CameraCalibration_RectifiedPixelToSource(int dst_x,
                                               int dst_y,
                                               int *src_x,
                                               int *src_y)
{
    if(src_x == NULL || src_y == NULL ||
       dst_x < 0 || dst_x >= MT9V03X_W ||
       dst_y < 0 || dst_y >= MT9V03X_H)
    {
        return 0U;
    }

#if CAMERA_UNDISTORT_RECT_MAP_HAS_DATA
    uint16 source_x_q8;
    uint16 source_y_q8;

    if(!CameraCalibration_RectifiedPixelToSourceQ8(dst_x, dst_y,
                                                   &source_x_q8, &source_y_q8))
    {
        return 0U;
    }
    *src_x = (int)((source_x_q8 + 128U) >> CAMERA_UNDISTORT_RECT_MAP_Q);
    *src_y = (int)((source_y_q8 + 128U) >> CAMERA_UNDISTORT_RECT_MAP_Q);
    return 1U;
#else
    *src_x = dst_x;
    *src_y = dst_y;
    return 1U;
#endif
}

uint8 CameraCalibration_RectifiedPixelToSourceQ8(int dst_x,
                                                 int dst_y,
                                                 uint16 *src_x_q8,
                                                 uint16 *src_y_q8)
{
    if(src_x_q8 == NULL || src_y_q8 == NULL ||
       dst_x < 0 || dst_x >= MT9V03X_W ||
       dst_y < 0 || dst_y >= MT9V03X_H)
    {
        return 0U;
    }

#if CAMERA_UNDISTORT_RECT_MAP_HAS_DATA
    *src_x_q8 = camera_undistort_rect_src_x_q8[dst_y][dst_x];
    *src_y_q8 = camera_undistort_rect_src_y_q8[dst_y][dst_x];
    return (uint8)(*src_x_q8 != CAMERA_UNDISTORT_RECT_MAP_INVALID &&
                   *src_y_q8 != CAMERA_UNDISTORT_RECT_MAP_INVALID);
#else
    *src_x_q8 = (uint16)((uint16)dst_x << 8);
    *src_y_q8 = (uint16)((uint16)dst_y << 8);
    return 1U;
#endif
}

uint8 CameraCalibration_FullRectifiedPixelToSourceQ8(int full_x,
                                                      int full_y,
                                                      uint16 *src_x_q8,
                                                      uint16 *src_y_q8)
{
    if(src_x_q8 == NULL || src_y_q8 == NULL ||
       full_x < 0 || full_x >= CAMERA_CALIBRATION_UNDISTORT_WIDTH ||
       full_y < 0 || full_y >= CAMERA_CALIBRATION_UNDISTORT_HEIGHT)
    {
        return 0U;
    }

#if CAMERA_UNDISTORT_RECT_MAP_HAS_DATA
    *src_x_q8 = camera_undistort_full_src_x_q8[full_y][full_x];
    *src_y_q8 = camera_undistort_full_src_y_q8[full_y][full_x];
    return (uint8)(*src_x_q8 != CAMERA_UNDISTORT_RECT_MAP_INVALID &&
                   *src_y_q8 != CAMERA_UNDISTORT_RECT_MAP_INVALID);
#else
    *src_x_q8 = CAMERA_UNDISTORT_RECT_MAP_INVALID;
    *src_y_q8 = CAMERA_UNDISTORT_RECT_MAP_INVALID;
    return 0U;
#endif
}
