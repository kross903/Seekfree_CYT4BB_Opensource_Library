/**
 * @file ipm_interface.c
 * @brief ????????? - ?? TC387 ??????
 * 
 * 整合IMU姿态数据和摄像头图像，提供简化的逆透视变换调用
 */

#include "ipm_interface.h"
#include "common/project_headfile.h"
#include "vision/camera_calibration.h"
#include <float.h>
#include <math.h>

/*
 * 模块定位:
 * - 上接按需轮廓识别与坐标输出
 * - 下接底层数学实现(drone_ipm_embedded)
 * - ????? IMU / ?? / ????????????????
 */

/* ???????? */
static uint8 g_birdview_buf[IPM_BIRDVIEW_W * IPM_BIRDVIEW_H];

/* ====================== ??? ====================== */

/**
 * @brief 初始化逆透视模块
 */
void IPM_Interface_Init(void)
{
    /* 初始化嵌入式IPM模块 */
    DroneIPM_Embedded_Init();
    
    /* 清空俯视图缓冲区 */
    memset(g_birdview_buf, 0, sizeof(g_birdview_buf));
}

/* ====================== 更新函数 ====================== */

/**
 * @brief 使用指定姿态更新逆透视变换
 *
 * ???????????????????????????
 */
void IPM_Interface_Update(float roll_deg, float pitch_deg, float height_cm)
{
    DroneIPM_Embedded_Update(roll_deg, pitch_deg, height_cm);
}

/**
 * @brief ???? IMU ??????
 * 
 * ? ICM.h ????????????
 * - Roll_a: ??????
 * - Pitch_a: ??????
 */
void IPM_Interface_UpdateFromIMU(float height_cm)
{
    /* 
     * ??????? IMU ???????????????
     * ?????
     * - Roll_a > 0 表示向右倾斜
     * - Pitch_a > 0 表示抬头
     * 
     * 如果实际情况相反，需要取负号
     */
#if CY_CORE_CM7_0
    float roll = Roll_a;   /* ? ICM.h ????? */
    float pitch = Pitch_a; /* ? ICM.h ????? */
#else
    float roll = 0.0f;
    float pitch = 0.0f;
#endif
    
    DroneIPM_Embedded_Update(roll, pitch, height_cm);
}

/* ====================== 坐标变换 ====================== */

/**
 * @brief ?????????????
 *
 * 这是上层视觉模块最常依赖的基础能力，保持包装层尽量薄，
 * ????????????? `DroneIPM_Embedded_Image2World()`?
 */
void IPM_Image2World(int img_x, int img_y, float *world_x, float *world_y)
{
    float result[2];
    if(!DroneIPM_Embedded_Image2WorldF((float)img_x, (float)img_y, result))
    {
        result[0] = 9999.0f;
        result[1] = 9999.0f;
    }
    if(world_x != NULL)
    {
        *world_x = result[0];
    }
    if(world_y != NULL)
    {
        *world_y = result[1];
    }
}

uint8 IPM_Image2WorldF(float img_x, float img_y, float *world_x, float *world_y)
{
    float result[2];

    if(world_x == NULL || world_y == NULL)
    {
        return 0U;
    }

    if(!DroneIPM_Embedded_Image2WorldF(img_x, img_y, result))
    {
        return 0U;
    }

    *world_x = result[0];
    *world_y = result[1];
    return 1U;
}

uint8 IPM_RectifiedImage2WorldF(float img_x, float img_y,
                               float *world_x, float *world_y)
{
    float source_x;
    float source_y;

    if(!CameraCalibration_RectifiedPointToSource(img_x, img_y,
                                                 &source_x, &source_y))
    {
        return 0U;
    }
    return IPM_Image2WorldF(source_x, source_y, world_x, world_y);
}

uint8 IPM_FullRectifiedImage2WorldF(float img_x, float img_y,
                                    float *world_x, float *world_y)
{
    int pixel_x = (int)floorf(img_x + 0.5f);
    int pixel_y = (int)floorf(img_y + 0.5f);
    uint16 source_x_q8;
    uint16 source_y_q8;

    if(fabsf(img_x - (float)pixel_x) < 1e-4f &&
       fabsf(img_y - (float)pixel_y) < 1e-4f &&
       CameraCalibration_FullRectifiedPixelToSourceQ8(
           pixel_x, pixel_y, &source_x_q8, &source_y_q8))
    {
        return IPM_Image2WorldF((float)source_x_q8 / 256.0f,
                                (float)source_y_q8 / 256.0f,
                                world_x, world_y);
    }
    float rectified_x = (img_x - 0.5f) * 0.5f;
    float rectified_y = (img_y - 0.5f) * 0.5f;
    return IPM_RectifiedImage2WorldF(rectified_x, rectified_y,
                                     world_x, world_y);
}

static uint8 IPM_GetHorizontalPlaneScale(float plane_height_cm,
                                         float *scale,
                                         float *camera_world_x,
                                         float *camera_world_y)
{
    float camera_to_ground_cm =
        -(g_ipm.Z0 + CAMERA_OFFSET_Z_CM);

    if(scale == NULL || camera_world_x == NULL || camera_world_y == NULL ||
       plane_height_cm < 0.0f ||
       camera_to_ground_cm <= plane_height_cm + 1e-3f)
    {
        return 0U;
    }

    *scale = (camera_to_ground_cm - plane_height_cm) /
             camera_to_ground_cm;
    *camera_world_x = -(g_ipm.X0 + CAMERA_OFFSET_X_CM);
    *camera_world_y = -(g_ipm.Y0 + CAMERA_OFFSET_Y_CM);
    return 1U;
}

uint8 IPM_FullRectifiedImage2PlaneF(float img_x, float img_y,
                                    float plane_height_cm,
                                    float *world_x, float *world_y)
{
    float ground_x = 0.0f;
    float ground_y = 0.0f;
    float camera_world_x = 0.0f;
    float camera_world_y = 0.0f;
    float scale = 0.0f;

    if(world_x == NULL || world_y == NULL ||
       !IPM_GetHorizontalPlaneScale(plane_height_cm, &scale,
                                    &camera_world_x, &camera_world_y) ||
       !IPM_FullRectifiedImage2WorldF(img_x, img_y,
                                     &ground_x, &ground_y))
    {
        return 0U;
    }

    *world_x = camera_world_x + scale * (ground_x - camera_world_x);
    *world_y = camera_world_y + scale * (ground_y - camera_world_y);
    return 1U;
}

/**
 * @brief ?????????????
 */
bool IPM_World2Image(float world_x, float world_y, int *img_x, int *img_y)
{
    int result[2];
    bool valid = DroneIPM_Embedded_World2Image(world_x, world_y, result);
    if (valid) {
        *img_x = result[0];
        *img_y = result[1];
    }
    return valid;
}

bool IPM_World2RectifiedImage(float world_x, float world_y,
                              int *img_x, int *img_y)
{
    float source[2];
    float rectified_x;
    float rectified_y;

    if(img_x == NULL || img_y == NULL ||
       !DroneIPM_Embedded_World2ImageF(world_x, world_y, source) ||
       !CameraCalibration_SourcePointToRectified(source[0], source[1],
                                                 &rectified_x, &rectified_y))
    {
        return false;
    }

    *img_x = (int)floorf(rectified_x + 0.5f);
    *img_y = (int)floorf(rectified_y + 0.5f);
    return *img_x >= 0 && *img_x < MT9V03X_W &&
           *img_y >= 0 && *img_y < MT9V03X_H;
}

bool IPM_World2FullRectifiedImage(float world_x, float world_y,
                                  int *img_x, int *img_y)
{
    float source[2];
    float rectified_x;
    float rectified_y;

    if(img_x == NULL || img_y == NULL ||
       !DroneIPM_Embedded_World2ImageF(world_x, world_y, source) ||
       !CameraCalibration_SourcePointToRectified(source[0], source[1],
                                                 &rectified_x, &rectified_y))
    {
        return false;
    }

    *img_x = (int)floorf((rectified_x * 2.0f) + 0.5f);
    *img_y = (int)floorf((rectified_y * 2.0f) + 0.5f);
    return *img_x >= 0 && *img_x < (MT9V03X_W * 2) &&
           *img_y >= 0 && *img_y < (MT9V03X_H * 2);
}

bool IPM_Plane2FullRectifiedImage(float world_x, float world_y,
                                  float plane_height_cm,
                                  int *img_x, int *img_y)
{
    float camera_world_x = 0.0f;
    float camera_world_y = 0.0f;
    float ground_x = 0.0f;
    float ground_y = 0.0f;
    float scale = 0.0f;

    if(!IPM_GetHorizontalPlaneScale(plane_height_cm, &scale,
                                    &camera_world_x, &camera_world_y))
    {
        return false;
    }

    ground_x = camera_world_x + (world_x - camera_world_x) / scale;
    ground_y = camera_world_y + (world_y - camera_world_y) / scale;
    return IPM_World2FullRectifiedImage(ground_x, ground_y, img_x, img_y);
}

/**
 * @brief ?????????????
 */
bool IPM_GetNadirPoint(int *img_x, int *img_y)
{
    int result[2];
    bool valid = DroneIPM_Embedded_GetNadir(result);
    if (valid) {
        *img_x = result[0];
        *img_y = result[1];
    }
    return valid;
}

/* ====================== ????? ====================== */

/**
 * @brief ?????
 *
 * ???????????????????? `drone_ipm_embedded.c`?
 */
static uint8 IPM_SampleGrayBilinear(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                    float x, float y, uint8 *value)
{
    int x0;
    int y0;
    int x1;
    int y1;
    float tx;
    float ty;
    float top;
    float bottom;
    float sample;

    if(src_img == NULL || value == NULL ||
       x < 0.0f || x > ((float)MT9V03X_W - 1.0f) ||
       y < 0.0f || y > ((float)MT9V03X_H - 1.0f))
    {
        return 0U;
    }

    x0 = (int)floorf(x);
    y0 = (int)floorf(y);
    x1 = (x0 + 1 < MT9V03X_W) ? (x0 + 1) : x0;
    y1 = (y0 + 1 < MT9V03X_H) ? (y0 + 1) : y0;
    tx = x - (float)x0;
    ty = y - (float)y0;
    top = ((1.0f - tx) * (float)src_img[y0][x0]) +
          (tx * (float)src_img[y0][x1]);
    bottom = ((1.0f - tx) * (float)src_img[y1][x0]) +
             (tx * (float)src_img[y1][x1]);
    sample = ((1.0f - ty) * top) + (ty * bottom);
    if(sample < 0.0f) sample = 0.0f;
    if(sample > 255.0f) sample = 255.0f;
    *value = (uint8)(sample + 0.5f);
    return 1U;
}

static uint8 IPM_SampleGrayNearest(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                   float x, float y, uint8 *value)
{
    int ix;
    int iy;

    if(src_img == NULL || value == NULL ||
       x < 0.0f || x > ((float)MT9V03X_W - 1.0f) ||
       y < 0.0f || y > ((float)MT9V03X_H - 1.0f))
    {
        return 0U;
    }

    ix = (int)floorf(x + 0.5f);
    iy = (int)floorf(y + 0.5f);
    if(ix < 0) ix = 0;
    if(ix >= MT9V03X_W) ix = MT9V03X_W - 1;
    if(iy < 0) iy = 0;
    if(iy >= MT9V03X_H) iy = MT9V03X_H - 1;
    *value = src_img[iy][ix];
    return 1U;
}

static uint8 IPM_SampleGrayQ8(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                              uint16 x_q8, uint16 y_q8, uint8 *value)
{
    uint32 x0;
    uint32 y0;
    uint32 fx;
    uint32 fy;
    uint32 inv_fx;
    uint32 inv_fy;
    uint32 sum;

    if(src_img == NULL || value == NULL)
    {
        return 0U;
    }

    x0 = (uint32)x_q8 >> 8;
    y0 = (uint32)y_q8 >> 8;
    if(x0 + 1U >= (uint32)MT9V03X_W || y0 + 1U >= (uint32)MT9V03X_H)
    {
        return 0U;
    }

    fx = (uint32)x_q8 & 0xFFU;
    fy = (uint32)y_q8 & 0xFFU;
    inv_fx = 256U - fx;
    inv_fy = 256U - fy;
    sum = ((uint32)src_img[y0][x0] * inv_fx * inv_fy) +
          ((uint32)src_img[y0][x0 + 1U] * fx * inv_fy) +
          ((uint32)src_img[y0 + 1U][x0] * inv_fx * fy) +
          ((uint32)src_img[y0 + 1U][x0 + 1U] * fx * fy);
    *value = (uint8)((sum + 32768U) >> 16);
    return 1U;
}

uint8 IPM_SampleFullUndistortedGrayQ8(
    const uint8 src_img[MT9V03X_H][MT9V03X_W],
    int full_x, int full_y,
    uint16 *source_x_q8, uint16 *source_y_q8,
    uint8 *value)
{
    if(source_x_q8 == NULL || source_y_q8 == NULL || value == NULL ||
       !CameraCalibration_FullRectifiedPixelToSourceQ8(
           full_x, full_y, source_x_q8, source_y_q8))
    {
        return 0U;
    }
    return IPM_SampleGrayQ8(src_img, *source_x_q8, *source_y_q8, value);
}

static uint8 IPM_SampleDynamicNearest(const uint8 *src_img,
                                      int width, int height,
                                      float x, float y,
                                      uint8 *value)
{
    int ix;
    int iy;

    if(src_img == NULL || value == NULL || width <= 0 || height <= 0 ||
       x < 0.0f || x > ((float)width - 1.0f) ||
       y < 0.0f || y > ((float)height - 1.0f))
    {
        return 0U;
    }

    ix = (int)floorf(x + 0.5f);
    iy = (int)floorf(y + 0.5f);
    if(ix < 0) ix = 0;
    if(ix >= width) ix = width - 1;
    if(iy < 0) iy = 0;
    if(iy >= height) iy = height - 1;
    *value = src_img[(size_t)iy * (size_t)width + (size_t)ix];
    return 1U;
}

void IPM_GenerateBirdView(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                          uint8 *dst_img, int dst_w, int dst_h, float scale)
{
    int dst_y;
    int dst_x;
    float center_px;
    float center_py;

    if(src_img == NULL || dst_img == NULL || dst_w <= 0 || dst_h <= 0 || scale <= 0.0f)
    {
        return;
    }

    center_px = (float)dst_w / 2.0f;
    center_py = (float)dst_h / 2.0f;
    for(dst_y = 0; dst_y < dst_h; dst_y++)
    {
        for(dst_x = 0; dst_x < dst_w; dst_x++)
        {
            float world_x = ((float)dst_y - center_py) * scale;
            float world_y = (center_px - (float)dst_x) * scale;
            float source[2];
            uint8 sample = 0U;

            if(DroneIPM_Embedded_World2ImageF(world_x, world_y, source) &&
               IPM_SampleGrayBilinear(src_img, source[0], source[1], &sample))
            {
                dst_img[dst_y * dst_w + dst_x] = sample;
            }
            else
            {
                dst_img[dst_y * dst_w + dst_x] = 0U;
            }
        }
    }
}

uint8 IPM_CalculateBirdViewBounds(float scale, IPM_BirdViewBounds_t *bounds)
{
    float min_x = FLT_MAX;
    float max_x = -FLT_MAX;
    float min_y = FLT_MAX;
    float max_y = -FLT_MAX;
    int valid_count = 0;
    int y;
    int x;

    if(bounds == NULL || scale <= 0.0f)
    {
        return 0U;
    }

    for(y = 0; y < MT9V03X_H; y++)
    {
        for(x = 0; x < MT9V03X_W; x++)
        {
            float world[2];
            if(DroneIPM_Embedded_Image2WorldF((float)x, (float)y, world))
            {
                if(world[0] < min_x) min_x = world[0];
                if(world[0] > max_x) max_x = world[0];
                if(world[1] < min_y) min_y = world[1];
                if(world[1] > max_y) max_y = world[1];
                valid_count++;
            }
        }
    }

    if(valid_count == 0)
    {
        return 0U;
    }

    bounds->min_world_x = floorf(min_x / scale) * scale;
    bounds->max_world_x = ceilf(max_x / scale) * scale;
    bounds->min_world_y = floorf(min_y / scale) * scale;
    bounds->max_world_y = ceilf(max_y / scale) * scale;
    bounds->scale = scale;
    bounds->width = (int)ceilf((bounds->max_world_y - bounds->min_world_y) / scale);
    bounds->height = (int)ceilf((bounds->max_world_x - bounds->min_world_x) / scale);
    if(bounds->width < 1) bounds->width = 1;
    if(bounds->height < 1) bounds->height = 1;
    return 1U;
}

uint8 IPM_ClampBirdViewBounds(IPM_BirdViewBounds_t *bounds,
                              float half_range_x,
                              float half_range_y)
{
    if(bounds == NULL || bounds->scale <= 0.0f || half_range_x <= 0.0f || half_range_y <= 0.0f)
    {
        return 0U;
    }

    if(bounds->min_world_x < -half_range_x) bounds->min_world_x = -half_range_x;
    if(bounds->max_world_x > half_range_x) bounds->max_world_x = half_range_x;
    if(bounds->min_world_y < -half_range_y) bounds->min_world_y = -half_range_y;
    if(bounds->max_world_y > half_range_y) bounds->max_world_y = half_range_y;
    if(bounds->min_world_x >= bounds->max_world_x || bounds->min_world_y >= bounds->max_world_y)
    {
        return 0U;
    }

    bounds->width = (int)ceilf((bounds->max_world_y - bounds->min_world_y) / bounds->scale);
    bounds->height = (int)ceilf((bounds->max_world_x - bounds->min_world_x) / bounds->scale);
    if(bounds->width < 1) bounds->width = 1;
    if(bounds->height < 1) bounds->height = 1;
    return 1U;
}

uint8 IPM_SetFixedBirdViewBounds(float scale,
                                 float half_range_x,
                                 float half_range_y,
                                 IPM_BirdViewBounds_t *bounds)
{
    if(bounds == NULL || scale <= 0.0f || half_range_x <= 0.0f || half_range_y <= 0.0f)
    {
        return 0U;
    }

    bounds->min_world_x = -half_range_x;
    bounds->max_world_x = half_range_x;
    bounds->min_world_y = -half_range_y;
    bounds->max_world_y = half_range_y;
    bounds->scale = scale;
    bounds->width = (int)ceilf((2.0f * half_range_y) / scale);
    bounds->height = (int)ceilf((2.0f * half_range_x) / scale);
    return (bounds->width > 0 && bounds->height > 0) ? 1U : 0U;
}

static uint8 IPM_GenerateBirdViewDynamicInternal(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                                 uint8 *dst_img,
                                                 uint8 *valid_mask,
                                                 const IPM_BirdViewBounds_t *bounds,
                                                 uint8 nearest)
{
    int dst_y;
    int dst_x;

    if(src_img == NULL || dst_img == NULL || valid_mask == NULL || bounds == NULL ||
       bounds->width <= 0 || bounds->height <= 0 || bounds->scale <= 0.0f)
    {
        return 0U;
    }

    for(dst_y = 0; dst_y < bounds->height; dst_y++)
    {
        for(dst_x = 0; dst_x < bounds->width; dst_x++)
        {
            float world_x = bounds->max_world_x - ((float)dst_y + 0.5f) * bounds->scale;
            float world_y = bounds->max_world_y - ((float)dst_x + 0.5f) * bounds->scale;
            float source[2];
            uint8 sample = 0U;
            size_t index = (size_t)dst_y * (size_t)bounds->width + (size_t)dst_x;

            if(DroneIPM_Embedded_World2ImageF(world_x, world_y, source) &&
               (nearest ? IPM_SampleGrayNearest(src_img, source[0], source[1], &sample) :
                          IPM_SampleGrayBilinear(src_img, source[0], source[1], &sample)))
            {
                dst_img[index] = sample;
                valid_mask[index] = 255U;
            }
            else
            {
                dst_img[index] = 0U;
                valid_mask[index] = 0U;
            }
        }
    }
    return 1U;
}

uint8 IPM_GenerateBirdViewDynamic(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                  uint8 *dst_img,
                                  uint8 *valid_mask,
                                  const IPM_BirdViewBounds_t *bounds)
{
    return IPM_GenerateBirdViewDynamicInternal(src_img, dst_img, valid_mask, bounds, 0U);
}

uint8 IPM_GenerateBirdViewDynamicNearest(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                         uint8 *dst_img,
                                         uint8 *valid_mask,
                                         const IPM_BirdViewBounds_t *bounds)
{
    return IPM_GenerateBirdViewDynamicInternal(src_img, dst_img, valid_mask, bounds, 1U);
}

uint8 IPM_GenerateBirdViewFromUndistortedBinary(const uint8 *src_img,
                                                const IPM_UndistortCanvas_t *src_canvas,
                                                uint8 *dst_img,
                                                uint8 *valid_mask,
                                                const IPM_BirdViewBounds_t *bounds)
{
    int dst_y;
    int dst_x;

    if(src_img == NULL || src_canvas == NULL || dst_img == NULL || valid_mask == NULL ||
       bounds == NULL || src_canvas->width <= 0 || src_canvas->height <= 0 ||
       bounds->width <= 0 || bounds->height <= 0 || bounds->scale <= 0.0f)
    {
        return 0U;
    }

    for(dst_y = 0; dst_y < bounds->height; dst_y++)
    {
        for(dst_x = 0; dst_x < bounds->width; dst_x++)
        {
            float world_x = bounds->max_world_x - ((float)dst_y + 0.5f) * bounds->scale;
            float world_y = bounds->max_world_y - ((float)dst_x + 0.5f) * bounds->scale;
            float distorted[2];
            float rectified_x = 0.0f;
            float rectified_y = 0.0f;
            uint8 sample = 0U;
            size_t index = (size_t)dst_y * (size_t)bounds->width + (size_t)dst_x;

            if(DroneIPM_Embedded_World2ImageF(world_x, world_y, distorted) &&
               CameraCalibration_UndistortPoint(distorted[0], distorted[1],
                                                &rectified_x, &rectified_y) &&
               IPM_SampleDynamicNearest(src_img,
                                        src_canvas->width, src_canvas->height,
                                        rectified_x - src_canvas->min_rectified_x,
                                        rectified_y - src_canvas->min_rectified_y,
                                        &sample))
            {
                dst_img[index] = sample;
                valid_mask[index] = 255U;
            }
            else
            {
                dst_img[index] = 0U;
                valid_mask[index] = 0U;
            }
        }
    }
    return 1U;
}

uint8 IPM_GenerateBirdViewFromRuntimeBinary(const uint8 *src_img,
                                            uint8 *dst_img,
                                            uint8 *valid_mask,
                                            const IPM_BirdViewBounds_t *bounds)
{
    int dst_y;
    int dst_x;

    if(src_img == NULL || dst_img == NULL || valid_mask == NULL || bounds == NULL ||
       bounds->width <= 0 || bounds->height <= 0 || bounds->scale <= 0.0f)
    {
        return 0U;
    }

    for(dst_y = 0; dst_y < bounds->height; dst_y++)
    {
        for(dst_x = 0; dst_x < bounds->width; dst_x++)
        {
            float world_x = bounds->max_world_x -
                ((float)dst_y + 0.5f) * bounds->scale;
            float world_y = bounds->max_world_y -
                ((float)dst_x + 0.5f) * bounds->scale;
            int source_x;
            int source_y;
            size_t index = (size_t)dst_y * (size_t)bounds->width +
                           (size_t)dst_x;

            if(IPM_World2FullRectifiedImage(world_x, world_y,
                                            &source_x, &source_y))
            {
                const int stride = ((MT9V03X_W * 2) + 7) / 8;
                uint8 mask = (uint8)(0x80U >> (source_x & 7));
                dst_img[index] =
                    (src_img[source_y * stride + (source_x >> 3)] & mask) ?
                    255U : 0U;
                valid_mask[index] = 255U;
            }
            else
            {
                dst_img[index] = 0U;
                valid_mask[index] = 0U;
            }
        }
    }
    return 1U;
}

/**
 * @brief 在IPS200屏幕上显示俯视图
 * 
 * ????????????????
 * ????????/????????????????????
 */
void IPM_DisplayBirdView(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                         int x, int y, float scale)
{
#if CY_CORE_CM7_0
    /* ????????? */
    IPM_GenerateBirdView(src_img, g_birdview_buf, 
                         IPM_BIRDVIEW_W, IPM_BIRDVIEW_H, scale);
    
    /* 绘制中心十字标记 */
    IPM_DrawCenterMark(g_birdview_buf, IPM_BIRDVIEW_W, IPM_BIRDVIEW_H);
    
    /* ????? */
    /* 注意：需要将灰度数据转换为IPS200可显示的格式 */
    for (int dy = 0; dy < IPM_BIRDVIEW_H; dy++) {
        for (int dx = 0; dx < IPM_BIRDVIEW_W; dx++) {
            uint8 gray = g_birdview_buf[dy * IPM_BIRDVIEW_W + dx];
            /* 简单显示：将灰度转为RGB565 */
            uint16 color = ((gray >> 3) << 11) | ((gray >> 2) << 5) | (gray >> 3);
            ips200_draw_point(x + dx, y + dy, color);
        }
    }
#else
    (void)src_img;
    (void)x;
    (void)y;
    (void)scale;
#endif
}

/**
 * @brief ?????????????
 */
void IPM_DrawCenterMark(uint8 *img, int w, int h)
{
    int cx = w / 2;
    int cy = h / 2;
    int len = 10;  /* ????? */
    
    /* ??? */
    for (int i = -len; i <= len; i++) {
        if (cx + i >= 0 && cx + i < w) {
            img[cy * w + cx + i] = 255;  /* 白色 */
        }
    }
    
    /* ??? */
    for (int i = -len; i <= len; i++) {
        if (cy + i >= 0 && cy + i < h) {
            img[(cy + i) * w + cx] = 255;  /* 白色 */
        }
    }
}

static void IPM_GenerateUndistortedDebugImageInternal(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                                      uint8 dst_img[MT9V03X_H][MT9V03X_W],
                                                      uint8 nearest)
{
    int y = 0;
    int x = 0;

    if(src_img == NULL || dst_img == NULL)
    {
        return;
    }

    for(y = 0; y < MT9V03X_H; y++)
    {
        for(x = 0; x < MT9V03X_W; x++)
        {
            uint16 sx_q8 = 0U;
            uint16 sy_q8 = 0U;
            uint8 sample = 0U;

            if(CameraCalibration_RectifiedPixelToSourceQ8(x, y, &sx_q8, &sy_q8) &&
               (nearest ?
                    IPM_SampleGrayNearest(src_img,
                                          (float)sx_q8 / 256.0f,
                                          (float)sy_q8 / 256.0f,
                                          &sample) :
                    IPM_SampleGrayQ8(src_img, sx_q8, sy_q8, &sample)))
            {
                dst_img[y][x] = sample;
            }
            else
            {
                dst_img[y][x] = 0U;
            }
        }
    }
}

void IPM_GenerateUndistortedDebugImage(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                       uint8 dst_img[MT9V03X_H][MT9V03X_W])
{
    IPM_GenerateUndistortedDebugImageInternal(src_img, dst_img, 0U);
}

void IPM_GenerateUndistortedDebugImageNearest(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                              uint8 dst_img[MT9V03X_H][MT9V03X_W])
{
    IPM_GenerateUndistortedDebugImageInternal(src_img, dst_img, 1U);
}

uint8 IPM_SetUndistortCanvas(IPM_UndistortCanvas_t *canvas)
{
    if(canvas == NULL)
    {
        return 0U;
    }

    canvas->width = CAMERA_CALIBRATION_UNDISTORT_WIDTH;
    canvas->height = CAMERA_CALIBRATION_UNDISTORT_HEIGHT;
    if(canvas->width <= 0 || canvas->height <= 0)
    {
        return 0U;
    }

    canvas->min_rectified_x = CAMERA_CALIBRATION_CX -
        0.5f * ((float)canvas->width - 1.0f);
    canvas->max_rectified_x = canvas->min_rectified_x + (float)canvas->width - 1.0f;
    canvas->min_rectified_y = CAMERA_CALIBRATION_CY -
        0.5f * ((float)canvas->height - 1.0f);
    canvas->max_rectified_y = canvas->min_rectified_y + (float)canvas->height - 1.0f;
    return 1U;
}

static uint8 IPM_GenerateFullUndistortedImageInternal(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                                      uint8 *dst_img,
                                                      uint8 *valid_mask,
                                                      const IPM_UndistortCanvas_t *canvas,
                                                      uint8 nearest)
{
    int y;
    int x;

    if(src_img == NULL || dst_img == NULL || valid_mask == NULL || canvas == NULL ||
       canvas->width <= 0 || canvas->height <= 0)
    {
        return 0U;
    }

    for(y = 0; y < canvas->height; y++)
    {
        for(x = 0; x < canvas->width; x++)
        {
            float source_x = 0.0f;
            float source_y = 0.0f;
            uint8 sample = 0U;
            size_t index = (size_t)y * (size_t)canvas->width + (size_t)x;
            float rectified_x = canvas->min_rectified_x + (float)x;
            float rectified_y = canvas->min_rectified_y + (float)y;

            if(CameraCalibration_RectifiedModelPointToSource(rectified_x, rectified_y,
                                                             &source_x, &source_y) &&
               (nearest ? IPM_SampleGrayNearest(src_img, source_x, source_y, &sample) :
                          IPM_SampleGrayBilinear(src_img, source_x, source_y, &sample)))
            {
                dst_img[index] = sample;
                valid_mask[index] = 255U;
            }
            else
            {
                dst_img[index] = 0U;
                valid_mask[index] = 0U;
            }
        }
    }
    return 1U;
}

uint8 IPM_GenerateFullUndistortedImage(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                       uint8 *dst_img,
                                       uint8 *valid_mask,
                                       const IPM_UndistortCanvas_t *canvas)
{
    return IPM_GenerateFullUndistortedImageInternal(src_img, dst_img, valid_mask,
                                                    canvas, 0U);
}

uint8 IPM_GenerateFullUndistortedImageNearest(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                              uint8 *dst_img,
                                              uint8 *valid_mask,
                                              const IPM_UndistortCanvas_t *canvas)
{
    return IPM_GenerateFullUndistortedImageInternal(src_img, dst_img, valid_mask,
                                                    canvas, 1U);
}

void IPM_GenerateUndistortedDebugMask(uint8 dst_mask[MT9V03X_H][MT9V03X_W])
{
    int y;
    int x;

    if(dst_mask == NULL)
    {
        return;
    }

    for(y = 0; y < MT9V03X_H; y++)
    {
        for(x = 0; x < MT9V03X_W; x++)
        {
            uint16 sx_q8 = 0U;
            uint16 sy_q8 = 0U;
            dst_mask[y][x] =
                CameraCalibration_RectifiedPixelToSourceQ8(x, y,
                                                           &sx_q8, &sy_q8) ?
                255U : 0U;
        }
    }
}

/* ====================== 调试函数 ====================== */

/**
 * @brief ???? IPM ???? printf ???
 * @param roll_deg   ???????????
 * @param pitch_deg  ???????????
 * @param height_cm  输出:当前高度(cm)
 * @param nadir_x    输出:正下方点X坐标(-1表示超出图像)
 * @param nadir_y    输出:正下方点Y坐标(-1表示超出图像)
 */
void IPM_GetStatus(float *roll_deg, float *pitch_deg, float *height_cm,
                   int *nadir_x, int *nadir_y)
{
    if (roll_deg)  *roll_deg = g_ipm.roll * RAD2DEG_F;
    if (pitch_deg) *pitch_deg = g_ipm.pitch * RAD2DEG_F;
    if (height_cm) *height_cm = g_ipm.height;
    
    if (nadir_x && nadir_y) {
        int nx, ny;
        if (IPM_GetNadirPoint(&nx, &ny)) {
            *nadir_x = nx;
            *nadir_y = ny;
        } else {
            *nadir_x = -1;
            *nadir_y = -1;
        }
    }
}

/**
 * @brief ??????????????
 * @param results   ???????? 8 ? float???? 4 ????????
 * 
 * 输出格式: [TL_x, TL_y, TR_x, TR_y, BL_x, BL_y, BR_x, BR_y]
 */
void IPM_TestCorners(float *results)
{
    if (!results) return;
    
    int corners[4][2] = {
        {0, 0},             /* TL */
        {MT9V03X_W-1, 0},   /* TR */
        {0, MT9V03X_H-1},   /* BL */
        {MT9V03X_W-1, MT9V03X_H-1}  /* BR */
    };
    
    for (int i = 0; i < 4; i++) {
        float wx, wy;
        IPM_Image2World(corners[i][0], corners[i][1], &wx, &wy);
        results[i * 2] = wx;
        results[i * 2 + 1] = wy;
    }
}
