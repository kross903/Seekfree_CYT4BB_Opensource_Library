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
 * - 上接业务算法(beacon_detect / ir_localization)
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
            float sx = 0.0f;
            float sy = 0.0f;
            uint8 sample = 0U;

            if(CameraCalibration_RectifiedPointToSource((float)x, (float)y, &sx, &sy) &&
               (nearest ? IPM_SampleGrayNearest(src_img, sx, sy, &sample) :
                          IPM_SampleGrayBilinear(src_img, sx, sy, &sample)))
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

uint8 IPM_CalculateFullUndistortCanvas(IPM_UndistortCanvas_t *canvas)
{
    float min_x = FLT_MAX;
    float max_x = -FLT_MAX;
    float min_y = FLT_MAX;
    float max_y = -FLT_MAX;
    int valid_count = 0;
    int y;
    int x;

    if(canvas == NULL)
    {
        return 0U;
    }

    for(y = 0; y < MT9V03X_H; y++)
    {
        for(x = 0; x < MT9V03X_W; x++)
        {
            float rectified_x = 0.0f;
            float rectified_y = 0.0f;
            if(CameraCalibration_UndistortPoint((float)x, (float)y,
                                                &rectified_x, &rectified_y) &&
               isfinite(rectified_x) && isfinite(rectified_y))
            {
                if(rectified_x < min_x) min_x = rectified_x;
                if(rectified_x > max_x) max_x = rectified_x;
                if(rectified_y < min_y) min_y = rectified_y;
                if(rectified_y > max_y) max_y = rectified_y;
                valid_count++;
            }
        }
    }

    if(valid_count == 0)
    {
        return 0U;
    }

    canvas->min_rectified_x = floorf(min_x);
    canvas->max_rectified_x = ceilf(max_x);
    canvas->min_rectified_y = floorf(min_y);
    canvas->max_rectified_y = ceilf(max_y);
    canvas->width = (int)(canvas->max_rectified_x - canvas->min_rectified_x) + 1;
    canvas->height = (int)(canvas->max_rectified_y - canvas->min_rectified_y) + 1;
    return (canvas->width > 0 && canvas->height > 0) ? 1U : 0U;
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
            float sx = 0.0f;
            float sy = 0.0f;
            dst_mask[y][x] =
                (CameraCalibration_RectifiedPointToSource((float)x, (float)y, &sx, &sy) &&
                 sx >= 0.0f && sx <= ((float)MT9V03X_W - 1.0f) &&
                 sy >= 0.0f && sy <= ((float)MT9V03X_H - 1.0f)) ? 255U : 0U;
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
