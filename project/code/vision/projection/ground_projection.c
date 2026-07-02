/**
 * @file ground_projection.c
 * @brief 地面投影项目接口 - 用于 TC387 无人机项目
 * 
 * 整合 IMU 姿态、高度、相机标定和底层 IPM 数学实现，
 * 为视觉业务模块提供统一的图像坐标与地面坐标转换入口。
 */

#include "vision/projection/ground_projection.h"
#include "common/project_headfile.h"
#include "vision/projection/camera_calibration.h"

/*
 * 模块定位:
 * - 上接业务算法(beacon_detection / ir_target_localization)
 * - 下接底层数学实现(ground_projection_ipm_core)
 * - 统一处理 IMU 姿态、高度和相机标定入口
 */

/* 俯视图显示缓冲区 */
static uint8 g_birdview_buf[IPM_BIRDVIEW_W * IPM_BIRDVIEW_H];

/* ====================== 初始化 ====================== */

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
 * 双核主流程中由 CM7_0 显式传入姿态和高度。
 */
void IPM_Interface_Update(float roll_deg, float pitch_deg, float height_cm)
{
    float calibrated_roll = roll_deg + g_camera_calibration.mount_roll_offset_deg;
    float calibrated_pitch = pitch_deg + g_camera_calibration.mount_pitch_offset_deg;
    float calibrated_height = height_cm + g_camera_calibration.offset_z_cm;

    DroneIPM_Embedded_Update(calibrated_roll, calibrated_pitch, calibrated_height);
}

/**
 * @brief 从当前 IMU 全局变量更新逆透视变换
 * 
 * 兼容单核或调试路径:
 * - Roll_a: 滚转角
 * - Pitch_a: 俯仰角
 */
void IPM_Interface_UpdateFromIMU(float height_cm)
{
    /* 
     * IMU 姿态角约定:
     * - Roll_a > 0 表示向右倾斜
     * - Pitch_a > 0 表示抬头
     * 
     * 如果实际情况相反，需要取负号
     */
#if CY_CORE_CM7_0
    float roll = Roll_a;   /* 来自 ICM 姿态解算 */
    float pitch = Pitch_a; /* 来自 ICM 姿态解算 */
#else
    float roll = 0.0f;
    float pitch = 0.0f;
#endif

    IPM_Interface_Update(roll, pitch, height_cm);
}

/* ====================== 坐标变换 ====================== */

/**
 * @brief 图像坐标转换为地面坐标
 *
 * 这是上层视觉模块最常依赖的基础能力。所有点级相机标定和去畸变
 * 都先在这里处理，再进入 `DroneIPM_Embedded_Image2World()`。
 */
void IPM_Image2World(int img_x, int img_y, float *world_x, float *world_y)
{
    float result[2];
    float undistorted_x = 0.0f;
    float undistorted_y = 0.0f;

    if(world_x == NULL || world_y == NULL)
    {
        return;
    }

    CameraCalibration_UndistortPoint((float)img_x, (float)img_y,
                                     &undistorted_x, &undistorted_y);
    DroneIPM_Embedded_Image2World((sint16)(undistorted_x + 0.5f),
                                  (sint16)(undistorted_y + 0.5f),
                                  result);
    *world_x = result[0] + g_camera_calibration.offset_x_cm;
    *world_y = result[1] + g_camera_calibration.offset_y_cm;
}

/**
 * @brief 地面坐标转换为图像坐标
 */
bool IPM_World2Image(float world_x, float world_y, int *img_x, int *img_y)
{
    int result[2];
    bool valid = false;

    if(img_x == NULL || img_y == NULL)
    {
        return false;
    }

    valid = DroneIPM_Embedded_World2Image(world_x - g_camera_calibration.offset_x_cm,
                                          world_y - g_camera_calibration.offset_y_cm,
                                          result);
    if (valid) {
        *img_x = result[0];
        *img_y = result[1];
    }
    return valid;
}

/**
 * @brief 获取正下方点在图像中的位置
 */
bool IPM_GetNadirPoint(int *img_x, int *img_y)
{
    int result[2];
    bool valid = false;

    if(img_x == NULL || img_y == NULL)
    {
        return false;
    }

    valid = DroneIPM_Embedded_GetNadir(result);
    if (valid) {
        *img_x = result[0];
        *img_y = result[1];
    }
    return valid;
}

/* ====================== 俯视图显示 ====================== */

/**
 * @brief 生成俯视图
 *
 * 运行时目标检测不依赖俯视图；该接口主要用于显示和调试。
 */
void IPM_GenerateBirdView(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                          uint8 *dst_img, int dst_w, int dst_h, float scale)
{
    DroneIPM_Embedded_BirdView((const uint8_t *)src_img, dst_img, 
                                dst_w, dst_h, scale);
}

/**
 * @brief 在IPS200屏幕上显示俯视图
 * 
 * 仅 CM7_0 本地显示路径实际绘制。
 */
void IPM_DisplayBirdView(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                         int x, int y, float scale)
{
#if CY_CORE_CM7_0
    /* 生成俯视图 */
    IPM_GenerateBirdView(src_img, g_birdview_buf, 
                         IPM_BIRDVIEW_W, IPM_BIRDVIEW_H, scale);
    
    /* 绘制中心十字标记 */
    IPM_DrawCenterMark(g_birdview_buf, IPM_BIRDVIEW_W, IPM_BIRDVIEW_H);
    
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
 * @brief 在俯视图上绘制中心十字
 */
void IPM_DrawCenterMark(uint8 *img, int w, int h)
{
    int cx = w / 2;
    int cy = h / 2;
    int len = 10;  /* 十字半长 */
    
    /* 横线 */
    for (int i = -len; i <= len; i++) {
        if (cx + i >= 0 && cx + i < w) {
            img[cy * w + cx + i] = 255;  /* 白色 */
        }
    }
    
    /* 竖线 */
    for (int i = -len; i <= len; i++) {
        if (cy + i >= 0 && cy + i < h) {
            img[(cy + i) * w + cx] = 255;  /* 白色 */
        }
    }
}

/* ====================== 调试函数 ====================== */

/**
 * @brief 获取当前 IPM 状态
 * @param roll_deg   输出:当前滚转角(度)
 * @param pitch_deg  输出:当前俯仰角(度)
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
 * @brief 测试四角坐标变换
 * @param results   输出缓冲区，至少 8 个 float，依次保存 4 个角点坐标
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

