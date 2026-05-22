/**
 * @file ipm_interface.c
 * @brief ????????? - ?? TC387 ??????
 * 
 * 整合IMU姿态数据和摄像头图像，提供简化的逆透视变换调用
 */

#include "ipm_interface.h"
#include "common/project_headfile.h"

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
#if CY_CORE_CM7_0
    /* 
     * ??????? IMU ???????????????
     * ?????
     * - Roll_a > 0 表示向右倾斜
     * - Pitch_a > 0 表示抬头
     * 
     * 如果实际情况相反，需要取负号
     */
    float roll = Roll_a;   /* ? ICM.h ????? */
    float pitch = Pitch_a; /* ? ICM.h ????? */
    
    DroneIPM_Embedded_Update(roll, pitch, height_cm);
#else
    DroneIPM_Embedded_Update(0.0f, 0.0f, height_cm);
#endif
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
    DroneIPM_Embedded_Image2World((sint16)img_x, (sint16)img_y, result);
    *world_x = result[0];
    *world_y = result[1];
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
void IPM_GenerateBirdView(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                          uint8 *dst_img, int dst_w, int dst_h, float scale)
{
    DroneIPM_Embedded_BirdView((const uint8_t *)src_img, dst_img, 
                                dst_w, dst_h, scale);
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

