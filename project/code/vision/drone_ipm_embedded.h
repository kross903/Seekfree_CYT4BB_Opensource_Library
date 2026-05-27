/**
 * @file drone_ipm_embedded.h
 * @brief 无人机正朝下摄像头逆透视变换 - 嵌入式优化版本
 *
 * 针对嵌入式平台(如TC387)的优化:
 * - 使用定点数或float减少计算量
 * - 预计算常用系数
 * - 查找表加速
 *
 * 适用于无人机正朝下摄像头，消除roll和pitch引起的透视畸变
 *
 * 说明:
 * - 这是当前工程真正依赖的 IPM 底层实现
 * - `ipm_interface` 在此基础上做项目级封装
 */

#ifndef _DRONE_IPM_EMBEDDED_H_
#define _DRONE_IPM_EMBEDDED_H_

/* 检测是否在TC387项目中(有zf_common_headfile.h) */
#if defined(__TASKING__) || defined(_USE_ZF_HEADFILE_)
    #include "zf_common_headfile.h"
    /* 使用逐飞库的图像尺寸定义 */
    #undef IMAGE_WIDTH
    #undef IMAGE_HEIGHT
    #define IMAGE_WIDTH   MT9V03X_W
    #define IMAGE_HEIGHT  MT9V03X_H
#else
    #include <stdint.h>
    #include <stdbool.h>
    /* 独立编译时的类型定义 */
    typedef int16_t  sint16;
    typedef uint16_t uint16;
    typedef int32_t  sint32;
#endif

#include <math.h>

/* ====================== 配置参数 ====================== */

/* 图像尺寸配置(如果未定义) */
#ifndef IMAGE_WIDTH
#define IMAGE_WIDTH   188   /* 根据实际摄像头修改 */
#endif

#ifndef IMAGE_HEIGHT
#define IMAGE_HEIGHT  120   /* 根据实际摄像头修改 */
#endif

/* 摄像头参数 - 根据MT9V03X实际标定修改 */
#ifndef CAM_FOV_X
#define CAM_FOV_X     50.0f  /* 水平半视场角(度) */
#endif

#ifndef CAM_FOV_Y
#define CAM_FOV_Y     35.0f  /* 垂直半视场角(度) */
#endif

/* 常量定义 */
#ifndef PI_F
#define PI_F          3.14159265f
#endif
#define DEG2RAD_F     (PI_F / 180.0f)
#define RAD2DEG_F     (180.0f / PI_F)

#ifndef DRONE_IPM_EMBEDDED_LUT_ENABLE
#define DRONE_IPM_EMBEDDED_LUT_ENABLE (0)
#endif

/* ====================== 预计算系数结构 ====================== */

/**
 * @brief IPM预计算系数
 *
 * 为了减少实时计算量，将与姿态相关的系数预先计算好
 */
typedef struct {
    /* 旋转矩阵元素 */
    float R[3][3];

    /* 摄像头位置(世界系) */
    float X0, Y0, Z0;

    /* FOV相关系数 */
    float K_fov_x;   /* = tan(fov_x) / (IMAGE_WIDTH/2) */
    float K_fov_y;   /* = tan(fov_y) / (IMAGE_HEIGHT/2) */

    /* 逆变换系数(世界->图像) */
    float K_inv;     /* = IMAGE_WIDTH / (2 * tan(fov_x)) */

    /* 当前状态 */
    float roll;      /* 弧度 */
    float pitch;     /* 弧度 */
    float height;    /* 高度 */
} DroneIPM_Coeff_t;

/* 全局系数变量 */
extern DroneIPM_Coeff_t g_ipm;

/* ====================== 函数声明 ====================== */

/**
 * @brief 初始化IPM系数(开机时调用一次)
 */
void DroneIPM_Embedded_Init(void);

/**
 * @brief 更新姿态(每帧或姿态变化时调用)
 * @param roll_deg 滚转角(度)
 * @param pitch_deg 俯仰角(度)
 * @param height 高度(与输出坐标单位一致)
 */
void DroneIPM_Embedded_Update(float roll_deg, float pitch_deg, float height);

/**
 * @brief 图像坐标到地面坐标(逆透视)
 * @param img_x 图像列坐标
 * @param img_y 图像行坐标
 * @param result 输出[world_x, world_y]
 */
void DroneIPM_Embedded_Image2World(sint16 img_x, sint16 img_y, float *result);

/**
 * @brief 地面坐标到图像坐标(正透视)
 * @param world_x 地面X坐标
 * @param world_y 地面Y坐标
 * @param result 输出[img_x, img_y]
 * @return true成功, false失败
 */
bool DroneIPM_Embedded_World2Image(float world_x, float world_y, int *result);

/**
 * @brief 获取正下方点在图像中的位置
 * @param result 输出[img_x, img_y]
 * @return true在图像内, false在图像外
 */
bool DroneIPM_Embedded_GetNadir(int *result);

/**
 * @brief 生成俯视图(查表法)
 * @param src 源图像
 * @param dst 目标图像
 * @param dst_w 目标宽度
 * @param dst_h 目标高度
 * @param scale 缩放系数(物理单位/像素)
 */
void DroneIPM_Embedded_BirdView(const uint8_t *src, uint8_t *dst,
                                 int dst_w, int dst_h, float scale);

/* ====================== 内联辅助函数 ====================== */

/**
 * @brief 快速计算sin和cos(查表或泰勒展开)
 */
static inline void fast_sincos(float angle_rad, float *s, float *c)
{
    /* 简单实现，可替换为查表或CORDIC */
    *s = sinf(angle_rad);
    *c = cosf(angle_rad);
}

#endif /* _DRONE_IPM_EMBEDDED_H_ */

