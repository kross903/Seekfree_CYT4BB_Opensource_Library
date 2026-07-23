/**
 * @file ipm_interface.h
 * @brief 逆透视变换接口 - 用于 TC387 无人机项目
 * 
 * 本文件是 `vision` 目录里所有视觉算法访问 IPM 的统一入口。
 * 它把 IMU 姿态、高度和底层 `drone_ipm_embedded` 数学实现封装到一起，
 * 让上层模块只关心“图像坐标 <-> 地面坐标”。
 */

#ifndef _IPM_INTERFACE_H_
#define _IPM_INTERFACE_H_

#include "zf_common_headfile.h"
#include "drone_ipm_embedded.h"

/* ====================== 配置宏 ====================== */

/* 摄像头图像尺寸(必须与MT9V03X一致) */
#define IPM_IMG_WIDTH     MT9V03X_W    /* 原图宽度: 188 像素 */
#define IPM_IMG_HEIGHT    MT9V03X_H    /* 原图高度: 120 像素 */

/* 俯视图输出尺寸 
 * 注意: 俯视图尺寸与原图不同是设计如此
 * - 俯视图是正方形，便于观察各方向等比例
 * - 俯视图中心对应正下方地面点
 * - 可根据需要调整大小
 */
#define IPM_BIRDVIEW_W    120   /* 俯视图宽度: 120 像素 */
#define IPM_BIRDVIEW_H    120   /* 俯视图高度: 120 像素 */

/* 俯视图缩放系数(cm/pixel) */
#define IPM_SCALE         0.5f

typedef struct
{
    float min_world_x;
    float max_world_x;
    float min_world_y;
    float max_world_y;
    float scale;
    int width;
    int height;
} IPM_BirdViewBounds_t;

typedef struct
{
    float min_rectified_x;
    float max_rectified_x;
    float min_rectified_y;
    float max_rectified_y;
    int width;
    int height;
} IPM_UndistortCanvas_t;

/* ====================== 函数声明 ====================== */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化逆透视模块
 * @note 在 Device_Init() 中调用一次
 */
void IPM_Interface_Init(void);

/**
 * @brief 使用IMU数据更新逆透视变换
 * @param roll_deg  IMU滚转角(度)
 * @param pitch_deg IMU俯仰角(度)
 * @param height_cm ToF测量高度(cm)
 * @note 每帧图像处理前调用
 */
void IPM_Interface_Update(float roll_deg, float pitch_deg, float height_cm);

/**
 * @brief 使用当前IMU状态自动更新(从全局变量读取)
 * @param height_cm 高度(cm)
 *
 * 这是工程主流程里最常用的更新入口。
 */
void IPM_Interface_UpdateFromIMU(float height_cm);

/**
 * @brief 图像坐标转换为地面物理坐标
 * @param img_x 图像列坐标(0 ~ MT9V03X_W-1)
 * @param img_y 图像行坐标(0 ~ MT9V03X_H-1)
 * @param world_x 输出:地面X坐标(向前为正,单位cm)
 * @param world_y 输出:地面Y坐标(向左为正,单位cm)
 *
 * Vision runtime 提取的连通域最终都会走到这里。
 */
void IPM_Image2World(int img_x, int img_y, float *world_x, float *world_y);

/**
 * @brief 浮点图像坐标转换为地面物理坐标
 * @return 1 成功，0 无效点或无法与地面相交
 */
uint8 IPM_Image2WorldF(float img_x, float img_y, float *world_x, float *world_y);

/* 188x120运行时去畸变坐标与地面物理坐标互转。 */
uint8 IPM_RectifiedImage2WorldF(float img_x, float img_y,
                               float *world_x, float *world_y);
uint8 IPM_FullRectifiedImage2WorldF(float img_x, float img_y,
                                    float *world_x, float *world_y);
/* 将完整分辨率去畸变坐标投影到离地 plane_height_cm 的水平目标平面。 */
uint8 IPM_FullRectifiedImage2PlaneF(float img_x, float img_y,
                                    float plane_height_cm,
                                    float *world_x, float *world_y);

/**
 * @brief 地面物理坐标转换为图像坐标
 * @param world_x 地面X坐标(cm)
 * @param world_y 地面Y坐标(cm)
 * @param img_x 输出:图像列坐标
 * @param img_y 输出:图像行坐标
 * @return true成功, false点在图像外
 */
bool IPM_World2Image(float world_x, float world_y, int *img_x, int *img_y);
bool IPM_World2RectifiedImage(float world_x, float world_y,
                              int *img_x, int *img_y);
bool IPM_World2FullRectifiedImage(float world_x, float world_y,
                                  int *img_x, int *img_y);
/* IPM_FullRectifiedImage2PlaneF() 的反向投影。 */
bool IPM_Plane2FullRectifiedImage(float world_x, float world_y,
                                  float plane_height_cm,
                                  int *img_x, int *img_y);

/**
 * @brief 获取正下方点在图像中的位置
 * @param img_x 输出:图像列坐标
 * @param img_y 输出:图像行坐标
 * @return true在图像内, false在图像外
 */
bool IPM_GetNadirPoint(int *img_x, int *img_y);

/**
 * @brief 生成俯视图
 * @param src_img 源摄像头图像(MT9V03X格式)
 * @param dst_img 输出俯视图
 * @param dst_w   输出宽度
 * @param dst_h   输出高度
 * @param scale   缩放(cm/pixel)
 */
void IPM_GenerateBirdView(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                          uint8 *dst_img, int dst_w, int dst_h, float scale);

/* 计算所有有效原图像素在地面上的覆盖范围。 */
uint8 IPM_CalculateBirdViewBounds(float scale, IPM_BirdViewBounds_t *bounds);

/* 将鸟瞰范围限制在以正下方为中心的物理边界内。 */
uint8 IPM_ClampBirdViewBounds(IPM_BirdViewBounds_t *bounds,
                              float half_range_x,
                              float half_range_y);

/* 设置以正下方为中心的固定物理画布。 */
uint8 IPM_SetFixedBirdViewBounds(float scale,
                                 float half_range_x,
                                 float half_range_y,
                                 IPM_BirdViewBounds_t *bounds);

/* 按动态地面范围生成鸟瞰图，并返回每个输出像素是否有效。 */
uint8 IPM_GenerateBirdViewDynamic(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                  uint8 *dst_img,
                                  uint8 *valid_mask,
                                  const IPM_BirdViewBounds_t *bounds);

/* 使用二值源图和最近邻采样生成鸟瞰图，保持输出为0/255。 */
uint8 IPM_GenerateBirdViewDynamicNearest(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                         uint8 *dst_img,
                                         uint8 *valid_mask,
                                         const IPM_BirdViewBounds_t *bounds);

/* 使用完整去畸变二值画布生成鸟瞰图。 */
uint8 IPM_GenerateBirdViewFromUndistortedBinary(const uint8 *src_img,
                                                const IPM_UndistortCanvas_t *src_canvas,
                                                uint8 *dst_img,
                                                uint8 *valid_mask,
                                                const IPM_BirdViewBounds_t *bounds);

/* 使用188x120运行时去畸变二值图生成动态鸟瞰图。 */
uint8 IPM_GenerateBirdViewFromRuntimeBinary(const uint8 *src_img,
                                            uint8 *dst_img,
                                            uint8 *valid_mask,
                                            const IPM_BirdViewBounds_t *bounds);

/**
 * @brief 在IPS200屏幕上显示俯视图
 * @param src_img 源摄像头图像
 * @param x 显示起始X坐标
 * @param y 显示起始Y坐标
 * @param scale 缩放系数
 */
void IPM_DisplayBirdView(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                         int x, int y, float scale);

/**
 * @brief 在俯视图上绘制十字标记(正下方)
 * @param img 俯视图
 * @param w 图像宽度
 * @param h 图像高度
 */
void IPM_DrawCenterMark(uint8 *img, int w, int h);

/**
 * @brief 生成整图调试去畸变图。无离线映射表时输出与输入相同。
 */
void IPM_GenerateUndistortedDebugImage(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                       uint8 dst_img[MT9V03X_H][MT9V03X_W]);

void IPM_GenerateUndistortedDebugImageNearest(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                              uint8 dst_img[MT9V03X_H][MT9V03X_W]);

void IPM_GenerateUndistortedDebugMask(uint8 dst_mask[MT9V03X_H][MT9V03X_W]);

/* 逐点采样完整376x240去畸变画布，供阈值前的全分辨率处理。 */
uint8 IPM_SampleFullUndistortedGrayQ8(
    const uint8 src_img[MT9V03X_H][MT9V03X_W],
    int full_x, int full_y,
    uint16 *source_x_q8, uint16 *source_y_q8,
    uint8 *value);

/* 使用标定配置设置原焦距尺度、居中的有限视场去畸变画布。 */
uint8 IPM_SetUndistortCanvas(IPM_UndistortCanvas_t *canvas);
uint8 IPM_GenerateFullUndistortedImage(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                       uint8 *dst_img,
                                       uint8 *valid_mask,
                                       const IPM_UndistortCanvas_t *canvas);
uint8 IPM_GenerateFullUndistortedImageNearest(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                              uint8 *dst_img,
                                              uint8 *valid_mask,
                                              const IPM_UndistortCanvas_t *canvas);

/* ====================== 状态获取函数 ====================== */

/**
 * @brief 获取当前IPM状态
 * @param roll_deg   输出:当前滚转角(度)
 * @param pitch_deg  输出:当前俯仰角(度)
 * @param height_cm  输出:当前高度(cm)
 * @param nadir_x    输出:正下方点X坐标(-1表示超出图像)
 * @param nadir_y    输出:正下方点Y坐标(-1表示超出图像)
 */
void IPM_GetStatus(float *roll_deg, float *pitch_deg, float *height_cm,
                   int *nadir_x, int *nadir_y);

/**
 * @brief 测试四角坐标变换
 * @param results   输出缓冲区(至少8个float)
 *                  格式: [TL_x, TL_y, TR_x, TR_y, BL_x, BL_y, BR_x, BR_y]
 */
void IPM_TestCorners(float *results);

#ifdef __cplusplus
}
#endif

#endif /* _IPM_INTERFACE_H_ */
