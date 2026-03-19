/**
 * @file drone_ipm.h
 * @brief 无人机正朝下摄像头逆透视变换 (Inverse Perspective Mapping)
 * 
 * 功能说明：
 * - 无人机有正朝下摄像头，捕捉地面平面信息
 * - 在有roll角和pitch角和高度z的情况下，实现图像的逆透视
 * - 确保正下方的成像始终在现实图像的中间
 * 
 * 坐标系定义：
 * - 世界坐标系：X向前，Y向右，Z向下（NED坐标系）
 * - 机体坐标系：与世界坐标系对齐时，X向前，Y向右，Z向下
 * - 图像坐标系：原点在图像左上角，u向右，v向下
 * 
 * 角度定义：
 * - Roll (φ)：绕X轴旋转，右倾为正
 * - Pitch (θ)：绕Y轴旋转，抬头为正
 *
 * 说明：
 * - 这是通用/双精度版本，更适合离线验证、桌面实验或推导参考
 * - 当前 TC387 工程运行时优先使用 `drone_ipm_embedded.*`
 */

#ifndef _DRONE_IPM_H_
#define _DRONE_IPM_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// 包含通用数学定义（M_PI, DEG2RAD, RAD2DEG等）
#include "../control/ControlMath.h"

/* 默认图像尺寸 */
#define DEFAULT_IMAGE_WIDTH   640
#define DEFAULT_IMAGE_HEIGHT  480

/* 默认摄像头参数 */
#define DEFAULT_FOV_X  60.0   /* 水平视场角(度) */
#define DEFAULT_FOV_Y  45.0   /* 垂直视场角(度) */

/* ====================== 数据结构 ====================== */

/**
 * @brief 四元数结构（双精度，用于IPM高精度计算）
 */
typedef struct {
    double w, x, y, z;
} QuaternionD_t;

/**
 * @brief 三维向量结构
 */
typedef struct {
    double x, y, z;
} Vector3_t;

/**
 * @brief 二维点结构(整数)
 */
typedef struct {
    int x, y;
} Point2i_t;

/**
 * @brief 二维点结构(浮点)
 */
typedef struct {
    double x, y;
} Point2d_t;

/**
 * @brief 摄像头内参结构
 */
typedef struct {
    int image_width;      /* 图像宽度(像素) */
    int image_height;     /* 图像高度(像素) */
    double fov_x;         /* 水平视场角(度) */
    double fov_y;         /* 垂直视场角(度) */
    double fx, fy;        /* 焦距(像素单位) */
    double cx, cy;        /* 主点坐标(像素) */
} CameraIntrinsics_t;

/**
 * @brief 无人机状态结构
 */
typedef struct {
    double roll;          /* 滚转角(度) - 绕X轴 */
    double pitch;         /* 俯仰角(度) - 绕Y轴 */
    double yaw;           /* 偏航角(度) - 绕Z轴(可选,逆透视通常不需要) */
    double height;        /* 飞行高度(单位与输出坐标一致,如cm或m) */
} DroneState_t;

/**
 * @brief IPM变换器结构
 */
typedef struct {
    CameraIntrinsics_t camera;  /* 摄像头内参 */
    DroneState_t state;         /* 无人机状态 */
    
    /* 预计算的旋转矩阵 (机体系到世界系) */
    double R[3][3];
    
    /* 预计算的中间变量 */
    double tan_half_fov_x;
    double tan_half_fov_y;
    
    /* 摄像头在世界坐标系中的位置 */
    double cam_pos_world[3];
    
    /* 输出图像的物理尺寸设置(可选) */
    double output_scale;        /* 输出缩放系数(cm/pixel 或 m/pixel) */
    double output_center_x;     /* 输出图像中心对应的物理X坐标 */
    double output_center_y;     /* 输出图像中心对应的物理Y坐标 */
} DroneIPM_t;

/* ====================== 函数声明 ====================== */

/**
 * @brief 初始化摄像头内参
 * @param cam 摄像头内参结构指针
 * @param width 图像宽度
 * @param height 图像高度
 * @param fov_x 水平视场角(度)
 * @param fov_y 垂直视场角(度)
 */
void DroneIPM_InitCamera(CameraIntrinsics_t *cam, int width, int height, 
                          double fov_x, double fov_y);

/**
 * @brief 初始化IPM变换器
 * @param ipm IPM变换器指针
 * @param cam 摄像头内参指针
 */
void DroneIPM_Init(DroneIPM_t *ipm, const CameraIntrinsics_t *cam);

/**
 * @brief 更新无人机状态并重新计算变换矩阵
 * @param ipm IPM变换器指针
 * @param roll 滚转角(度)
 * @param pitch 俯仰角(度)
 * @param height 高度
 */
void DroneIPM_Update(DroneIPM_t *ipm, double roll, double pitch, double height);

/**
 * @brief 设置输出缩放参数
 * @param ipm IPM变换器指针
 * @param scale 输出缩放系数(物理单位/像素)
 * @param center_x 输出中心对应的物理X坐标
 * @param center_y 输出中心对应的物理Y坐标
 */
void DroneIPM_SetOutputScale(DroneIPM_t *ipm, double scale, 
                              double center_x, double center_y);

/**
 * @brief 逆透视变换：图像坐标 -> 地面物理坐标
 * @param ipm IPM变换器指针
 * @param img_x 图像X坐标(列)
 * @param img_y 图像Y坐标(行)
 * @param world_x 输出:地面物理X坐标(向前为正)
 * @param world_y 输出:地面物理Y坐标(向右为正)
 * @return true成功，false失败(如光线与地面平行)
 */
bool DroneIPM_ImageToWorld(const DroneIPM_t *ipm, int img_x, int img_y,
                            double *world_x, double *world_y);

/**
 * @brief 正透视变换：地面物理坐标 -> 图像坐标
 * @param ipm IPM变换器指针
 * @param world_x 地面物理X坐标
 * @param world_y 地面物理Y坐标
 * @param img_x 输出:图像X坐标
 * @param img_y 输出:图像Y坐标
 * @return true成功，false失败(点在图像外)
 */
bool DroneIPM_WorldToImage(const DroneIPM_t *ipm, double world_x, double world_y,
                            int *img_x, int *img_y);

/**
 * @brief 生成逆透视变换后的俯视图
 * @param ipm IPM变换器指针
 * @param src_img 源图像数据(灰度)
 * @param dst_img 目标图像数据(俯视图)
 * @param dst_width 目标图像宽度
 * @param dst_height 目标图像高度
 * @param scale 物理尺寸缩放(物理单位/像素)
 */
void DroneIPM_GenerateBirdView(const DroneIPM_t *ipm,
                                const uint8_t *src_img,
                                uint8_t *dst_img,
                                int dst_width, int dst_height,
                                double scale);

/**
 * @brief 获取正下方点在图像中的位置
 * @param ipm IPM变换器指针
 * @param img_x 输出:正下方点的图像X坐标
 * @param img_y 输出:正下方点的图像Y坐标
 * @return true在图像内，false在图像外
 */
bool DroneIPM_GetNadirPoint(const DroneIPM_t *ipm, int *img_x, int *img_y);

/* ====================== 辅助函数 ====================== */

/**
 * @brief 从欧拉角创建旋转矩阵 (ZYX顺序: yaw -> pitch -> roll)
 * @param roll 滚转角(弧度)
 * @param pitch 俯仰角(弧度)
 * @param yaw 偏航角(弧度)
 * @param R 输出:3x3旋转矩阵
 */
void DroneIPM_EulerToRotationMatrix(double roll, double pitch, double yaw, 
                                     double R[3][3]);

/**
 * @brief 矩阵向量乘法
 * @param R 3x3矩阵
 * @param v 输入向量
 * @param result 输出向量
 */
void DroneIPM_MatVecMul(const double R[3][3], const double v[3], double result[3]);

/**
 * @brief 矩阵转置
 * @param R 输入3x3矩阵
 * @param Rt 输出转置矩阵
 */
void DroneIPM_MatTranspose(const double R[3][3], double Rt[3][3]);

#endif /* _DRONE_IPM_H_ */

