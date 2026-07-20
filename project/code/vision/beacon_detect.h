/**
 * @file beacon_detect.h
 * @brief 红外信标检测模块接口
 *
 * 模块职责:
 * 1. 对原始灰度图执行固定阈值二值化
 * 2. 用八邻域 BFS 提取连通域
 * 3. 按物理鸟瞰栅格的面积、宽高比、占空比过滤候选区域
 * 4. 通过 IPM 将像素坐标换算为地面坐标
 *
 * 说明:
 * - 本模块只负责“找信标 + 输出图像/地面坐标”
 * - 姿态补偿与逆透视由 ipm_interface / drone_ipm_embedded 提供
 * - 核心算法来自 case02 Unicycle_Beacon_2025/dual_camera 的移植
 */

#ifndef _BEACON_DETECT_H_
#define _BEACON_DETECT_H_

#include "zf_common_headfile.h"
#include "vision/vision_runtime.h"

/* ====================== 配置 ====================== */

#define BEACON_MAX_DETECTIONS   50          /* 单帧最大检测数量 */
#define BEACON_IMG_ROWS         MT9V03X_H   /* 120 */
#define BEACON_IMG_COLS         MT9V03X_W   /* 188 */
#define BEACON_DETECTION_CONFIDENCE_MIN (96U)
#define BEACON_PHYSICAL_AREA_MIN_CM2      (500.0f)
#define BEACON_PHYSICAL_AREA_IDEAL_CM2    (700.0f)
#define BEACON_PHYSICAL_AREA_MAX_CM2      (1000.0f)
#define BEACON_CONFIDENCE_BOX_SIZE_WEIGHT (0.45f)
#define BEACON_CONFIDENCE_STABILITY_WEIGHT (0.40f)
#define BEACON_CONFIDENCE_AREA_WEIGHT      (0.20f)
#define BEACON_CONFIDENCE_EDGE_WEIGHT      (0.10f)
#define BEACON_BOX_SIZE_MIN_EDGE           (0.05f)
#define BEACON_BOX_SIZE_MIN_CENTER         (0.50f)

/* ====================== 数据结构 ====================== */

/**
 * @brief 单个信标检测结果
 *
 * 与原 case02 中的连通域结果一一对应，只是额外补充了逆透视后的地面坐标。
 */
typedef struct {
    /* 图像域 */
    float centroid_row, centroid_col;      /* 质心坐标(行, 列) */
    int   area;                            /* 连通域像素面积 */
    int   min_row, max_row;                 /* 连通域外接框行范围 */
    int   min_col, max_col;                 /* 连通域外接框列范围 */
    int raw_min_row, raw_max_row;
    int raw_min_col, raw_max_col;
    float average_brightness;              /* 区域平均亮度 */
    float aspect_ratio;                    /* 外接矩形宽高比 */
    float occupancy_ratio;                 /* 面积 / 外接矩形面积 */
    float physical_area_cm2;               /* 局部物理鸟瞰栅格面积 */
    float physical_perimeter_cm;           /* 局部物理鸟瞰栅格周长 */
    float physical_circularity;             /* 局部物理鸟瞰栅格圆度 */
    float physical_aspect_ratio;            /* 局部物理鸟瞰栅格宽高比 */
    float physical_occupancy_ratio;         /* 局部物理鸟瞰栅格占空比 */
    uint16 physical_pixel_count;            /* 局部鸟瞰前景栅格数 */
    uint8 physical_valid;                   /* 物理鸟瞰指标有效 */
    int raw_box_area_pixels;                 /* 原图候选框像素面积 */
    /* 世界域 */
    float world_x, world_y;                /* 地面坐标(cm), X向前 Y向左 */
    float distance_cm;                     /* 相对正下方原点的距离(cm) */
    float angle_deg;                       /* 方位角(度), 正前方为0 */
    uint8 detection_confidence;            /* 0..255 */
    float confidence_box_size;
    float confidence_box_size_min;
    float confidence_stability;
    float confidence_area;
    float confidence_edge;
} BeaconResult_t;

/**
 * @brief 运行时可调的检测阈值
 */
typedef struct {
    int   binary_threshold;                /* 二值化阈值 */
    int   area_threshold;                  /* 最小连通域面积 */
    float min_aspect;                      /* 最小宽高比 */
    float max_aspect;                      /* 最大宽高比 */
    float min_occupancy;                   /* 最小占空比 */
    float min_circularity;                 /* 最小圆度 */
    float min_physical_area_cm2;
    float max_physical_area_cm2;
} BeaconConfig_t;

/* ====================== 全局变量 ====================== */

extern BeaconConfig_t g_beacon_config;

/* ====================== 函数接口 ====================== */

/* Runtime entry points used by cpu2_main.c */

/**
 * @brief 初始化信标检测模块
 * @note 开机时调用一次即可，当前主要用于保留统一初始化入口
 */
void BeaconDetect_Init(void);

/**
 * @brief 处理一帧图像并输出全部信标候选
 * @param image        MT9V03X 原始灰度图像
 * @param results      输出结果数组
 * @param max_results  结果数组容量(<= BEACON_MAX_DETECTIONS)
 * @return 本帧检测到的信标数量
 * @note 调用前需要先通过 IPM_Interface_UpdateFromIMU() 更新姿态与高度
 */
void BeaconDetect_BeginBlobFrame(BeaconResult_t *results, int max_results);
uint8 BeaconDetect_AppendBlob(VisionBlob_t *blob);
int BeaconDetect_EndBlobFrame(void);

/**
 * @brief 在检测结果中找出距离正下方最近的信标
 * @return 最近信标的下标，未检测到时返回 -1
 */
int BeaconDetect_GetNearestIndex(const BeaconResult_t *results, int count);

#endif /* _BEACON_DETECT_H_ */
