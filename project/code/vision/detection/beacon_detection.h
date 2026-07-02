/**
 * @file beacon_detection.h
 * @brief 红外信标检测模块接口
 *
 * 模块职责:
 * 1. 接收 VisionRuntime 已经提取出的较大连通域
 * 2. 按宽高比、占空比和容量过滤候选区域
 * 3. 输出信标图像信息和地面坐标
 *
 * 说明:
 * - 二值化、BFS 连通域、IPM_Image2World() 由 VisionRuntime 统一完成
 * - 本模块只负责“从连通域中筛出信标”
 * - 核心算法来自 case02 Unicycle_Beacon_2025/dual_camera 的移植
 */

#ifndef BEACON_DETECTION_H_
#define BEACON_DETECTION_H_

#include "zf_common_headfile.h"
#include "vision/pipeline/vision_frame_pipeline.h"

/* ====================== 配置 ====================== */

#define BEACON_MAX_DETECTIONS   50          /* 单帧最大检测数量 */
#define BEACON_IMG_ROWS         MT9V03X_H   /* 120 */
#define BEACON_IMG_COLS         MT9V03X_W   /* 188 */

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
    float average_brightness;              /* 区域平均亮度 */
    float aspect_ratio;                    /* 外接矩形宽高比 */
    float occupancy_ratio;                 /* 面积 / 外接矩形面积 */
    /* 世界域 */
    float world_x, world_y;                /* 地面坐标(cm), X向前 Y向左 */
    float distance_cm;                     /* 相对正下方原点的距离(cm) */
    float angle_deg;                       /* 方位角(度), 正前方为0 */
} BeaconResult_t;

/**
 * @brief 运行时可调的检测阈值
 */
typedef struct {
    int   binary_threshold;                /* 历史兼容字段: 当前统一使用 VISION_BINARY_THRESHOLD */
    int   area_threshold;                  /* 历史兼容字段: 当前分流使用 VISION_RUNTIME_BEACON_SPLIT_AREA_THRESHOLD */
    float min_aspect;                      /* 最小宽高比 */
    float max_aspect;                      /* 最大宽高比 */
    float min_occupancy;                   /* 最小占空比 */
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

/* 由 VisionRuntime 在单帧开始、每个候选连通域、单帧结束时依次调用。 */
void BeaconDetect_BeginBlobFrame(BeaconResult_t *results, int max_results);
uint8 BeaconDetect_AppendBlob(const VisionBlob_t *blob);
int BeaconDetect_EndBlobFrame(void);

/**
 * @brief 在检测结果中找出距离正下方最近的信标
 * @return 最近信标的下标，未检测到时返回 -1
 */
int BeaconDetect_GetNearestIndex(const BeaconResult_t *results, int count);

#endif /* BEACON_DETECTION_H_ */
