/**
 * @file ir_localization.h
 * @brief 三点红外定位模块接口
 *
 * 主流程:
 * 原始图像 -> 二值化 -> 三个亮点质心 -> IPM -> 三角形中心/边长 -> 高度估计
 */

#ifndef IR_LOCALIZATION_H_
#define IR_LOCALIZATION_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==================== 配置参数 ====================
#define IR_TRIANGLE_SIDE_M     (0.095f)
#define IR_THRESHOLD           (150)
#define IR_MAX_CANDIDATES      (20)
#define IR_MIN_POINT_DIST_PX   (4.0f)
#define IR_CENTROID_RADIUS     (3)

// ==================== 数据结构 ====================
typedef struct
{
    float u;
    float v;
    float intensity;
} IR_Point2D_t;

typedef struct
{
    float x;
    float y;
} IR_WorldPoint_t;

typedef struct
{
    float x;
    float y;
    float z;
    float triangle_side_cm;
    IR_Point2D_t img_pts[3];
    IR_WorldPoint_t world_pts[3];
    uint8 valid;
} IR_Pose_t;

// ==================== 对外接口 ====================
void IR_Localization_Init(void);

uint8 IR_Localization_Process(const uint8 image[MT9V03X_H][MT9V03X_W],
                              IR_Pose_t *out_pose);

uint8 IR_FindPoints(const uint8 image[MT9V03X_H][MT9V03X_W],
                    IR_Point2D_t out_pts[3]);

const uint8* IR_GetBinaryImage(void);

// ==================== 全局输出 ====================
extern IR_Pose_t ir_pose_estimate;

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 给高度融合侧提供“视觉样本是否刚刚更新”的判断依据。
 * 2. 避免旧视觉高度在多个融合周期里被重复使用。
 */
extern volatile uint32 ir_pose_update_seq;

#ifdef __cplusplus
}
#endif

#endif /* IR_LOCALIZATION_H_ */
