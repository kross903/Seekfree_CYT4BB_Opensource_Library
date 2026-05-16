/**
 * @file ir_localization.h
 * @brief 红外正方形定位模块接口，支持四点直接定位和三点补全定位
 *
 * 主流程:
 * 原始图像 -> 二值化 -> 高度自适应小圆亮斑候选 -> IPM -> 10cm 正方形几何筛选
 * -> 四点定位 / 三点补全 -> 中心滤波 -> 高度估计
 */

#ifndef IR_LOCALIZATION_H_
#define IR_LOCALIZATION_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==================== 配置参数 ====================
#define IR_SQUARE_POINT_COUNT  (4)
#define IR_SQUARE_MIN_DETECT_POINTS (3)
#define IR_SQUARE_SIDE_M       (0.10f)
#define IR_THRESHOLD           (150)
#define IR_MAX_CANDIDATES      (20)
#define IR_MIN_POINT_DIST_PX   (4.0f)
#define IR_CENTROID_RADIUS     (3)
#define IR_POSITION_LOST_HOLD_MS        (500U)
#define IR_POSITION_LOST_TASK_PERIOD_MS (5U)

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
    float raw_x;
    float raw_y;
    float z;
    float square_side_cm;
    IR_Point2D_t img_pts[IR_SQUARE_POINT_COUNT];
    IR_WorldPoint_t world_pts[IR_SQUARE_POINT_COUNT];
    uint8 detected_point_count;
    uint8 reconstructed_point;
    uint8 valid;       /* 1=当前中心可用于追踪，包含 0.5s 丢失保持期 */
} IR_Pose_t;

typedef struct
{
    uint8 raw_detect_valid;
    uint8 has_trackable_pose;
    uint8 lost_flag;
    uint8 pose_valid;
    uint16 lost_elapsed_ms;
    uint32 task_5ms_count;
    uint32 process_count;
    uint32 process_success_count;
    uint32 process_fail_count;
    uint32 mark_lost_count;
    uint32 timeout_count;
} IR_LossDebug_t;

// ==================== 对外接口 ====================
void IR_Localization_Init(void);

void IR_Localization_Task_5ms(void);

void IR_Localization_ServiceLostProtection(void);

void IR_Localization_GetLossDebug(IR_LossDebug_t *debug);

uint8 IR_Localization_Process(const uint8 image[MT9V03X_H][MT9V03X_W],
                              IR_Pose_t *out_pose);

uint8 IR_FindPoints(const uint8 image[MT9V03X_H][MT9V03X_W],
                    IR_Point2D_t out_pts[IR_SQUARE_POINT_COUNT]);

const uint8* IR_GetBinaryImage(void);

// ==================== 全局输出 ====================
extern IR_Pose_t ir_pose_estimate;
extern volatile uint8 ir_position_lost_flag;
extern volatile uint16 ir_position_lost_elapsed_ms;

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
