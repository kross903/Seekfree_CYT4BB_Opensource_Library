/**
 * @file vision_frame_pipeline.h
 * @brief 每帧视觉流水线公开接口
 */

#ifndef VISION_FRAME_PIPELINE_H_
#define VISION_FRAME_PIPELINE_H_

#include "zf_common_headfile.h"

/*
 * VisionRuntime 是当前视觉主流程入口:
 * - 统一读取 MT9V03X 图像
 * - 统一二值化与八邻域连通域提取
 * - 将小光斑分发给 IR_Localization，将大光斑分发给 BeaconDetect
 * - 汇总跨核回传和 VOFA/屏幕显示需要的诊断数据
 */

/* 全局二值化阈值。IR 和 Beacon 不再各自扫描整图，当前都复用该阈值。 */
#define VISION_BINARY_THRESHOLD (150)
#define VISION_RUNTIME_BEACON_SLOT_COUNT (5U)
#define VISION_RUNTIME_BLOB_AREA_COUNT (16U)

/* 连通域面积大于该值时进入 BeaconDetect，否则进入 IR_Localization。 */
#define VISION_RUNTIME_BEACON_SPLIT_AREA_THRESHOLD (17)

typedef struct
{
    /* 图像域连通域信息 */
    float centroid_row;
    float centroid_col;
    int area;
    int min_row;
    int max_row;
    int min_col;
    int max_col;
    float average_brightness;
    float aspect_ratio;
    float occupancy_ratio;

    /* 由 IPM_Image2World() 统一换算出的地面坐标(cm)。 */
    float world_x;
    float world_y;
} VisionBlob_t;

typedef struct
{
    /* 连通域与分发诊断 */
    int blob_count;
    int ir_split_count;
    int beacon_split_count;
    int ir_blob_count;
    int beacon_blob_count;
    int blob_area_rank[VISION_RUNTIME_BLOB_AREA_COUNT];
    int ir_reject_area_low_count;
    int ir_reject_area_high_count;
    int ir_reject_size_count;
    int ir_reject_aspect_count;
    int ir_reject_insert_count;

    /* IR 红外板定位诊断 */
    uint8 ir_raw_detect_valid;
    uint8 ir_track_valid;
    uint8 ir_detected_point_count;
    float ir_x_cm;
    float ir_y_cm;
    float ir_square_side_cm;
    float ir_height_cm;

    /* Beacon 信标诊断 */
    int beacon_count;
    uint8 beacon_has_nearest;
    float beacon_nearest_x_cm;
    float beacon_nearest_y_cm;
    float beacon_nearest_distance_cm;
    float beacon_nearest_area;
    float beacon_nearest_brightness;
    float beacon_nearest_occupancy;
    float beacon_nearest_aspect;
} VisionRuntimeDiag_t;

typedef struct
{
    uint8 valid;
    float x_cm;
    float y_cm;
} VisionRuntimeBeaconSlot_t;

void VisionRuntime_ProcessFrame(float height_cm);
void VisionRuntime_ProcessFrameWithPose(float roll_deg, float pitch_deg, float height_cm);
void VisionRuntime_GetDiag(VisionRuntimeDiag_t *diag);
void VisionRuntime_SetDiag(const VisionRuntimeDiag_t *diag);
void VisionRuntime_GetBeaconSlots(VisionRuntimeBeaconSlot_t *slots, uint8 max_slots);
void VisionRuntime_SetBeaconSlots(const VisionRuntimeBeaconSlot_t *slots, uint8 max_slots);

#endif /* VISION_FRAME_PIPELINE_H_ */
