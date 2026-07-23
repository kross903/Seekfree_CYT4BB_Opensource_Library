#ifndef BEACON_ADAPTIVE_THRESHOLD_H_
#define BEACON_ADAPTIVE_THRESHOLD_H_

/*
 * 信标轻量姿态自适应门限。
 * 每帧只更新一次高度和倾角尺度，每个候选仅执行少量定点友好的乘除法，
 * 不生成信标逆透视图，也不遍历轮廓执行 IPM。
 */

#define BEACON_ADAPTIVE_ENABLE                    (1U)

/* 由 vision/fit_beacon_adaptive_thresholds.py 对现有分类样本拟合。 */
#define BEACON_ADAPTIVE_REFERENCE_HEIGHT_CM       (155.0f)
#define BEACON_ADAPTIVE_AREA_RADIAL_LOSS          (0.04f)
#define BEACON_ADAPTIVE_AREA_TILT_LOSS            (0.09f)
#define BEACON_ADAPTIVE_SHAPE_RADIAL_RELAXATION   (0.23f)
/* 现有样本无法稳定分离纯倾角影响，保留小幅物理先验，防止过拟合为零。 */
#define BEACON_ADAPTIVE_SHAPE_TILT_RELAXATION     (0.10f)

#define BEACON_ADAPTIVE_TILT_FULL_RELAX_DEG       (15.0f)
#define BEACON_ADAPTIVE_HEIGHT_MIN_CM              (50.0f)
#define BEACON_ADAPTIVE_HEIGHT_MAX_CM             (220.0f)
#define BEACON_ADAPTIVE_AREA_SCALE_MIN              (0.25f)
#define BEACON_ADAPTIVE_AREA_SCALE_MAX              (6.00f)
#define BEACON_ADAPTIVE_MAX_RELAXATION               (0.45f)

typedef struct
{
    float min_area_px;
    float ideal_area_px;
    float max_area_px;
    float min_aspect;
    float max_aspect;
    float min_occupancy;
    float min_circularity;
} BeaconAdaptiveBaseGate_t;

typedef struct
{
    float normalized_radius_sq;
    float tilt_ratio;
    float area_scale;
    float normalized_area_px;
    float relaxation;
    float min_area_px;
    float ideal_area_px;
    float max_area_px;
    float min_aspect;
    float max_aspect;
    float min_occupancy;
    float min_circularity;
    float box_minimum_scale;
} BeaconAdaptiveGate_t;

void BeaconAdaptiveThreshold_Init(void);
void BeaconAdaptiveThreshold_UpdateFrame(float roll_deg,
                                         float pitch_deg,
                                         float height_cm);
void BeaconAdaptiveThreshold_Evaluate(float center_col,
                                      float center_row,
                                      float area_px,
                                      const BeaconAdaptiveBaseGate_t *base,
                                      BeaconAdaptiveGate_t *gate);

#endif /* BEACON_ADAPTIVE_THRESHOLD_H_ */
