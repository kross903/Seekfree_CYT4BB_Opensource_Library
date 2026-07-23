#include "pmw_flow.h"
#include "common/project_headfile.h"
#include <math.h>

#define PMW_FLOW_DEG2RAD                    (0.01745329251994329577f)
#define PMW_FLOW_RAD_PER_COUNT_X            FLOW_SENSOR_RAD_PER_COUNT_X
#define PMW_FLOW_RAD_PER_COUNT_Y            FLOW_SENSOR_RAD_PER_COUNT_Y
#define PMW_FLOW_SENSOR_X_SIGN              FLOW_SENSOR_X_SIGN
#define PMW_FLOW_SENSOR_Y_SIGN              FLOW_SENSOR_Y_SIGN
/* Rotational compensation mapping is sensor-specific; keep the cross-axis
 * relationship configurable so PMW3901 and UP-FLOW-302 can use different signs. */
#define PMW_FLOW_GYRO_X_DELTA_SIGN          (1.0f)
#define PMW_FLOW_GYRO_Y_DELTA_SIGN          (1.0f)
#define PMW_FLOW_COMP_X_FROM_GYRO_X_SIGN    FLOW_SENSOR_COMP_X_FROM_GYRO_X_SIGN
#define PMW_FLOW_COMP_X_FROM_GYRO_Y_SIGN    FLOW_SENSOR_COMP_X_FROM_GYRO_Y_SIGN
#define PMW_FLOW_COMP_Y_FROM_GYRO_X_SIGN    FLOW_SENSOR_COMP_Y_FROM_GYRO_X_SIGN
#define PMW_FLOW_COMP_Y_FROM_GYRO_Y_SIGN    FLOW_SENSOR_COMP_Y_FROM_GYRO_Y_SIGN

/*
 * 陀螺去旋转增益（按轴独立）。
 * 来源：历史光流标定数据中对 flow_x_rad ~ gyro_y_delta_rad、
 *       flow_y_rad ~ gyro_x_delta_rad 做最小二乘回归，斜率分别 ≈ 1.43 / 1.32，
 *       说明默认 K=1 时陀螺补偿不足，flow_comp 残留约 30~40% 同向旋转分量。
 * 这里把 K_X / K_Y 提取为可调增益：
 *   - 编译期默认值在下方宏；
 *   - 运行期可通过 PMW_Flow_SetGyroCompGain() 动态拨动（VOFA 调参用）。
 * 不动 RAD_PER_COUNT，速度尺度保持现状。
 */
#define PMW_FLOW_GYRO_COMP_GAIN_X_DEFAULT   (0.82f)
#define PMW_FLOW_GYRO_COMP_GAIN_Y_DEFAULT   (0.94f)
#define PMW_FLOW_GYRO_COMP_GAIN_MIN         (0.0f)
#define PMW_FLOW_GYRO_COMP_GAIN_MAX         (4.0f)
/*
 * Body velocity mapping from compensated optical-flow integrals.
 * Project convention after migration:
 * - body X positive: forward
 * - body Y positive: left
 */
#define PMW_FLOW_BODY_VX_FROM_FLOW_X_SIGN   (1.0f)
#define PMW_FLOW_BODY_VY_FROM_FLOW_Y_SIGN   (1.0f)
#define PMW_FLOW_MIN_DT_S                   (FLOW_SENSOR_NOMINAL_DT_S * 0.5f)
#define PMW_FLOW_MAX_DT_S                   (FLOW_SENSOR_NOMINAL_DT_S * 2.0f)
#define PMW_FLOW_MAX_ABS_DELTA_COUNT        (5000)
#define PMW_FLOW_TWO_PI                     (6.28318530717958647692f)

/*
 * 光流速度鲁棒化参数：坏数据先门控，速度变化只做重力加速度量级的斜率限制。
 * UP-FLOW-302 当前约 48Hz / dt=20.8ms，15Hz 一阶低通的 alpha 约 0.86。
 */
#define PMW_FLOW_RANGE_LPF_ALPHA            (0.40f)
#define PMW_FLOW_GRAVITY_CMS2               (980.665f)
#define PMW_FLOW_VEL_LPF_CUTOFF_HZ          (15.0f)
#define PMW_FLOW_VEL_DEADBAND_CMS           (5.0f)
#define PMW_FLOW_TILT_GATE_DEG              (25.0f)
#define PMW_FLOW_VEL_ABS_LIMIT_CMS          (300.0f)
#define PMW_FLOW_VEL_SLEW_LIMIT_CMS2        (1500.0f)
#define PMW_FLOW_VEL_SLEW_MIN_STEP_CMS      (12.0f)
#define PMW_FLOW_VEL_OUTPUT_SLEW_LIMIT_CMS2 PMW_FLOW_GRAVITY_CMS2
#define PMW_FLOW_FUSION_ALPHA_DEFAULT       (0.30f)
#define PMW_FLOW_CONFIDENCE_MIN_UPDATE      (0.05f)
#define PMW_FLOW_CONFIDENCE_TILT_GOOD_DEG   (8.0f)
#define PMW_FLOW_CONFIDENCE_RESIDUAL_GOOD_CMS (40.0f)
#define PMW_FLOW_CONFIDENCE_RESIDUAL_BAD_CMS  (160.0f)
#define PMW_FLOW_CONFIDENCE_DT_GOOD_RATIO   (0.20f)
#define PMW_FLOW_CONFIDENCE_DT_BAD_RATIO    (0.50f)
#define PMW_FLOW_CONFIDENCE_SPEED_GOOD_RATIO (0.50f)
#define PMW_FLOW_CONFIDENCE_SPEED_BAD_RATIO  (0.90f)
#define PMW_FLOW_CONFIDENCE_TOF_HOLD_SCALE  (0.75f)
#define PMW_FLOW_CONFIDENCE_MIN_VALID_OBSERVATION (0.30f)
#define PMW_FLOW_FUSION_IMU_X_SIGN          (1.0f)
#define PMW_FLOW_FUSION_IMU_Y_SIGN          (-1.0f)
#define PMW_FLOW_FUSION_IMU_DELTA_LIMIT_CMS2 PMW_FLOW_GRAVITY_CMS2
#define PMW_FLOW_INNOVATION_GATE_MIN_CMS    (60.0f)
#define PMW_FLOW_INNOVATION_GATE_ACCEL_MULT (2.0f)
#define PMW_FLOW_REACQUIRE_CONSISTENCY_CMS  (30.0f)
#define PMW_FLOW_REACQUIRE_CONFIRM_FRAMES   (3U)
#define PMW_FLOW_PREDICT_HOLD_S             (0.120f)
#ifndef PMW_FLOW_KALMAN_ENABLE
#define PMW_FLOW_KALMAN_ENABLE             (1U)
#endif
#ifndef PMW_FLOW_KALMAN_Q_RATE_CMS2_PER_S
#define PMW_FLOW_KALMAN_Q_RATE_CMS2_PER_S  (1000.0f)
#endif
#ifndef PMW_FLOW_KALMAN_R_BASE_CMS2
#define PMW_FLOW_KALMAN_R_BASE_CMS2        (900.0f)
#endif
#ifndef PMW_FLOW_KALMAN_P_INIT
#define PMW_FLOW_KALMAN_P_INIT             (625.0f)
#endif
#ifndef PMW_FLOW_KALMAN_R_MAX_SCALE
#define PMW_FLOW_KALMAN_R_MAX_SCALE        (20.0f)
#endif
#define PMW_FLOW_WORLD_TILT_GATE_DEG        (25.0f)

/*
 * 机体系 → 水平面投影时 1/cos 的下限保护：cos(60°)=0.5。
 * 正常情况倾角门把超过 25° 的样本冻结掉，下面的 1/cos 顶多放大 1/cos(25°) ≈ 1.10×；
 * 这个 floor 只是兜底，避免姿态读数瞬间出现极端值时除法爆掉。
 */
#define PMW_FLOW_TILT_COS_FLOOR             (0.5f)

typedef struct
{
    float x;
    float p;
    float q;
    float r_base;
    float k;
    float r;
    uint8 initialized;
} PMWFlowKalman1D_t;

typedef struct
{
    float velocity_filt_cms;
    float velocity_output_cms;
    float last_trusted_raw_cms;
    float candidate_cms;
    float prediction_cms;
    float innovation_cms;
    float gate_cms;
    float raw_jump_cms;
    float prediction_age_s;
    float kalman_k;
    uint32 reject_count;
    uint8 filter_initialized;
    uint8 output_initialized;
    uint8 last_trusted_valid;
    uint8 candidate_count;
    uint8 observation_accepted;
    uint8 recovery_blend;
    uint8 reject_reason_mask;
    uint8 state;
} PMWFlowAxisRuntime_t;

static float g_pmw_flow_gyro_flow_x_delta_rad = 0.0f;
static float g_pmw_flow_gyro_flow_y_delta_rad = 0.0f;
static float g_pmw_flow_gyro_comp_gain_x = PMW_FLOW_GYRO_COMP_GAIN_X_DEFAULT;
static float g_pmw_flow_gyro_comp_gain_y = PMW_FLOW_GYRO_COMP_GAIN_Y_DEFAULT;
static PMWFlowState_t g_pmw_flow_publish_slots[2] = {0};
static volatile uint8 g_pmw_flow_publish_index = 0U;
static volatile uint32 g_pmw_flow_publish_seq = 0U;
static PMWFlowState_t g_pmw_flow_mainloop_state = {0};
static uint32 g_pmw_flow_mainloop_seq = 0U;

static float g_pmw_flow_range_filt_cm = 0.0f;
static uint8 g_pmw_flow_range_filt_init = 0U;
static PMWFlowAxisRuntime_t g_pmw_flow_axis[2] = {{0}};
static float g_pmw_flow_fused_vx_cms = 0.0f;
static float g_pmw_flow_fused_vy_cms = 0.0f;
static float g_pmw_flow_fusion_imu_vx_last_cms = 0.0f;
static float g_pmw_flow_fusion_imu_vy_last_cms = 0.0f;
static uint8 g_pmw_flow_fusion_imu_last_initialized = 0U;
static PMWFlowKalman1D_t g_pmw_flow_kalman_x = {0};
static PMWFlowKalman1D_t g_pmw_flow_kalman_y = {0};
static float g_pmw_flow_world_vx_cms = 0.0f;
static float g_pmw_flow_world_vy_cms = 0.0f;
static float g_pmw_flow_world_x_cm = 0.0f;
static float g_pmw_flow_world_y_cm = 0.0f;
static float g_pmw_flow_delta_world_x_cm = 0.0f;
static float g_pmw_flow_delta_world_y_cm = 0.0f;
static float g_pmw_flow_direct_world_x_cm = 0.0f;
static float g_pmw_flow_direct_world_y_cm = 0.0f;

void PMW_Flow_ResetWorldPosition(void);

static uint8 pmw_flow_is_valid_number(float value)
{
    return (uint8)((value == value) && (value > -1000000.0f) && (value < 1000000.0f));
}

static uint8 pmw_flow_is_valid_delta_count(int16 value)
{
    return (uint8)((value > -PMW_FLOW_MAX_ABS_DELTA_COUNT) && (value < PMW_FLOW_MAX_ABS_DELTA_COUNT));
}

static float pmw_flow_apply_velocity_deadband(float velocity_cms)
{
    if(fabsf(velocity_cms) < PMW_FLOW_VEL_DEADBAND_CMS)
    {
        velocity_cms = 0.0f;
    }

    return velocity_cms;
}

static float pmw_flow_limit_float(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        value = max_value;
    }
    else if(value < min_value)
    {
        value = min_value;
    }

    return value;
}

static float pmw_flow_calc_lpf_alpha(float cutoff_hz, float dt_s)
{
    float alpha;

    if(!(cutoff_hz > 0.0f) || !(dt_s > 0.0f))
    {
        return 1.0f;
    }

    alpha = 1.0f - expf(-PMW_FLOW_TWO_PI * cutoff_hz * dt_s);
    return pmw_flow_limit_float(alpha, 0.0f, 1.0f);
}

static void pmw_flow_kalman_reset_axis(PMWFlowKalman1D_t *kf)
{
    if(kf == NULL)
    {
        return;
    }

    kf->x = 0.0f;
    kf->p = PMW_FLOW_KALMAN_P_INIT;
    kf->q = PMW_FLOW_KALMAN_Q_RATE_CMS2_PER_S;
    kf->r_base = PMW_FLOW_KALMAN_R_BASE_CMS2;
    kf->k = 0.0f;
    kf->r = PMW_FLOW_KALMAN_R_BASE_CMS2;
    kf->initialized = 0U;
}

static void pmw_flow_kalman_reset(void)
{
    pmw_flow_kalman_reset_axis(&g_pmw_flow_kalman_x);
    pmw_flow_kalman_reset_axis(&g_pmw_flow_kalman_y);
}

static void pmw_flow_kalman_init_axis(PMWFlowKalman1D_t *kf, float value)
{
    if(kf == NULL)
    {
        return;
    }

    kf->x = pmw_flow_limit_float(value, -PMW_FLOW_VEL_ABS_LIMIT_CMS, PMW_FLOW_VEL_ABS_LIMIT_CMS);
    kf->p = PMW_FLOW_KALMAN_P_INIT;
    kf->q = PMW_FLOW_KALMAN_Q_RATE_CMS2_PER_S;
    kf->r_base = PMW_FLOW_KALMAN_R_BASE_CMS2;
    kf->k = 0.0f;
    kf->r = PMW_FLOW_KALMAN_R_BASE_CMS2;
    kf->initialized = 1U;
}

#if PMW_FLOW_KALMAN_ENABLE
static float pmw_flow_kalman_calc_r(float confidence)
{
    float confidence_safe;
    float r;

    confidence_safe = pmw_flow_limit_float(confidence, PMW_FLOW_CONFIDENCE_MIN_UPDATE, 1.0f);
    r = PMW_FLOW_KALMAN_R_BASE_CMS2 / confidence_safe;
    return pmw_flow_limit_float(r,
                                PMW_FLOW_KALMAN_R_BASE_CMS2,
                                PMW_FLOW_KALMAN_R_BASE_CMS2 * PMW_FLOW_KALMAN_R_MAX_SCALE);
}

static void pmw_flow_kalman_predict_axis(PMWFlowKalman1D_t *kf, float delta_cms, float dt_s)
{
    if((kf == NULL) || !kf->initialized)
    {
        return;
    }

    kf->x = pmw_flow_limit_float(kf->x + delta_cms,
                                 -PMW_FLOW_VEL_ABS_LIMIT_CMS,
                                 PMW_FLOW_VEL_ABS_LIMIT_CMS);
    kf->k = 0.0f;

    if(pmw_flow_is_valid_number(kf->p) && (kf->p >= 0.0f) && (dt_s > 0.0f))
    {
        kf->p += kf->q * dt_s;
    }
    else
    {
        kf->p = PMW_FLOW_KALMAN_P_INIT;
    }
}

static void pmw_flow_kalman_update_axis(PMWFlowKalman1D_t *kf,
                                        float measurement_cms,
                                        float confidence,
                                        float innovation_limit_cms)
{
    float innovation_cms;
    float denominator;

    if((kf == NULL) || !pmw_flow_is_valid_number(measurement_cms))
    {
        return;
    }

    if(!kf->initialized)
    {
        pmw_flow_kalman_init_axis(kf, measurement_cms);
    }

    kf->r = pmw_flow_kalman_calc_r(confidence);
    denominator = kf->p + kf->r;
    if(!(denominator > 0.0f) || !pmw_flow_is_valid_number(denominator))
    {
        kf->p = PMW_FLOW_KALMAN_P_INIT;
        kf->k = 0.0f;
        return;
    }

    kf->k = pmw_flow_limit_float(kf->p / denominator, 0.0f, 1.0f);
    innovation_cms = measurement_cms - kf->x;
    if(innovation_limit_cms > 0.0f)
    {
        innovation_cms = pmw_flow_limit_float(innovation_cms,
                                              -innovation_limit_cms,
                                              innovation_limit_cms);
    }
    kf->x = pmw_flow_limit_float(kf->x + kf->k * innovation_cms,
                                 -PMW_FLOW_VEL_ABS_LIMIT_CMS,
                                 PMW_FLOW_VEL_ABS_LIMIT_CMS);
    kf->p = (1.0f - kf->k) * kf->p;
    if(!pmw_flow_is_valid_number(kf->p) || (kf->p < 0.0f))
    {
        kf->p = PMW_FLOW_KALMAN_P_INIT;
    }
}
#endif

static void pmw_flow_sync_kalman_diag(PMWFlowState_t *state)
{
    if(state == NULL)
    {
        return;
    }

    state->kalman_k_x = g_pmw_flow_axis[PMW_FLOW_AXIS_X].kalman_k;
    state->kalman_k_y = g_pmw_flow_axis[PMW_FLOW_AXIS_Y].kalman_k;
    state->kalman_p_x = g_pmw_flow_kalman_x.p;
    state->kalman_p_y = g_pmw_flow_kalman_y.p;
    state->kalman_r_x = g_pmw_flow_kalman_x.r;
    state->kalman_r_y = g_pmw_flow_kalman_y.r;
#if PMW_FLOW_KALMAN_ENABLE
    state->flow_fusion_alpha = (state->kalman_k_x + state->kalman_k_y) * 0.5f;
#endif
}

static float pmw_flow_ramp_down_confidence(float value, float good_value, float bad_value)
{
    float confidence;

    if(!(bad_value > good_value))
    {
        return 1.0f;
    }

    if(value <= good_value)
    {
        return 1.0f;
    }
    if(value >= bad_value)
    {
        return 0.0f;
    }

    confidence = 1.0f - ((value - good_value) / (bad_value - good_value));
    return pmw_flow_limit_float(confidence, 0.0f, 1.0f);
}

static float pmw_flow_calc_axis_confidence(const PMWFlowState_t *state,
                                           uint8 tof_source,
                                           float raw_velocity_cms,
                                           float measured_velocity_cms,
                                           float fused_velocity_cms)
{
    float confidence = 1.0f;
    float tilt_abs_max_deg;
    float residual_cms;
    float dt_error_ratio;
    float speed_ratio;

    if(state == NULL)
    {
        return 0.0f;
    }

    if(!pmw_flow_is_valid_number(raw_velocity_cms) ||
       !pmw_flow_is_valid_number(measured_velocity_cms) ||
       !pmw_flow_is_valid_number(fused_velocity_cms))
    {
        return 0.0f;
    }

    tilt_abs_max_deg = fabsf(state->roll_deg);
    if(fabsf(state->pitch_deg) > tilt_abs_max_deg)
    {
        tilt_abs_max_deg = fabsf(state->pitch_deg);
    }
    confidence *= pmw_flow_ramp_down_confidence(tilt_abs_max_deg,
                                                PMW_FLOW_CONFIDENCE_TILT_GOOD_DEG,
                                                PMW_FLOW_TILT_GATE_DEG);

    residual_cms = fabsf(measured_velocity_cms - fused_velocity_cms);
    confidence *= pmw_flow_ramp_down_confidence(residual_cms,
                                                PMW_FLOW_CONFIDENCE_RESIDUAL_GOOD_CMS,
                                                PMW_FLOW_CONFIDENCE_RESIDUAL_BAD_CMS);

    dt_error_ratio = fabsf(state->dt_s - FLOW_SENSOR_NOMINAL_DT_S) / FLOW_SENSOR_NOMINAL_DT_S;
    confidence *= pmw_flow_ramp_down_confidence(dt_error_ratio,
                                                PMW_FLOW_CONFIDENCE_DT_GOOD_RATIO,
                                                PMW_FLOW_CONFIDENCE_DT_BAD_RATIO);

    speed_ratio = fabsf(raw_velocity_cms) / PMW_FLOW_VEL_ABS_LIMIT_CMS;
    confidence *= pmw_flow_ramp_down_confidence(speed_ratio,
                                                PMW_FLOW_CONFIDENCE_SPEED_GOOD_RATIO,
                                                PMW_FLOW_CONFIDENCE_SPEED_BAD_RATIO);

    if(tof_source == PMW_FLOW_TOF_SOURCE_HOLD)
    {
        confidence *= PMW_FLOW_CONFIDENCE_TOF_HOLD_SCALE;
    }
    else if(tof_source != PMW_FLOW_TOF_SOURCE_LIVE)
    {
        confidence = 0.0f;
    }

    confidence = pmw_flow_limit_float(confidence, 0.0f, 1.0f);
    if((confidence > 0.0f) && (confidence < PMW_FLOW_CONFIDENCE_MIN_VALID_OBSERVATION))
    {
        confidence = PMW_FLOW_CONFIDENCE_MIN_VALID_OBSERVATION;
    }

    return confidence;
}

static float pmw_flow_limit_velocity_slew(float sample_cms, float previous_cms, float dt_s)
{
    float max_step_cms = PMW_FLOW_VEL_SLEW_LIMIT_CMS2 * dt_s;
    float delta_cms;

    if(max_step_cms < PMW_FLOW_VEL_SLEW_MIN_STEP_CMS)
    {
        max_step_cms = PMW_FLOW_VEL_SLEW_MIN_STEP_CMS;
    }

    delta_cms = sample_cms - previous_cms;
    delta_cms = pmw_flow_limit_float(delta_cms, -max_step_cms, max_step_cms);

    return previous_cms + delta_cms;
}

static float pmw_flow_limit_velocity_rate(float sample_cms, float previous_cms, float limit_cms2, float dt_s)
{
    float max_step_cms;
    float delta_cms;

    if(!(dt_s > 0.0f) || !(limit_cms2 > 0.0f))
    {
        return sample_cms;
    }

    max_step_cms = limit_cms2 * dt_s;
    delta_cms = sample_cms - previous_cms;
    delta_cms = pmw_flow_limit_float(delta_cms, -max_step_cms, max_step_cms);

    return previous_cms + delta_cms;
}

static PMWFlowKalman1D_t *pmw_flow_get_kalman(uint8 axis)
{
    return (axis == PMW_FLOW_AXIS_X) ? &g_pmw_flow_kalman_x : &g_pmw_flow_kalman_y;
}

static float *pmw_flow_get_fused_velocity(uint8 axis)
{
    return (axis == PMW_FLOW_AXIS_X) ? &g_pmw_flow_fused_vx_cms : &g_pmw_flow_fused_vy_cms;
}

static void pmw_flow_axis_mark_missing(uint8 axis, uint8 reason_mask)
{
    PMWFlowAxisRuntime_t *runtime = &g_pmw_flow_axis[axis];
    PMWFlowKalman1D_t *kf = pmw_flow_get_kalman(axis);

    if(reason_mask == PMW_FLOW_REJECT_NONE)
    {
        reason_mask = PMW_FLOW_REJECT_FLOW_INVALID;
    }
    reason_mask |= PMW_FLOW_REJECT_REACQUIRING;
    if(!kf->initialized)
    {
        reason_mask |= PMW_FLOW_REJECT_UNINITIALIZED;
    }

    runtime->observation_accepted = 0U;
    runtime->reject_reason_mask = reason_mask;
    runtime->kalman_k = 0.0f;
    runtime->prediction_cms = kf->initialized ? kf->x : 0.0f;
    runtime->innovation_cms = 0.0f;
    runtime->raw_jump_cms = 0.0f;
    if(runtime->state == PMW_FLOW_AXIS_LIVE)
    {
        runtime->state = PMW_FLOW_AXIS_REACQUIRING;
    }
    kf->k = 0.0f;
}

static uint8 pmw_flow_axis_accept_observation(uint8 axis, float raw_velocity_cms, float dt_s)
{
    PMWFlowAxisRuntime_t *runtime = &g_pmw_flow_axis[axis];
    PMWFlowKalman1D_t *kf = pmw_flow_get_kalman(axis);
    float candidate_error;
    uint8 within_innovation;
    uint8 within_jump;
    uint8 reason_mask = PMW_FLOW_REJECT_NONE;

    runtime->gate_cms = PMW_FLOW_INNOVATION_GATE_ACCEL_MULT * PMW_FLOW_GRAVITY_CMS2 * dt_s;
    if(runtime->gate_cms < PMW_FLOW_INNOVATION_GATE_MIN_CMS)
    {
        runtime->gate_cms = PMW_FLOW_INNOVATION_GATE_MIN_CMS;
    }
    runtime->prediction_cms = kf->initialized ? kf->x : 0.0f;
    runtime->innovation_cms = raw_velocity_cms - runtime->prediction_cms;
    runtime->raw_jump_cms = runtime->last_trusted_valid ?
                            raw_velocity_cms - runtime->last_trusted_raw_cms : 0.0f;
    runtime->observation_accepted = 0U;

    within_innovation = (uint8)(kf->initialized &&
                                fabsf(runtime->innovation_cms) <= runtime->gate_cms);
    within_jump = (uint8)(!runtime->last_trusted_valid ||
                          fabsf(runtime->raw_jump_cms) <= runtime->gate_cms);

    if(!kf->initialized)
    {
        reason_mask |= PMW_FLOW_REJECT_UNINITIALIZED;
    }
    else if(!within_innovation)
    {
        reason_mask |= PMW_FLOW_REJECT_INNOVATION;
    }
    if(!within_jump)
    {
        reason_mask |= PMW_FLOW_REJECT_RAW_JUMP;
    }

    if(runtime->state == PMW_FLOW_AXIS_LIVE)
    {
        if((runtime->recovery_blend && within_jump) ||
           (!runtime->recovery_blend && within_innovation && within_jump))
        {
            runtime->observation_accepted = 1U;
        }
    }
    else if(runtime->state == PMW_FLOW_AXIS_REACQUIRING &&
            within_innovation && within_jump && runtime->last_trusted_valid)
    {
        /* A short transient returned to the previous trusted trajectory. */
        runtime->observation_accepted = 1U;
        runtime->recovery_blend = 0U;
    }

    if(!runtime->observation_accepted)
    {
        runtime->state = PMW_FLOW_AXIS_REACQUIRING;
        reason_mask |= PMW_FLOW_REJECT_REACQUIRING;
        kf->k = 0.0f;

        if(runtime->candidate_count == 0U)
        {
            runtime->candidate_cms = raw_velocity_cms;
            runtime->candidate_count = 1U;
        }
        else
        {
            candidate_error = raw_velocity_cms - runtime->candidate_cms;
            if(fabsf(candidate_error) <= PMW_FLOW_REACQUIRE_CONSISTENCY_CMS)
            {
                if(runtime->candidate_count < PMW_FLOW_REACQUIRE_CONFIRM_FRAMES)
                {
                    runtime->candidate_count++;
                }
                runtime->candidate_cms += candidate_error / (float)runtime->candidate_count;
            }
            else
            {
                runtime->candidate_cms = raw_velocity_cms;
                runtime->candidate_count = 1U;
            }
        }

        if(runtime->candidate_count >= PMW_FLOW_REACQUIRE_CONFIRM_FRAMES)
        {
            raw_velocity_cms = runtime->candidate_cms;
            runtime->innovation_cms = raw_velocity_cms - runtime->prediction_cms;
            runtime->raw_jump_cms = runtime->last_trusted_valid ?
                                    raw_velocity_cms - runtime->last_trusted_raw_cms : 0.0f;
            runtime->observation_accepted = 1U;
            runtime->recovery_blend = kf->initialized ? 1U : 0U;
        }

        if(!runtime->observation_accepted && runtime->reject_count < 0xFFFFFFFFUL)
        {
            runtime->reject_count++;
        }
    }

    if(runtime->observation_accepted)
    {
        reason_mask = PMW_FLOW_REJECT_NONE;
        runtime->last_trusted_raw_cms = raw_velocity_cms;
        runtime->last_trusted_valid = 1U;
        runtime->candidate_count = 0U;
        runtime->candidate_cms = raw_velocity_cms;
        runtime->prediction_age_s = 0.0f;
        runtime->state = PMW_FLOW_AXIS_LIVE;
        if(runtime->recovery_blend &&
           fabsf(runtime->innovation_cms) <= runtime->gate_cms)
        {
            runtime->recovery_blend = 0U;
        }
    }

    runtime->reject_reason_mask = reason_mask;

    return runtime->observation_accepted;
}

static float pmw_flow_process_axis_observation(uint8 axis,
                                               float raw_velocity_cms,
                                               float dt_s,
                                               PMWFlowState_t *state)
{
    PMWFlowAxisRuntime_t *runtime = &g_pmw_flow_axis[axis];
    float slew_velocity_cms;
    float lpf_alpha;
    float output_velocity_cms;

    if(runtime->filter_initialized)
    {
        slew_velocity_cms = pmw_flow_limit_velocity_slew(raw_velocity_cms,
                                                         runtime->velocity_filt_cms,
                                                         dt_s);
        lpf_alpha = pmw_flow_calc_lpf_alpha(PMW_FLOW_VEL_LPF_CUTOFF_HZ, dt_s);
        runtime->velocity_filt_cms += lpf_alpha *
                                      (slew_velocity_cms - runtime->velocity_filt_cms);
    }
    else
    {
        slew_velocity_cms = raw_velocity_cms;
        runtime->velocity_filt_cms = raw_velocity_cms;
        runtime->filter_initialized = 1U;
    }

    if(runtime->output_initialized)
    {
        runtime->velocity_output_cms = pmw_flow_limit_velocity_rate(runtime->velocity_filt_cms,
                                                                    runtime->velocity_output_cms,
                                                                    PMW_FLOW_VEL_OUTPUT_SLEW_LIMIT_CMS2,
                                                                    dt_s);
    }
    else
    {
        runtime->velocity_output_cms = runtime->velocity_filt_cms;
        runtime->output_initialized = 1U;
    }
    output_velocity_cms = pmw_flow_apply_velocity_deadband(runtime->velocity_output_cms);

    if(axis == PMW_FLOW_AXIS_X)
    {
        state->velocity_x_slew_cms = slew_velocity_cms;
        state->velocity_x_lpf_cms = runtime->velocity_filt_cms;
        state->velocity_x_output_slew_cms = runtime->velocity_output_cms;
        state->velocity_x_cms = output_velocity_cms;
    }
    else
    {
        state->velocity_y_slew_cms = slew_velocity_cms;
        state->velocity_y_lpf_cms = runtime->velocity_filt_cms;
        state->velocity_y_output_slew_cms = runtime->velocity_output_cms;
        state->velocity_y_cms = output_velocity_cms;
    }

    return output_velocity_cms;
}

static void pmw_flow_publish_frozen_axis(uint8 axis, PMWFlowState_t *state)
{
    PMWFlowAxisRuntime_t *runtime = &g_pmw_flow_axis[axis];
    float value = runtime->filter_initialized ? runtime->velocity_filt_cms : 0.0f;
    float output = runtime->output_initialized ? runtime->velocity_output_cms : value;

    if(axis == PMW_FLOW_AXIS_X)
    {
        state->velocity_x_slew_cms = value;
        state->velocity_x_lpf_cms = value;
        state->velocity_x_output_slew_cms = output;
        state->velocity_x_cms = pmw_flow_apply_velocity_deadband(output);
    }
    else
    {
        state->velocity_y_slew_cms = value;
        state->velocity_y_lpf_cms = value;
        state->velocity_y_output_slew_cms = output;
        state->velocity_y_cms = pmw_flow_apply_velocity_deadband(output);
    }
}

static void pmw_flow_update_axis_feedback(PMWFlowState_t *state,
                                          uint8 axis,
                                          uint8 observation_accepted,
                                          uint8 tof_source,
                                          float raw_velocity_cms,
                                          float measured_velocity_cms)
{
    PMWFlowAxisRuntime_t *runtime = &g_pmw_flow_axis[axis];
    PMWFlowKalman1D_t *kf = pmw_flow_get_kalman(axis);
    float *fused_velocity = pmw_flow_get_fused_velocity(axis);
    float confidence = 0.0f;
    uint8 fused_valid;
    uint8 acc_used;

    if(observation_accepted)
    {
        confidence = pmw_flow_calc_axis_confidence(state,
                                                   tof_source,
                                                   raw_velocity_cms,
                                                   measured_velocity_cms,
                                                   kf->initialized ? kf->x : measured_velocity_cms);
#if PMW_FLOW_KALMAN_ENABLE
        if(!kf->initialized)
        {
            pmw_flow_kalman_init_axis(kf, measured_velocity_cms);
        }
        else if(confidence >= PMW_FLOW_CONFIDENCE_MIN_UPDATE)
        {
            pmw_flow_kalman_update_axis(kf,
                                        measured_velocity_cms,
                                        confidence,
                                        runtime->gate_cms);
        }
        else
        {
            kf->k = 0.0f;
        }
        runtime->kalman_k = kf->k;
        *fused_velocity = kf->x;
#else
        if(!kf->initialized)
        {
            pmw_flow_kalman_init_axis(kf, measured_velocity_cms);
            *fused_velocity = measured_velocity_cms;
        }
        else
        {
            *fused_velocity += PMW_FLOW_FUSION_ALPHA_DEFAULT *
                               (measured_velocity_cms - *fused_velocity);
        }
#endif
        fused_valid = 1U;
        acc_used = kf->initialized;
    }
    else
    {
        kf->k = 0.0f;
        runtime->kalman_k = 0.0f;
        fused_valid = (uint8)(kf->initialized &&
                              runtime->prediction_age_s <= PMW_FLOW_PREDICT_HOLD_S);
        acc_used = fused_valid;
    }

    *fused_velocity = pmw_flow_limit_float(*fused_velocity,
                                           -PMW_FLOW_VEL_ABS_LIMIT_CMS,
                                           PMW_FLOW_VEL_ABS_LIMIT_CMS);
    kf->x = *fused_velocity;

    if(axis == PMW_FLOW_AXIS_X)
    {
        state->velocity_x_fused_cms = *fused_velocity;
        state->velocity_x_fused_residual_cms = measured_velocity_cms - *fused_velocity;
        state->flow_x_used = observation_accepted;
        state->fused_velocity_x_valid = fused_valid;
        state->flow_confidence = confidence;
        if(acc_used) state->acc_used = 1U;
    }
    else
    {
        state->velocity_y_fused_cms = *fused_velocity;
        state->velocity_y_fused_residual_cms = measured_velocity_cms - *fused_velocity;
        state->flow_y_used = observation_accepted;
        state->fused_velocity_y_valid = fused_valid;
        if(state->flow_confidence == 0.0f ||
           (confidence > 0.0f && confidence < state->flow_confidence))
        {
            state->flow_confidence = confidence;
        }
        if(acc_used) state->acc_used = 1U;
    }
}

static void pmw_flow_sync_axis_diag(PMWFlowState_t *state)
{
    PMWFlowAxisRuntime_t *x = &g_pmw_flow_axis[PMW_FLOW_AXIS_X];
    PMWFlowAxisRuntime_t *y = &g_pmw_flow_axis[PMW_FLOW_AXIS_Y];

    state->flow_x_accepted = x->observation_accepted;
    state->flow_y_accepted = y->observation_accepted;
    state->flow_reject_reason_x = x->reject_reason_mask;
    state->flow_reject_reason_y = y->reject_reason_mask;
    state->flow_x_state = x->state;
    state->flow_y_state = y->state;
    state->flow_prediction_x_cms = x->prediction_cms;
    state->flow_prediction_y_cms = y->prediction_cms;
    state->flow_innovation_x_cms = x->innovation_cms;
    state->flow_innovation_y_cms = y->innovation_cms;
    state->flow_gate_x_cms = x->gate_cms;
    state->flow_gate_y_cms = y->gate_cms;
    state->flow_raw_jump_x_cms = x->raw_jump_cms;
    state->flow_raw_jump_y_cms = y->raw_jump_cms;
    state->flow_prediction_age_x_ms = x->prediction_age_s * 1000.0f;
    state->flow_prediction_age_y_ms = y->prediction_age_s * 1000.0f;
    state->flow_reacquire_count_x = x->candidate_count;
    state->flow_reacquire_count_y = y->candidate_count;
    state->flow_reject_count_x = x->reject_count;
    state->flow_reject_count_y = y->reject_count;
}

static void pmw_flow_update_world_odometry(PMWFlowState_t *state)
{
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float tilt_abs_max_deg;
    float cos_pitch;
    float cos_roll;
    float delta_body_x_cm;
    float delta_body_y_cm;
    uint8 direct_world_valid = 0U;

    if(state == NULL)
    {
        return;
    }

    state->yaw_deg = current_yaw;

    tilt_abs_max_deg = fabsf(state->roll_deg);
    if(fabsf(state->pitch_deg) > tilt_abs_max_deg)
    {
        tilt_abs_max_deg = fabsf(state->pitch_deg);
    }

    if(state->valid &&
       state->flow_used &&
             pmw_flow_is_valid_number(state->velocity_x_cms) &&
             pmw_flow_is_valid_number(state->velocity_y_cms) &&
             pmw_flow_is_valid_number(current_yaw) &&
       tilt_abs_max_deg <= PMW_FLOW_WORLD_TILT_GATE_DEG)
    {
        yaw_rad = current_yaw * PMW_FLOW_DEG2RAD;
        cos_yaw = cosf(yaw_rad);
        sin_yaw = sinf(yaw_rad);

        g_pmw_flow_world_vx_cms = cos_yaw * state->velocity_x_cms -
                                  sin_yaw * state->velocity_y_cms;
        g_pmw_flow_world_vy_cms = sin_yaw * state->velocity_x_cms +
                                  cos_yaw * state->velocity_y_cms;

        g_pmw_flow_world_x_cm += g_pmw_flow_world_vx_cms * state->dt_s;
        g_pmw_flow_world_y_cm += g_pmw_flow_world_vy_cms * state->dt_s;

        cos_pitch = cosf(state->pitch_deg * PMW_FLOW_DEG2RAD);
        cos_roll = cosf(state->roll_deg * PMW_FLOW_DEG2RAD);
        if(cos_pitch < PMW_FLOW_TILT_COS_FLOOR)
        {
            cos_pitch = PMW_FLOW_TILT_COS_FLOOR;
        }
        if(cos_roll < PMW_FLOW_TILT_COS_FLOOR)
        {
            cos_roll = PMW_FLOW_TILT_COS_FLOOR;
        }

        delta_body_x_cm = PMW_FLOW_BODY_VX_FROM_FLOW_X_SIGN *
                          (state->range_cm * state->flow_comp_x_rad / cos_pitch);
        delta_body_y_cm = PMW_FLOW_BODY_VY_FROM_FLOW_Y_SIGN *
                          (state->range_cm * state->flow_comp_y_rad / cos_roll);

        g_pmw_flow_delta_world_x_cm = cos_yaw * delta_body_x_cm -
                                      sin_yaw * delta_body_y_cm;
        g_pmw_flow_delta_world_y_cm = sin_yaw * delta_body_x_cm +
                                      cos_yaw * delta_body_y_cm;
        g_pmw_flow_direct_world_x_cm += g_pmw_flow_delta_world_x_cm;
        g_pmw_flow_direct_world_y_cm += g_pmw_flow_delta_world_y_cm;
    }
    else
    {
        g_pmw_flow_world_vx_cms = 0.0f;
        g_pmw_flow_world_vy_cms = 0.0f;
        g_pmw_flow_delta_world_x_cm = 0.0f;
        g_pmw_flow_delta_world_y_cm = 0.0f;
    }

    if(state->valid &&
       state->flow_used &&
       pmw_flow_is_valid_number(state->range_cm) &&
       pmw_flow_is_valid_number(state->flow_comp_x_rad) &&
       pmw_flow_is_valid_number(state->flow_comp_y_rad) &&
       pmw_flow_is_valid_number(state->yaw_deg))
    {
        yaw_rad = state->yaw_deg * PMW_FLOW_DEG2RAD;
        cos_yaw = cosf(yaw_rad);
        sin_yaw = sinf(yaw_rad);

        cos_pitch = cosf(state->pitch_deg * PMW_FLOW_DEG2RAD);
        cos_roll = cosf(state->roll_deg * PMW_FLOW_DEG2RAD);
        if(cos_pitch < PMW_FLOW_TILT_COS_FLOOR)
        {
            cos_pitch = PMW_FLOW_TILT_COS_FLOOR;
        }
        if(cos_roll < PMW_FLOW_TILT_COS_FLOOR)
        {
            cos_roll = PMW_FLOW_TILT_COS_FLOOR;
        }

        delta_body_x_cm = PMW_FLOW_BODY_VX_FROM_FLOW_X_SIGN *
                          (state->range_cm * state->flow_comp_x_rad / cos_pitch);
        delta_body_y_cm = PMW_FLOW_BODY_VY_FROM_FLOW_Y_SIGN *
                          (state->range_cm * state->flow_comp_y_rad / cos_roll);

        g_pmw_flow_direct_world_x_cm += cos_yaw * delta_body_x_cm -
                                        sin_yaw * delta_body_y_cm;
        g_pmw_flow_direct_world_y_cm += sin_yaw * delta_body_x_cm +
                                        cos_yaw * delta_body_y_cm;
        direct_world_valid = 1U;
    }
    else
    {
        direct_world_valid = 0U;
    }

    state->velocity_x_world_cms = g_pmw_flow_world_vx_cms;
    state->velocity_y_world_cms = g_pmw_flow_world_vy_cms;
    state->position_x_world_cm = g_pmw_flow_world_x_cm;
    state->position_y_world_cm = g_pmw_flow_world_y_cm;
    state->flow_delta_x_world_cm = g_pmw_flow_delta_world_x_cm;
    state->flow_delta_y_world_cm = g_pmw_flow_delta_world_y_cm;
    state->flow_position_x_world_cm = g_pmw_flow_direct_world_x_cm;
    state->flow_position_y_world_cm = g_pmw_flow_direct_world_y_cm;
    state->flow_position_valid = direct_world_valid;
}

static void pmw_flow_publish_state(const PMWFlowState_t *state)
{
    uint8 next_index;

    if(state == NULL)
    {
        return;
    }

    next_index = (uint8)(g_pmw_flow_publish_index ^ 1U);
    g_pmw_flow_publish_slots[next_index] = *state;
    __DMB();
    g_pmw_flow_publish_index = next_index;
    __DMB();
    g_pmw_flow_publish_seq++;
}

void PMW_Flow_Init(void)
{
    PMW_Flow_Reset();
}

void PMW_Flow_Reset(void)
{
    PMWFlowState_t cleared_state = {0};
    PMWFlowAxisRuntime_t cleared_axis = {0};

    g_pmw_flow_gyro_flow_x_delta_rad = 0.0f;
    g_pmw_flow_gyro_flow_y_delta_rad = 0.0f;
    g_pmw_flow_publish_slots[0] = cleared_state;
    g_pmw_flow_publish_slots[1] = cleared_state;
    g_pmw_flow_publish_index = 0U;
    g_pmw_flow_publish_seq = 0U;
    g_pmw_flow_mainloop_state = cleared_state;
    g_pmw_flow_mainloop_seq = 0U;
    g_pmw_flow_range_filt_cm = 0.0f;
    g_pmw_flow_range_filt_init = 0U;
    g_pmw_flow_axis[PMW_FLOW_AXIS_X] = cleared_axis;
    g_pmw_flow_axis[PMW_FLOW_AXIS_Y] = cleared_axis;
    g_pmw_flow_axis[PMW_FLOW_AXIS_X].state = PMW_FLOW_AXIS_INVALID;
    g_pmw_flow_axis[PMW_FLOW_AXIS_Y].state = PMW_FLOW_AXIS_INVALID;
    g_pmw_flow_fused_vx_cms = 0.0f;
    g_pmw_flow_fused_vy_cms = 0.0f;
    g_pmw_flow_fusion_imu_vx_last_cms = 0.0f;
    g_pmw_flow_fusion_imu_vy_last_cms = 0.0f;
    g_pmw_flow_fusion_imu_last_initialized = 0U;
    pmw_flow_kalman_reset();
    PMW_Flow_ResetWorldPosition();
}

void PMW_Flow_AccumulateGyro(float gyro_x_rate_dps, float gyro_y_rate_dps, float dt_s)
{
    if(!(dt_s > 0.0f) ||
       !pmw_flow_is_valid_number(gyro_x_rate_dps) ||
       !pmw_flow_is_valid_number(gyro_y_rate_dps))
    {
        return;
    }

    g_pmw_flow_gyro_flow_x_delta_rad += gyro_x_rate_dps * PMW_FLOW_DEG2RAD * dt_s * PMW_FLOW_GYRO_X_DELTA_SIGN;
    g_pmw_flow_gyro_flow_y_delta_rad += gyro_y_rate_dps * PMW_FLOW_DEG2RAD * dt_s * PMW_FLOW_GYRO_Y_DELTA_SIGN;
}

void PMW_Flow_PredictVelocity(float imu_velocity_x_cms, float imu_velocity_y_cms, float dt_s)
{
    float imu_vx_cms;
    float imu_vy_cms;
    float imu_delta_x_cms;
    float imu_delta_y_cms;

    if(!(dt_s > 0.0f) ||
       !pmw_flow_is_valid_number(imu_velocity_x_cms) ||
       !pmw_flow_is_valid_number(imu_velocity_y_cms))
    {
        return;
    }

    imu_vx_cms = imu_velocity_x_cms * PMW_FLOW_FUSION_IMU_X_SIGN;
    imu_vy_cms = imu_velocity_y_cms * PMW_FLOW_FUSION_IMU_Y_SIGN;

    if(!g_pmw_flow_fusion_imu_last_initialized)
    {
        g_pmw_flow_fusion_imu_vx_last_cms = imu_vx_cms;
        g_pmw_flow_fusion_imu_vy_last_cms = imu_vy_cms;
        g_pmw_flow_fusion_imu_last_initialized = 1U;
        return;
    }

    imu_delta_x_cms = pmw_flow_limit_velocity_rate(imu_vx_cms - g_pmw_flow_fusion_imu_vx_last_cms,
                                                   0.0f,
                                                   PMW_FLOW_FUSION_IMU_DELTA_LIMIT_CMS2,
                                                   dt_s);
    imu_delta_y_cms = pmw_flow_limit_velocity_rate(imu_vy_cms - g_pmw_flow_fusion_imu_vy_last_cms,
                                                   0.0f,
                                                   PMW_FLOW_FUSION_IMU_DELTA_LIMIT_CMS2,
                                                   dt_s);

#if PMW_FLOW_KALMAN_ENABLE
    pmw_flow_kalman_predict_axis(&g_pmw_flow_kalman_x, imu_delta_x_cms, dt_s);
    pmw_flow_kalman_predict_axis(&g_pmw_flow_kalman_y, imu_delta_y_cms, dt_s);
    g_pmw_flow_fused_vx_cms = g_pmw_flow_kalman_x.x;
    g_pmw_flow_fused_vy_cms = g_pmw_flow_kalman_y.x;
#else
    g_pmw_flow_fused_vx_cms += imu_delta_x_cms;
    g_pmw_flow_fused_vy_cms += imu_delta_y_cms;
#endif
    if(g_pmw_flow_kalman_x.initialized)
    {
        g_pmw_flow_axis[PMW_FLOW_AXIS_X].prediction_age_s += dt_s;
        g_pmw_flow_axis[PMW_FLOW_AXIS_X].prediction_cms = g_pmw_flow_kalman_x.x;
        if(g_pmw_flow_axis[PMW_FLOW_AXIS_X].prediction_age_s > PMW_FLOW_PREDICT_HOLD_S)
        {
            g_pmw_flow_axis[PMW_FLOW_AXIS_X].state = PMW_FLOW_AXIS_REACQUIRING;
        }
    }
    if(g_pmw_flow_kalman_y.initialized)
    {
        g_pmw_flow_axis[PMW_FLOW_AXIS_Y].prediction_age_s += dt_s;
        g_pmw_flow_axis[PMW_FLOW_AXIS_Y].prediction_cms = g_pmw_flow_kalman_y.x;
        if(g_pmw_flow_axis[PMW_FLOW_AXIS_Y].prediction_age_s > PMW_FLOW_PREDICT_HOLD_S)
        {
            g_pmw_flow_axis[PMW_FLOW_AXIS_Y].state = PMW_FLOW_AXIS_REACQUIRING;
        }
    }
    g_pmw_flow_fusion_imu_vx_last_cms = imu_vx_cms;
    g_pmw_flow_fusion_imu_vy_last_cms = imu_vy_cms;

    g_pmw_flow_fused_vx_cms = pmw_flow_limit_float(g_pmw_flow_fused_vx_cms,
                                                   -PMW_FLOW_VEL_ABS_LIMIT_CMS,
                                                   PMW_FLOW_VEL_ABS_LIMIT_CMS);
    g_pmw_flow_fused_vy_cms = pmw_flow_limit_float(g_pmw_flow_fused_vy_cms,
                                                   -PMW_FLOW_VEL_ABS_LIMIT_CMS,
                                                   PMW_FLOW_VEL_ABS_LIMIT_CMS);
#if PMW_FLOW_KALMAN_ENABLE
    g_pmw_flow_kalman_x.x = g_pmw_flow_fused_vx_cms;
    g_pmw_flow_kalman_y.x = g_pmw_flow_fused_vy_cms;
#endif
}

void PMW_Flow_ResetWorldPosition(void)
{
    g_pmw_flow_world_vx_cms = 0.0f;
    g_pmw_flow_world_vy_cms = 0.0f;
    g_pmw_flow_world_x_cm = 0.0f;
    g_pmw_flow_world_y_cm = 0.0f;
    g_pmw_flow_delta_world_x_cm = 0.0f;
    g_pmw_flow_delta_world_y_cm = 0.0f;
    g_pmw_flow_direct_world_x_cm = 0.0f;
    g_pmw_flow_direct_world_y_cm = 0.0f;
}

void PMW_Flow_UpdateSample(int16 flow_delta_x,
                           int16 flow_delta_y,
                           float roll_deg,
                           float pitch_deg,
                           float tof_height_cm,
                           uint8 tof_source,
                           uint8 flow_sample_valid,
                           float dt_s,
                           float imu_velocity_x_cms,
                           float imu_velocity_y_cms)
{
    PMWFlowState_t state = {0};
    float range_used_cm = g_pmw_flow_range_filt_init ? g_pmw_flow_range_filt_cm : tof_height_cm;
    float vx_raw = 0.0f;
    float vy_raw = 0.0f;
    float vx_measured;
    float vy_measured;
    float tilt_abs_max_deg;
    uint8 common_valid;
    uint8 x_sample_valid;
    uint8 y_sample_valid;
    uint8 x_accepted;
    uint8 y_accepted;
    uint8 base_reject_reason = PMW_FLOW_REJECT_NONE;
    uint8 x_reject_reason;
    uint8 y_reject_reason;
    int16 flow_input_x = flow_delta_x;
    int16 flow_input_y = flow_delta_y;

    state.dt_s = dt_s;
    state.tof_los_cm = tof_height_cm;
    state.roll_deg = roll_deg;
    state.pitch_deg = pitch_deg;
    state.gyro_flow_x_delta_rad = g_pmw_flow_gyro_flow_x_delta_rad;
    state.gyro_flow_y_delta_rad = g_pmw_flow_gyro_flow_y_delta_rad;
    g_pmw_flow_gyro_flow_x_delta_rad = 0.0f;
    g_pmw_flow_gyro_flow_y_delta_rad = 0.0f;

#if FLOW_SENSOR_SWAP_XY
    flow_input_x = flow_delta_y;
    flow_input_y = flow_delta_x;
#endif
    state.flow_delta_x_count = flow_input_x;
    state.flow_delta_y_count = flow_input_y;
    state.flow_x_rad = (float)flow_input_x * PMW_FLOW_RAD_PER_COUNT_X * PMW_FLOW_SENSOR_X_SIGN;
    state.flow_y_rad = (float)flow_input_y * PMW_FLOW_RAD_PER_COUNT_Y * PMW_FLOW_SENSOR_Y_SIGN;

    common_valid = (uint8)(flow_sample_valid &&
                           (dt_s >= PMW_FLOW_MIN_DT_S) &&
                           (dt_s <= PMW_FLOW_MAX_DT_S) &&
                           pmw_flow_is_valid_number(tof_height_cm) &&
                           pmw_flow_is_valid_number(roll_deg) &&
                           pmw_flow_is_valid_number(pitch_deg) &&
                           pmw_flow_is_valid_number(state.gyro_flow_x_delta_rad) &&
                           pmw_flow_is_valid_number(state.gyro_flow_y_delta_rad) &&
                           (tof_height_cm > 0.0f));

    if(common_valid && tof_source == PMW_FLOW_TOF_SOURCE_LIVE)
    {
        if(!g_pmw_flow_range_filt_init)
        {
            g_pmw_flow_range_filt_cm = tof_height_cm;
            g_pmw_flow_range_filt_init = 1U;
        }
        else
        {
            g_pmw_flow_range_filt_cm += PMW_FLOW_RANGE_LPF_ALPHA *
                                         (tof_height_cm - g_pmw_flow_range_filt_cm);
        }
        range_used_cm = g_pmw_flow_range_filt_cm;
    }
    state.range_cm = range_used_cm;

    state.flow_comp_x_rad = state.flow_x_rad +
                            g_pmw_flow_gyro_comp_gain_x *
                            (PMW_FLOW_COMP_X_FROM_GYRO_X_SIGN * state.gyro_flow_x_delta_rad +
                             PMW_FLOW_COMP_X_FROM_GYRO_Y_SIGN * state.gyro_flow_y_delta_rad);
    state.flow_comp_y_rad = state.flow_y_rad +
                            g_pmw_flow_gyro_comp_gain_y *
                            (PMW_FLOW_COMP_Y_FROM_GYRO_X_SIGN * state.gyro_flow_x_delta_rad +
                             PMW_FLOW_COMP_Y_FROM_GYRO_Y_SIGN * state.gyro_flow_y_delta_rad);

    if(common_valid)
    {
        float cos_pitch = cosf(pitch_deg * PMW_FLOW_DEG2RAD);
        float cos_roll = cosf(roll_deg * PMW_FLOW_DEG2RAD);

        if(cos_pitch < PMW_FLOW_TILT_COS_FLOOR) cos_pitch = PMW_FLOW_TILT_COS_FLOOR;
        if(cos_roll < PMW_FLOW_TILT_COS_FLOOR) cos_roll = PMW_FLOW_TILT_COS_FLOOR;
        vx_raw = PMW_FLOW_BODY_VX_FROM_FLOW_X_SIGN *
                 (range_used_cm * state.flow_comp_x_rad / dt_s) / cos_pitch;
        vy_raw = PMW_FLOW_BODY_VY_FROM_FLOW_Y_SIGN *
                 (range_used_cm * state.flow_comp_y_rad / dt_s) / cos_roll;
    }
    state.velocity_x_raw_cms = vx_raw;
    state.velocity_y_raw_cms = vy_raw;

    tilt_abs_max_deg = fabsf(roll_deg);
    if(fabsf(pitch_deg) > tilt_abs_max_deg) tilt_abs_max_deg = fabsf(pitch_deg);

    if(!flow_sample_valid ||
       !(dt_s >= PMW_FLOW_MIN_DT_S && dt_s <= PMW_FLOW_MAX_DT_S) ||
       !pmw_flow_is_valid_number(roll_deg) ||
       !pmw_flow_is_valid_number(pitch_deg) ||
       !pmw_flow_is_valid_number(state.gyro_flow_x_delta_rad) ||
       !pmw_flow_is_valid_number(state.gyro_flow_y_delta_rad))
    {
        base_reject_reason |= PMW_FLOW_REJECT_FLOW_INVALID;
    }
    if(flow_sample_valid &&
       (tof_source != PMW_FLOW_TOF_SOURCE_LIVE ||
        !pmw_flow_is_valid_number(tof_height_cm) ||
        !(tof_height_cm > 0.0f)))
    {
        base_reject_reason |= PMW_FLOW_REJECT_TOF_NOT_LIVE;
    }
    if(pmw_flow_is_valid_number(tilt_abs_max_deg) &&
       tilt_abs_max_deg > PMW_FLOW_TILT_GATE_DEG)
    {
        base_reject_reason |= PMW_FLOW_REJECT_TILT;
    }

    x_reject_reason = base_reject_reason;
    y_reject_reason = base_reject_reason;
    if(!pmw_flow_is_valid_delta_count(flow_input_x) ||
       (common_valid &&
        (!pmw_flow_is_valid_number(vx_raw) || fabsf(vx_raw) > PMW_FLOW_VEL_ABS_LIMIT_CMS)))
    {
        x_reject_reason |= PMW_FLOW_REJECT_RAW_LIMIT;
    }
    if(!pmw_flow_is_valid_delta_count(flow_input_y) ||
       (common_valid &&
        (!pmw_flow_is_valid_number(vy_raw) || fabsf(vy_raw) > PMW_FLOW_VEL_ABS_LIMIT_CMS)))
    {
        y_reject_reason |= PMW_FLOW_REJECT_RAW_LIMIT;
    }

    x_sample_valid = (uint8)(common_valid &&
                             tof_source == PMW_FLOW_TOF_SOURCE_LIVE &&
                             tilt_abs_max_deg <= PMW_FLOW_TILT_GATE_DEG &&
                             pmw_flow_is_valid_delta_count(flow_input_x) &&
                             pmw_flow_is_valid_number(vx_raw) &&
                             fabsf(vx_raw) <= PMW_FLOW_VEL_ABS_LIMIT_CMS);
    y_sample_valid = (uint8)(common_valid &&
                             tof_source == PMW_FLOW_TOF_SOURCE_LIVE &&
                             tilt_abs_max_deg <= PMW_FLOW_TILT_GATE_DEG &&
                             pmw_flow_is_valid_delta_count(flow_input_y) &&
                             pmw_flow_is_valid_number(vy_raw) &&
                             fabsf(vy_raw) <= PMW_FLOW_VEL_ABS_LIMIT_CMS);

    if(x_sample_valid)
    {
        x_accepted = pmw_flow_axis_accept_observation(PMW_FLOW_AXIS_X, vx_raw, dt_s);
    }
    else
    {
        pmw_flow_axis_mark_missing(PMW_FLOW_AXIS_X, x_reject_reason);
        x_accepted = 0U;
    }
    if(y_sample_valid)
    {
        y_accepted = pmw_flow_axis_accept_observation(PMW_FLOW_AXIS_Y, vy_raw, dt_s);
    }
    else
    {
        pmw_flow_axis_mark_missing(PMW_FLOW_AXIS_Y, y_reject_reason);
        y_accepted = 0U;
    }

    if(x_accepted)
    {
        vx_measured = pmw_flow_process_axis_observation(PMW_FLOW_AXIS_X, vx_raw, dt_s, &state);
    }
    else
    {
        pmw_flow_publish_frozen_axis(PMW_FLOW_AXIS_X, &state);
        vx_measured = state.velocity_x_cms;
    }
    if(y_accepted)
    {
        vy_measured = pmw_flow_process_axis_observation(PMW_FLOW_AXIS_Y, vy_raw, dt_s, &state);
    }
    else
    {
        pmw_flow_publish_frozen_axis(PMW_FLOW_AXIS_Y, &state);
        vy_measured = state.velocity_y_cms;
    }

    pmw_flow_update_axis_feedback(&state,
                                  PMW_FLOW_AXIS_X,
                                  x_accepted,
                                  tof_source,
                                  vx_raw,
                                  vx_measured);
    pmw_flow_update_axis_feedback(&state,
                                  PMW_FLOW_AXIS_Y,
                                  y_accepted,
                                  tof_source,
                                  vy_raw,
                                  vy_measured);

    state.valid = (uint8)(x_accepted && y_accepted);
    state.flow_used = (uint8)(state.flow_x_used && state.flow_y_used);
    state.fused_velocity_valid = (uint8)(state.fused_velocity_x_valid &&
                                         state.fused_velocity_y_valid);
    state.speed_cms = sqrtf(state.velocity_x_cms * state.velocity_x_cms +
                            state.velocity_y_cms * state.velocity_y_cms);
    state.flow_residual_norm_cms = sqrtf(state.velocity_x_fused_residual_cms *
                                          state.velocity_x_fused_residual_cms +
                                          state.velocity_y_fused_residual_cms *
                                          state.velocity_y_fused_residual_cms);
    pmw_flow_sync_axis_diag(&state);
    pmw_flow_sync_kalman_diag(&state);
    pmw_flow_update_world_odometry(&state);
    pmw_flow_publish_state(&state);

    (void)imu_velocity_x_cms;
    (void)imu_velocity_y_cms;
}

void PMW_Flow_SyncMainLoopBuffer(void)
{
    uint32 seq_before;
    uint32 seq_after;
    uint8 published_index;

    if(g_pmw_flow_mainloop_seq == g_pmw_flow_publish_seq)
    {
        return;
    }

    do
    {
        seq_before = g_pmw_flow_publish_seq;
        __DMB();
        published_index = g_pmw_flow_publish_index;
        g_pmw_flow_mainloop_state = g_pmw_flow_publish_slots[published_index];
        __DMB();
        seq_after = g_pmw_flow_publish_seq;
    } while((seq_before != seq_after) || (published_index != g_pmw_flow_publish_index));

    g_pmw_flow_mainloop_seq = seq_after;
}

void PMW_Flow_GetState(PMWFlowState_t *state)
{
    if(state == NULL)
    {
        return;
    }

    PMW_Flow_SyncMainLoopBuffer();
    *state = g_pmw_flow_mainloop_state;
    state->velocity_x_fused_cms = g_pmw_flow_fused_vx_cms;
    state->velocity_y_fused_cms = g_pmw_flow_fused_vy_cms;
    state->fused_velocity_x_valid = (uint8)(g_pmw_flow_kalman_x.initialized &&
                                            g_pmw_flow_axis[PMW_FLOW_AXIS_X].prediction_age_s <=
                                            PMW_FLOW_PREDICT_HOLD_S);
    state->fused_velocity_y_valid = (uint8)(g_pmw_flow_kalman_y.initialized &&
                                            g_pmw_flow_axis[PMW_FLOW_AXIS_Y].prediction_age_s <=
                                            PMW_FLOW_PREDICT_HOLD_S);
    state->fused_velocity_valid = (uint8)(state->fused_velocity_x_valid &&
                                          state->fused_velocity_y_valid);
    state->acc_used = (uint8)(state->fused_velocity_x_valid ||
                              state->fused_velocity_y_valid);
    if(!state->fused_velocity_x_valid) state->flow_x_used = 0U;
    if(!state->fused_velocity_y_valid) state->flow_y_used = 0U;
    state->flow_used = (uint8)(state->flow_x_used && state->flow_y_used);
    state->velocity_x_fused_residual_cms = state->velocity_x_cms - state->velocity_x_fused_cms;
    state->velocity_y_fused_residual_cms = state->velocity_y_cms - state->velocity_y_fused_cms;
    state->flow_residual_norm_cms = sqrtf(state->velocity_x_fused_residual_cms *
                                          state->velocity_x_fused_residual_cms +
                                          state->velocity_y_fused_residual_cms *
                                          state->velocity_y_fused_residual_cms);
    pmw_flow_sync_axis_diag(state);
    pmw_flow_sync_kalman_diag(state);
}

/*
 * 设置陀螺去旋转增益。kx / ky < 0 表示该轴不变更。
 * 范围被夹到 [PMW_FLOW_GYRO_COMP_GAIN_MIN, PMW_FLOW_GYRO_COMP_GAIN_MAX]，避免误调到极端值。
 * 调参建议：在 VOFA 里观察 flow_comp_x / flow_comp_y，让旋转主导段它们尽量回到 0 附近。
 */
void PMW_Flow_SetGyroCompGain(float kx, float ky)
{
    if(kx >= 0.0f)
    {
        if(kx < PMW_FLOW_GYRO_COMP_GAIN_MIN) kx = PMW_FLOW_GYRO_COMP_GAIN_MIN;
        if(kx > PMW_FLOW_GYRO_COMP_GAIN_MAX) kx = PMW_FLOW_GYRO_COMP_GAIN_MAX;
        g_pmw_flow_gyro_comp_gain_x = kx;
    }
    if(ky >= 0.0f)
    {
        if(ky < PMW_FLOW_GYRO_COMP_GAIN_MIN) ky = PMW_FLOW_GYRO_COMP_GAIN_MIN;
        if(ky > PMW_FLOW_GYRO_COMP_GAIN_MAX) ky = PMW_FLOW_GYRO_COMP_GAIN_MAX;
        g_pmw_flow_gyro_comp_gain_y = ky;
    }
}

void PMW_Flow_GetGyroCompGain(float *kx, float *ky)
{
    if(kx != NULL) *kx = g_pmw_flow_gyro_comp_gain_x;
    if(ky != NULL) *ky = g_pmw_flow_gyro_comp_gain_y;
}
