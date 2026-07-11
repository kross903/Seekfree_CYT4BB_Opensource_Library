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
#define PMW_FLOW_MIN_HEIGHT_CM              (5.0f)
#define PMW_FLOW_MAX_HEIGHT_CM              (400.0f)
#define PMW_FLOW_MIN_DT_S                   (FLOW_SENSOR_NOMINAL_DT_S * 0.5f)
#define PMW_FLOW_MAX_DT_S                   (FLOW_SENSOR_NOMINAL_DT_S * 2.0f)
#define PMW_FLOW_MAX_ABS_DELTA_COUNT        (5000)
#define PMW_FLOW_TWO_PI                     (6.28318530717958647692f)

/*
 * 光流速度鲁棒化参数：坏数据先门控，速度变化只做重力加速度量级的斜率限制。
 * UP-FLOW-302 当前约 40Hz / dt=25ms，VEL_LPF_CUTOFF_HZ=8Hz 时 alpha 约 0.72。
 */
#define PMW_FLOW_RANGE_LPF_ALPHA            (0.40f)
#define PMW_FLOW_TOF_JUMP_REJECT_CM         (15.0f)
#define PMW_FLOW_GRAVITY_CMS2               (980.665f)
#define PMW_FLOW_VEL_LPF_CUTOFF_HZ          (12.0f)
#define PMW_FLOW_VEL_DEADBAND_CMS           (5.0f)
#define PMW_FLOW_TILT_GATE_DEG              (25.0f)
#define PMW_FLOW_VEL_ABS_LIMIT_CMS          (300.0f)
#define PMW_FLOW_VEL_SLEW_LIMIT_CMS2        PMW_FLOW_GRAVITY_CMS2
#define PMW_FLOW_VEL_SLEW_MIN_STEP_CMS      (12.0f)
#define PMW_FLOW_VEL_OUTPUT_SLEW_LIMIT_CMS2 PMW_FLOW_GRAVITY_CMS2
/* Dynamic confidence fusion is enabled; FU! reports the active confidence and alpha. */
#define PMW_FLOW_CONFIDENCE_FUSION_ENABLE   (1U)
#define PMW_FLOW_FUSION_ALPHA_DEFAULT       (0.30f)
#define PMW_FLOW_FUSION_ALPHA_MIN           (0.30f)
#define PMW_FLOW_FUSION_ALPHA_MAX           (0.80f)
#define PMW_FLOW_CONFIDENCE_MIN_UPDATE      (0.05f)
#define PMW_FLOW_CONFIDENCE_TILT_GOOD_DEG   (8.0f)
#define PMW_FLOW_CONFIDENCE_RESIDUAL_GOOD_CMS (40.0f)
#define PMW_FLOW_CONFIDENCE_RESIDUAL_BAD_CMS  (160.0f)
#define PMW_FLOW_CONFIDENCE_DT_GOOD_RATIO   (0.20f)
#define PMW_FLOW_CONFIDENCE_DT_BAD_RATIO    (0.50f)
#define PMW_FLOW_CONFIDENCE_SPEED_GOOD_RATIO (0.50f)
#define PMW_FLOW_CONFIDENCE_SPEED_BAD_RATIO  (0.90f)
#define PMW_FLOW_CONFIDENCE_TOF_HOLD_SCALE  (0.75f)
#define PMW_FLOW_CONFIDENCE_TOF_JUMP_SCALE  (0.80f)
#define PMW_FLOW_CONFIDENCE_MIN_VALID_OBSERVATION (0.30f)
#define PMW_FLOW_FUSION_IMU_X_SIGN          (1.0f)
#define PMW_FLOW_FUSION_IMU_Y_SIGN          (-1.0f)
#define PMW_FLOW_FUSION_IMU_DELTA_LIMIT_CMS2 PMW_FLOW_GRAVITY_CMS2
#define PMW_FLOW_FUSION_RESIDUAL_RESET_CMS  (120.0f)
#define PMW_FLOW_FUSION_RESET_MIN_CONFIDENCE (0.60f)
#define PMW_FLOW_FUSION_RESET_CONFIRM_FRAMES (2U)
#define PMW_FLOW_FUSION_PREDICT_HOLD_FRAMES (25U)
#ifndef PMW_FLOW_KALMAN_ENABLE
#define PMW_FLOW_KALMAN_ENABLE             (1U)
#endif
#ifndef PMW_FLOW_KALMAN_Q_CMS2
#define PMW_FLOW_KALMAN_Q_CMS2             (2.0f)
#endif
#ifndef PMW_FLOW_KALMAN_R_BASE_CMS2
#define PMW_FLOW_KALMAN_R_BASE_CMS2        (625.0f)
#endif
#ifndef PMW_FLOW_KALMAN_P_INIT
#define PMW_FLOW_KALMAN_P_INIT             (625.0f)
#endif
#ifndef PMW_FLOW_KALMAN_R_MAX_SCALE
#define PMW_FLOW_KALMAN_R_MAX_SCALE        (20.0f)
#endif
#define PMW_FLOW_INVALID_HOLD_FRAMES        (3U)
#define PMW_FLOW_INVALID_DECAY_FRAMES       (8U)
#define PMW_FLOW_INVALID_DECAY_ALPHA        (0.85f)
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
static float g_pmw_flow_range_last_valid_cm = 0.0f;
static uint8 g_pmw_flow_range_filt_init = 0U;
static uint8 g_pmw_flow_range_last_valid_init = 0U;
static float g_pmw_flow_vel_x_filt_cms = 0.0f;
static float g_pmw_flow_vel_y_filt_cms = 0.0f;
static uint8 g_pmw_flow_vel_filt_init = 0U;
static float g_pmw_flow_vel_x_output_cms = 0.0f;
static float g_pmw_flow_vel_y_output_cms = 0.0f;
static uint8 g_pmw_flow_vel_output_init = 0U;
static uint8 g_pmw_flow_invalid_sample_count = 0U;
static float g_pmw_flow_fused_vx_cms = 0.0f;
static float g_pmw_flow_fused_vy_cms = 0.0f;
static float g_pmw_flow_fusion_imu_vx_last_cms = 0.0f;
static float g_pmw_flow_fusion_imu_vy_last_cms = 0.0f;
static uint8 g_pmw_flow_fusion_init = 0U;
static uint8 g_pmw_flow_fusion_residual_reset_count = 0U;
static uint8 g_pmw_flow_fusion_no_observation_frames = 0U;
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
    kf->q = PMW_FLOW_KALMAN_Q_CMS2;
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

#if PMW_FLOW_KALMAN_ENABLE
static void pmw_flow_kalman_init_axis(PMWFlowKalman1D_t *kf, float value)
{
    if(kf == NULL)
    {
        return;
    }

    kf->x = pmw_flow_limit_float(value, -PMW_FLOW_VEL_ABS_LIMIT_CMS, PMW_FLOW_VEL_ABS_LIMIT_CMS);
    kf->p = PMW_FLOW_KALMAN_P_INIT;
    kf->q = PMW_FLOW_KALMAN_Q_CMS2;
    kf->r_base = PMW_FLOW_KALMAN_R_BASE_CMS2;
    kf->k = 0.0f;
    kf->r = PMW_FLOW_KALMAN_R_BASE_CMS2;
    kf->initialized = 1U;
}

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

static void pmw_flow_kalman_predict_axis(PMWFlowKalman1D_t *kf, float delta_cms)
{
    if((kf == NULL) || !kf->initialized)
    {
        return;
    }

    kf->x = pmw_flow_limit_float(kf->x + delta_cms,
                                 -PMW_FLOW_VEL_ABS_LIMIT_CMS,
                                 PMW_FLOW_VEL_ABS_LIMIT_CMS);
    kf->k = 0.0f;

    if(pmw_flow_is_valid_number(kf->p) && (kf->p >= 0.0f))
    {
        kf->p += kf->q;
    }
    else
    {
        kf->p = PMW_FLOW_KALMAN_P_INIT;
    }
}

static void pmw_flow_kalman_update_axis(PMWFlowKalman1D_t *kf, float measurement_cms, float confidence)
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

    state->kalman_k_x = g_pmw_flow_kalman_x.k;
    state->kalman_k_y = g_pmw_flow_kalman_y.k;
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

static float pmw_flow_calc_confidence(const PMWFlowState_t *state,
                                      uint8 tof_source,
                                      uint8 tof_jump_rejected,
                                      float fused_vx_cms,
                                      float fused_vy_cms)
{
    float confidence = 1.0f;
    float tilt_abs_max_deg;
    float residual_x_cms;
    float residual_y_cms;
    float residual_norm_cms;
    float dt_error_ratio;
    float speed_ratio;

    if(state == NULL)
    {
        return 0.0f;
    }

    if(!state->valid ||
       !pmw_flow_is_valid_number(state->velocity_x_cms) ||
       !pmw_flow_is_valid_number(state->velocity_y_cms) ||
       !pmw_flow_is_valid_number(fused_vx_cms) ||
       !pmw_flow_is_valid_number(fused_vy_cms))
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

    residual_x_cms = state->velocity_x_cms - fused_vx_cms;
    residual_y_cms = state->velocity_y_cms - fused_vy_cms;
    residual_norm_cms = sqrtf(residual_x_cms * residual_x_cms +
                              residual_y_cms * residual_y_cms);
    confidence *= pmw_flow_ramp_down_confidence(residual_norm_cms,
                                                PMW_FLOW_CONFIDENCE_RESIDUAL_GOOD_CMS,
                                                PMW_FLOW_CONFIDENCE_RESIDUAL_BAD_CMS);

    dt_error_ratio = fabsf(state->dt_s - FLOW_SENSOR_NOMINAL_DT_S) / FLOW_SENSOR_NOMINAL_DT_S;
    confidence *= pmw_flow_ramp_down_confidence(dt_error_ratio,
                                                PMW_FLOW_CONFIDENCE_DT_GOOD_RATIO,
                                                PMW_FLOW_CONFIDENCE_DT_BAD_RATIO);

    speed_ratio = fabsf(state->velocity_x_raw_cms);
    if(fabsf(state->velocity_y_raw_cms) > speed_ratio)
    {
        speed_ratio = fabsf(state->velocity_y_raw_cms);
    }
    speed_ratio /= PMW_FLOW_VEL_ABS_LIMIT_CMS;
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

    if(tof_jump_rejected)
    {
        confidence *= PMW_FLOW_CONFIDENCE_TOF_JUMP_SCALE;
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

static void pmw_flow_apply_output_velocity_slew(float *velocity_x_cms, float *velocity_y_cms, float dt_s)
{
    if((velocity_x_cms == NULL) || (velocity_y_cms == NULL))
    {
        return;
    }

    if(!g_pmw_flow_vel_output_init)
    {
        g_pmw_flow_vel_x_output_cms = *velocity_x_cms;
        g_pmw_flow_vel_y_output_cms = *velocity_y_cms;
        g_pmw_flow_vel_output_init = 1U;
    }
    else
    {
        g_pmw_flow_vel_x_output_cms = pmw_flow_limit_velocity_rate(*velocity_x_cms,
                                                                   g_pmw_flow_vel_x_output_cms,
                                                                   PMW_FLOW_VEL_OUTPUT_SLEW_LIMIT_CMS2,
                                                                   dt_s);
        g_pmw_flow_vel_y_output_cms = pmw_flow_limit_velocity_rate(*velocity_y_cms,
                                                                   g_pmw_flow_vel_y_output_cms,
                                                                   PMW_FLOW_VEL_OUTPUT_SLEW_LIMIT_CMS2,
                                                                   dt_s);
    }

    *velocity_x_cms = g_pmw_flow_vel_x_output_cms;
    *velocity_y_cms = g_pmw_flow_vel_y_output_cms;
}

static void pmw_flow_apply_invalid_velocity_hold(PMWFlowState_t *state)
{
    uint8 publish_hold_velocity = 0U;
    uint8 invalid_window_frames = (uint8)(PMW_FLOW_INVALID_HOLD_FRAMES + PMW_FLOW_INVALID_DECAY_FRAMES);

    if(state == NULL)
    {
        return;
    }

    if(g_pmw_flow_vel_filt_init && (g_pmw_flow_invalid_sample_count < invalid_window_frames))
    {
        if(g_pmw_flow_invalid_sample_count >= PMW_FLOW_INVALID_HOLD_FRAMES)
        {
            g_pmw_flow_vel_x_filt_cms *= PMW_FLOW_INVALID_DECAY_ALPHA;
            g_pmw_flow_vel_y_filt_cms *= PMW_FLOW_INVALID_DECAY_ALPHA;
        }

        publish_hold_velocity = 1U;
    }

    if(g_pmw_flow_invalid_sample_count < 0xFFU)
    {
        g_pmw_flow_invalid_sample_count++;
    }

    if(publish_hold_velocity)
    {
        state->velocity_x_cms = g_pmw_flow_vel_x_filt_cms;
        state->velocity_y_cms = g_pmw_flow_vel_y_filt_cms;
        pmw_flow_apply_output_velocity_slew(&state->velocity_x_cms,
                                            &state->velocity_y_cms,
                                            state->dt_s);
        state->velocity_x_cms = pmw_flow_apply_velocity_deadband(state->velocity_x_cms);
        state->velocity_y_cms = pmw_flow_apply_velocity_deadband(state->velocity_y_cms);
        state->valid = 1U;
    }
    else
    {
        state->velocity_x_cms = 0.0f;
        state->velocity_y_cms = 0.0f;
        state->valid = 0U;
        g_pmw_flow_vel_filt_init = 0U;
        g_pmw_flow_range_filt_init = 0U;
        g_pmw_flow_range_last_valid_init = 0U;
        g_pmw_flow_vel_output_init = 0U;
    }

    state->speed_cms = sqrtf(state->velocity_x_cms * state->velocity_x_cms +
                             state->velocity_y_cms * state->velocity_y_cms);
}

static uint8 pmw_flow_fusion_predict_hold_active(void)
{
    return (uint8)(g_pmw_flow_fusion_init &&
                   (g_pmw_flow_fusion_no_observation_frames < PMW_FLOW_FUSION_PREDICT_HOLD_FRAMES));
}

static void pmw_flow_update_velocity_feedback(PMWFlowState_t *state,
                                              uint8 flow_observation_valid,
                                              uint8 tof_source,
                                              uint8 tof_jump_rejected,
                                              float imu_velocity_x_cms,
                                              float imu_velocity_y_cms)
{
    uint8 flow_used = 0U;
    uint8 acc_used = 0U;
    uint8 fused_velocity_valid = 0U;
    uint8 imu_velocity_valid;
    float imu_vx_cms;
    float imu_vy_cms;
    float residual_x_cms;
    float residual_y_cms;
    float residual_norm_cms;
    float flow_confidence;
#if !PMW_FLOW_KALMAN_ENABLE
    float confidence_for_alpha;
    float fusion_alpha;
#endif

    if(state == NULL)
    {
        return;
    }

    if(flow_observation_valid &&
       state->valid &&
       pmw_flow_is_valid_number(state->velocity_x_cms) &&
       pmw_flow_is_valid_number(state->velocity_y_cms))
    {
        flow_used = 1U;
        g_pmw_flow_fusion_no_observation_frames = 0U;

        imu_velocity_valid = (uint8)(pmw_flow_is_valid_number(imu_velocity_x_cms) &&
                                     pmw_flow_is_valid_number(imu_velocity_y_cms));
        imu_vx_cms = imu_velocity_x_cms * PMW_FLOW_FUSION_IMU_X_SIGN;
        imu_vy_cms = imu_velocity_y_cms * PMW_FLOW_FUSION_IMU_Y_SIGN;

        if(imu_velocity_valid)
        {
            if(!g_pmw_flow_fusion_init)
            {
                g_pmw_flow_fused_vx_cms = state->velocity_x_cms;
                g_pmw_flow_fused_vy_cms = state->velocity_y_cms;
                g_pmw_flow_fusion_imu_vx_last_cms = imu_vx_cms;
                g_pmw_flow_fusion_imu_vy_last_cms = imu_vy_cms;
                g_pmw_flow_fusion_init = 1U;
                g_pmw_flow_fusion_residual_reset_count = 0U;
#if PMW_FLOW_KALMAN_ENABLE
                pmw_flow_kalman_init_axis(&g_pmw_flow_kalman_x, state->velocity_x_cms);
                pmw_flow_kalman_init_axis(&g_pmw_flow_kalman_y, state->velocity_y_cms);
#endif
            }
            g_pmw_flow_fusion_imu_vx_last_cms = imu_vx_cms;
            g_pmw_flow_fusion_imu_vy_last_cms = imu_vy_cms;

            acc_used = 1U;
            residual_x_cms = state->velocity_x_cms - g_pmw_flow_fused_vx_cms;
            residual_y_cms = state->velocity_y_cms - g_pmw_flow_fused_vy_cms;
            residual_norm_cms = sqrtf(residual_x_cms * residual_x_cms +
                                      residual_y_cms * residual_y_cms);
            flow_confidence = pmw_flow_calc_confidence(state,
                                                       tof_source,
                                                       tof_jump_rejected,
                                                       g_pmw_flow_fused_vx_cms,
                                                       g_pmw_flow_fused_vy_cms);
            state->flow_confidence = flow_confidence;
            state->flow_residual_norm_cms = residual_norm_cms;

#if PMW_FLOW_KALMAN_ENABLE
            if(residual_norm_cms > PMW_FLOW_FUSION_RESIDUAL_RESET_CMS)
            {
                if(flow_confidence >= PMW_FLOW_FUSION_RESET_MIN_CONFIDENCE)
                {
                    if(g_pmw_flow_fusion_residual_reset_count < PMW_FLOW_FUSION_RESET_CONFIRM_FRAMES)
                    {
                        g_pmw_flow_fusion_residual_reset_count++;
                    }
                }
                else
                {
                    g_pmw_flow_fusion_residual_reset_count = 0U;
                }

                if(g_pmw_flow_fusion_residual_reset_count >= PMW_FLOW_FUSION_RESET_CONFIRM_FRAMES)
                {
                    pmw_flow_kalman_init_axis(&g_pmw_flow_kalman_x, state->velocity_x_cms);
                    pmw_flow_kalman_init_axis(&g_pmw_flow_kalman_y, state->velocity_y_cms);
                    g_pmw_flow_kalman_x.r = pmw_flow_kalman_calc_r(flow_confidence);
                    g_pmw_flow_kalman_y.r = g_pmw_flow_kalman_x.r;
                    g_pmw_flow_kalman_x.k = 1.0f;
                    g_pmw_flow_kalman_y.k = 1.0f;
                    g_pmw_flow_fusion_residual_reset_count = 0U;
                }
                else if(flow_confidence >= PMW_FLOW_CONFIDENCE_MIN_UPDATE)
                {
                    pmw_flow_kalman_update_axis(&g_pmw_flow_kalman_x, state->velocity_x_cms, flow_confidence);
                    pmw_flow_kalman_update_axis(&g_pmw_flow_kalman_y, state->velocity_y_cms, flow_confidence);
                }
                else
                {
                    g_pmw_flow_kalman_x.k = 0.0f;
                    g_pmw_flow_kalman_y.k = 0.0f;
                    g_pmw_flow_kalman_x.r = pmw_flow_kalman_calc_r(flow_confidence);
                    g_pmw_flow_kalman_y.r = g_pmw_flow_kalman_x.r;
                }
            }
            else if(flow_confidence >= PMW_FLOW_CONFIDENCE_MIN_UPDATE)
            {
                g_pmw_flow_fusion_residual_reset_count = 0U;
                pmw_flow_kalman_update_axis(&g_pmw_flow_kalman_x, state->velocity_x_cms, flow_confidence);
                pmw_flow_kalman_update_axis(&g_pmw_flow_kalman_y, state->velocity_y_cms, flow_confidence);
            }
            else
            {
                g_pmw_flow_fusion_residual_reset_count = 0U;
                g_pmw_flow_kalman_x.k = 0.0f;
                g_pmw_flow_kalman_y.k = 0.0f;
                g_pmw_flow_kalman_x.r = pmw_flow_kalman_calc_r(flow_confidence);
                g_pmw_flow_kalman_y.r = g_pmw_flow_kalman_x.r;
            }

            g_pmw_flow_fused_vx_cms = g_pmw_flow_kalman_x.x;
            g_pmw_flow_fused_vy_cms = g_pmw_flow_kalman_y.x;
#else
            if(flow_confidence > PMW_FLOW_CONFIDENCE_MIN_VALID_OBSERVATION)
            {
                confidence_for_alpha =
                    (flow_confidence - PMW_FLOW_CONFIDENCE_MIN_VALID_OBSERVATION) /
                    (1.0f - PMW_FLOW_CONFIDENCE_MIN_VALID_OBSERVATION);
            }
            else
            {
                confidence_for_alpha = 0.0f;
            }

            fusion_alpha = PMW_FLOW_FUSION_ALPHA_MIN +
                           confidence_for_alpha * (PMW_FLOW_FUSION_ALPHA_MAX - PMW_FLOW_FUSION_ALPHA_MIN);
            fusion_alpha = pmw_flow_limit_float(fusion_alpha,
                                                PMW_FLOW_FUSION_ALPHA_MIN,
                                                PMW_FLOW_FUSION_ALPHA_MAX);
            state->flow_confidence = flow_confidence;
            state->flow_fusion_alpha = fusion_alpha;
            state->flow_residual_norm_cms = residual_norm_cms;

#if PMW_FLOW_CONFIDENCE_FUSION_ENABLE
            if(residual_norm_cms > PMW_FLOW_FUSION_RESIDUAL_RESET_CMS)
            {
                if(flow_confidence >= PMW_FLOW_FUSION_RESET_MIN_CONFIDENCE)
                {
                    if(g_pmw_flow_fusion_residual_reset_count < PMW_FLOW_FUSION_RESET_CONFIRM_FRAMES)
                    {
                        g_pmw_flow_fusion_residual_reset_count++;
                    }
                }
                else
                {
                    g_pmw_flow_fusion_residual_reset_count = 0U;
                }

                if(g_pmw_flow_fusion_residual_reset_count >= PMW_FLOW_FUSION_RESET_CONFIRM_FRAMES)
                {
                    g_pmw_flow_fused_vx_cms = state->velocity_x_cms;
                    g_pmw_flow_fused_vy_cms = state->velocity_y_cms;
                    g_pmw_flow_fusion_residual_reset_count = 0U;
                }
                else if(flow_confidence >= PMW_FLOW_CONFIDENCE_MIN_UPDATE)
                {
                    g_pmw_flow_fused_vx_cms += fusion_alpha * residual_x_cms;
                    g_pmw_flow_fused_vy_cms += fusion_alpha * residual_y_cms;
                }
            }
            else if(flow_confidence >= PMW_FLOW_CONFIDENCE_MIN_UPDATE)
            {
                g_pmw_flow_fusion_residual_reset_count = 0U;
                g_pmw_flow_fused_vx_cms += fusion_alpha * residual_x_cms;
                g_pmw_flow_fused_vy_cms += fusion_alpha * residual_y_cms;
            }
            else
            {
                g_pmw_flow_fusion_residual_reset_count = 0U;
            }
#else
            if(fabsf(residual_x_cms) > PMW_FLOW_FUSION_RESIDUAL_RESET_CMS)
            {
                g_pmw_flow_fused_vx_cms = state->velocity_x_cms;
            }
            else
            {
                g_pmw_flow_fused_vx_cms += PMW_FLOW_FUSION_ALPHA_DEFAULT * residual_x_cms;
            }

            if(fabsf(residual_y_cms) > PMW_FLOW_FUSION_RESIDUAL_RESET_CMS)
            {
                g_pmw_flow_fused_vy_cms = state->velocity_y_cms;
            }
            else
            {
                g_pmw_flow_fused_vy_cms += PMW_FLOW_FUSION_ALPHA_DEFAULT * residual_y_cms;
            }
#endif
#endif

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

            state->velocity_x_fused_cms = g_pmw_flow_fused_vx_cms;
            state->velocity_y_fused_cms = g_pmw_flow_fused_vy_cms;
            fused_velocity_valid = 1U;
        }
        else
        {
            state->velocity_x_fused_cms = state->velocity_x_cms;
            state->velocity_y_fused_cms = state->velocity_y_cms;
            state->flow_confidence = 1.0f;
            state->flow_fusion_alpha = PMW_FLOW_FUSION_ALPHA_MAX;
            state->flow_residual_norm_cms = 0.0f;
            g_pmw_flow_fused_vx_cms = state->velocity_x_cms;
            g_pmw_flow_fused_vy_cms = state->velocity_y_cms;
            g_pmw_flow_fusion_init = 0U;
            g_pmw_flow_fusion_residual_reset_count = 0U;
            g_pmw_flow_fusion_no_observation_frames = 0U;
#if PMW_FLOW_KALMAN_ENABLE
            pmw_flow_kalman_reset();
#endif
            fused_velocity_valid = 1U;
        }
    }
    else
    {
        state->flow_confidence = 0.0f;
        state->flow_fusion_alpha = 0.0f;
        state->flow_residual_norm_cms = 0.0f;

        if(pmw_flow_fusion_predict_hold_active())
        {
            state->velocity_x_fused_cms = g_pmw_flow_fused_vx_cms;
            state->velocity_y_fused_cms = g_pmw_flow_fused_vy_cms;
#if PMW_FLOW_KALMAN_ENABLE
            g_pmw_flow_kalman_x.k = 0.0f;
            g_pmw_flow_kalman_y.k = 0.0f;
#endif
            fused_velocity_valid = 1U;
            acc_used = 1U;

            if(g_pmw_flow_fusion_no_observation_frames < 0xFFU)
            {
                g_pmw_flow_fusion_no_observation_frames++;
            }
        }
        else
        {
            state->velocity_x_fused_cms = 0.0f;
            state->velocity_y_fused_cms = 0.0f;
            g_pmw_flow_fused_vx_cms = 0.0f;
            g_pmw_flow_fused_vy_cms = 0.0f;
            g_pmw_flow_fusion_init = 0U;
            g_pmw_flow_fusion_residual_reset_count = 0U;
            g_pmw_flow_fusion_no_observation_frames = PMW_FLOW_FUSION_PREDICT_HOLD_FRAMES;
            pmw_flow_kalman_reset();
        }
    }

    state->velocity_x_fused_residual_cms = state->velocity_x_cms - state->velocity_x_fused_cms;
    state->velocity_y_fused_residual_cms = state->velocity_y_cms - state->velocity_y_fused_cms;
    state->flow_used = flow_used;
    state->acc_used = acc_used;
    state->fused_velocity_valid = fused_velocity_valid;
    pmw_flow_sync_kalman_diag(state);
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

    if(pmw_flow_is_valid_number(state->range_cm) &&
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

    g_pmw_flow_gyro_flow_x_delta_rad = 0.0f;
    g_pmw_flow_gyro_flow_y_delta_rad = 0.0f;
    g_pmw_flow_publish_slots[0] = cleared_state;
    g_pmw_flow_publish_slots[1] = cleared_state;
    g_pmw_flow_publish_index = 0U;
    g_pmw_flow_publish_seq = 0U;
    g_pmw_flow_mainloop_state = cleared_state;
    g_pmw_flow_mainloop_seq = 0U;
    g_pmw_flow_range_filt_cm = 0.0f;
    g_pmw_flow_range_last_valid_cm = 0.0f;
    g_pmw_flow_range_filt_init = 0U;
    g_pmw_flow_range_last_valid_init = 0U;
    g_pmw_flow_vel_x_filt_cms = 0.0f;
    g_pmw_flow_vel_y_filt_cms = 0.0f;
    g_pmw_flow_vel_filt_init = 0U;
    g_pmw_flow_vel_x_output_cms = 0.0f;
    g_pmw_flow_vel_y_output_cms = 0.0f;
    g_pmw_flow_vel_output_init = 0U;
    g_pmw_flow_invalid_sample_count = 0U;
    g_pmw_flow_fused_vx_cms = 0.0f;
    g_pmw_flow_fused_vy_cms = 0.0f;
    g_pmw_flow_fusion_imu_vx_last_cms = 0.0f;
    g_pmw_flow_fusion_imu_vy_last_cms = 0.0f;
    g_pmw_flow_fusion_init = 0U;
    g_pmw_flow_fusion_residual_reset_count = 0U;
    g_pmw_flow_fusion_no_observation_frames = 0U;
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

    if(!g_pmw_flow_fusion_init)
    {
        g_pmw_flow_fusion_imu_vx_last_cms = imu_vx_cms;
        g_pmw_flow_fusion_imu_vy_last_cms = imu_vy_cms;
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
    pmw_flow_kalman_predict_axis(&g_pmw_flow_kalman_x, imu_delta_x_cms);
    pmw_flow_kalman_predict_axis(&g_pmw_flow_kalman_y, imu_delta_y_cms);
    g_pmw_flow_fused_vx_cms = g_pmw_flow_kalman_x.x;
    g_pmw_flow_fused_vy_cms = g_pmw_flow_kalman_y.x;
#else
    g_pmw_flow_fused_vx_cms += imu_delta_x_cms;
    g_pmw_flow_fused_vy_cms += imu_delta_y_cms;
#endif
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
                           float tof_los_cm,
                           uint8 tof_source,
                           float dt_s,
                           float imu_velocity_x_cms,
                           float imu_velocity_y_cms)
{
    PMWFlowState_t new_state = {0};
    float range_used_cm;
    float range_sample_cm;
    float vx_raw;
    float vy_raw;
    float tilt_abs_max_deg;
    float velocity_lpf_alpha;
    uint8 tof_jump_rejected = 0U;
    uint8 flow_observation_valid = 0U;
    new_state.flow_delta_x_count = 0;
    new_state.flow_delta_y_count = 0;
    new_state.dt_s = dt_s;
    new_state.tof_los_cm = tof_los_cm;
    new_state.roll_deg = roll_deg;
    new_state.pitch_deg = pitch_deg;
    new_state.gyro_flow_x_delta_rad = g_pmw_flow_gyro_flow_x_delta_rad;
    new_state.gyro_flow_y_delta_rad = g_pmw_flow_gyro_flow_y_delta_rad;
    g_pmw_flow_gyro_flow_x_delta_rad = 0.0f;
    g_pmw_flow_gyro_flow_y_delta_rad = 0.0f;

    {
        int16 flow_input_x = flow_delta_x;
        int16 flow_input_y = flow_delta_y;

#if FLOW_SENSOR_SWAP_XY
        flow_input_x = flow_delta_y;
        flow_input_y = flow_delta_x;
#endif

        new_state.flow_delta_x_count = (int16)flow_input_x;
        new_state.flow_delta_y_count = (int16)flow_input_y;

        new_state.flow_x_rad = (float)flow_input_x * PMW_FLOW_RAD_PER_COUNT_X * PMW_FLOW_SENSOR_X_SIGN;
        new_state.flow_y_rad = (float)flow_input_y * PMW_FLOW_RAD_PER_COUNT_Y * PMW_FLOW_SENSOR_Y_SIGN;
    }

    if(!(dt_s >= PMW_FLOW_MIN_DT_S) ||
       !(dt_s <= PMW_FLOW_MAX_DT_S) ||
       !pmw_flow_is_valid_number(tof_los_cm) ||
       !pmw_flow_is_valid_number(roll_deg) ||
       !pmw_flow_is_valid_number(pitch_deg) ||
       !pmw_flow_is_valid_number(new_state.gyro_flow_x_delta_rad) ||
       !pmw_flow_is_valid_number(new_state.gyro_flow_y_delta_rad) ||
       !pmw_flow_is_valid_delta_count(flow_delta_x) ||
       !pmw_flow_is_valid_delta_count(flow_delta_y) ||
       (tof_los_cm < PMW_FLOW_MIN_HEIGHT_CM) ||
       (tof_los_cm > PMW_FLOW_MAX_HEIGHT_CM))
    {
        new_state.range_cm = tof_los_cm;
        pmw_flow_apply_invalid_velocity_hold(&new_state);
        pmw_flow_update_velocity_feedback(&new_state,
                                          flow_observation_valid,
                                          tof_source,
                                          tof_jump_rejected,
                                          imu_velocity_x_cms,
                                          imu_velocity_y_cms);
        pmw_flow_update_world_odometry(&new_state);
        pmw_flow_publish_state(&new_state);
        return;
    }

    /* (2) ToF 距离去毛刺 + LPF：v = range × flow_rate，距离单帧抖动会直接放大成速度噪声。 */
    range_sample_cm = tof_los_cm;
    if(g_pmw_flow_range_last_valid_init)
    {
        if(fabsf(range_sample_cm - g_pmw_flow_range_last_valid_cm) > PMW_FLOW_TOF_JUMP_REJECT_CM)
        {
            range_sample_cm = g_pmw_flow_range_last_valid_cm;
            tof_jump_rejected = 1U;
        }
        else
        {
            g_pmw_flow_range_last_valid_cm = range_sample_cm;
        }
    }
    else
    {
        g_pmw_flow_range_last_valid_cm = range_sample_cm;
        g_pmw_flow_range_last_valid_init = 1U;
    }

    if(!g_pmw_flow_range_filt_init)
    {
        g_pmw_flow_range_filt_cm = range_sample_cm;
        g_pmw_flow_range_filt_init = 1U;
    }
    else
    {
        g_pmw_flow_range_filt_cm += PMW_FLOW_RANGE_LPF_ALPHA * (range_sample_cm - g_pmw_flow_range_filt_cm);
    }
    range_used_cm = g_pmw_flow_range_filt_cm;
    new_state.range_cm = range_used_cm;

    /* (3) IMU 去旋转：用 4kHz 累加得到的陀螺角度增量抵消机体姿态变化对光流的污染。
     *     每个轴乘上独立的标定增益 K_X / K_Y，校准 RAD_PER_COUNT 与陀螺尺度的不匹配。 */
    {
        float kx = g_pmw_flow_gyro_comp_gain_x;
        float ky = g_pmw_flow_gyro_comp_gain_y;

        new_state.flow_comp_x_rad = new_state.flow_x_rad +
                                    kx * (PMW_FLOW_COMP_X_FROM_GYRO_X_SIGN * new_state.gyro_flow_x_delta_rad +
                                          PMW_FLOW_COMP_X_FROM_GYRO_Y_SIGN * new_state.gyro_flow_y_delta_rad);
        new_state.flow_comp_y_rad = new_state.flow_y_rad +
                                    ky * (PMW_FLOW_COMP_Y_FROM_GYRO_X_SIGN * new_state.gyro_flow_x_delta_rad +
                                          PMW_FLOW_COMP_Y_FROM_GYRO_Y_SIGN * new_state.gyro_flow_y_delta_rad);
    }

    /* (4) 机体系速度（去旋转后）。
     *
     * 约定：v_body_x 正 → 机体前进；v_body_y 正 → 机体左移。 */
    {
        float v_body_x = PMW_FLOW_BODY_VX_FROM_FLOW_X_SIGN * (range_used_cm * new_state.flow_comp_x_rad / dt_s);
        float v_body_y = PMW_FLOW_BODY_VY_FROM_FLOW_Y_SIGN * (range_used_cm * new_state.flow_comp_y_rad / dt_s);

        /* (4.1) 机体 → 水平面投影。
         *
         * pitch 正 = 低头，机体 X 轴向前下方倾斜，机体 X 速度比真实水平前进速度小 cos(pitch) 倍：
         *   v_body_x = v_horizontal_x × cos(pitch)   →   v_horizontal_x = v_body_x / cos(pitch)
         * Y/roll 同理（roll 正 = 右低）。
         *
         * 这样 vx_raw / vy_raw 直接是地面水平 X / Y 速度，给控制层用。 */
        float pitch_rad = pitch_deg * PMW_FLOW_DEG2RAD;
        float roll_rad  = roll_deg  * PMW_FLOW_DEG2RAD;
        float cos_pitch = cosf(pitch_rad);
        float cos_roll  = cosf(roll_rad);

        if(cos_pitch < PMW_FLOW_TILT_COS_FLOOR)
        {
            cos_pitch = PMW_FLOW_TILT_COS_FLOOR;
        }
        if(cos_roll < PMW_FLOW_TILT_COS_FLOOR)
        {
            cos_roll = PMW_FLOW_TILT_COS_FLOOR;
        }

        vx_raw = v_body_x / cos_pitch;
        vy_raw = v_body_y / cos_roll;
    }
    new_state.velocity_x_raw_cms = vx_raw;
    new_state.velocity_y_raw_cms = vy_raw;

    if((fabsf(vx_raw) > PMW_FLOW_VEL_ABS_LIMIT_CMS) ||
       (fabsf(vy_raw) > PMW_FLOW_VEL_ABS_LIMIT_CMS))
    {
        pmw_flow_apply_invalid_velocity_hold(&new_state);
        pmw_flow_update_velocity_feedback(&new_state,
                                          flow_observation_valid,
                                          tof_source,
                                          tof_jump_rejected,
                                          imu_velocity_x_cms,
                                          imu_velocity_y_cms);
        pmw_flow_update_world_odometry(&new_state);
        pmw_flow_publish_state(&new_state);
        return;
    }

    /* (5) 倾角门：roll/pitch 较大时，去旋转残差会放大、ToF LOS 也偏离真高，这种
     *     时刻 vx/vy 不可信。短时保持上一拍速度，持续无效后衰减并退出 valid。 */
    tilt_abs_max_deg = fabsf(roll_deg);
    if(fabsf(pitch_deg) > tilt_abs_max_deg)
    {
        tilt_abs_max_deg = fabsf(pitch_deg);
    }

    if(tilt_abs_max_deg > PMW_FLOW_TILT_GATE_DEG)
    {
        pmw_flow_apply_invalid_velocity_hold(&new_state);
    }
    else
    {
        /* (6) 速度样本预处理：用重力加速度量级限制变化率，再做轻低通。 */
        if(g_pmw_flow_vel_filt_init)
        {
            vx_raw = pmw_flow_limit_velocity_slew(vx_raw, g_pmw_flow_vel_x_filt_cms, dt_s);
            vy_raw = pmw_flow_limit_velocity_slew(vy_raw, g_pmw_flow_vel_y_filt_cms, dt_s);

            velocity_lpf_alpha = pmw_flow_calc_lpf_alpha(PMW_FLOW_VEL_LPF_CUTOFF_HZ, dt_s);
            g_pmw_flow_vel_x_filt_cms += velocity_lpf_alpha * (vx_raw - g_pmw_flow_vel_x_filt_cms);
            g_pmw_flow_vel_y_filt_cms += velocity_lpf_alpha * (vy_raw - g_pmw_flow_vel_y_filt_cms);
        }
        else
        {
            g_pmw_flow_vel_x_filt_cms = vx_raw;
            g_pmw_flow_vel_y_filt_cms = vy_raw;
            g_pmw_flow_vel_filt_init = 1U;
        }

        g_pmw_flow_invalid_sample_count = 0U;
        vx_raw = g_pmw_flow_vel_x_filt_cms;
        vy_raw = g_pmw_flow_vel_y_filt_cms;
        pmw_flow_apply_output_velocity_slew(&vx_raw, &vy_raw, dt_s);
        new_state.velocity_x_cms = pmw_flow_apply_velocity_deadband(vx_raw);
        new_state.velocity_y_cms = pmw_flow_apply_velocity_deadband(vy_raw);
        new_state.valid = 1U;
        flow_observation_valid = (uint8)((tof_source == PMW_FLOW_TOF_SOURCE_LIVE) ? 1U : 0U);
    }

    new_state.speed_cms = sqrtf(new_state.velocity_x_cms * new_state.velocity_x_cms +
                                new_state.velocity_y_cms * new_state.velocity_y_cms);
    pmw_flow_update_velocity_feedback(&new_state,
                                      flow_observation_valid,
                                      tof_source,
                                      tof_jump_rejected,
                                      imu_velocity_x_cms,
                                      imu_velocity_y_cms);
    pmw_flow_update_world_odometry(&new_state);

    pmw_flow_publish_state(&new_state);
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
    pmw_flow_sync_kalman_diag(state);

    if(pmw_flow_fusion_predict_hold_active())
    {
        state->velocity_x_fused_cms = g_pmw_flow_fused_vx_cms;
        state->velocity_y_fused_cms = g_pmw_flow_fused_vy_cms;
        state->velocity_x_fused_residual_cms = state->velocity_x_cms - state->velocity_x_fused_cms;
        state->velocity_y_fused_residual_cms = state->velocity_y_cms - state->velocity_y_fused_cms;
        state->flow_residual_norm_cms = sqrtf(state->velocity_x_fused_residual_cms *
                                              state->velocity_x_fused_residual_cms +
                                              state->velocity_y_fused_residual_cms *
                                              state->velocity_y_fused_residual_cms);
        state->acc_used = 1U;
        state->fused_velocity_valid = 1U;
    }
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
