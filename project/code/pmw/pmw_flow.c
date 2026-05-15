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
 * 来源：F0! 光流数据.csv 上对 flow_x_rad ~ gyro_y_delta_rad、
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
 * After axis verification, flow_comp_x maps to body X and flow_comp_y maps to body Y.
 */
#define PMW_FLOW_BODY_VX_FROM_FLOW_X_SIGN   (1.0f)
#define PMW_FLOW_BODY_VY_FROM_FLOW_Y_SIGN   (-1.0f)
#define PMW_FLOW_MIN_HEIGHT_CM              (5.0f)
#define PMW_FLOW_MAX_HEIGHT_CM              (400.0f)
#define PMW_FLOW_MIN_DT_S                   (0.001f)
#define PMW_FLOW_MAX_DT_S                   (0.100f)
#define PMW_FLOW_MAX_ABS_DELTA_COUNT        (5000)

/*
 * 滤波参数（采样率约 50Hz / dt=20ms）：
 *   - RANGE_LPF_ALPHA:   ToF 距离一阶 LPF, alpha=0.40 → 截止 ≈ 5Hz，
 *                        把单帧 ToF 抖动平滑掉，避免直接乘到速度上。
 *   - VEL_X_LPF_ALPHA:   X 方向机体系速度 LPF, alpha=0.30，保留 Pitch 速度环响应。
 *   - VEL_Y_LPF_ALPHA:   Y 方向机体系速度 LPF, alpha=0.18，压低 Roll 目标角抖动。
 *   - TILT_GATE_DEG:     |roll|/|pitch| 超过此值时认为光流姿态补偿残差不可信，
 *                        冻结速度 LPF 状态，输出保持上一次的滤波值。
 *   - VEL_OUTLIER_LIMIT: 单帧速度跳变超过该阈值视为离群（多半是 ToF 跳变 +
 *                        光流大瞬变叠加），用上一次滤波值替代喂进 LPF。
 */
#define PMW_FLOW_RANGE_LPF_ALPHA            (0.40f)
#define PMW_FLOW_VEL_X_LPF_ALPHA            (0.30f)
#define PMW_FLOW_VEL_Y_LPF_ALPHA            (0.18f)
#define PMW_FLOW_TILT_GATE_DEG              (25.0f)
#define PMW_FLOW_VEL_OUTLIER_LIMIT_CMS      (300.0f)
#define PMW_FLOW_WORLD_TILT_GATE_DEG        (25.0f)

/*
 * 机体系 → 水平面投影时 1/cos 的下限保护：cos(60°)=0.5。
 * 正常情况倾角门把超过 25° 的样本冻结掉，下面的 1/cos 顶多放大 1/cos(25°) ≈ 1.10×；
 * 这个 floor 只是兜底，避免姿态读数瞬间出现极端值时除法爆掉。
 */
#define PMW_FLOW_TILT_COS_FLOOR             (0.5f)

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
static float g_pmw_flow_vel_x_filt_cms = 0.0f;
static float g_pmw_flow_vel_y_filt_cms = 0.0f;
static uint8 g_pmw_flow_vel_filt_init = 0U;
static float g_pmw_flow_world_vx_cms = 0.0f;
static float g_pmw_flow_world_vy_cms = 0.0f;
static float g_pmw_flow_world_x_cm = 0.0f;
static float g_pmw_flow_world_y_cm = 0.0f;
static float g_pmw_flow_delta_world_x_cm = 0.0f;
static float g_pmw_flow_delta_world_y_cm = 0.0f;
static float g_pmw_flow_direct_world_x_cm = 0.0f;
static float g_pmw_flow_direct_world_y_cm = 0.0f;
static uint8 g_pmw_flow_world_valid = 0U;

void PMW_Flow_ResetWorldPosition(void);

static uint8 pmw_flow_is_valid_number(float value)
{
    return (uint8)((value == value) && (value > -1000000.0f) && (value < 1000000.0f));
}

static uint8 pmw_flow_is_valid_delta_count(int16 value)
{
    return (uint8)((value > -PMW_FLOW_MAX_ABS_DELTA_COUNT) && (value < PMW_FLOW_MAX_ABS_DELTA_COUNT));
}

static void pmw_flow_update_velocity_feedback(PMWFlowState_t *state)
{
    uint8 flow_used = 0U;

    if(state == NULL)
    {
        return;
    }

    if(state->valid &&
       pmw_flow_is_valid_number(state->velocity_x_cms) &&
       pmw_flow_is_valid_number(state->velocity_y_cms))
    {
        state->velocity_x_fused_cms = state->velocity_x_cms;
        state->velocity_y_fused_cms = state->velocity_y_cms;
        flow_used = 1U;
    }
    else
    {
        state->velocity_x_fused_cms = 0.0f;
        state->velocity_y_fused_cms = 0.0f;
    }

    state->velocity_x_fused_residual_cms = state->velocity_x_cms - state->velocity_x_fused_cms;
    state->velocity_y_fused_residual_cms = state->velocity_y_cms - state->velocity_y_fused_cms;
    state->flow_used = flow_used;
    state->acc_used = 0U;
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
        g_pmw_flow_world_valid = 1U;
    }
    else
    {
        g_pmw_flow_world_vx_cms = 0.0f;
        g_pmw_flow_world_vy_cms = 0.0f;
        g_pmw_flow_delta_world_x_cm = 0.0f;
        g_pmw_flow_delta_world_y_cm = 0.0f;
        g_pmw_flow_world_valid = 0U;
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
    state->reserved0 = g_pmw_flow_world_valid;
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
    g_pmw_flow_range_filt_init = 0U;
    g_pmw_flow_vel_x_filt_cms = 0.0f;
    g_pmw_flow_vel_y_filt_cms = 0.0f;
    g_pmw_flow_vel_filt_init = 0U;
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
    g_pmw_flow_world_valid = 0U;
}

void PMW_Flow_UpdateSample(int16 flow_delta_x,
                           int16 flow_delta_y,
                           float roll_deg,
                           float pitch_deg,
                           float tof_los_cm,
                           float dt_s)
{
    PMWFlowState_t new_state = {0};
    float range_used_cm;
    float vx_raw;
    float vy_raw;
    float tilt_abs_max_deg;
    new_state.flow_delta_x_count = flow_delta_x;
    new_state.flow_delta_y_count = flow_delta_y;
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
        pmw_flow_update_velocity_feedback(&new_state);
        pmw_flow_update_world_odometry(&new_state);
        pmw_flow_publish_state(&new_state);
        return;
    }

    /* (2) ToF 距离 LPF：v = range × flow_rate，距离单帧抖动会直接放大成速度噪声。
     *     先把 range 平滑成 ~5Hz 截止的稳定信号再去乘。 */
    if(!g_pmw_flow_range_filt_init)
    {
        g_pmw_flow_range_filt_cm = tof_los_cm;
        g_pmw_flow_range_filt_init = 1U;
    }
    else
    {
        g_pmw_flow_range_filt_cm += PMW_FLOW_RANGE_LPF_ALPHA * (tof_los_cm - g_pmw_flow_range_filt_cm);
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
     * 约定：v_body_x 正 → 机体前进；v_body_y 正 → 机体右移。 */
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

    /* (5) 倾角门：roll/pitch 较大时，去旋转残差会放大、ToF LOS 也偏离真高，这种
     *     时刻 vx/vy 不可信。直接冻结速度 LPF 状态，保持上一次的滤波输出。 */
    tilt_abs_max_deg = fabsf(roll_deg);
    if(fabsf(pitch_deg) > tilt_abs_max_deg)
    {
        tilt_abs_max_deg = fabsf(pitch_deg);
    }

    if(tilt_abs_max_deg > PMW_FLOW_TILT_GATE_DEG)
    {
        new_state.velocity_x_cms = g_pmw_flow_vel_filt_init ? g_pmw_flow_vel_x_filt_cms : 0.0f;
        new_state.velocity_y_cms = g_pmw_flow_vel_filt_init ? g_pmw_flow_vel_y_filt_cms : 0.0f;
    }
    else
    {
        /* (6) 离群剔除：单帧速度跳变超过门限的样本，多半是 ToF 抖动 + 大瞬时光流叠加。
         *     用上一次滤波值替代当前样本喂进 LPF，避免追到尖峰上去。 */
        if(g_pmw_flow_vel_filt_init)
        {
            if(fabsf(vx_raw - g_pmw_flow_vel_x_filt_cms) > PMW_FLOW_VEL_OUTLIER_LIMIT_CMS)
            {
                vx_raw = g_pmw_flow_vel_x_filt_cms;
            }
            if(fabsf(vy_raw - g_pmw_flow_vel_y_filt_cms) > PMW_FLOW_VEL_OUTLIER_LIMIT_CMS)
            {
                vy_raw = g_pmw_flow_vel_y_filt_cms;
            }
            g_pmw_flow_vel_x_filt_cms += PMW_FLOW_VEL_X_LPF_ALPHA * (vx_raw - g_pmw_flow_vel_x_filt_cms);
            g_pmw_flow_vel_y_filt_cms += PMW_FLOW_VEL_Y_LPF_ALPHA * (vy_raw - g_pmw_flow_vel_y_filt_cms);
        }
        else
        {
            g_pmw_flow_vel_x_filt_cms = vx_raw;
            g_pmw_flow_vel_y_filt_cms = vy_raw;
            g_pmw_flow_vel_filt_init = 1U;
        }

        new_state.velocity_x_cms = g_pmw_flow_vel_x_filt_cms;
        new_state.velocity_y_cms = g_pmw_flow_vel_y_filt_cms;
    }

    new_state.speed_cms = sqrtf(new_state.velocity_x_cms * new_state.velocity_x_cms +
                                new_state.velocity_y_cms * new_state.velocity_y_cms);
    new_state.valid = 1U;
    pmw_flow_update_velocity_feedback(&new_state);
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
