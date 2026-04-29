#include "pmw_flow.h"
#include "common/project_headfile.h"
#include <math.h>

#define PMW_FLOW_DEG2RAD                    (0.01745329251994329577f)
#define PMW_FLOW_RAD_PER_COUNT_X            (1.0f / 385.0f)
#define PMW_FLOW_RAD_PER_COUNT_Y            (1.0f / 385.0f)
#define PMW_FLOW_SENSOR_X_SIGN              (1.0f)
#define PMW_FLOW_SENSOR_Y_SIGN              (1.0f)
/*
 * Flight-test result for rotational compensation:
 * - flow X follows gyro Y
 * - flow Y follows gyro X
 * Keep the cross-axis mapping explicit so later tuning only needs macro changes.
 */
#define PMW_FLOW_GYRO_X_DELTA_SIGN          (1.0f)
#define PMW_FLOW_GYRO_Y_DELTA_SIGN          (1.0f)
#define PMW_FLOW_COMP_X_FROM_GYRO_X_SIGN    (0.0f)
#define PMW_FLOW_COMP_X_FROM_GYRO_Y_SIGN    (-1.0f)
#define PMW_FLOW_COMP_Y_FROM_GYRO_X_SIGN    (-1.0f)
#define PMW_FLOW_COMP_Y_FROM_GYRO_Y_SIGN    (0.0f)
/*
 * PX4-style body velocity mapping from compensated flow integral.
 * Keep the mapping configurable because the final body-frame sign depends on
 * camera installation and the project's body-axis convention.
 */
#define PMW_FLOW_BODY_VX_FROM_FLOW_Y_SIGN   (-1.0f)
#define PMW_FLOW_BODY_VY_FROM_FLOW_X_SIGN   (1.0f)
#define PMW_FLOW_MIN_HEIGHT_CM              (5.0f)
#define PMW_FLOW_MAX_HEIGHT_CM              (400.0f)
#define PMW_FLOW_MIN_DT_S                   (0.001f)
#define PMW_FLOW_MAX_DT_S                   (0.100f)
#define PMW_FLOW_MAX_ABS_DELTA_COUNT        (5000)

/*
 * 滤波参数（采样率约 50Hz / dt=20ms）：
 *   - RANGE_LPF_ALPHA:   ToF 距离一阶 LPF, alpha=0.40 → 截止 ≈ 5Hz，
 *                        把单帧 ToF 抖动平滑掉，避免直接乘到速度上。
 *   - VEL_LPF_ALPHA:     最终体系速度 LPF, alpha=0.30 → 截止 ≈ 3Hz，
 *                        给上层（控制 / VOFA）提供稳定的速度量。
 *   - TILT_GATE_DEG:     |roll|/|pitch| 超过此值时认为光流姿态补偿残差不可信，
 *                        冻结速度 LPF 状态，输出保持上一次的滤波值。
 *   - VEL_OUTLIER_LIMIT: 单帧速度跳变超过该阈值视为离群（多半是 ToF 跳变 +
 *                        光流大瞬变叠加），用上一次滤波值替代喂进 LPF。
 */
#define PMW_FLOW_RANGE_LPF_ALPHA            (0.40f)
#define PMW_FLOW_VEL_LPF_ALPHA              (0.30f)
#define PMW_FLOW_TILT_GATE_DEG              (25.0f)
#define PMW_FLOW_VEL_OUTLIER_LIMIT_CMS      (300.0f)

/*
 * 机体系 → 水平面投影时 1/cos 的下限保护：cos(60°)=0.5。
 * 正常情况倾角门把超过 25° 的样本冻结掉，下面的 1/cos 顶多放大 1/cos(25°) ≈ 1.10×；
 * 这个 floor 只是兜底，避免姿态读数瞬间出现极端值时除法爆掉。
 */
#define PMW_FLOW_TILT_COS_FLOOR             (0.5f)

static float g_pmw_flow_gyro_flow_x_delta_rad = 0.0f;
static float g_pmw_flow_gyro_flow_y_delta_rad = 0.0f;
static PMWFlowState_t g_pmw_flow_publish_slots[2] = {0};
static volatile uint8 g_pmw_flow_publish_index = 0U;
static volatile uint32 g_pmw_flow_publish_seq = 0U;
static PMWFlowState_t g_pmw_flow_mainloop_state = {0};
static uint32 g_pmw_flow_mainloop_seq = 0U;

/* 中值-3 历史窗（kill 单帧光流尖峰），LPF 状态以及惯性侧锁存初始化标志。 */
static int16 g_pmw_flow_dx_hist[2] = {0, 0};
static int16 g_pmw_flow_dy_hist[2] = {0, 0};
static uint8 g_pmw_flow_count_hist_n = 0U;
static float g_pmw_flow_range_filt_cm = 0.0f;
static uint8 g_pmw_flow_range_filt_init = 0U;
static float g_pmw_flow_vel_x_filt_cms = 0.0f;
static float g_pmw_flow_vel_y_filt_cms = 0.0f;
static uint8 g_pmw_flow_vel_filt_init = 0U;

static int16 pmw_flow_median3_i16(int16 a, int16 b, int16 c)
{
    int16 lo = (a < b) ? a : b;
    int16 hi = (a > b) ? a : b;
    if(c < lo) return lo;
    if(c > hi) return hi;
    return c;
}

static uint8 pmw_flow_is_valid_number(float value)
{
    return (uint8)((value == value) && (value > -1000000.0f) && (value < 1000000.0f));
}

static uint8 pmw_flow_is_valid_delta_count(int16 value)
{
    return (uint8)((value > -PMW_FLOW_MAX_ABS_DELTA_COUNT) && (value < PMW_FLOW_MAX_ABS_DELTA_COUNT));
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

    g_pmw_flow_dx_hist[0] = 0;
    g_pmw_flow_dx_hist[1] = 0;
    g_pmw_flow_dy_hist[0] = 0;
    g_pmw_flow_dy_hist[1] = 0;
    g_pmw_flow_count_hist_n = 0U;
    g_pmw_flow_range_filt_cm = 0.0f;
    g_pmw_flow_range_filt_init = 0U;
    g_pmw_flow_vel_x_filt_cms = 0.0f;
    g_pmw_flow_vel_y_filt_cms = 0.0f;
    g_pmw_flow_vel_filt_init = 0U;
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

void PMW_Flow_UpdateSample(int16 flow_delta_x,
                           int16 flow_delta_y,
                           float roll_deg,
                           float pitch_deg,
                           float tof_los_cm,
                           float dt_s)
{
    PMWFlowState_t new_state = {0};
    int16 dx_med;
    int16 dy_med;
    float range_used_cm;
    float vx_raw;
    float vy_raw;
    float tilt_abs_max_deg;

    /* (1) 中值-3 去单帧尖峰：PMW3901 偶尔会蹦出明显偏离邻帧的脉冲（贴地纹理切换、
     *     强反光），这一级把孤立尖峰整体替换成中值。历史窗永远更新，保证后续两帧
     *     恢复正常时立刻跟上节拍。 */
    if(g_pmw_flow_count_hist_n >= 2U)
    {
        dx_med = pmw_flow_median3_i16(flow_delta_x, g_pmw_flow_dx_hist[0], g_pmw_flow_dx_hist[1]);
        dy_med = pmw_flow_median3_i16(flow_delta_y, g_pmw_flow_dy_hist[0], g_pmw_flow_dy_hist[1]);
    }
    else
    {
        dx_med = flow_delta_x;
        dy_med = flow_delta_y;
        g_pmw_flow_count_hist_n++;
    }
    g_pmw_flow_dx_hist[1] = g_pmw_flow_dx_hist[0];
    g_pmw_flow_dx_hist[0] = flow_delta_x;
    g_pmw_flow_dy_hist[1] = g_pmw_flow_dy_hist[0];
    g_pmw_flow_dy_hist[0] = flow_delta_y;

    /* 原始计数 / dt / 姿态等基础字段；flow_delta_*_count 暴露中值后的值，
     * 让 VOFA / 监视页直接看到下游真正使用的输入。 */
    new_state.flow_delta_x_count = dx_med;
    new_state.flow_delta_y_count = dy_med;
    new_state.dt_s = dt_s;
    new_state.tof_los_cm = tof_los_cm;
    new_state.roll_deg = roll_deg;
    new_state.pitch_deg = pitch_deg;
    new_state.gyro_flow_x_delta_rad = g_pmw_flow_gyro_flow_x_delta_rad;
    new_state.gyro_flow_y_delta_rad = g_pmw_flow_gyro_flow_y_delta_rad;
    g_pmw_flow_gyro_flow_x_delta_rad = 0.0f;
    g_pmw_flow_gyro_flow_y_delta_rad = 0.0f;

    new_state.flow_x_rad = (float)dx_med * PMW_FLOW_RAD_PER_COUNT_X * PMW_FLOW_SENSOR_X_SIGN;
    new_state.flow_y_rad = (float)dy_med * PMW_FLOW_RAD_PER_COUNT_Y * PMW_FLOW_SENSOR_Y_SIGN;

    if(!(dt_s >= PMW_FLOW_MIN_DT_S) ||
       !(dt_s <= PMW_FLOW_MAX_DT_S) ||
       !pmw_flow_is_valid_number(tof_los_cm) ||
       !pmw_flow_is_valid_number(roll_deg) ||
       !pmw_flow_is_valid_number(pitch_deg) ||
       !pmw_flow_is_valid_number(new_state.gyro_flow_x_delta_rad) ||
       !pmw_flow_is_valid_number(new_state.gyro_flow_y_delta_rad) ||
       !pmw_flow_is_valid_delta_count(dx_med) ||
       !pmw_flow_is_valid_delta_count(dy_med) ||
       (tof_los_cm < PMW_FLOW_MIN_HEIGHT_CM) ||
       (tof_los_cm > PMW_FLOW_MAX_HEIGHT_CM))
    {
        new_state.range_cm = tof_los_cm;
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

    /* (3) IMU 去旋转：用 4kHz 累加得到的陀螺角度增量抵消机体姿态变化对光流的污染。 */
    new_state.flow_comp_x_rad = new_state.flow_x_rad +
                                PMW_FLOW_COMP_X_FROM_GYRO_X_SIGN * new_state.gyro_flow_x_delta_rad +
                                PMW_FLOW_COMP_X_FROM_GYRO_Y_SIGN * new_state.gyro_flow_y_delta_rad;
    new_state.flow_comp_y_rad = new_state.flow_y_rad +
                                PMW_FLOW_COMP_Y_FROM_GYRO_X_SIGN * new_state.gyro_flow_x_delta_rad +
                                PMW_FLOW_COMP_Y_FROM_GYRO_Y_SIGN * new_state.gyro_flow_y_delta_rad;

    /* (4) 机体系速度（去旋转后）。
     *
     * 约定：v_body_x 正 → 机体前进；v_body_y 正 → 机体右移。 */
    {
        float v_body_x = PMW_FLOW_BODY_VX_FROM_FLOW_Y_SIGN * (range_used_cm * new_state.flow_comp_y_rad / dt_s);
        float v_body_y = PMW_FLOW_BODY_VY_FROM_FLOW_X_SIGN * (range_used_cm * new_state.flow_comp_x_rad / dt_s);

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
            g_pmw_flow_vel_x_filt_cms += PMW_FLOW_VEL_LPF_ALPHA * (vx_raw - g_pmw_flow_vel_x_filt_cms);
            g_pmw_flow_vel_y_filt_cms += PMW_FLOW_VEL_LPF_ALPHA * (vy_raw - g_pmw_flow_vel_y_filt_cms);
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
