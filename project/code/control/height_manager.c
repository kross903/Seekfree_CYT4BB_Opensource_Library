#include "control/height_manager.h"
#include "common/project_headfile.h"
#include <math.h>

extern float current_roll;
extern float current_pitch;
extern volatile uint8 armed;

/* TOF_DT 对应 PIT_CH11 的周期（见 All_Init.c 里 TOF_UPDATE_PIT 的 pit_us_init）。
 * 轮询周期改成 10ms 后同步更新此处的差分步长。
 *
 * LPF α 没跟着调：轮询率上升、α 不变意味着等效截止频率等比上升（40Hz→100Hz 大约
 * 1.4Hz→3.5Hz 截止）。如果发现高度读数太跳，把 α 降到 0.1 左右就能把截止重新压回去。 */
#define TOF_DT                  0.010f
#define TOF_HEIGHT_LPF_ALPHA    0.2f
#define TOF_VEL_LPF_ALPHA       0.3f

/* 高度闭环目标高度，以 cm 为单位用于 HeightPID。 */
#define HEIGHT_TARGET_DEFAULT_CM       110.0f
#define HEIGHT_TARGET_START_CM         50.0f
#define HEIGHT_TARGET_ERROR_LIMIT_CM   40.0f
#define HEIGHT_TARGET_RAMP_DURATION_S  3.0f
/* 高度环使能门限：实际高度与目标高度差值不超过此值才允许激活闭环（cm）。 */
#define HEIGHT_LOOP_ENABLE_THRESHOLD_CM  50.0f
#define TOF_VALID_MIN_M         0.05f
#define TOF_VALID_MAX_M         4.0f
#define TOF_JUMP_LIMIT_M        0.15f
#define TOF_NO_UPDATE_TIMEOUT_TICKS  30U   /* 30 * 10ms = 300ms without a fresh sample -> invalid */

/*
 * ToF / optical-flow install compensation.
 *
 * Axis convention used here:
 *   - Y is the body pitch axis.
 *   - ToF is mounted at +Y from body center by 7cm.
 *   - optical flow is mounted at -Y from body center by 7cm.
 *   - ToF beam is tilted 10deg toward +Y.
 *
 * If flight-test shows the correction direction is opposite, change the sign of
 * TOF_SENSOR_OFFSET_Y_CM / FLOW_SENSOR_OFFSET_Y_CM / TOF_SENSOR_TILT_Y_DEG only.
 */
#define TOF_SENSOR_OFFSET_Y_CM       (7.0f)
#define FLOW_SENSOR_OFFSET_Y_CM      (-7.0f)
#define TOF_SENSOR_TILT_Y_DEG        (15.0f)
#define TOF_BEAM_VERTICAL_COS_MIN    (0.20f)

HeightFusionResult_t g_height_result = {0};

PID_Controller HeightPID;

float HeightSpeedPID_output = 0.0f;
float tof_height_true_filt = 0.0f;
float g_height_target_cm = HEIGHT_TARGET_DEFAULT_CM;

/* 高度环使能锁存：一旦条件满足置1，disarm/复位前不再关闭，避免反复开关振荡。 */
static uint8 s_height_loop_enabled = 0U;

static float height_m = 0.0f;
static float tof_height_m = 0.0f;
static float tof_height_true_m = 0.0f;
static float tof_height_true_prev = 0.0f;
static float tof_vel_mps_filt = 0.0f;
static uint8 tof_valid = 0;
static uint8 tof_filter_initialized = 0;
static uint16 tof_no_update_ticks = 0U;
static uint8 s_height_takeoff_target_active = 0U;
static float s_height_takeoff_target_cm = HEIGHT_TARGET_START_CM;
static float s_height_takeoff_target_step_cm = 0.0f;

static void publish_height_result(float height_cm, float velocity_cms)
{
    g_height_result.height_cm = height_cm;
    g_height_result.velocity_cms = velocity_cms;
}

void HeightManager_Set_Target(float target_cm)
{
    if(target_cm < TOF_VALID_MIN_M * 100.0f)
    {
        target_cm = TOF_VALID_MIN_M * 100.0f;
    }
    else if(target_cm > TOF_VALID_MAX_M * 100.0f)
    {
        target_cm = TOF_VALID_MAX_M * 100.0f;
    }

    g_height_target_cm = target_cm;
}

float HeightManager_Get_Target(void)
{
    return g_height_target_cm;
}

float HeightManager_Get_Active_Target(void)
{
    return s_height_takeoff_target_active ? s_height_takeoff_target_cm : g_height_target_cm;
}

static float body_y_down_offset_cm(float offset_y_cm, float roll_deg, float pitch_deg)
{
    float roll_rad = roll_deg * DEG2RAD;
    float pitch_rad = pitch_deg * DEG2RAD;

    return offset_y_cm * sinf(roll_rad) * cosf(pitch_rad);
}

static float tof_beam_vertical_projection(float roll_deg, float pitch_deg)
{
    float roll_rad = roll_deg * DEG2RAD;
    float pitch_rad = pitch_deg * DEG2RAD;
    float tilt_rad = TOF_SENSOR_TILT_Y_DEG * DEG2RAD;
    float projection;

    projection = cosf(pitch_rad) *
                 (sinf(roll_rad) * sinf(tilt_rad) +
                  cosf(roll_rad) * cosf(tilt_rad));

    if(projection < TOF_BEAM_VERTICAL_COS_MIN)
    {
        projection = TOF_BEAM_VERTICAL_COS_MIN;
    }

    return projection;
}

float HeightManager_GetCenterHeightFromTofCm(float tof_los_cm, float roll_deg, float pitch_deg)
{
    float tof_vertical_cm;
    float tof_down_offset_cm;
    float center_height_cm;

    if(tof_los_cm <= 0.0f)
    {
        return 0.0f;
    }

    tof_vertical_cm = tof_los_cm * tof_beam_vertical_projection(roll_deg, pitch_deg);
    tof_down_offset_cm = body_y_down_offset_cm(TOF_SENSOR_OFFSET_Y_CM, roll_deg, pitch_deg);
    center_height_cm = tof_vertical_cm + tof_down_offset_cm;

    return center_height_cm > 0.0f ? center_height_cm : 0.0f;
}

float HeightManager_GetFlowHeightFromTofCm(float tof_los_cm, float roll_deg, float pitch_deg)
{
    float center_height_cm;
    float flow_down_offset_cm;
    float flow_height_cm;

    center_height_cm = HeightManager_GetCenterHeightFromTofCm(tof_los_cm, roll_deg, pitch_deg);
    flow_down_offset_cm = body_y_down_offset_cm(FLOW_SENSOR_OFFSET_Y_CM, roll_deg, pitch_deg);
    flow_height_cm = center_height_cm - flow_down_offset_cm;

    return flow_height_cm > 0.0f ? flow_height_cm : 0.0f;
}

static void update_tof_height_filter(void)
{
    float vel = 0.0f;
    float tof_los_cm = tof_height_m * 100.0f;

    tof_height_true_m = HeightManager_GetCenterHeightFromTofCm(tof_los_cm,
                                                               current_roll,
                                                               current_pitch) * 0.01f;

    if(!tof_filter_initialized)
    {
        tof_height_true_filt = tof_height_true_m;
        tof_height_true_prev = tof_height_true_m;
        tof_vel_mps_filt = 0.0f;
        tof_filter_initialized = 1U;
        return;
    }

    tof_height_true_filt += TOF_HEIGHT_LPF_ALPHA * (tof_height_true_m - tof_height_true_filt);

    vel = (tof_height_true_filt - tof_height_true_prev) / TOF_DT;
    tof_height_true_prev = tof_height_true_filt;
    tof_vel_mps_filt += TOF_VEL_LPF_ALPHA * (vel - tof_vel_mps_filt);
}

void HeightManager_Init(float initial_height_cm)
{
    if(initial_height_cm < 1.0f)
    {
        initial_height_cm = 100.0f;
    }

    height_m = 0.0f;
    tof_height_m = 0.0f;
    tof_height_true_m = 0.0f;
    tof_height_true_filt = initial_height_cm * 0.01f;
    tof_height_true_prev = tof_height_true_filt;
    tof_vel_mps_filt = 0.0f;
    tof_valid = 0;
    tof_filter_initialized = 0;
    tof_no_update_ticks = 0U;
    s_height_loop_enabled = 0U;
    s_height_takeoff_target_active = 0U;
    s_height_takeoff_target_cm = HEIGHT_TARGET_START_CM;
    s_height_takeoff_target_step_cm = 0.0f;

    HeightSpeedPID_output = 0.0f;
    HeightManager_Set_Target(HEIGHT_TARGET_DEFAULT_CM);
    publish_height_result(initial_height_cm, 0.0f);
}

void HeightManager_Task_10ms(void)
{
    uint8 tof_new_sample = 0U;

    dl1b_get_distance();

    if(dl1b_finsh_flag && dl1b_distance_mm <= 4000)
    {
        height_m = dl1b_distance_mm * 0.001f;
        tof_new_sample = 1U;
        tof_no_update_ticks = 0U;

        if(tof_valid && fabsf(height_m - tof_height_m) > TOF_JUMP_LIMIT_M)
        {
            height_m = tof_height_m;
        }

        if(height_m >= TOF_VALID_MIN_M && height_m <= TOF_VALID_MAX_M)
        {
            tof_height_m = height_m;
            tof_valid = 1U;
        }
        else
        {
            tof_valid = 0U;
        }
    }
    else
    {
        /* dl1b_finsh_flag=0 can mean either "not ready yet" or a real invalid sample.
         * dl1b_get_distance() now keeps the last distance for "not ready" and only
         * writes 8192 for real invalid/out-of-range cases, so use distance to tell
         * them apart. */
        if(dl1b_distance_mm > 4000U)
        {
            tof_valid = 0U;
            tof_no_update_ticks = 0U;
        }
        else
        {
            if(tof_no_update_ticks < 0xFFFFU)
            {
                tof_no_update_ticks++;
            }
            if(tof_no_update_ticks >= TOF_NO_UPDATE_TIMEOUT_TICKS)
            {
                tof_valid = 0U;
            }
        }
    }

    if(tof_valid)
    {
        if(tof_new_sample)
        {
            update_tof_height_filter();
        }
        else
        {
            tof_height_true_prev = tof_height_true_filt;
            tof_vel_mps_filt = 0.0f;
        }
        publish_height_result(tof_height_true_filt * 100.0f, tof_vel_mps_filt * 100.0f);
    }
    else
    {
        tof_height_true_prev = tof_height_true_filt;
        tof_vel_mps_filt = 0.0f;
        publish_height_result(tof_height_true_filt * 100.0f, 0.0f);
    }

    /* 高度环条件门控：
     * 1. ToF 必须有效（tof_valid）
     * 2. armed 状态下，实际高度与目标高度差值首次进入 ±HEIGHT_LOOP_ENABLE_THRESHOLD_CM
     *    时锁存使能，之后保持激活直到 disarm 或复位（HeightManager_Init）。
     * 3. ToF 临时失效时输出清零、积分重置，但锁存不清除（恢复后继续闭环）。
     */
    if(!armed)
    {
        /* disarm：复位锁存，等待下次起飞重新触发 */
        s_height_loop_enabled = 0U;
        s_height_takeoff_target_active = 0U;
        s_height_takeoff_target_cm = HEIGHT_TARGET_START_CM;
        HeightSpeedPID_output = 0.0f;
        PID_Reset(&HeightPID);
        return;
    }

    if(!tof_valid)
    {
        /* 传感器失效：清零输出和积分，保留锁存状态 */
        HeightSpeedPID_output = 0.0f;
        PID_Reset(&HeightPID);
        return;
    }

    {
        float actual_cm = tof_height_true_filt * 100.0f;
        float target_cm = g_height_target_cm;
        float error_cm = 0.0f;

        if(base_throttle < FLIGHT_TAKEOFF_RAMP_TARGET_THROTTLE)
        {
            s_height_loop_enabled = 0U;
            s_height_takeoff_target_active = 0U;
            s_height_takeoff_target_cm = HEIGHT_TARGET_START_CM;
            HeightSpeedPID_output = 0.0f;
            PID_Reset(&HeightPID);
            return;
        }

        if(!s_height_loop_enabled)
        {
            s_height_loop_enabled = 1U;
            s_height_takeoff_target_active = 1U;
            s_height_takeoff_target_cm = HEIGHT_TARGET_START_CM;
            s_height_takeoff_target_step_cm = (g_height_target_cm - HEIGHT_TARGET_START_CM) /
                                              (HEIGHT_TARGET_RAMP_DURATION_S / TOF_DT);
            PID_Reset(&HeightPID);
        }

        if(s_height_takeoff_target_active)
        {
            s_height_takeoff_target_cm += s_height_takeoff_target_step_cm;

            if((s_height_takeoff_target_step_cm >= 0.0f && s_height_takeoff_target_cm >= g_height_target_cm) ||
               (s_height_takeoff_target_step_cm < 0.0f && s_height_takeoff_target_cm <= g_height_target_cm))
            {
                s_height_takeoff_target_cm = g_height_target_cm;
                s_height_takeoff_target_active = 0U;
            }

            target_cm = s_height_takeoff_target_cm;
        }

        error_cm = target_cm - actual_cm;
        if(error_cm > HEIGHT_TARGET_ERROR_LIMIT_CM)
        {
            error_cm = HEIGHT_TARGET_ERROR_LIMIT_CM;
        }
        else if(error_cm < -HEIGHT_TARGET_ERROR_LIMIT_CM)
        {
            error_cm = -HEIGHT_TARGET_ERROR_LIMIT_CM;
        }

        HeightSpeedPID_output = PID_Calculate(&HeightPID, actual_cm + error_cm, actual_cm, TOF_DT);
    }
}
