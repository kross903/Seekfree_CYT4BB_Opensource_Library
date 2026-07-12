#include "control/height_manager.h"
#include "common/project_headfile.h"
#include <math.h>
#include <string.h>

extern float current_roll;
extern float current_pitch;
extern volatile uint8 armed;

#define TOF_DT                            0.010f
#define TOF_HEIGHT_LPF_ALPHA              0.20f
#define TOF_VEL_LPF_ALPHA                 0.15f
#define TOF_VEL_NO_SAMPLE_DECAY           0.90f
#define TOF_MEDIAN_WINDOW                 3U

#define HEIGHT_CASCADE_ENABLE             1U
#define HEIGHT_TARGET_DEFAULT_CM          110.0f
#define HEIGHT_TARGET_START_CM            50.0f
#define HEIGHT_TARGET_ERROR_LIMIT_CM      40.0f
#define HEIGHT_TARGET_RAMP_DURATION_S     3.0f
#define HEIGHT_VEL_TARGET_LIMIT_CMS       40.0f

#define HEIGHT_ACC_GRAVITY_BIAS_MG        997.0f
#define HEIGHT_ACC_CMS2_PER_MG            0.981f
#define HEIGHT_ACC_DEADBAND_MG            6.0f
#define HEIGHT_ACC_MAX_CMS2               250.0f
#define HEIGHT_ACC_VEL_LPF_ALPHA          0.10f
#define HEIGHT_ACC_VEL_LEAK               0.995f
#define HEIGHT_ACC_ATTITUDE_GATE_DEG      25.0f
#define HEIGHT_ACC_NORM_GATE_MG           400.0f
#define HEIGHT_VEL_FUSION_ACC_ALPHA       0.20f

#define TOF_VALID_MIN_M                   0.00f
#define TOF_VALID_MAX_M                   2.0f
#define TOF_JUMP_LIMIT_M                  0.15f
#define TOF_NO_UPDATE_TIMEOUT_TICKS       10U

/*
 * ToF / optical-flow install compensation.
 *
 * Axis convention used here:
 *   - Y is the body pitch axis.
 *   - ToF is mounted at +Y from body center by 7cm.
 *   - optical flow is mounted at -Y from body center by 7cm.
 *   - ToF beam is tilted toward +Y.
 */
#define TOF_SENSOR_OFFSET_Y_CM            (7.0f)
#define FLOW_SENSOR_OFFSET_Y_CM           (-7.0f)
#define TOF_SENSOR_TILT_Y_DEG             (15.0f)
#define TOF_BEAM_VERTICAL_COS_MIN         (0.20f)

HeightFusionResult_t g_height_result = {0};

PID_Controller HeightPID;
PID_Controller HeightVelPID;

float HeightSpeedPID_output = 0.0f;
float tof_height_true_filt = 0.0f;
float g_height_target_cm = HEIGHT_TARGET_DEFAULT_CM;

static HeightManagerDiag_t s_height_diag = {0};

static uint8 s_height_loop_enabled = 0U;

static float height_m = 0.0f;
static float tof_height_m = 0.0f;
static float tof_height_true_m = 0.0f;
static float tof_height_true_prev = 0.0f;
static float tof_vel_mps_filt = 0.0f;
static float tof_sample_dt_s = TOF_DT;
static uint8 tof_valid = 0U;
static uint8 tof_filter_initialized = 0U;
static uint16 tof_no_update_ticks = 0U;

static float s_tof_median_history_cm[TOF_MEDIAN_WINDOW] = {0.0f};
static uint8 s_tof_median_count = 0U;
static uint8 s_tof_median_index = 0U;

static float s_acc_velocity_cms = 0.0f;
static float s_acc_velocity_filt_cms = 0.0f;

static uint8 s_height_takeoff_target_active = 0U;
static float s_height_takeoff_target_cm = HEIGHT_TARGET_START_CM;
static float s_height_takeoff_target_step_cm = 0.0f;
static uint8 s_soft_takeoff_prepared = 0U;
static uint8 s_soft_takeoff_active = 0U;
static float s_soft_takeoff_output_blend = 1.0f;

static float height_clamp_float(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }
    if(value < min_value)
    {
        return min_value;
    }
    return value;
}

static float height_median3(float a, float b, float c)
{
    float t;

    if(a > b)
    {
        t = a;
        a = b;
        b = t;
    }
    if(b > c)
    {
        t = b;
        b = c;
        c = t;
    }
    if(a > b)
    {
        t = a;
        a = b;
        b = t;
    }

    return b;
}

static void height_reset_tof_median_filter(void)
{
    uint8 i;

    for(i = 0U; i < TOF_MEDIAN_WINDOW; i++)
    {
        s_tof_median_history_cm[i] = 0.0f;
    }

    s_tof_median_count = 0U;
    s_tof_median_index = 0U;
}

static float height_update_tof_median(float height_cm)
{
    s_tof_median_history_cm[s_tof_median_index] = height_cm;
    s_tof_median_index++;
    if(s_tof_median_index >= TOF_MEDIAN_WINDOW)
    {
        s_tof_median_index = 0U;
    }

    if(s_tof_median_count < TOF_MEDIAN_WINDOW)
    {
        s_tof_median_count++;
    }

    if(s_tof_median_count < TOF_MEDIAN_WINDOW)
    {
        return height_cm;
    }

    return height_median3(s_tof_median_history_cm[0],
                          s_tof_median_history_cm[1],
                          s_tof_median_history_cm[2]);
}

static void height_clear_control_diag(void)
{
    s_height_diag.target_velocity_cms = 0.0f;
    s_height_diag.velocity_error_cms = 0.0f;
    s_height_diag.height_pid_output_cms = 0.0f;
    s_height_diag.velocity_pid_output = 0.0f;
    s_height_diag.cascade_enabled = HEIGHT_CASCADE_ENABLE;
}

static void height_reset_velocity_estimator(void)
{
    s_acc_velocity_cms = 0.0f;
    s_acc_velocity_filt_cms = 0.0f;
    s_height_diag.acc_velocity_cms = 0.0f;
    s_height_diag.fused_velocity_cms = tof_vel_mps_filt * 100.0f;
}

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

void HeightManager_Get_Diag(HeightManagerDiag_t *diag)
{
    if(diag != NULL)
    {
        *diag = s_height_diag;
    }
}

uint8 HeightManager_IsTofValid(void)
{
    return tof_valid;
}

uint8 HeightManager_IsReadyForTakeoff(void)
{
    return (uint8)(tof_valid &&
                   tof_filter_initialized &&
                   tof_no_update_ticks < TOF_NO_UPDATE_TIMEOUT_TICKS);
}

void HeightManager_PrepareSoftTakeoff(void)
{
    if(!s_soft_takeoff_prepared || s_soft_takeoff_active)
    {
        s_height_loop_enabled = 0U;
        s_height_takeoff_target_active = 0U;
        s_height_takeoff_target_cm = g_height_result.height_cm;
        s_height_takeoff_target_step_cm = 0.0f;
        PID_Reset(&HeightPID);
        PID_Reset(&HeightVelPID);
        height_reset_velocity_estimator();
        height_clear_control_diag();
    }

    s_soft_takeoff_prepared = 1U;
    s_soft_takeoff_active = 0U;
    s_soft_takeoff_output_blend = 0.0f;
    HeightSpeedPID_output = 0.0f;
}

void HeightManager_BeginSoftTakeoff(void)
{
    float start_height_cm = g_height_result.height_cm;

    HeightManager_Set_Target(HEIGHT_TARGET_DEFAULT_CM);

    if(start_height_cm < TOF_VALID_MIN_M * 100.0f)
    {
        start_height_cm = TOF_VALID_MIN_M * 100.0f;
    }
    else if(start_height_cm > TOF_VALID_MAX_M * 100.0f)
    {
        start_height_cm = TOF_VALID_MAX_M * 100.0f;
    }

    s_soft_takeoff_prepared = 1U;
    s_soft_takeoff_active = 1U;
    s_soft_takeoff_output_blend = 0.0f;
    s_height_loop_enabled = 1U;
    s_height_takeoff_target_active = 1U;
    s_height_takeoff_target_cm = start_height_cm;
    s_height_takeoff_target_step_cm = (g_height_target_cm - start_height_cm) /
                                      (HEIGHT_TARGET_RAMP_DURATION_S / TOF_DT);
    HeightSpeedPID_output = 0.0f;
    PID_Reset(&HeightPID);
    PID_Reset(&HeightVelPID);
    height_reset_velocity_estimator();
    height_clear_control_diag();
}

void HeightManager_SetOutputBlend(float blend)
{
    s_soft_takeoff_output_blend = height_clamp_float(blend, 0.0f, 1.0f);
}

void HeightManager_AbortSoftTakeoff(void)
{
    s_soft_takeoff_prepared = 0U;
    s_soft_takeoff_active = 0U;
    s_soft_takeoff_output_blend = 1.0f;
    s_height_loop_enabled = 0U;
    s_height_takeoff_target_active = 0U;
    s_height_takeoff_target_cm = HEIGHT_TARGET_START_CM;
    s_height_takeoff_target_step_cm = 0.0f;
    HeightSpeedPID_output = 0.0f;
    PID_Reset(&HeightPID);
    PID_Reset(&HeightVelPID);
    height_reset_velocity_estimator();
    height_clear_control_diag();
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
    float tof_comp_cm;
    float tof_median_cm;

    tof_comp_cm = HeightManager_GetCenterHeightFromTofCm(tof_los_cm,
                                                         current_roll,
                                                         current_pitch);
    tof_median_cm = height_update_tof_median(tof_comp_cm);
    tof_height_true_m = tof_median_cm * 0.01f;

    s_height_diag.tof_raw_cm = tof_los_cm;
    s_height_diag.tof_comp_cm = tof_comp_cm;
    s_height_diag.tof_median_cm = tof_median_cm;

    if(!tof_filter_initialized)
    {
        tof_height_true_filt = tof_height_true_m;
        tof_height_true_prev = tof_height_true_m;
        tof_vel_mps_filt = 0.0f;
        tof_sample_dt_s = TOF_DT;
        tof_filter_initialized = 1U;
    }
    else
    {
        tof_height_true_filt += TOF_HEIGHT_LPF_ALPHA * (tof_height_true_m - tof_height_true_filt);

        vel = (tof_height_true_filt - tof_height_true_prev) / tof_sample_dt_s;
        tof_height_true_prev = tof_height_true_filt;
        tof_sample_dt_s = TOF_DT;
        tof_vel_mps_filt += TOF_VEL_LPF_ALPHA * (vel - tof_vel_mps_filt);
    }

    s_height_diag.tof_height_filt_cm = tof_height_true_filt * 100.0f;
    s_height_diag.tof_velocity_cms = tof_vel_mps_filt * 100.0f;
}

static void update_acc_velocity_estimator(uint8 estimator_valid)
{
    float ax_level_mg = 0.0f;
    float ay_level_mg = 0.0f;
    float az_level_mg = 0.0f;
    float acc_norm_mg;
    float acc_z_mg;
    float acc_z_cms2;
    uint8 allow_integrate;

    Body_To_RollPitch_Level_Frame(Roll_acc_F, Pitch_acc_F, Yaw_acc_F,
                                  &ax_level_mg, &ay_level_mg, &az_level_mg);
    (void)ax_level_mg;
    (void)ay_level_mg;

    acc_norm_mg = sqrtf(Roll_acc_F * Roll_acc_F +
                        Pitch_acc_F * Pitch_acc_F +
                        Yaw_acc_F * Yaw_acc_F);

    acc_z_mg = az_level_mg - HEIGHT_ACC_GRAVITY_BIAS_MG;
    if(fabsf(acc_z_mg) < HEIGHT_ACC_DEADBAND_MG)
    {
        acc_z_mg = 0.0f;
    }

    acc_z_cms2 = height_clamp_float(acc_z_mg * HEIGHT_ACC_CMS2_PER_MG,
                                    -HEIGHT_ACC_MAX_CMS2,
                                    HEIGHT_ACC_MAX_CMS2);

    allow_integrate = (uint8)(armed &&
                              base_throttle >= FLIGHT_TAKEOFF_RAMP_TARGET_THROTTLE &&
                              estimator_valid &&
                              fabsf(current_roll) <= HEIGHT_ACC_ATTITUDE_GATE_DEG &&
                              fabsf(current_pitch) <= HEIGHT_ACC_ATTITUDE_GATE_DEG &&
                              fabsf(acc_norm_mg - HEIGHT_ACC_GRAVITY_BIAS_MG) <= HEIGHT_ACC_NORM_GATE_MG);

    if(allow_integrate)
    {
        s_acc_velocity_cms += acc_z_cms2 * TOF_DT;
        s_acc_velocity_cms *= HEIGHT_ACC_VEL_LEAK;
        s_acc_velocity_filt_cms += HEIGHT_ACC_VEL_LPF_ALPHA *
                                   (s_acc_velocity_cms - s_acc_velocity_filt_cms);
        s_height_diag.acc_used = 1U;
    }
    else
    {
        s_acc_velocity_cms = 0.0f;
        s_acc_velocity_filt_cms = 0.0f;
        s_height_diag.acc_used = 0U;
    }

    s_height_diag.acc_z_level_mg = az_level_mg;
    s_height_diag.gravity_bias_mg = HEIGHT_ACC_GRAVITY_BIAS_MG;
    s_height_diag.acc_z_cms2 = acc_z_cms2;
    s_height_diag.acc_velocity_cms = s_acc_velocity_filt_cms;
}

static void publish_estimator_result(void)
{
    float tof_velocity_cms = tof_vel_mps_filt * 100.0f;
    float fused_velocity_cms = tof_velocity_cms;

    if(s_height_diag.acc_used)
    {
        fused_velocity_cms = HEIGHT_VEL_FUSION_ACC_ALPHA * s_acc_velocity_filt_cms +
                             (1.0f - HEIGHT_VEL_FUSION_ACC_ALPHA) * tof_velocity_cms;
    }

    s_height_diag.tof_valid = tof_valid;
    s_height_diag.tof_height_filt_cm = tof_height_true_filt * 100.0f;
    s_height_diag.tof_velocity_cms = tof_velocity_cms;
    s_height_diag.fused_velocity_cms = fused_velocity_cms;

    publish_height_result(tof_height_true_filt * 100.0f, fused_velocity_cms);
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
    tof_sample_dt_s = TOF_DT;
    tof_valid = 0U;
    tof_filter_initialized = 0U;
    tof_no_update_ticks = 0U;
    s_height_loop_enabled = 0U;
    s_height_takeoff_target_active = 0U;
    s_height_takeoff_target_cm = HEIGHT_TARGET_START_CM;
    s_height_takeoff_target_step_cm = 0.0f;
    s_soft_takeoff_prepared = 0U;
    s_soft_takeoff_active = 0U;
    s_soft_takeoff_output_blend = 1.0f;

    memset(&s_height_diag, 0, sizeof(s_height_diag));
    s_height_diag.gravity_bias_mg = HEIGHT_ACC_GRAVITY_BIAS_MG;
    s_height_diag.tof_height_filt_cm = initial_height_cm;
    s_height_diag.cascade_enabled = HEIGHT_CASCADE_ENABLE;

    height_reset_tof_median_filter();
    height_reset_velocity_estimator();

    HeightSpeedPID_output = 0.0f;
    HeightManager_Set_Target(HEIGHT_TARGET_DEFAULT_CM);
    PID_Reset(&HeightPID);
    PID_Reset(&HeightVelPID);
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
        /* DL1B 轮询快于传感器出数时，驱动会用 finish=0/8192 表示“当前没有新帧”。
         * 这不是测距立即失效；保留上一有效值，连续超时后再置无效。 */
        if(tof_no_update_ticks < 0xFFFFU)
        {
            tof_no_update_ticks++;
        }
        if(tof_no_update_ticks >= TOF_NO_UPDATE_TIMEOUT_TICKS)
        {
            tof_valid = 0U;
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
            tof_sample_dt_s += TOF_DT;
            if(tof_sample_dt_s > 0.20f)
            {
                tof_sample_dt_s = 0.20f;
            }
            tof_vel_mps_filt *= TOF_VEL_NO_SAMPLE_DECAY;
            s_height_diag.tof_velocity_cms = tof_vel_mps_filt * 100.0f;
            s_height_diag.tof_height_filt_cm = tof_height_true_filt * 100.0f;
        }
    }
    else
    {
        tof_height_true_prev = tof_height_true_filt;
        tof_vel_mps_filt = 0.0f;
        tof_sample_dt_s = TOF_DT;
        tof_filter_initialized = 0U;
        height_reset_tof_median_filter();
    }

    update_acc_velocity_estimator(tof_valid);
    publish_estimator_result();

    if(!armed)
    {
        s_soft_takeoff_prepared = 0U;
        s_soft_takeoff_active = 0U;
        s_soft_takeoff_output_blend = 1.0f;
        s_height_loop_enabled = 0U;
        s_height_takeoff_target_active = 0U;
        s_height_takeoff_target_cm = HEIGHT_TARGET_START_CM;
        HeightSpeedPID_output = 0.0f;
        PID_Reset(&HeightPID);
        PID_Reset(&HeightVelPID);
        height_clear_control_diag();
        return;
    }

    if(!tof_valid)
    {
        HeightSpeedPID_output = 0.0f;
        PID_Reset(&HeightPID);
        PID_Reset(&HeightVelPID);
        height_reset_velocity_estimator();
        height_clear_control_diag();
        return;
    }

    {
        float actual_cm = g_height_result.height_cm;
        float target_cm = g_height_target_cm;
        float error_cm;

        if(s_soft_takeoff_prepared && !s_soft_takeoff_active)
        {
            s_height_loop_enabled = 0U;
            s_height_takeoff_target_active = 0U;
            s_height_takeoff_target_cm = actual_cm;
            HeightSpeedPID_output = 0.0f;
            PID_Reset(&HeightPID);
            PID_Reset(&HeightVelPID);
            height_reset_velocity_estimator();
            height_clear_control_diag();
            return;
        }

        if(!s_soft_takeoff_active && base_throttle < FLIGHT_TAKEOFF_RAMP_TARGET_THROTTLE)
        {
            s_height_loop_enabled = 0U;
            s_height_takeoff_target_active = 0U;
            s_height_takeoff_target_cm = HEIGHT_TARGET_START_CM;
            HeightSpeedPID_output = 0.0f;
            PID_Reset(&HeightPID);
            PID_Reset(&HeightVelPID);
            height_reset_velocity_estimator();
            height_clear_control_diag();
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
            PID_Reset(&HeightVelPID);
            height_reset_velocity_estimator();
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
        error_cm = height_clamp_float(error_cm,
                                      -HEIGHT_TARGET_ERROR_LIMIT_CM,
                                      HEIGHT_TARGET_ERROR_LIMIT_CM);

#if HEIGHT_CASCADE_ENABLE
        {
            float height_pid_vz_cms;
            float target_vz_cms;
            float raw_velocity_output;

            height_pid_vz_cms = PID_Calculate(&HeightPID,
                                              actual_cm + error_cm,
                                              actual_cm,
                                              TOF_DT);
            target_vz_cms = height_clamp_float(height_pid_vz_cms,
                                               -HEIGHT_VEL_TARGET_LIMIT_CMS,
                                               HEIGHT_VEL_TARGET_LIMIT_CMS);

            raw_velocity_output = PID_Calculate(&HeightVelPID,
                                                target_vz_cms,
                                                g_height_result.velocity_cms,
                                                TOF_DT);
            HeightSpeedPID_output = raw_velocity_output *
                                    (s_soft_takeoff_active ? s_soft_takeoff_output_blend : 1.0f);

            s_height_diag.height_pid_output_cms = height_pid_vz_cms;
            s_height_diag.target_velocity_cms = target_vz_cms;
            s_height_diag.velocity_error_cms = target_vz_cms - g_height_result.velocity_cms;
            s_height_diag.velocity_pid_output = HeightSpeedPID_output;
            s_height_diag.cascade_enabled = 1U;
        }
#else
        HeightSpeedPID_output = PID_Calculate(&HeightPID,
                                              actual_cm + error_cm,
                                              actual_cm,
                                              TOF_DT) *
                                (s_soft_takeoff_active ? s_soft_takeoff_output_blend : 1.0f);
        s_height_diag.height_pid_output_cms = HeightSpeedPID_output;
        s_height_diag.target_velocity_cms = 0.0f;
        s_height_diag.velocity_error_cms = 0.0f;
        s_height_diag.velocity_pid_output = HeightSpeedPID_output;
        s_height_diag.cascade_enabled = 0U;
#endif
    }
}
