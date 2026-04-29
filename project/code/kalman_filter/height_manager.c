#include "height_manager.h"
#include "common/project_headfile.h"
#include <math.h>

extern float current_roll;
extern float current_pitch;

/* TOF_DT 对应 PIT_CH11 的周期（见 All_Init.c 里 TOF_UPDATE_PIT 的 pit_us_init）。
 * 轮询周期改成 10ms 后同步更新此处的差分步长。
 *
 * LPF α 没跟着调：轮询率上升、α 不变意味着等效截止频率等比上升（40Hz→100Hz 大约
 * 1.4Hz→3.5Hz 截止）。如果发现高度读数太跳，把 α 降到 0.1 左右就能把截止重新压回去。 */
#define TOF_DT                  0.010f
#define TOF_HEIGHT_LPF_ALPHA    0.2f
#define TOF_VEL_LPF_ALPHA       0.3f

/* 高度闭环目标高度 1.38m，以 cm 为单位用于 HeightPID。 */
#define HEIGHT_TARGET_CM        138.0f
#define TOF_VALID_MIN_M         0.05f
#define TOF_VALID_MAX_M         2.0f
#define TOF_JUMP_LIMIT_M        0.10f

HeightFusionResult_t g_height_result = {0};

PID_Controller HeightPID;

float HeightSpeedPID_output = 0.0f;
float tof_height_true_filt = 0.0f;

static float height_m = 0.0f;
static float tof_height_m = 0.0f;
static float tof_height_true_m = 0.0f;
static float tof_height_true_prev = 0.0f;
static float tof_vel_mps_filt = 0.0f;
static uint8 tof_valid = 0;
static uint8 tof_filter_initialized = 0;

static void publish_height_result(float height_cm, float velocity_cms)
{
    g_height_result.height_cm = height_cm;
    g_height_result.velocity_cms = velocity_cms;
}

static void update_tof_height_filter(void)
{
    float roll_rad = current_roll * DEG2RAD;
    float pitch_rad = current_pitch * DEG2RAD;
    float tilt_cos = cosf(roll_rad) * cosf(pitch_rad);
    float vel = 0.0f;

    tof_height_true_m = tof_height_m * tilt_cos;

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

    HeightSpeedPID_output = 0.0f;
    publish_height_result(initial_height_cm, 0.0f);
}

void HeightManager_Task_10ms(void)
{
    dl1b_get_distance();

    if(dl1b_finsh_flag && dl1b_distance_mm <= 4000)
    {
        height_m = dl1b_distance_mm * 0.001f;

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
        tof_valid = 0U;
    }

    if(tof_valid)
    {
        update_tof_height_filter();
        publish_height_result(tof_height_true_filt * 100.0f, tof_vel_mps_filt * 100.0f);
    }
    else
    {
        tof_height_true_prev = tof_height_true_filt;
        tof_vel_mps_filt = 0.0f;
        publish_height_result(tof_height_true_filt * 100.0f, 0.0f);
    }

    /* 高度闭环 PID：解锁且 ToF 有效时反馈到油门混合，否则强制清零并复位积分。
     * Why: disarm/失效情况下若仍计算输出，base_throttle 起跳瞬间会把残余积分一次抛出。 */
    if(armed && tof_valid)
    {
        HeightSpeedPID_output = PID_Calculate(&HeightPID,
                                              HEIGHT_TARGET_CM,
                                              tof_height_true_filt * 100.0f,
                                              TOF_DT);
    }
    else
    {
        HeightSpeedPID_output = 0.0f;
        PID_Reset(&HeightPID);
    }
}
