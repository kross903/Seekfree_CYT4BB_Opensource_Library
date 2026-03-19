#include "height_manager.h"
#include "ControlMath.h"
#include "ir_localization.h"
#include "zf_device_dl1b.h"
#include <math.h>

extern float current_roll;
extern float current_pitch;

/* -------------------------------------------------------------------------- */
/* 高度参数配置                                                               */
/* -------------------------------------------------------------------------- */

#define TOF_UPDATE_HZ           40
#define TOF_UPDATE_DIV          (4000 / TOF_UPDATE_HZ)
#define TOF_DT                  0.025f
#define TOF_HEIGHT_LPF_ALPHA    0.2f
#define TOF_VEL_LPF_ALPHA       0.3f
#define TOF_VALID_MIN_M         0.05f
#define TOF_VALID_MAX_M         2.0f

#define HF_TOF_R                4.0f
#define HF_IR_R                 25.0f
#define HF_Q_HEIGHT             0.1f
#define HF_Q_VELOCITY           1.0f

#define HF_TOF_MIN_CM           5.0f
#define HF_TOF_MAX_CM           200.0f
#define HF_IR_MIN_CM            10.0f
#define HF_IR_MAX_CM            300.0f
#define HF_OUTLIER_THRESH       30.0f

/* -------------------------------------------------------------------------- */
/* 高度对外对象                                                               */
/* -------------------------------------------------------------------------- */

HeightFusionResult_t g_height_result = {0};

PID_Controller HeightPID;
PID_Controller HeightSpeedPID;

float HeightSpeedPID_output = 0.0f;
float tof_height_true_filt = 0.0f;

/* -------------------------------------------------------------------------- */
/* 高度内部运行状态                                                           */
/* -------------------------------------------------------------------------- */

typedef struct
{
    float x[2];
    float P[2][2];
    float Q[2][2];
    float R_tof;
    float R_ir;
    uint8 initialized;
    uint8 tof_valid;
    uint8 ir_valid;
    uint32 tof_update_count;
    uint32 ir_update_count;
    uint32 outlier_count;
} HeightFusionState_t;

static HeightFusionState_t g_height_fusion = {{0}};

static uint16 tof_divider = 0;
static float height_m = 0.0f;
static float tof_height_m = 0.0f;
static float tof_height_true_m = 0.0f;
static float tof_height_true_prev = 0.0f;
static float tof_vel_mps_filt = 0.0f;
static uint8 tof_valid = 0;

/* -------------------------------------------------------------------------- */
/* 高度融合核心                                                               */
/* -------------------------------------------------------------------------- */

static void fusion_publish_result(uint8 confidence)
{
    g_height_result.height_cm = g_height_fusion.x[0];
    g_height_result.velocity_cms = g_height_fusion.x[1];
    g_height_result.height_std = sqrtf(g_height_fusion.P[0][0]);
    g_height_result.confidence = confidence;
}

static void fusion_predict(float dt)
{
    float h_prior = 0.0f;
    float v_prior = 0.0f;
    float p00 = 0.0f;
    float p01 = 0.0f;
    float p10 = 0.0f;
    float p11 = 0.0f;

    if(!g_height_fusion.initialized)
    {
        return;
    }

    if(dt < 0.001f)
    {
        dt = 0.001f;
    }
    if(dt > 1.0f)
    {
        dt = 1.0f;
    }

    g_height_fusion.tof_valid = 0;
    g_height_fusion.ir_valid = 0;

    h_prior = g_height_fusion.x[0] + g_height_fusion.x[1] * dt;
    v_prior = g_height_fusion.x[1];

    p00 = g_height_fusion.P[0][0]
        + dt * g_height_fusion.P[1][0]
        + dt * (g_height_fusion.P[0][1] + dt * g_height_fusion.P[1][1])
        + g_height_fusion.Q[0][0];
    p01 = g_height_fusion.P[0][1] + dt * g_height_fusion.P[1][1];
    p10 = g_height_fusion.P[1][0] + dt * g_height_fusion.P[1][1];
    p11 = g_height_fusion.P[1][1] + g_height_fusion.Q[1][1];

    if(p00 > 10000.0f)
    {
        p00 = 10000.0f;
    }
    if(p11 > 10000.0f)
    {
        p11 = 10000.0f;
    }

    g_height_fusion.x[0] = (h_prior < 0.0f) ? 0.0f : h_prior;
    g_height_fusion.x[1] = v_prior;
    g_height_fusion.P[0][0] = p00;
    g_height_fusion.P[0][1] = p01;
    g_height_fusion.P[1][0] = p10;
    g_height_fusion.P[1][1] = p11;

    fusion_publish_result(30);
}

static float fusion_update(float measurement, float noise_r)
{
    float innovation = measurement - g_height_fusion.x[0];
    float s = g_height_fusion.P[0][0] + noise_r;
    float k0 = 0.0f;
    float k1 = 0.0f;
    float p00_new = 0.0f;
    float p01_new = 0.0f;
    float p10_new = 0.0f;
    float p11_new = 0.0f;

    if(s < 0.001f)
    {
        s = 0.001f;
    }

    k0 = g_height_fusion.P[0][0] / s;
    k1 = g_height_fusion.P[1][0] / s;

    if(k0 < 0.0f)
    {
        k0 = 0.0f;
    }
    if(k0 > 1.0f)
    {
        k0 = 1.0f;
    }
    if(k1 < -1.0f)
    {
        k1 = -1.0f;
    }
    if(k1 > 1.0f)
    {
        k1 = 1.0f;
    }

    g_height_fusion.x[0] = g_height_fusion.x[0] + k0 * innovation;
    g_height_fusion.x[1] = g_height_fusion.x[1] + k1 * innovation;

    p00_new = (1.0f - k0) * g_height_fusion.P[0][0];
    p01_new = (1.0f - k0) * g_height_fusion.P[0][1];
    p10_new = -k1 * g_height_fusion.P[0][0] + g_height_fusion.P[1][0];
    p11_new = -k1 * g_height_fusion.P[0][1] + g_height_fusion.P[1][1];

    if(p00_new < 0.001f)
    {
        p00_new = 0.001f;
    }
    if(p11_new < 0.001f)
    {
        p11_new = 0.001f;
    }

    g_height_fusion.P[0][0] = p00_new;
    g_height_fusion.P[0][1] = p01_new;
    g_height_fusion.P[1][0] = p10_new;
    g_height_fusion.P[1][1] = p11_new;

    if(g_height_fusion.x[0] < 0.0f)
    {
        g_height_fusion.x[0] = 0.0f;
    }

    return g_height_fusion.x[0];
}

static void fusion_init(float initial_height_cm)
{
    if(initial_height_cm < 1.0f)
    {
        initial_height_cm = 100.0f;
    }

    g_height_fusion = (HeightFusionState_t){{0}};

    g_height_fusion.x[0] = initial_height_cm;
    g_height_fusion.x[1] = 0.0f;

    g_height_fusion.P[0][0] = 25.0f;
    g_height_fusion.P[0][1] = 0.0f;
    g_height_fusion.P[1][0] = 0.0f;
    g_height_fusion.P[1][1] = 25.0f;

    g_height_fusion.Q[0][0] = HF_Q_HEIGHT;
    g_height_fusion.Q[0][1] = 0.0f;
    g_height_fusion.Q[1][0] = 0.0f;
    g_height_fusion.Q[1][1] = HF_Q_VELOCITY;

    g_height_fusion.R_tof = HF_TOF_R;
    g_height_fusion.R_ir = HF_IR_R;
    g_height_fusion.initialized = 1;

    g_height_result.height_cm = initial_height_cm;
    g_height_result.velocity_cms = 0.0f;
    g_height_result.height_std = 5.0f;
    g_height_result.confidence = 50;
}

static float fusion_update_tof(float tof_height_cm, float dt)
{
    float diff = 0.0f;

    (void)dt;

    if(tof_height_cm <= 0.0f || !isfinite(tof_height_cm))
    {
        g_height_fusion.tof_valid = 0;
        return g_height_fusion.x[0];
    }

    if(!g_height_fusion.initialized)
    {
        fusion_init(tof_height_cm);
        return tof_height_cm;
    }

    if(tof_height_cm < HF_TOF_MIN_CM || tof_height_cm > HF_TOF_MAX_CM)
    {
        g_height_fusion.tof_valid = 0;
        fusion_publish_result(50);
        return g_height_fusion.x[0];
    }

    if(g_height_fusion.tof_update_count > 10)
    {
        diff = fabsf(tof_height_cm - g_height_fusion.x[0]);
        if(diff > HF_OUTLIER_THRESH)
        {
            g_height_fusion.outlier_count++;
            g_height_fusion.tof_valid = 0;
            fusion_publish_result(50);
            return g_height_fusion.x[0];
        }
    }

    g_height_fusion.tof_valid = 1;
    g_height_fusion.tof_update_count++;

    (void)fusion_update(tof_height_cm, g_height_fusion.R_tof);

    if(g_height_fusion.tof_valid && g_height_fusion.ir_valid)
    {
        fusion_publish_result(95);
    }
    else
    {
        fusion_publish_result(80);
    }

    return g_height_fusion.x[0];
}

static float fusion_update_ir(float ir_height_cm, float dt)
{
    float diff = 0.0f;

    (void)dt;

    if(ir_height_cm <= 0.0f || !isfinite(ir_height_cm))
    {
        g_height_fusion.ir_valid = 0;
        return g_height_fusion.x[0];
    }

    if(!g_height_fusion.initialized)
    {
        fusion_init(ir_height_cm);
        return ir_height_cm;
    }

    if(ir_height_cm < HF_IR_MIN_CM || ir_height_cm > HF_IR_MAX_CM)
    {
        g_height_fusion.ir_valid = 0;
        return g_height_fusion.x[0];
    }

    if(g_height_fusion.tof_update_count > 10)
    {
        diff = fabsf(ir_height_cm - g_height_fusion.x[0]);
        if(diff > HF_OUTLIER_THRESH * 2.0f)
        {
            g_height_fusion.ir_valid = 0;
            g_height_fusion.outlier_count++;
            return g_height_fusion.x[0];
        }
    }

    g_height_fusion.ir_valid = 1;
    g_height_fusion.ir_update_count++;

    (void)fusion_update(ir_height_cm, g_height_fusion.R_ir);

    if(g_height_fusion.tof_valid && g_height_fusion.ir_valid)
    {
        fusion_publish_result(95);
    }
    else
    {
        fusion_publish_result(70);
    }

    return g_height_fusion.x[0];
}

/* -------------------------------------------------------------------------- */
/* 高度传感器辅助逻辑                                                         */
/* -------------------------------------------------------------------------- */

static uint8 consume_new_ir_height_measurement(float *ir_height_cm)
{
    static uint32 last_ir_pose_seq = 0;
    uint32 current_seq = ir_pose_update_seq;

    if(current_seq == last_ir_pose_seq)
    {
        return 0;
    }

    last_ir_pose_seq = current_seq;

    if(ir_pose_estimate.valid &&
       ir_pose_estimate.z > 5.0f &&
       ir_pose_estimate.z < 300.0f)
    {
        if(ir_height_cm != NULL)
        {
            *ir_height_cm = ir_pose_estimate.z;
        }
        return 1;
    }

    return 0;
}

static void update_height_fusion_step(uint8 tof_measurement_valid, float tof_height_cm)
{
    float ir_height_cm = 0.0f;
    uint8 ir_measurement_valid = consume_new_ir_height_measurement(&ir_height_cm);

    fusion_predict(TOF_DT);

    if(tof_measurement_valid)
    {
        fusion_update_tof(tof_height_cm, TOF_DT);
    }

    if(ir_measurement_valid)
    {
        fusion_update_ir(ir_height_cm, TOF_DT);
    }
}

static void update_tof_height_filter(void)
{
    float roll_rad = current_roll * DEG2RAD;
    float pitch_rad = current_pitch * DEG2RAD;
    float tilt_cos = cosf(roll_rad) * cosf(pitch_rad);
    float vel = 0.0f;

    tof_height_true_m = tof_height_m * tilt_cos;
    tof_height_true_filt += TOF_HEIGHT_LPF_ALPHA * (tof_height_true_m - tof_height_true_filt);

    vel = (tof_height_true_filt - tof_height_true_prev) / TOF_DT;
    tof_height_true_prev = tof_height_true_filt;
    tof_vel_mps_filt += TOF_VEL_LPF_ALPHA * (vel - tof_vel_mps_filt);
}

/* -------------------------------------------------------------------------- */
/* 高度模块对外入口                                                           */
/* -------------------------------------------------------------------------- */

/*
 * 初始化高度管理模块。
 * initial_height_cm: 高度融合器的初始高度，单位 cm。
 * 当 ToF 和 IR 还没有给出首个有效高度时，先用它作为融合状态初值。
 */
void HeightManager_Init(float initial_height_cm)
{
    tof_divider = 0;                             // ToF 更新分频计数器
    height_m = 0.0f;                             // 当前周期读取到的原始测距值，单位 m
    tof_height_m = 0.0f;                         // 通过有效性判定后的 ToF 原始高度，单位 m
    tof_height_true_m = 0.0f;                    // 经过姿态修正后的垂直高度，单位 m
    tof_height_true_filt = 0.0f;                 // 低通滤波后的垂直高度，单位 m
    tof_height_true_prev = 0.0f;                 // 上一拍滤波后高度，用于差分求速度
    tof_vel_mps_filt = 0.0f;                     // 由高度差分得到并滤波后的垂直速度，单位 m/s
    tof_valid = 0;                               // 当前 ToF 高度是否有效
    HeightSpeedPID_output = 0.0f;                // 高度环输出到总推力的修正量
    g_height_result = (HeightFusionResult_t){0}; // 对外发布的高度融合结果快照

    fusion_init(initial_height_cm);
}

void HeightManager_Update(void)
{
    if(++tof_divider < TOF_UPDATE_DIV)
    {
        return;
    }

    tof_divider = 0;
    dl1b_get_distance();

    if(dl1b_finsh_flag && dl1b_distance_mm <= 4000)
    {
        height_m = dl1b_distance_mm * 0.001f;

        if(tof_valid && fabsf(height_m - tof_height_m) > 0.10f)
        {
            height_m = tof_height_m;
        }

        if(height_m >= TOF_VALID_MIN_M && height_m <= TOF_VALID_MAX_M)
        {
            tof_height_m = height_m;
            tof_valid = 1;
        }
        else
        {
            tof_valid = 0;
        }
    }
    else
    {
        tof_valid = 0;
    }

    if(tof_valid)
    {
        update_tof_height_filter();
        update_height_fusion_step(1, tof_height_true_m * 100.0f);
    }
    else
    {
        update_height_fusion_step(0, 0.0f);
    }
}
