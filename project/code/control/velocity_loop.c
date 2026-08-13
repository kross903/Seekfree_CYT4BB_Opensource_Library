#include "control/velocity_loop.h"
#include "control/angle_loop.h"
#include "control/motor_mixer.h"
#include <math.h>

#define VELOCITY_LOOP_GRAVITY_CMS2        (981.0f) /* 重力加速度，单位cm/s2 */
#define VELOCITY_LOOP_MAX_TILT_DEG        (25.0f)  /* 速度环允许叠加的最大倾角 */
#define VELOCITY_LOOP_RATE_FF_LPF_HZ      (15.0f)  /* 目标角速度前馈低通截止频率 */
#define VELOCITY_LOOP_RATE_FF_GAIN_DEFAULT (0.2f)  /* 目标角速度前馈默认增益 */
#define VELOCITY_LOOP_RATE_FF_GAIN_MAX    (2.0f)   /* VOFA允许设置的最大前馈增益 */
#define VELOCITY_LOOP_RATE_FF_LIMIT_DEFAULT (60.0f)/* 目标角速度前馈默认限幅，单位deg/s */
#define VELOCITY_LOOP_RATE_FF_LIMIT_MAX   (350.0f) /* VOFA允许设置的最大前馈限幅 */
#define VELOCITY_LOOP_TWO_PI              (6.28318530718f) /* 低通滤波指数形式使用的2π */
#define VELOCITY_LOOP_DEG2RAD             (0.01745329252f) /* 角度转弧度系数 */
#define VELOCITY_LOOP_RAD2DEG             (57.295779513f)  /* 弧度转角度系数 */

typedef struct
{
    float previous_velocity_cms;          /* 上一帧速度反馈，用于计算测量加速度 */
    float filtered_negative_accel_cms2;   /* 低通后的负测量加速度，作为D项输入 */
    volatile float cutoff_hz;             /* D支路低通截止频率，可由VOFA调整 */
    uint8 initialized;                    /* 首帧只初始化，不产生D项 */
} VelocityLoopDState_t;

PID_Controller vel_pid_x;
PID_Controller vel_pid_y;

static volatile uint8 g_enabled = 0U;                    /* 速度环总使能 */
static volatile float g_target_vx_earth_cms = 0.0f;      /* 惯性系X目标速度 */
static volatile float g_target_vy_earth_cms = 0.0f;      /* 惯性系Y目标速度 */
static volatile float g_target_ax_earth_cms2 = 0.0f;     /* 惯性系X加速度前馈 */
static volatile float g_target_ay_earth_cms2 = 0.0f;     /* 惯性系Y加速度前馈 */
static volatile float g_pitch_corr_deg = 0.0f;           /* X向加速度换算出的Pitch修正角 */
static volatile float g_roll_corr_deg = 0.0f;            /* Y向加速度换算出的Roll修正角 */
static volatile float g_fb_vx_earth_cms = 0.0f;          /* 惯性系X速度反馈缓存 */
static volatile float g_fb_vy_earth_cms = 0.0f;          /* 惯性系Y速度反馈缓存 */
static volatile uint8 g_active = 0U;                     /* 最近一次速度环是否有效工作 */
static volatile float g_pitch_ff_gain = VELOCITY_LOOP_RATE_FF_GAIN_DEFAULT; /* X轴对应Pitch角速度前馈增益 */
static volatile float g_roll_ff_gain = VELOCITY_LOOP_RATE_FF_GAIN_DEFAULT;  /* Y轴对应Roll角速度前馈增益 */
static volatile float g_pitch_ff_limit_dps = VELOCITY_LOOP_RATE_FF_LIMIT_DEFAULT; /* Pitch角速度前馈限幅 */
static volatile float g_roll_ff_limit_dps = VELOCITY_LOOP_RATE_FF_LIMIT_DEFAULT;  /* Roll角速度前馈限幅 */
static float g_prev_pitch_corr_deg = 0.0f;               /* 上一帧Pitch修正角，用于估算目标角速度 */
static float g_prev_roll_corr_deg = 0.0f;                /* 上一帧Roll修正角，用于估算目标角速度 */
static uint8 g_rate_ff_initialized = 0U;                 /* 角速度前馈首帧初始化标志 */
static VelocityLoopRateFeedforwardDiag_t g_rate_ff_diag;
static volatile VelocityLoopAxisDiag_t g_axis_diag[2];
static volatile VelocityLoopHorizontalDiag_t g_horizontal_diag;
static VelocityLoopDState_t g_d_state[2] = {
    {0.0f, 0.0f, VELOCITY_LOOP_D_FILTER_DEFAULT_HZ, 0U},
    {0.0f, 0.0f, VELOCITY_LOOP_D_FILTER_DEFAULT_HZ, 0U}
};

/* 标量限幅，保证PID输出和调参值不会超过允许范围。 */
static float velocity_loop_limit_float(float value, float minimum, float maximum)
{
    if(value > maximum) return maximum;
    if(value < minimum) return minimum;
    return value;
}

/* 二维向量限幅，保持方向不变，只压缩幅值。 */
static uint8 velocity_loop_limit_vector(float *x, float *y, float limit)
{
    float length;
    float scale;

    if((x == NULL) || (y == NULL) || !(limit > 0.0f)) return 0U;
    length = sqrtf((*x) * (*x) + (*y) * (*y));
    if(!(length > limit)) return 0U;
    scale = limit / length;
    *x *= scale;
    *y *= scale;
    return 1U;
}

/* 机体系速度转惯性系速度，Yaw为IMU绝对偏航角。 */
static void velocity_loop_body_to_earth(float body_x,
                                        float body_y,
                                        float yaw_deg,
                                        float *earth_x,
                                        float *earth_y)
{
    float yaw_rad = yaw_deg * VELOCITY_LOOP_DEG2RAD;
    float cosine = cosf(yaw_rad);
    float sine = sinf(yaw_rad);

    *earth_x = cosine * body_x - sine * body_y;
    *earth_y = sine * body_x + cosine * body_y;
}

/* 惯性系加速度转机体系加速度，供后续换算Roll/Pitch目标角。 */
static void velocity_loop_earth_to_body(float earth_x,
                                        float earth_y,
                                        float yaw_deg,
                                        float *body_x,
                                        float *body_y)
{
    float yaw_rad = yaw_deg * VELOCITY_LOOP_DEG2RAD;
    float cosine = cosf(yaw_rad);
    float sine = sinf(yaw_rad);

    *body_x = cosine * earth_x + sine * earth_y;
    *body_y = -sine * earth_x + cosine * earth_y;
}

/* 清空单轴D支路历史速度和滤波状态。 */
static void velocity_loop_reset_d_axis(VelocityLoopDState_t *state)
{
    state->previous_velocity_cms = 0.0f;
    state->filtered_negative_accel_cms2 = 0.0f;
    state->initialized = 0U;
}

static void velocity_loop_reset_d_state(void)
{
    velocity_loop_reset_d_axis(&g_d_state[VELOCITY_LOOP_AXIS_X]);
    velocity_loop_reset_d_axis(&g_d_state[VELOCITY_LOOP_AXIS_Y]);
}

/* D项使用测量速度变化率，带低通滤波，降低光流噪声放大。 */
static float velocity_loop_update_d(uint8 axis,
                                    float measured_velocity_cms,
                                    float dt_s)
{
    VelocityLoopDState_t *state = &g_d_state[axis];
    float raw_negative_accel;
    float alpha;

    if(!state->initialized)
    {
        state->previous_velocity_cms = measured_velocity_cms;
        state->initialized = 1U;
        return 0.0f;
    }
    raw_negative_accel = -(measured_velocity_cms - state->previous_velocity_cms) /
                         dt_s;
    state->previous_velocity_cms = measured_velocity_cms;
    alpha = 1.0f - expf(-VELOCITY_LOOP_TWO_PI * state->cutoff_hz *
                        dt_s);
    state->filtered_negative_accel_cms2 += alpha *
        (raw_negative_accel - state->filtered_negative_accel_cms2);
    return state->filtered_negative_accel_cms2;
}

/* 清空速度环给角度环提供的角速度前馈。 */
static void velocity_loop_reset_rate_feedforward(void)
{
    VelocityLoopRateFeedforwardDiag_t empty = {0};

    g_prev_pitch_corr_deg = 0.0f;
    g_prev_roll_corr_deg = 0.0f;
    g_rate_ff_initialized = 0U;
    g_rate_ff_diag = empty;
    Angle_Loop_Set_Rate_Feedforward(0.0f, 0.0f);
}

/* 将目标Roll/Pitch修正角变化率转换为角速度前馈，减小角度环滞后。 */
static void velocity_loop_update_rate_feedforward(float roll_corr_deg,
                                                  float pitch_corr_deg,
                                                  float dt_s)
{
    float alpha = 1.0f - expf(-VELOCITY_LOOP_TWO_PI *
                              VELOCITY_LOOP_RATE_FF_LPF_HZ *
                              dt_s);

    if(!g_rate_ff_initialized)
    {
        g_prev_roll_corr_deg = roll_corr_deg;
        g_prev_pitch_corr_deg = pitch_corr_deg;
        g_rate_ff_initialized = 1U;
        Angle_Loop_Set_Rate_Feedforward(0.0f, 0.0f);
        return;
    }

    g_rate_ff_diag.roll_target_rate_raw_dps =
        (roll_corr_deg - g_prev_roll_corr_deg) / dt_s;
    g_rate_ff_diag.pitch_target_rate_raw_dps =
        (pitch_corr_deg - g_prev_pitch_corr_deg) / dt_s;
    g_prev_roll_corr_deg = roll_corr_deg;
    g_prev_pitch_corr_deg = pitch_corr_deg;
    g_rate_ff_diag.roll_target_rate_filt_dps += alpha *
        (g_rate_ff_diag.roll_target_rate_raw_dps -
         g_rate_ff_diag.roll_target_rate_filt_dps);
    g_rate_ff_diag.pitch_target_rate_filt_dps += alpha *
        (g_rate_ff_diag.pitch_target_rate_raw_dps -
         g_rate_ff_diag.pitch_target_rate_filt_dps);
    g_rate_ff_diag.roll_rate_ff_dps = velocity_loop_limit_float(
        g_rate_ff_diag.roll_target_rate_filt_dps * g_roll_ff_gain,
        -g_roll_ff_limit_dps, g_roll_ff_limit_dps);
    g_rate_ff_diag.pitch_rate_ff_dps = velocity_loop_limit_float(
        g_rate_ff_diag.pitch_target_rate_filt_dps * g_pitch_ff_gain,
        -g_pitch_ff_limit_dps, g_pitch_ff_limit_dps);
    Angle_Loop_Set_Rate_Feedforward(g_rate_ff_diag.roll_rate_ff_dps,
                                    g_rate_ff_diag.pitch_rate_ff_dps);
}

/* 清空VOFA和菜单使用的速度环诊断缓存。 */
static void velocity_loop_reset_diag(void)
{
    VelocityLoopAxisDiag_t empty_axis = {0};
    VelocityLoopHorizontalDiag_t empty_horizontal = {0};

    g_axis_diag[0] = empty_axis;
    g_axis_diag[1] = empty_axis;
    g_horizontal_diag = empty_horizontal;
    g_pitch_corr_deg = 0.0f;
    g_roll_corr_deg = 0.0f;
    g_fb_vx_earth_cms = 0.0f;
    g_fb_vy_earth_cms = 0.0f;
}

void Velocity_Loop_Set_Enabled(uint8 enabled)
{
    g_enabled = enabled ? 1U : 0U;
    Velocity_Loop_Reset_Runtime();
}

uint8 Velocity_Loop_Is_Enabled(void)
{
    return g_enabled;
}

void Velocity_Loop_Set_Target_Velocity(float vx_earth_cms, float vy_earth_cms)
{
    Velocity_Loop_Set_Target_State(vx_earth_cms, vy_earth_cms, 0.0f, 0.0f);
}

void Velocity_Loop_Set_Target_State(float vx_earth_cms,
                                    float vy_earth_cms,
                                    float ax_earth_cms2,
                                    float ay_earth_cms2)
{
    g_target_vx_earth_cms = isfinite(vx_earth_cms) ? vx_earth_cms : 0.0f;
    g_target_vy_earth_cms = isfinite(vy_earth_cms) ? vy_earth_cms : 0.0f;
    g_target_ax_earth_cms2 = isfinite(ax_earth_cms2) ? ax_earth_cms2 : 0.0f;
    g_target_ay_earth_cms2 = isfinite(ay_earth_cms2) ? ay_earth_cms2 : 0.0f;
}

void Velocity_Loop_Get_Target_Velocity(float *vx_earth_cms, float *vy_earth_cms)
{
    if(vx_earth_cms != NULL) *vx_earth_cms = g_target_vx_earth_cms;
    if(vy_earth_cms != NULL) *vy_earth_cms = g_target_vy_earth_cms;
}

void Velocity_Loop_Reset_Runtime(void)
{
    g_target_vx_earth_cms = 0.0f;
    g_target_vy_earth_cms = 0.0f;
    g_target_ax_earth_cms2 = 0.0f;
    g_target_ay_earth_cms2 = 0.0f;
    PID_Reset(&vel_pid_x);
    PID_Reset(&vel_pid_y);
    velocity_loop_reset_d_state();
    velocity_loop_reset_diag();
    velocity_loop_reset_rate_feedforward();
    g_active = 0U;
}

void Velocity_Loop_Reset_D_Filter_State(void)
{
    velocity_loop_reset_d_state();
}

uint8 Velocity_Loop_Set_D_Filter_Hz(uint8 axis, float cutoff_hz)
{
    if((axis > VELOCITY_LOOP_AXIS_Y) || !isfinite(cutoff_hz) ||
       (cutoff_hz < VELOCITY_LOOP_D_FILTER_MIN_HZ) ||
       (cutoff_hz > VELOCITY_LOOP_D_FILTER_MAX_HZ))
    {
        return 0U;
    }
    g_d_state[axis].cutoff_hz = cutoff_hz;
    return 1U;
}

float Velocity_Loop_Get_D_Filter_Hz(uint8 axis)
{
    return (axis <= VELOCITY_LOOP_AXIS_Y) ? g_d_state[axis].cutoff_hz : 0.0f;
}

uint8 Velocity_Loop_Set_Rate_Feedforward_Parameters(uint8 axis,
                                                    float gain,
                                                    float limit_dps)
{
    if(!isfinite(gain) || !isfinite(limit_dps) ||
       (gain < 0.0f) || (gain > VELOCITY_LOOP_RATE_FF_GAIN_MAX) ||
       (limit_dps < 0.0f) || (limit_dps > VELOCITY_LOOP_RATE_FF_LIMIT_MAX))
    {
        return 0U;
    }
    if(axis == VELOCITY_LOOP_AXIS_X)
    {
        g_pitch_ff_gain = gain;
        g_pitch_ff_limit_dps = limit_dps;
    }
    else if(axis == VELOCITY_LOOP_AXIS_Y)
    {
        g_roll_ff_gain = gain;
        g_roll_ff_limit_dps = limit_dps;
    }
    else return 0U;
    return 1U;
}

void Velocity_Loop_Get_Rate_Feedforward_Parameters(uint8 axis,
                                                   float *gain,
                                                   float *limit_dps)
{
    float selected_gain = 0.0f;
    float selected_limit = 0.0f;

    if(axis == VELOCITY_LOOP_AXIS_X)
    {
        selected_gain = g_pitch_ff_gain;
        selected_limit = g_pitch_ff_limit_dps;
    }
    else if(axis == VELOCITY_LOOP_AXIS_Y)
    {
        selected_gain = g_roll_ff_gain;
        selected_limit = g_roll_ff_limit_dps;
    }
    if(gain != NULL) *gain = selected_gain;
    if(limit_dps != NULL) *limit_dps = selected_limit;
}

void Velocity_Loop_Get_Rate_Feedforward_Diag(VelocityLoopRateFeedforwardDiag_t *diag)
{
    if(diag != NULL) *diag = g_rate_ff_diag;
}

uint8 Velocity_Loop_Get_Axis_Diag(uint8 axis, VelocityLoopAxisDiag_t *diag)
{
    uint32 irq_state;

    if((axis > VELOCITY_LOOP_AXIS_Y) || (diag == NULL)) return 0U;
    irq_state = __get_PRIMASK();
    __disable_irq();
    *diag = g_axis_diag[axis];
    if(irq_state == 0U) __enable_irq();
    return 1U;
}

void Velocity_Loop_Get_Horizontal_Diag(VelocityLoopHorizontalDiag_t *diag)
{
    uint32 irq_state;

    if(diag == NULL) return;
    irq_state = __get_PRIMASK();
    __disable_irq();
    *diag = g_horizontal_diag;
    if(irq_state == 0U) __enable_irq();
}

void Velocity_Loop_Update_Attitude_Target(uint8 velocity_ready,
                                          float measured_vx_body_cms,
                                          float measured_vy_body_cms,
                                          float yaw_deg,
                                          float dt_s,
                                          float captured_roll,
                                          float captured_pitch,
                                          float *target_roll,
                                          float *target_pitch)
{
    float measured_vx_earth;
    float measured_vy_earth;
    float error_x;
    float error_y;
    float d_measure_x;
    float d_measure_y;
    float previous_i_x;
    float previous_i_y;
    float candidate_i_x;
    float candidate_i_y;
    float p_x;
    float p_y;
    float d_x;
    float d_y;
    float correction_x;
    float correction_y;
    float acceleration_x;
    float acceleration_y;
    float acceleration_body_x;
    float acceleration_body_y;
    float acceleration_limit;
    MotorMixerLimitDiag_t mixer_diag = {0};
    uint8 motor_limited;
    uint8 status = 0U;
    uint8 accel_limited;

    if((target_roll == NULL) || (target_pitch == NULL)) return;
    /* 速度反馈或控制周期无效时，回到起飞捕获角，防止脏数据进入姿态环。 */
    if(!g_enabled || !velocity_ready || !isfinite(yaw_deg) ||
       !isfinite(dt_s) || (dt_s < 0.002f) || (dt_s > 0.008f) ||
       !isfinite(measured_vx_body_cms) || !isfinite(measured_vy_body_cms))
    {
        *target_roll = captured_roll;
        *target_pitch = captured_pitch;
        Velocity_Loop_Reset_Runtime();
        return;
    }

    velocity_loop_body_to_earth(measured_vx_body_cms, measured_vy_body_cms,
                                yaw_deg, &measured_vx_earth, &measured_vy_earth);
    /* 速度环统一在惯性系计算，避免Yaw变化时目标方向跟着机头漂移。 */
    error_x = g_target_vx_earth_cms - measured_vx_earth;
    error_y = g_target_vy_earth_cms - measured_vy_earth;
    d_measure_x = velocity_loop_update_d(VELOCITY_LOOP_AXIS_X,
                                         measured_vx_earth,
                                         dt_s);
    d_measure_y = velocity_loop_update_d(VELOCITY_LOOP_AXIS_Y,
                                         measured_vy_earth,
                                         dt_s);
    p_x = vel_pid_x.Kp * error_x;
    p_y = vel_pid_y.Kp * error_y;
    d_x = vel_pid_x.Kd * d_measure_x;
    d_y = vel_pid_y.Kd * d_measure_y;
    previous_i_x = vel_pid_x.integral;
    previous_i_y = vel_pid_y.integral;

    Motor_Mixer_Get_Limit_Diag(&mixer_diag);
    /* Yaw或公共油门受限时Roll/Pitch仍完整可实现，不应冻结水平速度积分。 */
    motor_limited = (uint8)((mixer_diag.roll_limit_flags != 0U) ||
                            (mixer_diag.pitch_limit_flags != 0U));
    if(motor_limited) status |= VELOCITY_LOOP_STATUS_MOTOR_LIMITED;

    /* 电机饱和时只允许I项向减小饱和的方向恢复，避免继续顶死。 */
    candidate_i_x = previous_i_x;
    candidate_i_y = previous_i_y;
    if((!motor_limited || (error_x * previous_i_x < 0.0f)) && (vel_pid_x.Ki > 0.0f))
    {
        candidate_i_x += vel_pid_x.Ki * error_x * dt_s;
    }
    else if(motor_limited)
    {
        status |= VELOCITY_LOOP_STATUS_INTEGRATOR_HELD;
    }
    if((!motor_limited || (error_y * previous_i_y < 0.0f)) && (vel_pid_y.Ki > 0.0f))
    {
        candidate_i_y += vel_pid_y.Ki * error_y * dt_s;
    }
    else if(motor_limited)
    {
        status |= VELOCITY_LOOP_STATUS_INTEGRATOR_HELD;
    }
    candidate_i_x = velocity_loop_limit_float(candidate_i_x,
                                               -vel_pid_x.integral_limit,
                                               vel_pid_x.integral_limit);
    candidate_i_y = velocity_loop_limit_float(candidate_i_y,
                                               -vel_pid_y.integral_limit,
                                               vel_pid_y.integral_limit);

    correction_x = p_x + candidate_i_x + d_x;
    correction_y = p_y + candidate_i_y + d_y;
    acceleration_x = g_target_ax_earth_cms2 + correction_x;
    acceleration_y = g_target_ay_earth_cms2 + correction_y;
    acceleration_limit = VELOCITY_LOOP_GRAVITY_CMS2 *
                         tanf(VELOCITY_LOOP_MAX_TILT_DEG * VELOCITY_LOOP_DEG2RAD);
    /* 最终加速度限幅同时受最大倾角和各轴PID输出限幅约束。 */
    if((vel_pid_x.output_limit > 0.0f) &&
       (vel_pid_x.output_limit < acceleration_limit))
    {
        acceleration_limit = vel_pid_x.output_limit;
    }
    if((vel_pid_y.output_limit > 0.0f) &&
       (vel_pid_y.output_limit < acceleration_limit))
    {
        acceleration_limit = vel_pid_y.output_limit;
    }
    accel_limited = velocity_loop_limit_vector(&acceleration_x,
                                               &acceleration_y,
                                               acceleration_limit);
    if(accel_limited)
    {
        float integral_delta_x = candidate_i_x - previous_i_x;
        float integral_delta_y = candidate_i_y - previous_i_y;
        status |= VELOCITY_LOOP_STATUS_ACCEL_LIMITED;
        /* 若新增I项会把限幅后的加速度继续往外推，则撤销本周期I项增长。 */
        if((integral_delta_x * acceleration_x + integral_delta_y * acceleration_y) > 0.0f)
        {
            candidate_i_x = previous_i_x;
            candidate_i_y = previous_i_y;
            correction_x = p_x + candidate_i_x + d_x;
            correction_y = p_y + candidate_i_y + d_y;
            acceleration_x = g_target_ax_earth_cms2 + correction_x;
            acceleration_y = g_target_ay_earth_cms2 + correction_y;
            (void)velocity_loop_limit_vector(&acceleration_x,
                                             &acceleration_y,
                                             acceleration_limit);
            status |= VELOCITY_LOOP_STATUS_INTEGRATOR_HELD;
        }
    }
    vel_pid_x.integral = candidate_i_x;
    vel_pid_y.integral = candidate_i_y;
    vel_pid_x.derivative = d_measure_x;
    vel_pid_y.derivative = d_measure_y;
    vel_pid_x.prev_error = error_x;
    vel_pid_y.prev_error = error_y;

    velocity_loop_earth_to_body(acceleration_x, acceleration_y, yaw_deg,
                                &acceleration_body_x, &acceleration_body_y);
    /* 小角度近似的反向形式：期望水平加速度换算为Pitch/Roll修正角。 */
    g_pitch_corr_deg = atan2f(acceleration_body_x,
                              VELOCITY_LOOP_GRAVITY_CMS2) * VELOCITY_LOOP_RAD2DEG;
    g_roll_corr_deg = -atan2f(acceleration_body_y,
                              VELOCITY_LOOP_GRAVITY_CMS2) * VELOCITY_LOOP_RAD2DEG;
    *target_pitch = captured_pitch + g_pitch_corr_deg;
    *target_roll = captured_roll + g_roll_corr_deg;
    velocity_loop_update_rate_feedforward(g_roll_corr_deg, g_pitch_corr_deg, dt_s);

    g_axis_diag[0].target_velocity_cms = g_target_vx_earth_cms;
    g_axis_diag[0].actual_velocity_cms = measured_vx_earth;
    g_axis_diag[0].error_velocity_cms = error_x;
    g_axis_diag[0].p_output_cms2 = p_x;
    g_axis_diag[0].i_output_cms2 = candidate_i_x;
    g_axis_diag[0].d_output_cms2 = d_x;
    g_axis_diag[0].acceleration_ff_cms2 = g_target_ax_earth_cms2;
    g_axis_diag[0].raw_output_cms2 = g_target_ax_earth_cms2 + correction_x;
    g_axis_diag[0].output_cms2 = acceleration_x;
    g_axis_diag[0].saturated = accel_limited;
    g_axis_diag[0].active = 1U;
    g_axis_diag[1].target_velocity_cms = g_target_vy_earth_cms;
    g_axis_diag[1].actual_velocity_cms = measured_vy_earth;
    g_axis_diag[1].error_velocity_cms = error_y;
    g_axis_diag[1].p_output_cms2 = p_y;
    g_axis_diag[1].i_output_cms2 = candidate_i_y;
    g_axis_diag[1].d_output_cms2 = d_y;
    g_axis_diag[1].acceleration_ff_cms2 = g_target_ay_earth_cms2;
    g_axis_diag[1].raw_output_cms2 = g_target_ay_earth_cms2 + correction_y;
    g_axis_diag[1].output_cms2 = acceleration_y;
    g_axis_diag[1].saturated = accel_limited;
    g_axis_diag[1].active = 1U;
    g_horizontal_diag.target_vx_earth_cms = g_target_vx_earth_cms;
    g_horizontal_diag.target_vy_earth_cms = g_target_vy_earth_cms;
    g_horizontal_diag.actual_vx_earth_cms = measured_vx_earth;
    g_horizontal_diag.actual_vy_earth_cms = measured_vy_earth;
    g_horizontal_diag.acceleration_ff_x_earth_cms2 = g_target_ax_earth_cms2;
    g_horizontal_diag.acceleration_ff_y_earth_cms2 = g_target_ay_earth_cms2;
    g_horizontal_diag.acceleration_correction_x_earth_cms2 = correction_x;
    g_horizontal_diag.acceleration_correction_y_earth_cms2 = correction_y;
    g_horizontal_diag.acceleration_target_x_earth_cms2 = acceleration_x;
    g_horizontal_diag.acceleration_target_y_earth_cms2 = acceleration_y;
    g_horizontal_diag.target_roll_deg = *target_roll;
    g_horizontal_diag.target_pitch_deg = *target_pitch;
    g_horizontal_diag.dt_s = dt_s;
    g_horizontal_diag.status_flags = status;
    g_horizontal_diag.timing_valid = 1U;
    g_fb_vx_earth_cms = measured_vx_earth;
    g_fb_vy_earth_cms = measured_vy_earth;
    g_active = 1U;
}

void Velocity_Loop_Get_Diag(float *pitch_corr_deg,
                            float *roll_corr_deg,
                            float *fb_vx_cms,
                            float *fb_vy_cms,
                            float *x_i_out,
                            float *y_i_out,
                            uint8 *active)
{
    if(pitch_corr_deg != NULL) *pitch_corr_deg = g_pitch_corr_deg;
    if(roll_corr_deg != NULL) *roll_corr_deg = g_roll_corr_deg;
    if(fb_vx_cms != NULL) *fb_vx_cms = g_fb_vx_earth_cms;
    if(fb_vy_cms != NULL) *fb_vy_cms = g_fb_vy_earth_cms;
    if(x_i_out != NULL) *x_i_out = vel_pid_x.integral;
    if(y_i_out != NULL) *y_i_out = vel_pid_y.integral;
    if(active != NULL) *active = g_active;
}
