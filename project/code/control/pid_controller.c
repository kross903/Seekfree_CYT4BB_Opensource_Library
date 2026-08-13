/*********************************************************************************************************************
* 文件名称: pid_controller.c
* 功能说明: PID 控制器实现
********************************************************************************************************************/

#include "control/pid_controller.h"
#include "common/project_headfile.h"
#include <math.h>

/*
 * PID 参数初始化入口:
 * 开机时由 All_Init.c 统一写入各个控制环的默认参数，
 * 后续也可以在外部调参后再次调用这里重建控制器状态。
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float out_limit, float int_limit, float integral_threshold, float alpha)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = out_limit;
    pid->integral_limit = int_limit;
    pid->IntegralThreshold = integral_threshold;
    pid->alpha = alpha;
    pid->reference_dt_s = 0.0f;
    pid->cached_dt_s = 0.0f;
    pid->cached_alpha = -1.0f;
    pid->integral_step_scale = 1.0f;
    pid->derivative_step_scale = 1.0f;
    pid->runtime_alpha = alpha;
}

void PID_SetReferenceDt(PID_Controller *pid, float reference_dt_s)
{
    if(pid == NULL)
    {
        return;
    }

    pid->reference_dt_s = (isfinite(reference_dt_s) && reference_dt_s > 0.0f)
                            ? reference_dt_s
                            : 0.0f;
    pid->cached_dt_s = 0.0f;
    pid->cached_alpha = -1.0f;
}

static void pid_update_timing_cache(PID_Controller *pid, float dt)
{
    float reference_dt_s;
    float ratio;

    if(!isfinite(dt) || dt <= 0.0f)
    {
        dt = (pid->reference_dt_s > 0.0f) ? pid->reference_dt_s : 1.0f;
    }
    if((pid->cached_dt_s == dt) && (pid->cached_alpha == pid->alpha))
    {
        return;
    }

    reference_dt_s = (isfinite(pid->reference_dt_s) && pid->reference_dt_s > 0.0f)
                       ? pid->reference_dt_s
                       : dt;
    ratio = dt / reference_dt_s;
    if(!isfinite(ratio) || ratio <= 0.0f)
    {
        ratio = 1.0f;
    }

    pid->integral_step_scale = ratio;
    pid->derivative_step_scale = 1.0f / ratio;
    if(pid->alpha <= 0.0f)
    {
        pid->runtime_alpha = 0.0f;
    }
    else if(pid->alpha >= 1.0f)
    {
        pid->runtime_alpha = 1.0f;
    }
    else
    {
        pid->runtime_alpha = 1.0f - powf(1.0f - pid->alpha, ratio);
    }
    pid->cached_dt_s = dt;
    pid->cached_alpha = pid->alpha;
}

void PID_PrepareRuntimeDt(PID_Controller *pid, float runtime_dt_s)
{
    if(pid != NULL)
    {
        pid_update_timing_cache(pid, runtime_dt_s);
    }
}

/*
 * PID 输出计算入口:
 * 按“误差 -> 积分 -> 微分 -> 限幅”的顺序算出本次控制输出。
 */
static float pid_calculate_error_internal(PID_Controller *pid,
                                          float error,
                                          float dt,
                                          int use_external_derivative,
                                          float external_derivative,
                                          int negative_limited,
                                          int positive_limited)
{
    float output = 0.0f;
    float integral_factor = 1.0f;
    float raw_derivative = 0.0f;
    float integral_before;
    float integral_output_delta;

    pid_update_timing_cache(pid, dt);

    if(fabsf(error) > 0.0f)
    {
        integral_factor = (fabsf(error) < pid->IntegralThreshold) ? 1.0f
                                                                 : (pid->IntegralThreshold / fabsf(error));
    }

    integral_before = pid->integral;
    // 执行器受限时只禁止积分继续推向受限方向，反向误差仍可释放已有积分。
    if(!((negative_limited && (error < 0.0f)) ||
         (positive_limited && (error > 0.0f))))
    {
        // 误差越大，积分缩放越强，保持原工程的积分抑制策略不变。
        pid->integral += integral_factor * error * pid->integral_step_scale;
    }

    // 积分限幅
    if(pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if(pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }

    if(use_external_derivative)
    {
        pid->derivative = isfinite(external_derivative) ? external_derivative : 0.0f;
    }
    else
    {
        // 微分项采用一阶低通滤波，降低测量噪声带来的尖峰。
        raw_derivative = (error - pid->prev_error) * pid->derivative_step_scale;
        pid->derivative = pid->runtime_alpha * raw_derivative +
                          (1.0f - pid->runtime_alpha) * pid->derivative;
    }

    // 计算 PID 输出
    output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * pid->derivative;

    /* PID自身输出受限时也撤销推动饱和的新增积分，反向释放积分不受影响。 */
    integral_output_delta = pid->Ki * (pid->integral - integral_before);
    if(((output > pid->output_limit) && (integral_output_delta > 0.0f)) ||
       ((output < -pid->output_limit) && (integral_output_delta < 0.0f)))
    {
        pid->integral = integral_before;
        output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * pid->derivative;
    }

    // 输出限幅
    if(output > pid->output_limit)
    {
        output = pid->output_limit;
    }
    else if(output < -pid->output_limit)
    {
        output = -pid->output_limit;
    }

    // 保存当前误差，供下一次微分计算使用。
    pid->prev_error = error;

    return output;
}

float PID_CalculateError(PID_Controller *pid, float error, float dt)
{
    return pid_calculate_error_internal(pid, error, dt, 0, 0.0f, 0, 0);
}

float PID_CalculateErrorLimited(PID_Controller *pid,
                                float error,
                                float dt,
                                int negative_limited,
                                int positive_limited)
{
    return pid_calculate_error_internal(pid,
                                        error,
                                        dt,
                                        0,
                                        0.0f,
                                        negative_limited,
                                        positive_limited);
}

float PID_CalculateErrorWithDerivative(PID_Controller *pid,
                                       float error,
                                       float derivative,
                                       float dt)
{
    return pid_calculate_error_internal(pid, error, dt, 1, derivative, 0, 0);
}

float PID_Calculate(PID_Controller *pid, float target, float current, float dt)
{
    return PID_CalculateError(pid, target - current, dt);
}

float PID_CalculateLimited(PID_Controller *pid,
                           float target,
                           float current,
                           float dt,
                           int negative_limited,
                           int positive_limited)
{
    return PID_CalculateErrorLimited(pid,
                                     target - current,
                                     dt,
                                     negative_limited,
                                     positive_limited);
}

/*
 * PID 状态复位入口:
 * 清空积分和历史误差，常用于切模式、重新解锁或参数大改后。
 */
void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->prev_error = 0.0f;
}
