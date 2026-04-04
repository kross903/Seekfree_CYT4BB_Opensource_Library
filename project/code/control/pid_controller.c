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
}

/*
 * PID 输出计算入口:
 * 按“误差 -> 积分 -> 微分 -> 限幅”的顺序算出本次控制输出。
 */
float PID_Calculate(PID_Controller *pid, float target, float current, float dt)
{
    float error = 0.0f;
    float output = 0.0f;
    float integral_factor = 1.0f;
    float raw_derivative = 0.0f;

    (void)dt;

    // 计算误差
    error = target - current;

    // 当前工程仍按固定周期运行，积分暂不直接乘 dt。
    if(fabsf(error) > 0.0f)
    {
        integral_factor = (fabsf(error) < pid->IntegralThreshold) ? 1.0f
                                                                 : (pid->IntegralThreshold / fabsf(error));
    }

    // 误差越大，积分缩放越强，保持原工程的积分抑制策略不变。
    pid->integral += integral_factor * error;

    // 积分限幅
    if(pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if(pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }

    // 微分项采用一阶低通滤波，降低测量噪声带来的尖峰。
    raw_derivative = error - pid->prev_error;
    pid->derivative = pid->alpha * raw_derivative + (1.0f - pid->alpha) * pid->derivative;

    // 计算 PID 输出
    output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * pid->derivative;

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

/*
 * PID 状态复位入口:
 * 清空积分和历史误差，常用于切模式、重新解锁或参数大改后。
 */
void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
