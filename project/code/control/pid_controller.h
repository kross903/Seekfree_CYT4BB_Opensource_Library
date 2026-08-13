#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

// PID 运行时参数，既保存增益，也保存积分、微分等历史状态。
typedef struct
{
    float Kp;                 // 比例增益
    float Ki;                 // 积分增益
    float Kd;                 // 微分增益
    float integral;           // 当前积分累计值
    float derivative;         // 当前微分项
    float prev_error;         // 上一次误差
    float output_limit;       // 输出限幅
    float integral_limit;     // 积分限幅
    float IntegralThreshold;  // 启用积分缩放的误差阈值
    float alpha;              // 参考周期下的微分一阶低通滤波系数
    float reference_dt_s;     // 当前参数的原调参周期
    float cached_dt_s;        // 已缓存的实际运行周期
    float cached_alpha;       // 已缓存的参考滤波系数
    float integral_step_scale;
    float derivative_step_scale;
    float runtime_alpha;
} PID_Controller;

// 按给定参数初始化一个 PID 控制器。
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float out_limit, float int_limit, float integral_threshold, float alpha);

// 设置现有 PID 参数对应的原调参周期。
void PID_SetReferenceDt(PID_Controller *pid, float reference_dt_s);

// 在中断启动前预计算实际运行周期对应的缩放和滤波系数。
void PID_PrepareRuntimeDt(PID_Controller *pid, float runtime_dt_s);

// 根据目标值、当前值和实际运行周期计算新的控制输出。
float PID_Calculate(PID_Controller *pid, float target, float current, float dt);

// 带方向性执行器限幅的PID计算：受限方向上的误差不再累积积分，反向解除饱和仍允许积分更新。
float PID_CalculateLimited(PID_Controller *pid,
                           float target,
                           float current,
                           float dt,
                           int negative_limited,
                           int positive_limited);

// 使用调用方已经处理好的误差计算输出，适用于周期角度等非普通减法误差。
float PID_CalculateError(PID_Controller *pid, float error, float dt);

// 使用调用方误差和方向性执行器限幅计算PID输出。
float PID_CalculateErrorLimited(PID_Controller *pid,
                                float error,
                                float dt,
                                int negative_limited,
                                int positive_limited);

// 使用调用方已经滤波好的微分值计算PID，P/I和总输出限幅仍沿用当前控制器配置。
float PID_CalculateErrorWithDerivative(PID_Controller *pid,
                                       float error,
                                       float derivative,
                                       float dt);

// 清空 PID 内部历史量，通常在解锁、切模式或急停后调用。
void PID_Reset(PID_Controller *pid);

#endif /* PID_CONTROLLER_H_ */
