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
    float alpha;              // 微分一阶低通滤波系数
} PID_Controller;

// 按给定参数初始化一个 PID 控制器。
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float out_limit, float int_limit, float integral_threshold, float alpha);

// 根据目标值、当前值和采样周期计算新的控制输出。
// 当前实现仍按固定中断节拍工作，dt 参数保留用于接口兼容。
float PID_Calculate(PID_Controller *pid, float target, float current, float dt);

// 清空 PID 内部历史量，通常在解锁、切模式或急停后调用。
void PID_Reset(PID_Controller *pid);

#endif /* PID_CONTROLLER_H_ */
