#ifndef VELOCITY_LOOP_H_
#define VELOCITY_LOOP_H_

#include "zf_common_headfile.h"
#include "control/pid_controller.h"

extern PID_Controller vel_pid_x; /* 惯性系X速度PID，输出X向加速度修正 */
extern PID_Controller vel_pid_y; /* 惯性系Y速度PID，输出Y向加速度修正 */

#define VELOCITY_LOOP_AXIS_X    (0U)   /* 惯性系X轴编号，通常对应前后速度 */
#define VELOCITY_LOOP_AXIS_Y    (1U)   /* 惯性系Y轴编号，通常对应左右速度 */
#define VELOCITY_LOOP_D_FILTER_DEFAULT_HZ (5.0f)  /* 速度D支路低通默认截止频率 */
#define VELOCITY_LOOP_D_FILTER_MIN_HZ     (1.0f)  /* VOFA允许设置的最小D滤波截止频率 */
#define VELOCITY_LOOP_D_FILTER_MAX_HZ     (50.0f) /* VOFA允许设置的最大D滤波截止频率 */
#define VELOCITY_LOOP_D_BIQUAD_SAMPLE_HZ (250.0f) /* 水平速度环实际更新频率 */
#define VELOCITY_LOOP_D_BIQUAD_Q_DEFAULT (0.7071f) /* Butterworth品质因数 */
#define VELOCITY_LOOP_D_RAW_ACCEL_LIMIT_DEFAULT (400.0f) /* 微分原始值限幅，cm/s² */
#define VELOCITY_LOOP_D_OUTPUT_LIMIT_DEFAULT (160.0f) /* D项输出限幅，cm/s² */

#define VELOCITY_LOOP_STATUS_ACCEL_LIMITED  (0x01U) /* 目标加速度被倾角或PID输出限幅 */
#define VELOCITY_LOOP_STATUS_MOTOR_LIMITED  (0x02U) /* 电机混控已触顶或触底 */
#define VELOCITY_LOOP_STATUS_INTEGRATOR_HELD (0x04U)/* I项因限幅保护被暂停增长 */

typedef struct
{
    float roll_target_rate_raw_dps;  /* Roll目标角变化率原始值 */
    float pitch_target_rate_raw_dps; /* Pitch目标角变化率原始值 */
    float roll_target_rate_filt_dps; /* Roll目标角变化率低通后值 */
    float pitch_target_rate_filt_dps;/* Pitch目标角变化率低通后值 */
    float roll_rate_ff_dps;          /* 送入角度环的Roll角速度前馈 */
    float pitch_rate_ff_dps;         /* 送入角度环的Pitch角速度前馈 */
} VelocityLoopRateFeedforwardDiag_t;

typedef struct
{
    float target_velocity_cms;  /* 该轴目标速度，单位cm/s */
    float actual_velocity_cms;  /* 该轴反馈速度，单位cm/s */
    float error_velocity_cms;   /* 速度误差，目标减反馈 */
    float p_output_cms2;        /* P项输出的加速度修正 */
    float i_output_cms2;        /* I项输出的加速度修正 */
    float d_output_cms2;        /* D项输出的加速度修正 */
    float acceleration_ff_cms2; /* 上层轨迹给定的加速度前馈 */
    float raw_output_cms2;      /* 限幅前的目标加速度 */
    float output_cms2;          /* 限幅后的目标加速度 */
    uint8 saturated;            /* 该轴本周期是否被加速度限幅 */
    uint8 active;               /* 速度环本周期是否参与控制 */
} VelocityLoopAxisDiag_t;

typedef struct
{
    float target_vx_earth_cms;              /* 惯性系X目标速度 */
    float target_vy_earth_cms;              /* 惯性系Y目标速度 */
    float actual_vx_earth_cms;              /* 惯性系X反馈速度 */
    float actual_vy_earth_cms;              /* 惯性系Y反馈速度 */
    float acceleration_ff_x_earth_cms2;     /* 惯性系X加速度前馈 */
    float acceleration_ff_y_earth_cms2;     /* 惯性系Y加速度前馈 */
    float acceleration_correction_x_earth_cms2; /* 惯性系X速度PID修正加速度 */
    float acceleration_correction_y_earth_cms2; /* 惯性系Y速度PID修正加速度 */
    float acceleration_target_x_earth_cms2; /* 惯性系X最终目标加速度 */
    float acceleration_target_y_earth_cms2; /* 惯性系Y最终目标加速度 */
    float target_roll_deg;                  /* 速度环输出后的Roll目标角 */
    float target_pitch_deg;                 /* 速度环输出后的Pitch目标角 */
    float dt_s;                             /* 本次速度环计算周期 */
    uint8 status_flags;                     /* VELOCITY_LOOP_STATUS_*状态位 */
    uint8 timing_valid;                     /* dt_s是否在有效控制周期范围内 */
} VelocityLoopHorizontalDiag_t;

/* 速度环使能切换，关闭或重新开启时会清空运行状态。 */
void Velocity_Loop_Set_Enabled(uint8 enabled);
uint8 Velocity_Loop_Is_Enabled(void);
/* 设置目标速度，单位cm/s；加速度前馈清零。 */
void Velocity_Loop_Set_Target_Velocity(float vx_earth_cms, float vy_earth_cms);
/* 设置目标速度和加速度前馈，均在惯性系下表达。 */
void Velocity_Loop_Set_Target_State(float vx_earth_cms,
                                    float vy_earth_cms,
                                    float ax_earth_cms2,
                                    float ay_earth_cms2);
void Velocity_Loop_Get_Target_Velocity(float *vx_earth_cms, float *vy_earth_cms);
/* 清空PID积分、D滤波、诊断和角速度前馈。 */
void Velocity_Loop_Reset_Runtime(void);
void Velocity_Loop_Reset_D_Filter_State(void);
/* 设置/读取指定轴速度D支路低通截止频率。 */
uint8 Velocity_Loop_Set_D_Filter_Hz(uint8 axis, float cutoff_hz);
float Velocity_Loop_Get_D_Filter_Hz(uint8 axis);
/* 设置/读取由目标角变化率生成的角速度前馈参数。 */
uint8 Velocity_Loop_Set_Rate_Feedforward_Parameters(uint8 axis,
                                                    float gain,
                                                    float limit_dps);
void Velocity_Loop_Get_Rate_Feedforward_Parameters(uint8 axis,
                                                   float *gain,
                                                   float *limit_dps);
void Velocity_Loop_Get_Rate_Feedforward_Diag(VelocityLoopRateFeedforwardDiag_t *diag);
uint8 Velocity_Loop_Get_Axis_Diag(uint8 axis, VelocityLoopAxisDiag_t *diag);
void Velocity_Loop_Get_Horizontal_Diag(VelocityLoopHorizontalDiag_t *diag);
/* 主控制入口：根据速度误差输出Roll/Pitch目标角。 */
void Velocity_Loop_Update_Attitude_Target(uint8 velocity_ready,
                                          float measured_vx_body_cms,
                                          float measured_vy_body_cms,
                                          float yaw_deg,
                                          float dt_s,
                                          float captured_roll,
                                          float captured_pitch,
                                          float *target_roll,
                                          float *target_pitch);
/* 兼容旧监控页面的简化诊断接口。 */
void Velocity_Loop_Get_Diag(float *pitch_corr_deg,
                            float *roll_corr_deg,
                            float *fb_vx_cms,
                            float *fb_vy_cms,
                            float *x_i_out,
                            float *y_i_out,
                            uint8 *active);

#endif
