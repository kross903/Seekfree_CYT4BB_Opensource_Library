# FP Log Diagnostic Report

## Basic Info
- input: `FP.csv`
- rows: `854`
- segment: `all`
- time axis: `time_s`
- sample rate: `169.0`

## Sample Validity
- 位置环未参与：`position_loop_active` 全程为 0，本次日志不支持位置环闭环性能判断。
- all-zero fields: `x_pos_fb_cm, x_v_target_cms, y_pos_fb_cm, y_v_target_cms, position_loop_active, reserved`

## X/Pitch Diagnostics
- 异常段摘要：velocity[543:545], velocity[768:770], angle[688:690], rate[687:690]

| Stage | MAE | RMS | Peak | Corr | Lag(samples) | Lag(s) | Dom Freq | Dom Period |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| velocity | 3.785 | 7.048 | 48.170 | n/a | n/a | n/a | 7.718 | 0.130 |
| angle | 0.490 | 0.601 | 3.964 | -0.016 | -391 | -2.314 | 0.198 | 5.053 |
| rate | 2.330 | 4.027 | 31.508 | 0.512 | 0 | 0.000 | 22.362 | 0.045 |

## Y/Roll Diagnostics
- 速度环: 目标基本为 0，但速度反馈持续波动，优先检查光流/融合速度估计。
- 速度环: 速度环跟踪差，但内环基本正常，优先检查速度环参数或速度反馈链。
- 异常段摘要：velocity[396:398], velocity[405:409], velocity[557:559], velocity[611:614], velocity[669:673], rate[487:491]

| Stage | MAE | RMS | Peak | Corr | Lag(samples) | Lag(s) | Dom Freq | Dom Period |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| velocity | 7.415 | 12.943 | 67.628 | n/a | n/a | n/a | 7.520 | 0.133 |
| angle | 1.585 | 2.143 | 4.299 | -0.659 | -516 | -3.053 | 0.198 | 5.053 |
| rate | 3.620 | 5.813 | 30.025 | 0.532 | 0 | 0.000 | 9.301 | 0.108 |

## Abnormal Segment Summary
- X/Pitch velocity: [543:545], [768:770]
- X/Pitch angle: [688:690]
- X/Pitch rate: [687:690]
- Y/Roll velocity: [396:398], [405:409], [557:559], [611:614], [669:673]
- Y/Roll rate: [487:491]

## Suggested Additional Fields
- 补采位置目标 `x/y setpoint`，否则无法严格判断位置环误差。
- 补采最终执行器输出或 PWM/油门，用于判断限幅与混控问题。
- 补采限幅标志、视觉有效标志和实际发送频率，提升自动诊断准确性。
