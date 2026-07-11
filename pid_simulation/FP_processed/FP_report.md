# FP Log Diagnostic Report

## Basic Info
- input: `FP.csv`
- rows: `671`
- segment: `all`
- time axis: `sample`
- sample rate: `not provided`

## Sample Validity
- 位置环未参与：`position_loop_active` 全程为 0，本次日志不支持位置环闭环性能判断。
- all-zero fields: `x_pos_fb_cm, x_v_target_cms, y_pos_fb_cm, position_loop_active`

## X/Pitch Diagnostics
- 速度环: 目标基本为 0，但速度反馈持续波动，优先检查光流/融合速度估计。
- 异常段摘要：velocity[496:498], velocity[507:509], velocity[519:522], velocity[549:551], velocity[621:623], velocity[629:635], velocity[640:644], velocity[652:656]

| Stage | MAE | RMS | Peak | Corr | Lag(samples) | Lag(s) | Dom Freq | Dom Period |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| velocity | 3.497 | 9.193 | 45.366 | n/a | n/a | n/a | 0.049 | 20.333 |
| angle | 1.385 | 1.546 | 5.119 | -0.326 | -433 | n/a | 0.001 | 671.000 |
| rate | 1.366 | 4.230 | 65.155 | 0.721 | 0 | n/a | 0.055 | 18.135 |

## Y/Roll Diagnostics
- 角速度环: 跟踪较差，优先检查速率环 PID、滤波和 IMU 噪声。
- 速度环: 速度环跟踪差，但内环基本正常，优先检查速度环参数或速度反馈链。
- 异常段摘要：velocity[517:521], velocity[559:561], velocity[627:630], velocity[636:638], velocity[648:653], velocity[656:663], rate[436:438], rate[655:658]

| Stage | MAE | RMS | Peak | Corr | Lag(samples) | Lag(s) | Dom Freq | Dom Period |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| velocity | 4.510 | 11.705 | 69.380 | -0.001 | -651 | n/a | 0.054 | 18.639 |
| angle | 1.897 | 2.139 | 8.636 | -0.143 | 189 | n/a | 0.003 | 335.500 |
| rate | 1.718 | 4.541 | 47.110 | 0.418 | 0 | n/a | 0.054 | 18.639 |

## Abnormal Segment Summary
- X/Pitch velocity: [496:498], [507:509], [519:522], [549:551], [621:623], [629:635], [640:644], [652:656]
- X/Pitch rate: [436:438], [500:502]
- Y/Roll velocity: [517:521], [559:561], [627:630], [636:638], [648:653], [656:663]
- Y/Roll rate: [436:438], [655:658]

## Suggested Additional Fields
- 补采位置目标 `x/y setpoint`，否则无法严格判断位置环误差。
- 补采最终执行器输出或 PWM/油门，用于判断限幅与混控问题。
- 补采限幅标志、视觉有效标志和实际发送频率，提升自动诊断准确性。
