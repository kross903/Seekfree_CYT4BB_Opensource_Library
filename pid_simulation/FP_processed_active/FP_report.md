# FP Log Diagnostic Report

## Basic Info
- input: `FP.csv`
- rows: `0`
- segment: `active`
- time axis: `sample`
- sample rate: `not provided`

## Sample Validity
- 筛选后的日志为空，无法分析该分段。
- all-zero fields: `x_pos_fb_cm, x_v_target_cms, x_v_fb_cms, pitch_target_deg, pitch_current_deg, pitch_rate_target_dps, pitch_rate_current_dps, y_pos_fb_cm, y_v_target_cms, y_v_fb_cms, roll_target_deg, roll_current_deg, roll_rate_target_dps, roll_rate_current_dps, position_loop_active, reserved`

## X/Pitch Diagnostics

| Stage | MAE | RMS | Peak | Corr | Lag(samples) | Lag(s) | Dom Freq | Dom Period |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| velocity | 0.000 | 0.000 | 0.000 | n/a | n/a | n/a | n/a | n/a |
| angle | 0.000 | 0.000 | 0.000 | n/a | n/a | n/a | n/a | n/a |
| rate | 0.000 | 0.000 | 0.000 | n/a | n/a | n/a | n/a | n/a |

## Y/Roll Diagnostics

| Stage | MAE | RMS | Peak | Corr | Lag(samples) | Lag(s) | Dom Freq | Dom Period |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| velocity | 0.000 | 0.000 | 0.000 | n/a | n/a | n/a | n/a | n/a |
| angle | 0.000 | 0.000 | 0.000 | n/a | n/a | n/a | n/a | n/a |
| rate | 0.000 | 0.000 | 0.000 | n/a | n/a | n/a | n/a | n/a |

## Abnormal Segment Summary
- No strong abnormal segments detected by the current rule thresholds.

## Suggested Additional Fields
- 检查 `--segment` 选择是否与当前日志匹配。
- 若要分析位置环区段，请重新采集 `position_loop_active=1` 的日志。
