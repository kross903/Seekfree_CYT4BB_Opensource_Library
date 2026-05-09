# VOFA 参数与指令说明

本文根据当前工程中的 [vofa.c](./vofa.c) 与相关控制接口整理，适用于当前无人机项目。

## 串口配置

- 串口：`UART_1`
- 波特率：`115200`
- TX 引脚：`UART1_TX_P04_1`
- RX 引脚：`UART1_RX_P04_0`

## 指令格式

当前 VOFA 指令采用如下格式：

```text
<命令><项目>=<数值>!
```

例如：

```text
R1=3.50!
T2=0.21!
S1=450!
```

说明：

- 第 1 个字符：命令类别
- 第 2 个字符：项目编号
- `=` 后：数值
- `!`：一条命令结束符

## 可调参数总览

### 1. Roll 姿态环 PID

命令前缀：`R`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `R1` | `R1=3.50!` | Roll 姿态环 `Kp` |
| `R2` | `R2=0.00!` | Roll 姿态环 `Ki` |
| `R3` | `R3=0.10!` | Roll 姿态环 `Kd` |

### 2. Roll 角速度环 PID

命令前缀：`r`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `r1` | `r1=1.60!` | Roll 角速度环 `Kp` |
| `r2` | `r2=0.00!` | Roll 角速度环 `Ki` |
| `r3` | `r3=0.12!` | Roll 角速度环 `Kd` |
| `r4` | `r4=100.0!` | Roll 角速度环积分限幅 `integral_limit` |

注意：

- `r4` 当前不是输出限幅，而是积分限幅。

### 3. Pitch 姿态环 PID

命令前缀：`P`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `P1` | `P1=3.50!` | Pitch 姿态环 `Kp` |
| `P2` | `P2=0.00!` | Pitch 姿态环 `Ki` |
| `P3` | `P3=0.10!` | Pitch 姿态环 `Kd` |

### 4. Pitch 角速度环 PID

命令前缀：`p`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `p1` | `p1=1.60!` | Pitch 角速度环 `Kp` |
| `p2` | `p2=0.00!` | Pitch 角速度环 `Ki` |
| `p3` | `p3=0.12!` | Pitch 角速度环 `Kd` |
| `p4` | `p4=100.0!` | Pitch 角速度环积分限幅 `integral_limit` |

### 5. Yaw 姿态环 PID

命令前缀：`Y`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `Y1` | `Y1=3.00!` | Yaw 姿态环 `Kp` |
| `Y2` | `Y2=0.00!` | Yaw 姿态环 `Ki` |
| `Y3` | `Y3=0.00!` | Yaw 姿态环 `Kd` |

### 6. Yaw 角速度环 PID

命令前缀：`y`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `y1` | `y1=1.00!` | Yaw 角速度环 `Kp` |
| `y2` | `y2=0.00!` | Yaw 角速度环 `Ki` |
| `y3` | `y3=0.00!` | Yaw 角速度环 `Kd` |
| `y4` | `y4=10.0!` | Yaw 角速度环积分限幅 `integral_limit` |

### 7. 高度环 PID

命令前缀：`H`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `H1` | `H1=125.00!` | 高度环 `Kp` |
| `H2` | `H2=0.00!` | 高度环 `Ki` |
| `H3` | `H3=0.00!` | 高度环 `Kd` |

### 8. 目标油门

命令前缀：`S`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `S1` | `S1=450!` | 设置目标油门 |

注意：

- 当前代码只检查第 1 个字符是 `S`，项目位不会单独分支，因此通常约定发 `S1=...!`

### 9. 姿态补偿角

命令前缀：`T`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `T1` | `T1=-0.60!` | Roll 补偿角 |
| `T2` | `T2=0.21!` | Pitch 补偿角 |
| `T3` | `T3=0.00!` | Yaw 补偿角 |

### 10. Roll 摇杆 PID

命令前缀：`J`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `J1` | `J1=1.00!` | Roll 摇杆 PID `Kp` |
| `J2` | `J2=0.00!` | Roll 摇杆 PID `Ki` |
| `J3` | `J3=0.00!` | Roll 摇杆 PID `Kd` |
| `J4` | `J4=20.0!` | Roll 摇杆 PID 输出限幅 `output_limit` |
| `J5` | `J5=10.0!` | Roll 摇杆 PID 积分限幅 `integral_limit` |

### 11. Pitch 摇杆 PID

命令前缀：`K`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `K1` | `K1=1.00!` | Pitch 摇杆 PID `Kp` |
| `K2` | `K2=0.00!` | Pitch 摇杆 PID `Ki` |
| `K3` | `K3=0.00!` | Pitch 摇杆 PID `Kd` |
| `K4` | `K4=20.0!` | Pitch 摇杆 PID 输出限幅 `output_limit` |
| `K5` | `K5=10.0!` | Pitch 摇杆 PID 积分限幅 `integral_limit` |

## 非参数控制指令

### 1. 切换回传模式

命令前缀：`F`

| 指令 | 含义 |
|---|---|
| `F0=0!` 或 `F0!` | 默认飞行状态回传 |
| `F1=0!` 或 `F1!` | PID 参数回读 |
| `F2=0!` 或 `F2!` | 定位 / 高度观察 |
| `F3=0!` 或 `F3!` | VOFA 链路测试 |
| `F4=0!` 或 `F4!` | 陀螺仪原始值与补偿观察 |
| `F5=0!` 或 `F5!` | 加速度原始值与补偿观察 |
| `F6=0!` 或 `F6!` | Roll/Pitch 角度与角速度对比 |
| `F7=0!` 或 `F7!` | ADC 原始值、滤波值与目标角 |
| `F8=0!` 或 `F8!` | 光流速度闭环综合监视 |
| `F9=0!` 或 `F9!` | 加速度积分速度与光流速度对比 |
| `FA=0!` 或 `FA!` | 加速度水平化残差诊断 |
| `FB=0!` 或 `FB!` | 融合速度旋转到世界系后的速度与积分位置 |
| `FC=0!` 或 `FC!` | 速度积分位置与光流单帧位移积分位置对比 |

说明：

- 代码实际只读取第 2 个字符来决定模式，因此 `F0!` 和 `F0=0!` 都能被识别。

### 2. 强制失锁

| 指令 | 含义 |
|---|---|
| `00=0!` | 调用 `Flight_Control_Disarm()` |

说明：

- 只有第 1 个字符和第 2 个字符分别为 `'0'`、`'0'` 时才会触发失锁。

## 回传数据说明

当前 `send_date_back()` 会根据 `send_back_flag` 返回 8 路浮点数据。

### `F0`：默认飞行状态回传

返回顺序：

1. `optical_flow_x_rad`
2. `optical_flow_gyro_y_delta_rad`
3. `optical_flow_comp_x_rad`
4. `optical_flow_velocity_x_raw_cms`
5. `optical_flow_y_rad`
6. `optical_flow_gyro_x_delta_rad`
7. `optical_flow_comp_y_rad`
8. `optical_flow_velocity_y_raw_cms`

### `F1`：PID 参数回读

会根据最近一次设置命令的类别来决定回读哪一组参数：

| 最近发送的命令前缀 | 回读内容 |
|---|---|
| `R` | Roll 姿态环 `Kp Ki Kd` |
| `r` | Roll 角速度环 `Kp Ki Kd` |
| `P` | Pitch 姿态环 `Kp Ki Kd` |
| `p` | Pitch 角速度环 `Kp Ki Kd` |
| `Y` | Yaw 姿态环 `Kp Ki Kd` |
| `y` | Yaw 角速度环 `Kp Ki Kd` |
| `H` | 高度环 `Kp Ki Kd` |
| `J` | Roll 摇杆 PID `Kp Ki Kd OutLim IntLim` |
| `K` | Pitch 摇杆 PID `Kp Ki Kd OutLim IntLim` |
| `S` | 当前目标油门 |
| `T` | 特殊姿态/补偿调试数据 |

其中：

- `T` 回读的不是单纯三个补偿参数，而是：
  1. `current_roll`
  2. `current_pitch`
  3. `current_yaw`
  4. `icm42688_acc_x`
  5. `icm42688_acc_y`
  6. `icm42688_acc_z`
  7. `roll_compensation`
  8. `pitch_compensation`

### `F2`：定位 / 高度观察

返回顺序：

1. `ir_pose_estimate.x`
2. `g_height_result.height_cm`
3. `ir_pose_estimate.z`
4. `tof_height_true_filt * 100.0f`
5. `0`
6. `current_roll`
7. `current_pitch`
8. `current_yaw`

### `F3`：链路测试

返回顺序：

1. 测试波形 `build_vofa_test_channel()`
2. `current_roll`
3. `current_pitch`
4. `current_yaw`
5. `g_height_result.height_cm`
6. `dl1b_distance_mm`
7. `dl1b_finsh_flag`
8. `0`

### `F4`：陀螺仪调试

返回顺序：

1. `icm42688_gyro_x_r`
2. `icm42688_gyro_y_r`
3. `icm42688_gyro_z_r`
4. `icm42688_gyro_x`
5. `icm42688_gyro_y`
6. `icm42688_gyro_z`
7. `angle_roll_compensation`
8. `angle_pitch_compensation`

### `F5`：加速度调试

返回顺序：

1. `icm42688_acc_x_r`
2. `icm42688_acc_y_r`
3. `icm42688_acc_z_r`
4. `icm42688_acc_x`
5. `icm42688_acc_y`
6. `icm42688_acc_z`
7. `angle_roll_compensation`
8. `angle_pitch_compensation`

### `F6`：Roll / Pitch 角度与角速度对比

返回顺序：

1. `current_roll`
2. `target_angle_roll`
3. `current_gyro_roll`
4. `target_roll_rate`
5. `current_pitch`
6. `target_angle_pitch`
7. `current_gyro_pitch`
8. `target_pitch_rate`

### `F7`：ADC 原始值、滤波值与目标角

返回顺序：

1. `adc_x`
2. `adc_y`
3. `adc_x_filtered`
4. `adc_y_filtered`
5. `adc_x_nomalize`
6. `adc_y_nomalize`
7. `target_angle_roll`
8. `target_angle_pitch`

### `F8`：光流速度闭环综合监视

返回顺序：

1. `pmw_flow_state.velocity_x_raw_cms`
2. `pmw_flow_state.velocity_x_cms`
3. `vx_target_cms`
4. `pitch_corr_deg`
5. `pmw_flow_state.velocity_y_raw_cms`
6. `pmw_flow_state.velocity_y_cms`
7. `vy_target_cms`
8. `roll_corr_deg`

### `F9`：加速度积分速度与光流速度对比

返回顺序：

1. `X_V`
2. `pmw_flow_state.velocity_x_cms`
3. `pmw_flow_state.velocity_x_fused_cms`
4. `pmw_flow_state.velocity_x_fused_residual_cms`
5. `Y_V`
6. `pmw_flow_state.velocity_y_cms`
7. `pmw_flow_state.velocity_y_fused_cms`
8. `pmw_flow_state.velocity_y_fused_residual_cms`

F9 当前用于观察速度融合效果：`X_V` / `Y_V` 是加速度积分速度，`velocity_*_cms` 是光流滤波速度，`velocity_*_fused_cms` 是融合速度，`velocity_*_fused_residual_cms = 光流速度 - 融合速度`。融合速度目前只用于观察，未接入速度环 PID。

## 上位机常用发送示例

### 设置 Roll 姿态环

```text
R1=3.50!
R2=0.00!
R3=0.10!
```

### 设置 Pitch 角速度环

```text
p1=1.60!
p2=0.00!
p3=0.12!
p4=100.0!
```

### 设置高度环

```text
H1=125.00!
H2=0.00!
H3=0.00!
```

### 设置油门

```text
S1=450!
```

### 设置姿态补偿角

```text
T1=-0.60!
T2=0.21!
T3=0.00!
```

### 设置 Roll 摇杆 PID

```text
J1=1.00!
J2=0.00!
J3=0.00!
J4=20.0!
J5=10.0!
```

### 设置 Pitch 摇杆 PID

```text
K1=1.00!
K2=0.00!
K3=0.00!
K4=20.0!
K5=10.0!
```

### 切换回传模式

```text
F0!
F1!
F2!
F3!
F4!
F5!
F6!
F7!
F8!
F9!
```

### 强制失锁

```text
00=0!
```

## 使用建议

- 一次发送一条完整命令，必须以 `!` 结尾
- 建议连续命令之间留出少量时间，避免无线串口链路堆积
- 如果需要看某类参数回读，先发送对应设置命令，再切到 `F1`
- 如果需要观察实时状态，优先使用 `F0 / F2 / F3 / F4 / F5`

## 附加诊断回传

### `FA`：加速度水平化残差诊断

发送：

```text
FA!
```

返回顺序：

1. `current_roll`
2. `current_pitch`
3. `ax_level_raw_mg`
4. `ay_level_raw_mg`
5. `acc_level_x_bias_mg`
6. `acc_level_y_bias_mg`
7. `ax_level_after_bias_mg`
8. `ay_level_after_bias_mg`

用途：固定不同 roll / pitch 倾角时，观察 `ax_level_after_bias_mg`、`ay_level_after_bias_mg` 是否随倾角线性变化，用于判断重力污染来自姿态误差、安装角误差或加速度比例误差。

### `FB`：世界系速度与位置积分诊断

发送：

```text
FB!
```

返回顺序：

1. `pmw_flow_state.velocity_x_fused_cms`
2. `pmw_flow_state.velocity_y_fused_cms`
3. `pmw_flow_state.yaw_deg`
4. `pmw_flow_state.velocity_x_world_cms`
5. `pmw_flow_state.velocity_y_world_cms`
6. `pmw_flow_state.position_x_world_cm`
7. `pmw_flow_state.position_y_world_cm`
8. `world_position_valid`

说明：世界系速度由融合速度按当前 yaw 旋转得到，世界系位置由世界系速度积分得到。当前只作为相对里程计观察量，未接入控制。

### `FC`：两种世界系位置积分路径对比

发送：

```text
FC!
```

返回顺序：

1. `pmw_flow_state.position_x_world_cm`
2. `pmw_flow_state.position_y_world_cm`
3. `pmw_flow_state.flow_position_x_world_cm`
4. `pmw_flow_state.flow_position_y_world_cm`
5. `pmw_flow_state.flow_delta_x_world_cm`
6. `pmw_flow_state.flow_delta_y_world_cm`
7. `pmw_flow_state.yaw_deg`
8. `flow_position_valid`

说明：`position_*_world_cm` 是原有“融合速度旋转到世界系后再积分”的位置；`flow_position_*_world_cm` 是新增“光流补偿后角位移 -> 单帧机体系位移 -> yaw 旋转 -> 世界系位移累加”的位置。两条路径并行存在，互不覆盖。
