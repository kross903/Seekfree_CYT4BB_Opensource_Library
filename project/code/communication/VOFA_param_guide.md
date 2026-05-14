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

### 10. X 方向速度环 PID

命令前缀：`V`

`V` 组调 `vel_pid_x`，反馈量是光流滤波后的 `velocity_x_cms`，PID 输出是前后方向水平加速度指令 `ax_cmd_cms2`，再换算成 `pitch_corr_deg` 送入姿态环。

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `V1` | `V1=1.70!` | X 速度环 `Kp` |
| `V2` | `V2=0.00!` | X 速度环 `Ki` |
| `V3` | `V3=0.00!` | X 速度环 `Kd` |
| `V4` | `V4=85.0!` | X 速度环加速度指令限幅 `output_limit`，单位 `cm/s^2` |
| `V5` | `V5=20.0!` | X 速度环积分限幅 `integral_limit` |

### 11. Y 方向速度环 PID

命令前缀：`W`

`W` 组调 `vel_pid_y`，反馈量是光流滤波后的 `velocity_y_cms`，PID 输出是左右方向水平加速度指令 `ay_cmd_cms2`，再换算成 `roll_corr_deg` 送入姿态环。

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `W1` | `W1=1.70!` | Y 速度环 `Kp` |
| `W2` | `W2=0.00!` | Y 速度环 `Ki` |
| `W3` | `W3=0.00!` | Y 速度环 `Kd` |
| `W4` | `W4=85.0!` | Y 速度环加速度指令限幅 `output_limit`，单位 `cm/s^2` |
| `W5` | `W5=20.0!` | Y 速度环积分限幅 `integral_limit` |

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
| `FB=0!` 或 `FB!` | 光流速度旋转到世界系后的速度与积分位置 |
| `FC=0!` 或 `FC!` | 速度积分位置与光流单帧位移积分位置对比 |
| `FD=0!` 或 `FD!` | 高度环监视：I0 实际高度, I1 目标高度, I2 误差, I3 PID输出, I4 P项, I5 I项, I6 D项, I7 base_throttle |
| `FE=0!` 或 `FE!` | 光流原始输入诊断：UP-FLOW-302 模式下回传原始 x/y、valid、状态、新帧标志、帧间隔 |
| `FF=0!` 或 `FF!` | Roll/Pitch 角度环监视：目标角、实际角、I项输出、低通后的总输出 |

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

旋转补偿路径说明：

- `optical_flow_x_rad` / `optical_flow_y_rad` 是光流原始积分值经过传感器选通配置后的后端角位移。UP-FLOW-302 当前先交换 X/Y；`X_SIGN` / `Y_SIGN` 作用在交换后的机体 / 后端轴上。
- `optical_flow_gyro_y_delta_rad` / `optical_flow_gyro_x_delta_rad` 来自 250us 控制环中对 `icm42688_gyro_y` / `icm42688_gyro_x` 的累计，到光流更新周期时一次性消费。
- 去旋转补偿公式为 `flow_comp_x = flow_x + Kx * sign_xy * gyro_y_delta`，`flow_comp_y = flow_y + Ky * sign_yx * gyro_x_delta`。
- PMW3901 旧光流中 `flow_y_rad` 与 `gyro_x_delta_rad` 同号，因此 `sign_yx = -1`。
- UP-FLOW-302 当前配置中，机体 Y 轴光流使用 `Y_SIGN = +1` 修正后，`flow_y_rad` 应与 `gyro_x_delta_rad` 同号；若再次观察到二者反号，优先检查 Y 轴光流方向配置，而不是先改补偿符号。

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
| `V` | X 方向速度环 `Kp Ki Kd OutLim IntLim` |
| `W` | Y 方向速度环 `Kp Ki Kd OutLim IntLim` |
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
7. `acc_raw_norm`
8. `acc_filtered_norm`

说明：

- `acc_raw_norm` 是原始三轴加速度模值
- `acc_filtered_norm` 是 Roll / Pitch / Yaw 三轴 Biquad 滤波后的模值，也是当前 Mahony 姿态解算传入的 `acc_norm`

### `F6`：Roll / Pitch 角度与角速度对比

返回顺序：

1. `current_roll`
2. `target_angle_roll`
3. `current_gyro_roll`
4. `target_roll_rate`（角度环输出低通后的目标 Roll 角速度）
5. `current_pitch`
6. `target_angle_pitch`
7. `current_gyro_pitch`
8. `target_pitch_rate`（角度环输出低通后的目标 Pitch 角速度）

### `FF`：Roll / Pitch 角度环监视

返回顺序：

1. `target_angle_roll`
2. `current_roll`
3. `angle_pid_roll.Ki * angle_pid_roll.integral`
4. `target_roll_rate`（角度环总输出，已低通后送入 Roll 角速度环）
5. `target_angle_pitch`
6. `current_pitch`
7. `angle_pid_pitch.Ki * angle_pid_pitch.integral`
8. `target_pitch_rate`（角度环总输出，已低通后送入 Pitch 角速度环）

Roll/Pitch 角度环输出经过 `ANGLE_RATE_CMD_LPF_ALPHA = 0.04` 的一阶低通后再送入角速度环。FF 用于观察角度目标、实际追踪、积分项贡献和最终喂给角速度环的目标角速度。

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

### `F8`：速度环监视

返回顺序：

1. `fb_vx_cms`——速度环当前反馈，来自光流滤波速度
2. `vx_target_cms`
3. `x_i_out`
4. `pitch_corr_deg`
5. `fb_vy_cms`——速度环当前反馈，来自光流滤波速度
6. `vy_target_cms`
7. `y_i_out`
8. `roll_corr_deg`

### `F9`：加速度积分速度与光流速度对比

返回顺序：

1. `X_V`
2. `pmw_flow_state.velocity_x_cms`
3. `pmw_flow_state.velocity_x_fused_cms`（当前等于 `velocity_x_cms`）
4. `pmw_flow_state.velocity_x_fused_residual_cms`（关闭融合后应为 0）
5. `Y_V`
6. `pmw_flow_state.velocity_y_cms`
7. `pmw_flow_state.velocity_y_fused_cms`（当前等于 `velocity_y_cms`）
8. `pmw_flow_state.velocity_y_fused_residual_cms`（关闭融合后应为 0）

F9 当前用于观察加速度积分速度与光流速度差异：`X_V` / `Y_V` 是原 IMU 加速度积分速度，`velocity_*_cms` 是光流滤波速度。加速度融合已关闭，`velocity_*_fused_cms` 仅作为兼容字段保留并等于 `velocity_*_cms`，速度环 PID 使用 `velocity_*_cms` 作为反馈。

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
H1=3.00!
H2=0.00!
H3=0.00!
```

注意：

- 目标高度固定为 100cm（1m），修改需重新编译 `height_manager.c` 中的 `HEIGHT_TARGET_CM`
- Kp=3 时误差 100cm 输出=300（打满限幅），误差缩小到 ~33cm 才开始线性收敛
- 高度环具备中心使能锁存机制：实际高度与目标差异 ≤ 50cm 才激活，disarm 后自动复位

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

### 设置 X 方向速度环 PID

```text
V1=1.70!
V2=0.00!
V3=0.00!
V4=85.0!
V5=20.0!
```

### 设置 Y 方向速度环 PID

```text
W1=1.70!
W2=0.00!
W3=0.00!
W4=85.0!
W5=20.0!
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
FA!
FB!
FC!
FD!
FE!
FF!
```

### 强制失锁

```text
00=0!
```

## 使用建议

- 一次发送一条完整命令，必须以 `!` 结尾
- 建议连续命令之间留出少量时间，避免无线串口链路堆积
- 如果需要看某类参数回读，先发送对应设置命令，再切到 `F1`
- 如果需要观察实时状态，优先使用 `F0 / F2 / F3 / F4 / F5 / F6 / FF`
- 高度环调参建议先开 `FD!` 观察 I0/I1/I2/I3，只调 Kp（H1）直到高度收敛稳定再考虑加 Kd

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

1. `pmw_flow_state.velocity_x_cms`
2. `pmw_flow_state.velocity_y_cms`
3. `pmw_flow_state.yaw_deg`
4. `pmw_flow_state.velocity_x_world_cms`
5. `pmw_flow_state.velocity_y_world_cms`
6. `pmw_flow_state.position_x_world_cm`
7. `pmw_flow_state.position_y_world_cm`
8. `world_position_valid`

说明：世界系速度由光流速度按当前 yaw 旋转得到，世界系位置由世界系速度积分得到。当前只作为相对里程计观察量，未接入控制。

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
### `FD`：高度环全要素监视（新增）

发送：

```text
FD!
```

返回顺序：

1. `actual_cm`——ToF 滤波后实际高度（cm）
2. `target_cm`——目标高度，当前固定 100cm（1m）
3. `error_cm`——高度误差 = 目标 - 实际（cm）
4. `HeightSpeedPID_output`——高度 PID 整体输出，叠加到四路混控油门上
5. `p_term`——比例项 = Kp × error
6. `i_term`——积分项 = Ki × integral（Ki=0 时始终为 0）
7. `d_term`——微分项 = Kd × derivative（Kd=0 时始终为 0）
8. `base_throttle`——当前基础油门，便于对比 PID 修正量与基础油门的比例

调参常用观察方式：

- **高度是否收敛**：观察 I0（实际）是否靠近 I1（目标）
- **PID 修正幅度**：观察 I3（PID 输出）vs I7（base_throttle），I3 超过 I7 的一半说明增益偏大
- **各项贡献分离**：观察 I4/I5/I6，判断是 P 项还是 I/D 项导致问题
- **高度环尚未激活时**：飞机离地后 I3 始终为 0，说明实际高度与目标差超过 50cm，高度环仍在等待进入范围

### `FE`：光流原始输入诊断

发送：

```text
FE!
```

UP-FLOW-302 模式返回顺序：

1. `FLOW_SENSOR_SELECTED`——当前光流源，`1` 表示 UP-FLOW-302
2. `upflow302_x`——UP-FLOW-302 原始 X 通道；当前配置中映射到机体 / 后端 Y 轴，`Y_SIGN = +1`，不取反
3. `upflow302_y`——UP-FLOW-302 原始 Y 通道；当前配置中映射到机体 / 后端 X 轴，且符号取反
4. `upflow302_valid`——模块有效标志，正常应为 `245`
5. `upflow302_state_flag`——最近一帧校验通过标志
6. `upflow302_finsh_flag`——最近是否收到新帧，PIT 消费后会清零
7. `upflow302_us`——模块回传的帧间隔，单位按模块协议为 us
8. `PMWFlowState.dt_s`——当前后端实际使用的 dt

当前光流方向配置：

- PMW3901：原始 `delta_x` 对应后端 X，原始 `delta_y` 对应后端 Y，不交换轴，不取反。
- UP-FLOW-302：原始 `upflow302_x` 对应机体 / 后端 Y，原始 `upflow302_y` 对应机体 / 后端 X；代码中 `FLOW_SENSOR_UPFLOW302_SWAP_XY = 1`。符号作用在交换后的机体 / 后端轴上：`X_SIGN = -1` 作用于机体 X，也就是原始 `upflow302_y`；`Y_SIGN = +1` 作用于机体 Y，也就是原始 `upflow302_x`。

## 当前方向关系速查

工程中涉及方向的传感器和坐标关系如下：

| 模块 | 当前方向关系 | 代码位置 |
|---|---|---|
| 通用机体 / 世界系约定 | X 向前，Y 向右，Z 向下。下视视觉和光流世界系位置都按这个约定解释。 | `vision/drone_ipm.c`, `vision/ipm_interface.h`, `pmw/pmw_flow.c` |
| ICM42688 IMU | 原始 X/Y/Z 经过固定安装补偿后进入姿态解算；当前未启用额外 X/Y/Z 取反。`icm42688_acc_x -> Roll_acc`，`icm42688_acc_y -> Pitch_acc`，`icm42688_acc_z -> Yaw_acc`；`icm42688_gyro_x -> Roll_g`，`icm42688_gyro_y -> Pitch_g`，`icm42688_gyro_z -> Yaw_g`。 | `icm/icm42688.c`, `icm/icm.c` |
| PMW3901 光流 | 原始 `pmw3901_delta_x -> 后端 X`，原始 `pmw3901_delta_y -> 后端 Y`，不交换轴，X/Y 符号均为 `+1`。这是旧光流已验证正确的方向约定。 | `libraries/zf_device/zf_device_pmw3901.c`, `pmw/flow_sensor_config.h` |
| UP-FLOW-302 光流 | 原始 `upflow302_x -> 机体 / 后端 Y`，原始 `upflow302_y -> 机体 / 后端 X`，因此 `FLOW_SENSOR_UPFLOW302_SWAP_XY = 1`；符号作用在交换后的机体 / 后端轴上，当前 `X_SIGN = -1`、`Y_SIGN = +1`。 | `libraries/zf_device/zf_device_upflow302.c`, `pmw/flow_sensor_config.h`, `pmw/pmw_flow.c` |
| 光流速度 / 位置后端 | `flow_comp_x` 解释为机体 X 向光流角位移，`flow_comp_y` 解释为机体 Y 向光流角位移；速度映射中 X 符号为 `+1`，Y 符号为 `-1`。世界系速度 / 位置再由当前 yaw 旋转得到。 | `pmw/pmw_flow.c` |
| DL1B ToF | 标量视线距离，不提供 X/Y 方向；代码按下向高度使用，并补偿安装关系：ToF 位于机体 `+Y` 约 7cm，光流位于机体 `-Y` 约 7cm，ToF 光束朝 `+Y` 倾斜约 10 度。 | `kalman_filter/height_manager.c` |
| 下视摄像头 / IPM | 图像 `u` 向右，对应机体 / 世界 Y 向右；图像 `v` 向下，对应机体 / 世界 X 向前；图像中心对应正下方地面点。 | `vision/drone_ipm.c`, `vision/drone_ipm_embedded.c`, `vision/ipm_interface.h` |
| 红外定位 | 使用 IPM 输出的地面坐标，因此 `ir_pose_estimate.x` 为前向坐标，`ir_pose_estimate.y` 为右向坐标，`ir_pose_estimate.z` 为估计高度 / 尺度相关量。 | `vision/ir_localization.c`, `vision/ir_localization.h` |
