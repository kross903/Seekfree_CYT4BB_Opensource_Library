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
B1=-2.10!
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
| `R1` | `R1=5.50!` | Roll 姿态环 `Kp` |
| `R2` | `R2=0.0001!` | Roll 姿态环 `Ki` |
| `R3` | `R3=0.00!` | Roll 姿态环 `Kd` |

### 2. Roll 角速度环 PID

命令前缀：`r`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `r1` | `r1=2.00!` | Roll 角速度环 `Kp` |
| `r2` | `r2=0.00!` | Roll 角速度环 `Ki` |
| `r3` | `r3=0.00!` | Roll 角速度环 `Kd` |
| `r4` | `r4=100.0!` | Roll 角速度环积分限幅 `integral_limit` |

注意：

- `r4` 当前不是输出限幅，而是积分限幅。

### 3. Pitch 姿态环 PID

命令前缀：`P`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `P1` | `P1=5.50!` | Pitch 姿态环 `Kp` |
| `P2` | `P2=0.0001!` | Pitch 姿态环 `Ki` |
| `P3` | `P3=0.00!` | Pitch 姿态环 `Kd` |

### 4. Pitch 角速度环 PID

命令前缀：`p`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `p1` | `p1=2.50!` | Pitch 角速度环 `Kp` |
| `p2` | `p2=0.00!` | Pitch 角速度环 `Ki` |
| `p3` | `p3=0.00!` | Pitch 角速度环 `Kd` |
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
| `y1` | `y1=5.50!` | Yaw 角速度环 `Kp` |
| `y2` | `y2=0.00!` | Yaw 角速度环 `Ki` |
| `y3` | `y3=0.00!` | Yaw 角速度环 `Kd` |
| `y4` | `y4=10.0!` | Yaw 角速度环积分限幅 `integral_limit` |

### 7. 高度环 PID

命令前缀：`H`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `H1` | `H1=4.00!` | 高度环 `Kp` |
| `H2` | `H2=0.0001!` | 高度环 `Ki` |
| `H3` | `H3=0.10!` | 高度环 `Kd` |

### 8. 目标油门

命令前缀：`S`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `S1` | `S1=450!` | 设置目标油门 |

注意：

- 当前代码只检查第 1 个字符是 `S`，项目位不会单独分支，因此通常约定发 `S1=...!`

### 9. IMU 安装补偿角

命令前缀：`T`

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `T1` | `T1=-0.60!` | IMU 加速度 Roll 安装补偿角 |
| `T2` | `T2=0.21!` | IMU 加速度 Pitch 安装补偿角 |
| `T3` | `T3=0.00!` | 预留 Yaw 补偿角 |

注意：`T1/T2` 会参与 ICM42688 加速度安装补偿，不用于调悬停姿态目标。

### 10. 悬停基准角补偿

命令前缀：`B`

`B` 组调 `Flight_Takeoff_Get_Captured_Angles()` 返回的悬停基准角。当前起飞采样基准为 `0deg / 0deg`，因此 `B1/B2` 会直接成为速度环叠加前的 Roll / Pitch 基准角。内部限幅为 `±10deg`。

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `B1` | `B1=-2.10!` | Roll 悬停基准角补偿，叠加到 `target_angle_roll` |
| `B2` | `B2=0.65!` | Pitch 悬停基准角补偿，叠加到 `target_angle_pitch` |

### 11. X 方向速度环 PID

命令前缀：`V`

`V` 组调 `vel_pid_x`，反馈量是 4kHz ICM 预测 + 25ms 光流校正后的 `velocity_x_fused_cms`，PID 输出是前后方向水平加速度指令 `ax_cmd_cms2`，再换算成 `pitch_corr_deg` 送入姿态环。当前 X 方向光流校正链路为：ToF 跳变拒绝、原始速度硬限幅、重力加速度量级输入斜率限制，然后按 `PMW_FLOW_VEL_LPF_CUTOFF_HZ = 8.0` 计算动态 alpha 做单级低通，再经过重力加速度量级输出斜率限制，最后用 `PMW_FLOW_VEL_DEADBAND_CMS = 5.0` 做输出零速死区。

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `V1` | `V1=3.00!` | X 速度环 `Kp` |
| `V2` | `V2=0.00!` | X 速度环 `Ki` |
| `V3` | `V3=0.00!` | X 速度环 `Kd` |
| `V4` | `V4=40.0!` | X 速度环加速度指令限幅 `output_limit`，单位 `cm/s^2` |
| `V5` | `V5=20.0!` | X 速度环积分限幅 `integral_limit` |

### 12. Y 方向速度环 PID

命令前缀：`W`

`W` 组调 `vel_pid_y`，反馈量是 4kHz ICM 预测 + 25ms 光流校正后的 `velocity_y_fused_cms`，PID 输出是左右方向水平加速度指令 `ay_cmd_cms2`，再换算成 `roll_corr_deg` 送入姿态环。当前 Y 方向光流校正链路为：ToF 跳变拒绝、原始速度硬限幅、重力加速度量级输入斜率限制，然后按 `PMW_FLOW_VEL_LPF_CUTOFF_HZ = 8.0` 计算动态 alpha 做单级低通，再经过重力加速度量级输出斜率限制，最后用 `PMW_FLOW_VEL_DEADBAND_CMS = 5.0` 做输出零速死区。

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `W1` | `W1=3.00!` | Y 速度环 `Kp` |
| `W2` | `W2=0.00!` | Y 速度环 `Ki` |
| `W3` | `W3=0.00!` | Y 速度环 `Kd` |
| `W4` | `W4=40.0!` | Y 速度环加速度指令限幅 `output_limit`，单位 `cm/s^2` |
| `W5` | `W5=20.0!` | Y 速度环积分限幅 `integral_limit` |

### 13. 红外 X 位置环 PID

命令前缀：`I`

`I` 组调 `pos_pid_x`。视觉原始识别有效时，位置环使用 `ir_pose_estimate.x` 作为 X 方向追踪误差；视觉原始识别丢失后，`FI` 中的 X 位置改为由光流 + ICM 融合速度积分预测，速度积分位移达到丢失瞬间锁存的位置距离或预测位置进入死区后回到 0。PID 输出成为 X 方向目标速度 `vx_target_cms`，交给已有光流速度环。起飞 ramp 完成并进入巡航分支后会自动接入位置环；调试时也可发送 `I0=1!` 或 `J0=1!` 手动开启，关闭发送 `I0=0!`。

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `I0` | `I0=1!` | 位置环使能，`1` 开启，`0` 关闭 |
| `I1` | `I1=0.60!` | X 位置环 `Kp`，单位近似 `cm/s per cm` |
| `I2` | `I2=0.00!` | X 位置环 `Ki` |
| `I3` | `I3=0.00!` | X 位置环 `Kd` |
| `I4` | `I4=35.0!` | X 位置环目标速度限幅 `output_limit`，单位 `cm/s` |
| `I5` | `I5=50.0!` | X 位置环积分限幅 `integral_limit` |

### 14. 红外 Y 位置环 PID

命令前缀：`J`

`J` 组调 `pos_pid_y`。视觉原始识别有效时，位置环使用 `ir_pose_estimate.y` 作为 Y 方向追踪误差；视觉原始识别丢失后，`FI` 中的 Y 位置同样由融合速度积分预测，直到积分位移达到锁存距离或进入死区后回到 0。PID 输出成为 Y 方向目标速度 `vy_target_cms`，交给已有光流速度环。

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `J0` | `J0=1!` | 位置环使能，`1` 开启，`0` 关闭 |
| `J1` | `J1=0.60!` | Y 位置环 `Kp`，单位近似 `cm/s per cm` |
| `J2` | `J2=0.00!` | Y 位置环 `Ki` |
| `J3` | `J3=0.00!` | Y 位置环 `Kd` |
| `J4` | `J4=35.0!` | Y 位置环目标速度限幅 `output_limit`，单位 `cm/s` |
| `J5` | `J5=50.0!` | Y 位置环积分限幅 `integral_limit` |

### 15. 光流陀螺去旋转增益

命令前缀：`G`

`G` 组调 `PMW_Flow_SetGyroCompGain()`，用于修正光流角位移中由机体旋转带来的残留量。`Gx` 对应 X 方向补偿增益，`Gy` 对应 Y 方向补偿增益；负数会被忽略，不会修改参数。

| 参数 | 指令示例 | 含义 |
|---|---|---|
| `Gx` | `Gx=0.82!` | X 方向去旋转增益 `g_pmw_flow_gyro_comp_gain_x` |
| `Gy` | `Gy=0.94!` | Y 方向去旋转增益 `g_pmw_flow_gyro_comp_gain_y` |

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
| `F9=0!` 或 `F9!` | 加速度积分速度、光流速度与融合速度对比 |
| `FA=0!` 或 `FA!` | 加速度水平化残差诊断 |
| `FB=0!` 或 `FB!` | 光流速度旋转到世界系后的速度与积分位置 |
| `FC=0!` 或 `FC!` | 速度积分位置与光流单帧位移积分位置对比 |
| `FD=0!` 或 `FD!` | 高度环监视：I0 实际高度, I1 目标高度, I2 误差, I3 PID输出, I4 P项, I5 I项, I6 D项, I7 base_throttle |
| `FE=0!` 或 `FE!` | 光流原始输入诊断：UP-FLOW-302 模式下回传原始 x/y、valid、状态、新帧标志、帧间隔 |
| `FF=0!` 或 `FF!` | Roll/Pitch 角度环监视：目标角、实际角、I项输出、低通后的总输出 |
| `FG=0!` 或 `FG!` | 红外正方形中心点：I0 中心 X, I1 中心 Y, I2 视觉高度, I3 正方形边长, I4 可追踪, I5 丢失标志, I7 本帧识别点数 |
| `FH=0!` 或 `FH!` | 红外丢失保护状态机诊断：I0 中心 X, I1 中心 Y, I2 原始检测成功, I3 丢失标志, I4 丢失计时, I5 有历史位姿, I6 5ms任务计数, I7 0.5s超时次数 |
| `FI=0!` 或 `FI!` | 红外位置闭环诊断：I0 位置环输入X, I1 位置环输入Y, I2 目标速度X, I3 目标速度Y, I4 位置环使能, I5 位置环激活, I6 位置输入有效, I7 光流速度可用 |
| `FJ=0!` 或 `FJ!` | 速度环/光流门控诊断：I0 速度环使能, I1 速度环激活, I2 光流valid, I3 光流used, I4 ToF高度, I5 ToF来源, I6 ToF失效计数, I7 光流dt |
| `FK=0!` 或 `FK!` | 加速度/速度反馈/光流原始值对齐诊断：I0/I1 水平化加速度, I2/I3 进入速度环的测量速度, I4/I5 光流原始水平速度, I6/I7 光流原始 count |
| `FL=0!` 或 `FL!` | 红外位置环 PID 调参诊断：I0/I4 死区后位置误差, I1/I5 P项, I2/I6 I项, I3/I7 输出目标速度 |
| `FM=0!` 或 `FM!` | X 轴多环控制链路：位置输入、目标速度、实际速度、Pitch 目标/实际、目标/实际 Pitch 角速度 |
| `FN=0!` 或 `FN!` | Y 轴多环控制链路：位置输入、目标速度、实际速度、Roll 目标/实际、目标/实际 Roll 角速度 |

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

返回通道：

- `I0`: `optical_flow_x_rad`——X 方向光流原始角位移，单位 rad
- `I1`: `optical_flow_gyro_y_delta_rad`——同一采样周期内由 Gyro Y 积分得到的 X 方向旋转补偿量，单位 rad
- `I2`: `optical_flow_comp_x_rad`——X 方向去旋转后的光流角位移，单位 rad
- `I3`: `optical_flow_velocity_x_raw_cms`——X 方向原始水平速度，单位 cm/s
- `I4`: `optical_flow_y_rad`——Y 方向光流原始角位移，单位 rad
- `I5`: `optical_flow_gyro_x_delta_rad`——同一采样周期内由 Gyro X 积分得到的 Y 方向旋转补偿量，单位 rad
- `I6`: `optical_flow_comp_y_rad`——Y 方向去旋转后的光流角位移，单位 rad
- `I7`: `optical_flow_velocity_y_raw_cms`——Y 方向原始水平速度，单位 cm/s

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
| `I` | 红外 X 位置环 `Kp Ki Kd OutLim IntLim Enable` |
| `J` | 红外 Y 位置环 `Kp Ki Kd OutLim IntLim Enable` |
| `S` | 当前目标油门 |
| `B` | 悬停基准角补偿与当前最终基准角 |
| `T` | 特殊姿态/补偿调试数据 |
| `G` | 光流陀螺去旋转增益 `Kx Ky` |

普通 PID / 单值组返回通道：

- `R/r/P/p/Y/y/H`：`I0`=`Kp`，`I1`=`Ki`，`I2`=`Kd`；`I3-I7` 为 0 占位
- `S`：`I0`=当前目标油门；`I1-I7` 为 0 占位
- `V/W`：`I0`=`Kp`，`I1`=`Ki`，`I2`=`Kd`，`I3`=`output_limit`，`I4`=`integral_limit`；`I5-I7` 为 0 占位
- `I/J`：`I0`=`Kp`，`I1`=`Ki`，`I2`=`Kd`，`I3`=`output_limit`，`I4`=`integral_limit`，`I5`=位置环使能状态；`I6-I7` 为 0 占位

其中：

- `T` 回读的不是单纯三个补偿参数，而是：
  - `I0`: `current_roll`——当前 Roll 姿态角，单位 deg
  - `I1`: `current_pitch`——当前 Pitch 姿态角，单位 deg
  - `I2`: `current_yaw`——当前 Yaw 姿态角，单位 deg
  - `I3`: `icm42688_acc_x`——补偿/滤波后的 X 轴加速度
  - `I4`: `icm42688_acc_y`——补偿/滤波后的 Y 轴加速度
  - `I5`: `icm42688_acc_z`——补偿/滤波后的 Z 轴加速度
  - `I6`: `roll_compensation`——Roll 安装/调试补偿角
  - `I7`: `pitch_compensation`——Pitch 安装/调试补偿角

- `B` 回读：
  - `I0`: `hover_roll_comp_deg`——Roll 悬停基准补偿角，来自 `B1`
  - `I1`: `hover_pitch_comp_deg`——Pitch 悬停基准补偿角，来自 `B2`
  - `I2`: `hover_base_roll_deg`——当前 Roll 悬停基准角，当前等于 `0 + B1`
  - `I3`: `hover_base_pitch_deg`——当前 Pitch 悬停基准角，当前等于 `0 + B2`
  - `I4`: `target_angle_roll`——最终 Roll 目标角，单位 deg
  - `I5`: `target_angle_pitch`——最终 Pitch 目标角，单位 deg
  - `I6`: `current_roll`——当前 Roll 姿态角，单位 deg
  - `I7`: `current_pitch`——当前 Pitch 姿态角，单位 deg

- `G` 回读：
  - `I0`: `gyro_comp_gain_x`——光流 X 方向陀螺去旋转补偿增益
  - `I1`: `gyro_comp_gain_y`——光流 Y 方向陀螺去旋转补偿增益
  - `I2`: `0`——占位，当前未使用
  - `I3`: `0`——占位，当前未使用
  - `I4`: `0`——占位，当前未使用
  - `I5`: `0`——占位，当前未使用
  - `I6`: `0`——占位，当前未使用
  - `I7`: `0`——占位，当前未使用

### `F2`：定位 / 高度观察

返回通道：

- `I0`: `ir_pose_estimate.x`——红外中心 X 坐标，单位 cm
- `I1`: `g_height_result.height_cm`——高度管理器融合高度，单位 cm
- `I2`: `ir_pose_estimate.z`——红外正方形尺度估计高度，单位 cm
- `I3`: `tof_height_true_filt * 100.0f`——ToF 滤波高度，单位 cm
- `I4`: `0`——占位，当前未使用
- `I5`: `current_roll`——当前 Roll 姿态角，单位 deg
- `I6`: `current_pitch`——当前 Pitch 姿态角，单位 deg
- `I7`: `current_yaw`——当前 Yaw 姿态角，单位 deg

### `F3`：链路测试

返回通道：

- `I0`: `build_vofa_test_channel()`——VOFA 链路测试波形
- `I1`: `current_roll`——当前 Roll 姿态角，单位 deg
- `I2`: `current_pitch`——当前 Pitch 姿态角，单位 deg
- `I3`: `current_yaw`——当前 Yaw 姿态角，单位 deg
- `I4`: `g_height_result.height_cm`——高度管理器融合高度，单位 cm
- `I5`: `dl1b_distance_mm`——DL1B ToF 原始距离，单位 mm
- `I6`: `dl1b_finsh_flag`——DL1B 本帧测距完成标志，1 表示有效完成
- `I7`: `0`——占位，当前未使用

### `F4`：陀螺仪调试

返回通道：

- `I0`: `icm42688_gyro_x_r`——ICM X 轴原始角速度
- `I1`: `icm42688_gyro_y_r`——ICM Y 轴原始角速度
- `I2`: `icm42688_gyro_z_r`——ICM Z 轴原始角速度
- `I3`: `icm42688_gyro_x`——补偿/滤波后的 X 轴角速度
- `I4`: `icm42688_gyro_y`——补偿/滤波后的 Y 轴角速度
- `I5`: `icm42688_gyro_z`——补偿/滤波后的 Z 轴角速度
- `I6`: `angle_roll_compensation`——Roll 姿态安装补偿角
- `I7`: `angle_pitch_compensation`——Pitch 姿态安装补偿角

### `F5`：加速度调试

返回通道：

- `I0`: `icm42688_acc_x_r`——ICM X 轴原始加速度
- `I1`: `icm42688_acc_y_r`——ICM Y 轴原始加速度
- `I2`: `icm42688_acc_z_r`——ICM Z 轴原始加速度
- `I3`: `icm42688_acc_x`——补偿/滤波后的 X 轴加速度
- `I4`: `icm42688_acc_y`——补偿/滤波后的 Y 轴加速度
- `I5`: `icm42688_acc_z`——补偿/滤波后的 Z 轴加速度
- `I6`: `acc_raw_norm`——原始三轴加速度模值
- `I7`: `acc_filtered_norm`——滤波后三轴加速度模值

说明：

- `acc_raw_norm` 是原始三轴加速度模值
- `acc_filtered_norm` 是 Roll / Pitch / Yaw 三轴 Biquad 滤波后的模值，也是当前 Mahony 姿态解算传入的 `acc_norm`

### `F6`：Roll / Pitch 角度与角速度对比

返回通道：

- `I0`: `current_roll`——当前 Roll 姿态角，单位 deg
- `I1`: `target_angle_roll`——Roll 目标角，单位 deg
- `I2`: `current_gyro_roll`——当前 Roll 角速度
- `I3`: `target_roll_rate`——角度环输出低通后的目标 Roll 角速度
- `I4`: `current_pitch`——当前 Pitch 姿态角，单位 deg
- `I5`: `target_angle_pitch`——Pitch 目标角，单位 deg
- `I6`: `current_gyro_pitch`——当前 Pitch 角速度
- `I7`: `target_pitch_rate`——角度环输出低通后的目标 Pitch 角速度

当前巡航阶段 `target_angle_roll = hover_base_roll_deg + roll_corr_deg`，其中 `hover_base_roll_deg` 由 `B1` 调节，`roll_corr_deg` 来自 Y 方向速度环。`target_angle_pitch` 同理由 `B2` 与 X 方向速度环共同决定。

### `F7`：ADC 原始值、滤波值与目标角

返回通道：

- `I0`: `adc_x`——X 通道 ADC 原始值
- `I1`: `adc_y`——Y 通道 ADC 原始值
- `I2`: `adc_x_filtered`——X 通道 ADC 滤波值
- `I3`: `adc_y_filtered`——Y 通道 ADC 滤波值
- `I4`: `adc_x_nomalize`——X 通道归一化值
- `I5`: `adc_y_nomalize`——Y 通道归一化值
- `I6`: `target_angle_roll`——当前 Roll 目标角，单位 deg
- `I7`: `target_angle_pitch`——当前 Pitch 目标角，单位 deg

### `F8`：速度环监视

返回通道：

- `I0`: `fb_vx_cms`——X 方向速度环反馈速度，即 ICM 预测 + 光流校正后的融合速度，单位 cm/s
- `I1`: `vx_target_cms`——X 方向速度环目标速度，单位 cm/s
- `I2`: `x_i_out`——X 方向速度环积分项输出
- `I3`: `pitch_corr_deg`——X 速度环输出换算得到的 Pitch 修正角，单位 deg
- `I4`: `fb_vy_cms`——Y 方向速度环反馈速度，即 ICM 预测 + 光流校正后的融合速度，单位 cm/s
- `I5`: `vy_target_cms`——Y 方向速度环目标速度，单位 cm/s
- `I6`: `y_i_out`——Y 方向速度环积分项输出
- `I7`: `roll_corr_deg`——Y 速度环输出换算得到的 Roll 修正角，单位 deg

说明：`fb_vx_cms` / `fb_vy_cms` 直接来自当前光流融合状态，因此即使速度环尚未 active，也能用 `F8!` 判断融合反馈速度是否已经有数；目标速度、积分项和姿态修正仍来自速度环诊断量。

### `F9`：加速度积分速度、光流速度与融合速度对比

返回通道：

- `I0`: `X_V_F`——4kHz ICM 预测使用的 X 方向滤波速度，单位 cm/s
- `I1`: `pmw_flow_state.velocity_x_cms`——X 方向光流滤波速度，单位 cm/s
- `I2`: `pmw_flow_state.velocity_x_fused_cms`——X 方向光流 + ICM 融合速度，单位 cm/s
- `I3`: `pmw_flow_state.velocity_x_fused_residual_cms`——X 方向光流滤波速度减去融合速度，单位 cm/s
- `I4`: `Y_V_F`——4kHz ICM 预测使用的 Y 方向滤波速度，单位 cm/s
- `I5`: `pmw_flow_state.velocity_y_cms`——Y 方向光流滤波速度，单位 cm/s
- `I6`: `pmw_flow_state.velocity_y_fused_cms`——Y 方向光流 + ICM 融合速度，单位 cm/s
- `I7`: `pmw_flow_state.velocity_y_fused_residual_cms`——Y 方向光流滤波速度减去融合速度，单位 cm/s

F9 当前用于观察 ICM 预测速度、光流校正速度和融合速度的差异：`X_V_F` / `Y_V_F` 是 4kHz 预测使用的 ICM 滤波速度，`velocity_*_cms` 是 25ms 光流校正速度，`velocity_*_fused_cms` 是速度环 PID 使用的快速融合反馈速度。

### `FA`：加速度水平化残差诊断

返回通道：

- `I0`: `current_roll`——当前 Roll 姿态角，单位 deg
- `I1`: `current_pitch`——当前 Pitch 姿态角，单位 deg
- `I2`: `ax_level_mg`——水平化后的 X 轴加速度，单位 mg
- `I3`: `ay_level_mg`——水平化后的 Y 轴加速度，单位 mg
- `I4`: `acc_level_x_bias_mg`——X 轴水平加速度零偏估计，单位 mg
- `I5`: `acc_level_y_bias_mg`——Y 轴水平加速度零偏估计，单位 mg
- `I6`: `ax_level_mg - acc_level_x_bias_mg`——扣零偏后的 X 轴水平加速度，单位 mg
- `I7`: `ay_level_mg - acc_level_y_bias_mg`——扣零偏后的 Y 轴水平加速度，单位 mg

用途：固定不同 Roll / Pitch 倾角时，观察 `I6/I7` 是否随倾角线性变化，用于判断重力污染来自姿态误差、安装角误差或加速度比例误差。

### `FB`：世界系速度与位置积分诊断

返回通道：

- `I0`: `pmw_flow_state.velocity_x_cms`——机体系 X 方向光流滤波速度，单位 cm/s
- `I1`: `pmw_flow_state.velocity_y_cms`——机体系 Y 方向光流滤波速度，单位 cm/s
- `I2`: `pmw_flow_state.yaw_deg`——当前用于旋转到世界系的 Yaw 角，单位 deg
- `I3`: `pmw_flow_state.velocity_x_world_cms`——世界系 X 方向速度，单位 cm/s
- `I4`: `pmw_flow_state.velocity_y_world_cms`——世界系 Y 方向速度，单位 cm/s
- `I5`: `pmw_flow_state.position_x_world_cm`——世界系 X 方向速度积分位置，单位 cm
- `I6`: `pmw_flow_state.position_y_world_cm`——世界系 Y 方向速度积分位置，单位 cm
- `I7`: `pmw_flow_state.reserved0`——当前作为 `world_position_valid` 标志

说明：世界系速度由光流速度按当前 yaw 旋转得到，世界系位置由世界系速度积分得到。当前只作为相对里程计观察量，未接入控制。

### `FC`：两种世界系位置积分路径对比

返回通道：

- `I0`: `pmw_flow_state.position_x_world_cm`——速度旋转到世界系后积分得到的 X 位置，单位 cm
- `I1`: `pmw_flow_state.position_y_world_cm`——速度旋转到世界系后积分得到的 Y 位置，单位 cm
- `I2`: `pmw_flow_state.flow_position_x_world_cm`——单帧光流位移旋转到世界系后累加的 X 位置，单位 cm
- `I3`: `pmw_flow_state.flow_position_y_world_cm`——单帧光流位移旋转到世界系后累加的 Y 位置，单位 cm
- `I4`: `pmw_flow_state.flow_delta_x_world_cm`——本帧光流世界系 X 位移，单位 cm
- `I5`: `pmw_flow_state.flow_delta_y_world_cm`——本帧光流世界系 Y 位移，单位 cm
- `I6`: `pmw_flow_state.yaw_deg`——当前用于坐标旋转的 Yaw 角，单位 deg
- `I7`: `pmw_flow_state.flow_position_valid`——单帧位移积分位置有效标志

说明：`position_*_world_cm` 是“速度旋转到世界系后再积分”的位置；`flow_position_*_world_cm` 是“光流补偿后角位移 -> 单帧机体系位移 -> yaw 旋转 -> 世界系位移累加”的位置。两条路径并行存在，互不覆盖。

### `FD`：高度环全要素监视

返回通道：

- `I0`: `actual_cm`——ToF 滤波后实际高度，单位 cm
- `I1`: `target_cm`——高度管理器当前目标高度，单位 cm
- `I2`: `error_cm`——高度误差，已限幅到 `[-40, 40]cm`
- `I3`: `HeightSpeedPID_output`——高度 PID 整体输出
- `I4`: `p_term`——比例项，`HeightPID.Kp * error_cm`
- `I5`: `i_term`——积分项，`HeightPID.Ki * HeightPID.integral`
- `I6`: `d_term`——微分项，`HeightPID.Kd * HeightPID.derivative`
- `I7`: `base_throttle`——当前基础油门

### `FE`：光流原始输入诊断

UP-FLOW-302 模式返回通道：

- `I0`: `FLOW_SENSOR_SELECTED`——当前光流传感器选择，`1` 表示 UP-FLOW-302
- `I1`: `upflow302_receive.upflow302_x`——UP-FLOW-302 原始 X 数据
- `I2`: `upflow302_receive.upflow302_y`——UP-FLOW-302 原始 Y 数据
- `I3`: `upflow302_receive.upflow302_valid`——UP-FLOW-302 本帧数据有效标志
- `I4`: `upflow302_state_flag`——UP-FLOW-302 接收状态标志
- `I5`: `upflow302_finsh_flag`——UP-FLOW-302 本帧解析完成标志
- `I6`: `upflow302_receive.upflow302_us`——UP-FLOW-302 帧时间戳/周期字段，单位 us
- `I7`: `pmw_flow_state.dt_s`——光流后端本次采样间隔，单位 s

PMW3901 模式返回通道：

- `I0`: `FLOW_SENSOR_SELECTED`——当前光流传感器选择，`0` 表示 PMW3901
- `I1`: `pmw_flow_state.flow_delta_x_count`——PMW3901 本帧 X 原始 count
- `I2`: `pmw_flow_state.flow_delta_y_count`——PMW3901 本帧 Y 原始 count
- `I3`: `pmw_flow_state.valid`——光流后端速度状态有效标志
- `I4`: `pmw_flow_state.dt_s`——光流后端本次采样间隔，单位 s
- `I5`: `pmw_flow_state.flow_x_rad`——X 方向光流角位移，单位 rad
- `I6`: `pmw_flow_state.flow_y_rad`——Y 方向光流角位移，单位 rad
- `I7`: `pmw_flow_state.tof_los_cm`——参与光流速度换算的 ToF 视线距离，单位 cm

### `FF`：Roll / Pitch 角度环监视

返回通道：

- `I0`: `target_angle_roll`——Roll 目标角，单位 deg
- `I1`: `current_roll`——当前 Roll 姿态角，单位 deg
- `I2`: `angle_pid_roll.Ki * angle_pid_roll.integral`——Roll 角度环积分项输出
- `I3`: `target_roll_rate`——Roll 角度环总输出，已低通后送入 Roll 角速度环
- `I4`: `target_angle_pitch`——Pitch 目标角，单位 deg
- `I5`: `current_pitch`——当前 Pitch 姿态角，单位 deg
- `I6`: `angle_pid_pitch.Ki * angle_pid_pitch.integral`——Pitch 角度环积分项输出
- `I7`: `target_pitch_rate`——Pitch 角度环总输出，已低通后送入 Pitch 角速度环

### `FG`：红外正方形中心点

返回通道：

- `I0`: `ir_pose_estimate.x`——Kalman 滤波后的正方形中心点 X 坐标，单位 cm，正方向为机体 / 世界前方
- `I1`: `ir_pose_estimate.y`——Kalman 滤波后的正方形中心点 Y 坐标，单位 cm，正方向为机体 / 世界右方
- `I2`: `ir_pose_estimate.z`——根据正方形尺度估计的视觉高度 / 尺度量，单位 cm
- `I3`: `ir_pose_estimate.square_side_cm`——IPM 后计算出的正方形平均边长，正常应接近 10cm
- `I4`: `ir_track_valid`——中心点可追踪标志，1 表示当前中心可用于追踪；识别丢失后的 0.5 秒保持期内仍为 1
- `I5`: `ir_position_lost_flag`——位置丢失标志，1 表示超过 0.5 秒仍未恢复识别，中心 X/Y 已清零，不允许继续追踪
- `I6`: `ir_position_lost_elapsed_ms`——距离最近一次成功识别的时间，单位 ms；达到 500ms 后触发丢失保护
- `I7`: `ir_pose_estimate.detected_point_count`——通过几何筛选的小车红外点数，正常四点为 4；单点遮挡三点补全时为 3

说明：识别链路不再按面积取最大的 4 个亮点。原始图像中先提取小圆亮斑候选，候选面积和最大宽高会根据当前 `g_ipm.height` 做高度自适应缩放：高度升高时允许更小的亮点通过，高度降低时允许更大的近距离亮点通过；面积过大的地面信标灯仍会被排除出小车灯候选。随后只把候选质心做 `IPM_Image2World()` 转到地面 / 世界坐标，在世界坐标中枚举组合并按 10cm 正方形边长、对角线、边长一致性和上一帧中心连续性评分。看到 4 点时使用最佳四点组合；看到 3 点且满足“正方形缺一个角”的几何关系时，先在世界坐标系里补出第四角，再求中心。中心坐标写入 `ir_pose_estimate.x/y` 前会经过一维 Kalman 滤波，位置 PID 使用的也是这一路滤波后的坐标。

### `FH`：红外丢失保护状态机诊断

返回通道：

- `I0`: `ir_pose_estimate.x`——当前红外中心 X 坐标，单位 cm
- `I1`: `ir_pose_estimate.y`——当前红外中心 Y 坐标，单位 cm
- `I2`: `ir_loss_debug.raw_detect_valid`——本帧原始红外几何检测成功标志
- `I3`: `ir_loss_debug.lost_flag`——红外位置丢失标志
- `I4`: `ir_loss_debug.lost_elapsed_ms`——距离最近一次成功识别的时间，单位 ms
- `I5`: `ir_loss_debug.has_trackable_pose`——视觉模块内部是否仍有可保持的历史位姿
- `I6`: `ir_loss_debug.task_5ms_count`——红外丢失保护 5ms 任务运行计数
- `I7`: `ir_loss_debug.timeout_count`——红外丢失保持超时次数

诊断方法：遮挡或移走红外灯后，如果 I2 仍然为 1，说明检测器仍然在画面里找到了足够的亮点；如果 I6 不增长，说明 5ms 任务没有运行；如果 I7 增长但 I0/I1 又很快变成非 0，说明清零后被新的成功检测写回。

### `FI`：红外位置闭环诊断

返回通道：

- `I0`: `pos_diag.fb_x_cm`——位置环实际使用的 X 输入；视觉正常时与 `FG` 的 I0 同步，视觉原始识别丢失后为融合速度积分预测值
- `I1`: `pos_diag.fb_y_cm`——位置环实际使用的 Y 输入；视觉正常时与 `FG` 的 I1 同步，视觉原始识别丢失后为融合速度积分预测值
- `I2`: `pos_diag.vx_cmd_cms`——位置环输出给速度环的 X 目标速度，单位 cm/s
- `I3`: `pos_diag.vy_cmd_cms`——位置环输出给速度环的 Y 目标速度，单位 cm/s
- `I4`: `pos_diag.enabled`——位置环使能状态，1 表示已开启
- `I5`: `pos_diag.active`——位置环本周期是否实际参与控制
- `I6`: `pos_diag.ir_track_valid`——位置环输入有效标志；视觉正常或预测保持中为 1，预测释放后为 0
- `I7`: `pos_diag.velocity_ready`——速度反馈是否可用；为 0 时位置环目标速度会退回 0

使用建议：`FI!` 中 I0/I1 有值且 I6=1，说明位置环正在获得有效输入；若 `FH` 的 I2=0 但 `FI` 的 I6=1，说明当前处于视觉丢失后的速度积分预测保持。此时若 I4=0，位置环未使能，发送 `I0=1!` 或等待起飞 ramp 完成后的自动接入。I4=1 但 I5=0 时，不要继续加大 PID，先看 I7：I7=0 是光流速度不可用。位置环关闭或任一反馈失效时，代码会把目标速度自动退回 0，并 reset 位置环 PID。

位置环在进入 PID 前对 X/Y 分别加入 `+-3cm` 死区。某一轴的 `FI` 位置绝对值不超过 3cm 时，该轴 PID 输入按 0 处理并复位该轴 PID，`I2/I3` 对应轴目标速度应回到 0。视觉丢失预测保持会在速度积分位移达到锁存位置距离，或预测剩余距离进入死区时释放，此后 `FI` 的 I0/I1 和 I2/I3 都应回到 0。

### `FJ`：速度环 / 光流门控诊断

返回通道：

- `I0`: `Velocity_Loop_Is_Enabled()`——速度环使能状态，1 表示已开启
- `I1`: `vel_loop_active`——速度环本周期是否实际参与控制
- `I2`: `pmw_flow_state.valid`——光流后端速度状态有效标志
- `I3`: `pmw_flow_state.flow_used`——本帧光流是否被速度链路采用
- `I4`: `tof_los_cm`——传给光流速度换算的 ToF 视线距离，单位 cm
- `I5`: `tof_source`——ToF 来源，0=NONE，1=LIVE，2=HOLD
- `I6`: `tof_stale_ticks`——ToF hold 缓存连续使用计数
- `I7`: `pmw_flow_state.dt_s`——光流后端本次采样间隔，单位 s

排查 `F8!` 全 0 时优先看这一路：I2/I3 为 0 说明光流速度本身不可用；I4 为 0 或 I5 为 0 说明高度链路没有喂给光流；I0 为 0 说明还没有进入速度环接管状态；I0=1 但 I1=0 通常是光流门控没过，或还处在起飞/手动分支。

### `FK`：加速度 / 速度反馈 / 光流原始值对齐诊断

返回通道：

- `I0`: `ax_level_mg`——水平化并扣除 X 零偏后的 X 轴水平加速度，单位 mg
- `I1`: `ay_level_mg`——水平化并扣除 Y 零偏后的 Y 轴水平加速度，单位 mg
- `I2`: `pmw_flow_state.velocity_x_fused_cms`——进入 X 速度环 PID 的融合测量速度，单位 cm/s
- `I3`: `pmw_flow_state.velocity_y_fused_cms`——进入 Y 速度环 PID 的融合测量速度，单位 cm/s
- `I4`: `pmw_flow_state.velocity_x_raw_cms`——光流解算出的 X 原始水平速度，未经过斜率限制、低通和死区，单位 cm/s
- `I5`: `pmw_flow_state.velocity_y_raw_cms`——光流解算出的 Y 原始水平速度，未经过斜率限制、低通和死区，单位 cm/s
- `I6`: `pmw_flow_state.flow_delta_x_count`——本帧光流原始 X count
- `I7`: `pmw_flow_state.flow_delta_y_count`——本帧光流原始 Y count

用途：同屏比较加速度变化、速度环实际测量速度和光流原始输入。如果 I6/I7 已变化但 I4/I5 原始水平速度异常，优先查高度/轴映射/去旋转；如果 I4/I5 已变化但 I2/I3 滞后，优先查光流滤波；如果 I0/I1 先变化而光流迟迟不动，说明机体已受力但光流反馈没及时反映。

### `FL`：红外位置环 PID 调参诊断

返回通道：

- `I0`: `pos_diag.error_x_cm`——X 方向进入位置 PID 的误差，已经过 `+-3cm` 死区处理，单位 cm
- `I1`: `pos_pid_x.Kp * pos_diag.error_x_cm`——X 位置环比例项输出，单位 cm/s
- `I2`: `pos_diag.x_i_out`——X 位置环积分项输出，单位 cm/s
- `I3`: `pos_diag.vx_cmd_cms`——X 位置环最终输出给速度环的目标速度，单位 cm/s
- `I4`: `pos_diag.error_y_cm`——Y 方向进入位置 PID 的误差，已经过 `+-3cm` 死区处理，单位 cm
- `I5`: `pos_pid_y.Kp * pos_diag.error_y_cm`——Y 位置环比例项输出，单位 cm/s
- `I6`: `pos_diag.y_i_out`——Y 位置环积分项输出，单位 cm/s
- `I7`: `pos_diag.vy_cmd_cms`——Y 位置环最终输出给速度环的目标速度，单位 cm/s

调位置环 PID 时建议同时打开 `FI!` 和 `FL!`。`FI` 看位置输入是否有效、速度反馈是否可用以及最终目标速度；`FL` 看死区后的真实 PID 输入、P/I 项占比和输出是否被 `output_limit` 限住。若 `I0/I4` 已经接近 0 但 `I2/I6` 长时间不回落，优先减小 `I2/J2` 或积分限幅 `I5/J5`；若 `I1/I5` 很大且 `I3/I7` 直接顶到限幅，优先降低 `I1/J1` 或增大位置死区。

### `FM`：X 轴位置-速度-Pitch-角速度链路诊断

返回通道：

- `I0`: `pos_diag.fb_x_cm`——X 方向位置环输入位置，单位 cm；视觉正常时来自红外中心，视觉丢失保持时来自速度积分预测
- `I1`: `pos_diag.vx_cmd_cms`——X 方向速度环输入目标速度，单位 cm/s；由 X 位置环输出
- `I2`: `pmw_flow_state.velocity_x_fused_cms`——X 方向速度环实际反馈速度，单位 cm/s；光流 + ICM 融合速度
- `I3`: `target_angle_pitch`——Pitch 角度环输入目标角，单位 deg；X 方向速度环通过 Pitch 修正实现前后加速度控制
- `I4`: `current_pitch`——实际 Pitch 姿态角，单位 deg
- `I5`: `target_pitch_rate`——Pitch 角速度环输入目标角速度，角度环输出经低通后的结果
- `I6`: `current_gyro_pitch`——实际 Pitch 角速度
- `I7`: `pos_diag.active`——位置环本周期是否实际参与控制，1 表示位置链路正在输出目标速度

使用方法：用 `FM!` 可以从左到右看 X 轴闭环是否逐级传递：位置输入 `I0` 产生目标速度 `I1`，速度误差 `I1-I2` 推动 Pitch 目标角 `I3`，角度误差 `I3-I4` 生成目标 Pitch 角速度 `I5`，最终由角速度环追踪 `I6`。如果 `I1` 有值但 `I3` 长时间不变化，优先查速度环门控和光流反馈；如果 `I3` 有值但 `I5/I6` 不跟，优先查角度环/角速度环。

### `FN`：Y 轴位置-速度-Roll-角速度链路诊断

返回通道：

- `I0`: `pos_diag.fb_y_cm`——Y 方向位置环输入位置，单位 cm；视觉正常时来自红外中心，视觉丢失保持时来自速度积分预测
- `I1`: `pos_diag.vy_cmd_cms`——Y 方向速度环输入目标速度，单位 cm/s；由 Y 位置环输出
- `I2`: `pmw_flow_state.velocity_y_fused_cms`——Y 方向速度环实际反馈速度，单位 cm/s；光流 + ICM 融合速度
- `I3`: `target_angle_roll`——Roll 角度环输入目标角，单位 deg；Y 方向速度环通过 Roll 修正实现左右加速度控制
- `I4`: `current_roll`——实际 Roll 姿态角，单位 deg
- `I5`: `target_roll_rate`——Roll 角速度环输入目标角速度，角度环输出经低通后的结果
- `I6`: `current_gyro_roll`——实际 Roll 角速度
- `I7`: `pos_diag.active`——位置环本周期是否实际参与控制，1 表示位置链路正在输出目标速度

使用方法：用 `FN!` 可以从左到右看 Y 轴闭环是否逐级传递：位置输入 `I0` 产生目标速度 `I1`，速度误差 `I1-I2` 推动 Roll 目标角 `I3`，角度误差 `I3-I4` 生成目标 Roll 角速度 `I5`，最终由角速度环追踪 `I6`。如果 `I2` 方向和 `I1` 反着走，优先查 Y 轴光流/融合速度符号；如果 `I3/I4` 跟随方向反了，优先查 Roll 控制极性。

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

### 设置 IMU 安装补偿角

```text
T1=-0.60!
T2=0.21!
T3=0.00!
```

### 设置悬停基准角补偿

```text
B1=-2.10!
B2=0.65!
```

### 设置 X 方向速度环 PID

```text
V1=3.00!
V2=0.00!
V3=0.00!
V4=40.0!
V5=20.0!
```

### 设置 Y 方向速度环 PID

```text
W1=3.00!
W2=0.00!
W3=0.00!
W4=40.0!
W5=20.0!
```

### 设置光流陀螺去旋转增益

```text
Gx=0.82!
Gy=0.94!
```

### 设置红外位置环 PID 并开启

```text
I1=0.60!
I2=0.00!
I3=0.00!
I4=35.0!
I5=50.0!
J1=0.60!
J2=0.00!
J3=0.00!
J4=35.0!
J5=50.0!
I0=1!
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
FG!
FH!
FI!
FJ!
FK!
FL!
FM!
FN!
```

### 强制失锁

```text
00=0!
```

## 使用建议

- 一次发送一条完整命令，必须以 `!` 结尾
- 建议连续命令之间留出少量时间，避免无线串口链路堆积
- 如果需要看某类参数回读，先发送对应设置命令，再切到 `F1`
- 如果需要观察实时状态，优先使用 `F0 / F2 / F3 / F4 / F5 / F6 / FF`；观察红外正方形中心点和丢失保护状态用 `FG!`
- 高度环调参建议先开 `FD!` 观察 I0/I1/I2/I3，只调 Kp（H1）直到高度收敛稳定再考虑加 Kd

## 当前方向关系速查

工程中涉及方向的传感器和坐标关系如下：

| 模块 | 当前方向关系 | 代码位置 |
|---|---|---|
| 通用机体 / 世界系约定 | X 向前，Y 向右，Z 向下。下视视觉和光流世界系位置都按这个约定解释。 | `vision/drone_ipm.c`, `vision/ipm_interface.h`, `pmw/pmw_flow.c` |
| ICM42688 IMU | 原始 X/Y/Z 经过固定安装补偿后进入姿态解算；当前未启用额外 X/Y/Z 取反。`icm42688_acc_x -> Roll_acc`，`icm42688_acc_y -> Pitch_acc`，`icm42688_acc_z -> Yaw_acc`；`icm42688_gyro_x -> Roll_g`，`icm42688_gyro_y -> Pitch_g`，`icm42688_gyro_z -> Yaw_g`。 | `icm/icm42688.c`, `icm/icm.c` |
| PMW3901 光流 | 原始 `pmw3901_delta_x -> 后端 X`，原始 `pmw3901_delta_y -> 后端 Y`，不交换轴，X/Y 符号均为 `+1`。这是旧光流已验证正确的方向约定。 | `libraries/zf_device/zf_device_pmw3901.c`, `pmw/flow_sensor_config.h` |
| UP-FLOW-302 光流 | 原始 `upflow302_x -> 机体 / 后端 Y`，原始 `upflow302_y -> 机体 / 后端 X`，因此 `FLOW_SENSOR_UPFLOW302_SWAP_XY = 1`；符号作用在交换后的机体 / 后端轴上，当前 `X_SIGN = -1`、`Y_SIGN = +1`。 | `libraries/zf_device/zf_device_upflow302.c`, `pmw/flow_sensor_config.h`, `pmw/pmw_flow.c` |
| 光流速度 / 位置后端 | `flow_comp_x` 解释为机体 X 向光流角位移，`flow_comp_y` 解释为机体 Y 向光流角位移；速度映射中 X 符号为 `+1`，Y 符号为 `-1`。世界系速度 / 位置再由当前 yaw 旋转得到。 | `pmw/pmw_flow.c` |
| DL1B ToF | 标量视线距离，不提供 X/Y 方向；代码按下向高度使用，并补偿安装关系：ToF 位于机体 `+Y` 约 7cm，光流位于机体 `-Y` 约 7cm，ToF 光束朝 `+Y` 倾斜约 10 度。 | `control/height_manager.c` |
| 下视摄像头 / IPM | 图像 `u` 向右，对应机体 / 世界 Y 向右；图像 `v` 向下，对应机体 / 世界 X 向前；图像中心对应正下方地面点。 | `vision/drone_ipm.c`, `vision/drone_ipm_embedded.c`, `vision/ipm_interface.h` |
| 红外定位 | 使用 IPM 输出的地面坐标，因此 `ir_pose_estimate.x` 为前向坐标，`ir_pose_estimate.y` 为右向坐标，`ir_pose_estimate.z` 为估计高度 / 尺度相关量。 | `vision/ir_localization.c`, `vision/ir_localization.h` |
