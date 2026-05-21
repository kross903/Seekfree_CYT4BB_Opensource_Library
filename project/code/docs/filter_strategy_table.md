# 飞控滤波策略表

本文档用于记录当前无人机工程后续滤波设计的原则、候选位置和风险。目标不是立刻增加滤波，而是在后续修改前先明确：哪些信号可以滤、哪些信号不应重滤、notch 和 D-term 滤波应该放在哪里。

## 总原则

当前飞控控制链路应保持不变：

```text
位置环
  ↓ 输出目标速度
速度环
  ↓ 输出目标姿态角
角度环
  ↓ 输出目标角速度
角速度环
  ↓ 输出 roll_output / pitch_output / yaw_output
混控
  ↓ 输出 motor_throttle[4]
```

滤波策略遵循以下原则：

1. 越靠近内环，越不能重滤。
2. 外环反馈可以适当滤，内环反馈必须谨慎。
3. 目标值平滑通常比反馈值重低通更安全。
4. gyro 反馈不建议靠重低通压噪声。
5. 固定频率振动应优先考虑 notch，而不是一刀切低通。
6. D 项容易放大噪声，应优先考虑 D-term 单独滤波。
7. 任何会影响飞控闭环的滤波，都应先默认关闭或保守配置，再逐步验证。
8. 滤波修改后必须做无桨测试，再做固定机架低油门测试，最后才考虑飞行测试。

## 滤波优先级概览

| 优先级 | 方向 | 推荐程度 | 原因 |
| --- | --- | --- | --- |
| 高 | D-term 滤波 | 推荐 | 只抑制 D 项噪声，不必重滤整个 gyro 反馈 |
| 高 | 固定 notch | 推荐但需实测频率 | 针对电机/桨叶/机架固定频率振动 |
| 中 | 目标值限斜率 | 推荐 | 减少目标突变，不延迟反馈 |
| 中 | 外环反馈 LPF | 可用 | 位置、视觉、高度等慢信号允许一定延迟 |
| 低 | gyro 整体低通 | 谨慎 | 会直接引入角速度环反馈相位滞后 |
| 低 | dynamic notch | 后期再考虑 | 需要频率检测和更复杂的稳定性处理 |
| 低 | 完整 EKF | 长期学习对象 | 当前工程阶段不建议直接上复杂估计器 |

## 信号级滤波策略表

| 信号 | 所在链路 | 当前作用 | 推荐滤波方式 | 延迟敏感度 | 建议优先级 | 说明 |
| --- | --- | --- | --- | --- | --- | --- |
| `current_gyro_roll/pitch/yaw` | 角速度环反馈 | rate PID 输入 | 不重低通；必要时固定 notch | 极高 | 高风险 | 这是最内环反馈，低通会直接造成相位滞后。若有固定频率振动，优先 notch。 |
| gyro D 项输入 | 角速度环 D 项 | D 项变化率来源 | D-term LPF | 高 | 高 | D 项最容易放大噪声，适合单独滤波，不应让 P 项一起被重滤。 |
| `roll_output/pitch_output/yaw_output` | rate PID 输出 | 混控输入 | 不建议滤 | 高 | 不推荐 | PID 输出后再滤会拖慢控制修正，不如在 gyro notch 或 D-term 内处理噪声。 |
| `target_roll_rate/target_pitch_rate` | 角度环输出 | rate loop setpoint | 轻微 LPF 或限斜率 | 高 | 中 | 可以抑制目标角速度突变，但不能过重，否则角度响应变慢。 |
| `target_angle_roll/pitch` | 速度环输出 / 手动目标 | angle loop setpoint | 限斜率优先 | 中 | 中 | 目标姿态变化可以平滑，避免姿态目标突然跳变。 |
| `target_angle_yaw` | yaw 目标 | angle loop setpoint | 限斜率或轻滤 | 中 | 中 | yaw 响应通常可比 roll/pitch 慢一些，但仍不应过重。 |
| `vx_target/vy_target` | 位置环输出 | 速度环目标 | 限斜率或轻 LPF | 中低 | 中 | 位置环较慢，允许平滑；优先限制目标速度变化率。 |
| `measured_vx/measured_vy` | 光流速度反馈 | 速度环反馈 | LPF/PT2，异常值剔除 | 中 | 中 | 光流速度有噪声，可以滤，但过重会让速度环滞后。 |
| `ir_pose_estimate.x/y` | 视觉位置反馈 | 位置环输入 | 轻 LPF、中值/异常值剔除 | 低 | 中 | 位置环较慢，视觉结果可能跳点，适合做异常值处理。 |
| `ir_pose_estimate.valid` | 视觉有效性 | 位置环 valid 条件 | 不滤；做状态机/丢失保护 | 低 | 高 | valid 应明确，不应被数值滤波模糊。 |
| `tof_height_true_filt` | 高度估计 | 高度/IPM 输入 | LPF、异常值剔除 | 中低 | 中 | 高度对 IPM 和高度环都有影响，应重视跳变处理。 |
| `HeightSpeedPID_output` | 高度控制输出 | 混控总推力修正 | 不建议额外滤输出 | 中 | 谨慎 | 若高度输出抖，应优先检查高度测量和 PID 参数。 |
| `car_bat_voltage` | 电池电压 | 显示/低压保护 | LPF 或滑动平均 | 低 | 低 | 电压慢变量，适合滤波；不影响高速控制。 |
| `adc_x/adc_y` | 地面输入 ADC | 归一化输入 | LPF/死区 | 低 | 已有 | 低频人工/地面输入可以滤，延迟影响小。 |
| `motor_throttle[4]` | 电机输出 | 硬件输出 | 不建议在最终输出滤波 | 极高 | 不推荐 | 电机输出层应尽量直接反映控制器输出；需要保护时做限幅/饱和检测。 |

## Notch 滤波策略

### 适用对象

Notch 主要用于 gyro 中的固定频率振动：

```text
电机振动
桨叶不平衡
机架共振
IMU 安装振动
```

### 推荐位置

建议放在角速度环内部：

```text
current_gyro_*
  ↓
gyro notch filter
  ↓
Rate_Loop_Update() 中的 PID_Calculate()
```

推荐落点：

```text
project/code/control/rate_loop.c
```

不建议放在：

```text
PIT_Demo.c
PID 输出 roll_output/pitch_output/yaw_output 后面
motor_mixer.c
motor_hardware.c
```

### 参数来源

固定 notch 需要：

```text
采样频率 fs
中心频率 f0
Q 值或带宽
```

当前角速度环周期：

```text
PIT_Control_Task_250us() = 4 kHz
fs = 4000 Hz
```

`f0` 不应猜测，应通过 VOFA 或日志观察 gyro 波形后确定。后续可先记录：

```text
gyro_roll 原始波形
gyro_pitch 原始波形
gyro_yaw 原始波形
电机不同油门下的主要振动频率
```

### 初始建议

```text
Notch 默认关闭
测到明确频率后再开启
Q 初始可考虑 5 左右
确认频率稳定后可尝试 8 ~ 10
```

注意：

- Q 太低，陷波太宽，相位影响更大。
- Q 太高，陷波太窄，频率漂移时可能压不住。
- 没有明确固定频率时，不要盲目加 notch。

## D-term 滤波策略

### 适用对象

D-term 滤波用于抑制 PID 的 D 项噪声。

D 项对噪声敏感，因为它关注变化率：

```text
测量噪声
  ↓
变化率被放大
  ↓
D 项抖动
  ↓
电机输出抖动
```

### 推荐位置

推荐在通用 PID 内部支持，但默认关闭：

```text
project/code/control/pid_controller.c
project/code/control/pid_controller.h
```

只给 gyro PID 打开：

```text
gyro_pid_roll
gyro_pid_pitch
gyro_pid_yaw
```

不要默认影响：

```text
angle_pid_*
vel_pid_*
pos_pid_*
HeightPID
```

### 推荐结构

理想计算结构：

```text
error
  ↓
P = kp * error

error 或 measurement 的变化率
  ↓
D raw
  ↓
D-term LPF
  ↓
D = kd * D filtered

P + I + D
  ↓
PID output
```

### 后续实现约束

如果后续修改 `PID_Controller`，应满足：

```text
默认行为完全不变
D-term 滤波默认关闭
只有显式启用的 PID 才使用 D-term 滤波
PID_Reset() 必须同时 reset D-term filter state
参数保存/菜单调参不能破坏原有 PID 参数
```

## 目标值平滑策略

目标值平滑优先使用限斜率，而不是重低通。

适合对象：

```text
target_throttle
base_throttle
vx_target / vy_target
target_roll / target_pitch
target_roll_rate / target_pitch_rate
```

原因：

```text
目标值平滑会让指令更柔和
反馈值重滤会让控制器更晚看到真实状态
```

推荐落点：

```text
project/code/control/throttle_runtime.c
project/code/control/velocity_loop.c
project/code/control/angle_loop.c
```

其中：

- 油门 ramp 已有基础。
- 角度环目标角速度已有轻微平滑思路。
- 速度环输出目标姿态后续可考虑限斜率。

## 外环反馈滤波策略

外环比内环慢，可以更接受滤波延迟。

适合对象：

```text
红外位置 x/y
光流速度 vx/vy
ToF 高度
高度速度
电池电压
```

推荐方法：

```text
轻 LPF
PT2
异常值剔除
valid / lost 状态机
短时间 hold / predict
```

需要注意：

- 外环滤波不应隐藏传感器失效。
- valid 状态要明确，不要只靠数值滤波。
- 视觉/ToF 跳变应优先做异常值剔除，而不是单纯加重低通。

## 不建议当前立即做的事

1. 不要在 gyro 反馈上加重低通。
2. 不要把 notch 放到 PID 输出后面。
3. 不要把滤波逻辑写进 `PIT_Demo.c`。
4. 不要一次性打开多个滤波器。
5. 不要在没测频谱的情况下启用固定 notch。
6. 不要直接做 dynamic notch。
7. 不要直接移植 PX4 EKF2。
8. 不要在未无桨验证前接入飞行测试。

## 推荐后续实施顺序

### 第一步：只建立工具和开关

新增或整理：

```text
project/code/control/filter.c
project/code/control/filter.h
```

提供：

```text
一阶 LPF
PT2 / PT3
Biquad notch
限斜率工具
```

所有新滤波默认不改变现有控制行为。

### 第二步：加入 D-term 滤波框架

修改：

```text
project/code/control/pid_controller.c/.h
```

要求：

```text
默认关闭
PID_Reset() 同步 reset filter state
只在 gyro PID 初始化时显式开启
```

### 第三步：加入 gyro notch 框架

修改：

```text
project/code/control/rate_loop.c/.h
```

要求：

```text
默认关闭
notch 参数集中配置
保留原始 gyro 和 filtered gyro 的调试入口
```

### 第四步：增加 VOFA 观测量

建议观测：

```text
current_gyro_roll / pitch / yaw
filtered_gyro_roll / pitch / yaw
D_raw
D_filtered
roll_output / pitch_output / yaw_output
motor_output[4]
```

### 第五步：测试顺序

```text
编译通过
  ↓
无桨静态看 gyro 噪声
  ↓
固定机架低油门看振动频率
  ↓
确定 notch f0
  ↓
开启单个滤波器测试
  ↓
对比开启前后波形
  ↓
确认无异常后再考虑飞行测试
```

## 成熟飞控可学习点

### Betaflight

值得学习：

```text
gyro 滤波尽量少而精准
D-term 单独滤波
notch 用于固定/动态振动频率
不要靠重低通解决所有噪声
滤波和 PID 调参必须一起看
```

不建议当前直接照搬：

```text
RPM filter
dynamic notch 自动频率追踪
完整滤波调参体系
```

### PX4

值得学习：

```text
传感器 valid 状态
估计器健康状态
观测失效 reset/hold 逻辑
外环估计结果和控制器解耦
```

不建议当前直接照搬：

```text
EKF2 完整状态估计
复杂协方差参数
多传感器延迟补偿框架
```

## 当前推荐结论

当前工程最适合的滤波路线是：

```text
1. 先文档化每个信号的滤波策略。
2. 后续新增通用 filter 工具模块。
3. 优先实现 D-term 滤波框架，默认关闭。
4. 再实现 gyro 固定 notch 框架，默认关闭。
5. 外环信号继续使用轻滤波、异常值剔除和 valid 状态管理。
6. 不在 gyro 反馈上使用重低通。
```

最终目标是：

```text
减少电机/IMU 高频噪声对控制输出的影响，
同时尽量避免角速度环反馈相位滞后。
```
