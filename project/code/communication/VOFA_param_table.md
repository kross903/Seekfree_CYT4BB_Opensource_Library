# VOFA 参数表（当前默认值）

本文整理当前无人机工程中与 VOFA 调参相关、以及调参时需要重点关注的默认参数数值。

相关来源：

- [All_Init.c](./../common/All_Init.c)
- [PIT_Demo.h](./../common/PIT_Demo.h)
- [PIT_Demo.c](./../common/PIT_Demo.c)
- [flight_control.c](./../control/flight_control.c)
- [vofa.c](./vofa.c)

## 1. 可通过 VOFA 调整的 PID 参数

### 1.1 Roll 姿态环 PID

| 参数 | 默认值 | VOFA 指令 |
|---|---:|---|
| `Kp` | `3.50` | `R1=3.50!` |
| `Ki` | `0.00` | `R2=0.00!` |
| `Kd` | `0.00` | `R3=0.00!` |
| 输出限幅 `output_limit` | `200.0` | 不直接调，当前 `R` 组未开放 |
| 积分限幅 `integral_limit` | `30.0` | 不直接调，当前 `R` 组未开放 |

### 1.2 Roll 角速度环 PID

| 参数 | 默认值 | VOFA 指令 |
|---|---:|---|
| `Kp` | `1.60` | `r1=1.60!` |
| `Ki` | `0.00` | `r2=0.00!` |
| `Kd` | `0.120` | `r3=0.120!` |
| 输出限幅 `output_limit` | `600.0` | 当前未开放单独指令 |
| 积分限幅 `integral_limit` | `100.0` | `r4=100.0!` |
| 微分限幅 / 滤波参数 | `10.0 / 0.8` | 不通过当前 VOFA 指令修改 |

### 1.3 Pitch 姿态环 PID

| 参数 | 默认值 | VOFA 指令 |
|---|---:|---|
| `Kp` | `3.50` | `P1=3.50!` |
| `Ki` | `0.00` | `P2=0.00!` |
| `Kd` | `0.00` | `P3=0.00!` |
| 输出限幅 `output_limit` | `200.0` | 不直接调，当前 `P` 组未开放 |
| 积分限幅 `integral_limit` | `30.0` | 不直接调，当前 `P` 组未开放 |

### 1.4 Pitch 角速度环 PID

| 参数 | 默认值 | VOFA 指令 |
|---|---:|---|
| `Kp` | `1.60` | `p1=1.60!` |
| `Ki` | `0.00` | `p2=0.00!` |
| `Kd` | `0.120` | `p3=0.120!` |
| 输出限幅 `output_limit` | `600.0` | 当前未开放单独指令 |
| 积分限幅 `integral_limit` | `100.0` | `p4=100.0!` |
| 微分限幅 / 滤波参数 | `10.0 / 0.8` | 不通过当前 VOFA 指令修改 |

### 1.5 Yaw 姿态环 PID

| 参数 | 默认值 | VOFA 指令 |
|---|---:|---|
| `Kp` | `3.00` | `Y1=3.00!` |
| `Ki` | `0.00` | `Y2=0.00!` |
| `Kd` | `0.00` | `Y3=0.00!` |
| 输出限幅 `output_limit` | `100.0` | 不直接调，当前 `Y` 组未开放 |
| 积分限幅 `integral_limit` | `20.0` | 不直接调，当前 `Y` 组未开放 |

### 1.6 Yaw 角速度环 PID

| 参数 | 默认值 | VOFA 指令 |
|---|---:|---|
| `Kp` | `1.00` | `y1=1.00!` |
| `Ki` | `0.00` | `y2=0.00!` |
| `Kd` | `0.00` | `y3=0.00!` |
| 输出限幅 `output_limit` | `400.0` | 当前未开放单独指令 |
| 积分限幅 `integral_limit` | `10.0` | `y4=10.0!` |
| 微分限幅 / 滤波参数 | `0.0 / 0.0` | 不通过当前 VOFA 指令修改 |

### 1.7 高度环 PID

| 参数 | 默认值 | VOFA 指令 |
|---|---:|---|
| `Kp` | `5.00` | `H1=5.00!` |
| `Ki` | `0.00` | `H2=0.00!` |
| `Kd` | `0.00` | `H3=0.00!` |
| 输出限幅 `output_limit` | `200.0` | 当前未开放单独指令 |
| 积分限幅 `integral_limit` | `50.0` | 当前未开放单独指令 |
| 微分限幅 / 滤波参数 | `10.0 / 0.0` | 不通过当前 VOFA 指令修改 |

### 1.8 摇杆到姿态的 PID

这组参数当前已经接入 VOFA 调参，但只覆盖 `Roll / Pitch` 两轴，**不控制 Yaw**。

| 参数 | Roll 默认值 | Pitch 默认值 |
|---|---:|---:|
| `Kp` | `1.00` | `1.00` |
| `Ki` | `0.00` | `0.00` |
| `Kd` | `0.00` | `0.00` |
| 输出限幅 `output_limit` | `20.0` | `20.0` |
| 积分限幅 `integral_limit` | `10.0` | `10.0` |

对应指令：

| 轴 | 参数 | VOFA 指令 |
|---|---|---|
| Roll | `Kp` | `J1=<value>!` |
| Roll | `Ki` | `J2=<value>!` |
| Roll | `Kd` | `J3=<value>!` |
| Roll | `output_limit` | `J4=<value>!` |
| Roll | `integral_limit` | `J5=<value>!` |
| Pitch | `Kp` | `K1=<value>!` |
| Pitch | `Ki` | `K2=<value>!` |
| Pitch | `Kd` | `K3=<value>!` |
| Pitch | `output_limit` | `K4=<value>!` |
| Pitch | `integral_limit` | `K5=<value>!` |

## 2. 可通过 VOFA 调整的非 PID 参数

### 2.1 油门目标

| 参数 | 默认值 | VOFA 指令 |
|---|---:|---|
| 目标油门 `target_throttle` | `0.0` | `S1=<value>!` |

### 2.2 姿态补偿角

| 参数 | 默认值 | VOFA 指令 |
|---|---:|---|
| Roll 补偿角 `angle_roll_compensation` | `0.0` | `T1=<value>!` |
| Pitch 补偿角 `angle_pitch_compensation` | `0.0` | `T2=<value>!` |
| Yaw 补偿角 `angle_yaw_compensation` | `0.0` | `T3=<value>!` |

## 3. 重要限幅与安全参数

### 3.1 油门与电机相关

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `MOTOR_OUTPUT_SCALE_MAX` | `1800` | 电机输出缩放上限 |
| `SAFE_MAX_MOTOR_OUTPUT_LIMIT` | `1800` | 安全输出上限 |
| `SAFE_MAX_THROTTLE` | `1800.0` | 最大允许油门 |
| `SAFE_MIN_THROTTLE` | `300.0` | 安全最小油门阈值 |
| `MOTOR_TEST_MAX_THROTTLE` | `450` | 电机测试页最大油门 |
| `MOTOR_TEST_STEP` | `50` | 电机测试页步进 |

### 3.2 倾角保护

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `SAFE_MAX_ANGLE_ROLL` | `40.0` 度 | Roll 超限急停阈值 |
| `SAFE_MAX_ANGLE_PITCH` | `40.0` 度 | Pitch 超限急停阈值 |
| `SAFE_MAX_ANGLE_YAW` | `180.0` 度 | 当前未用于姿态急停 |

### 3.3 手动摇杆映射

这些参数当前不通过 VOFA 单独调，但在控制效果上很关键。

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `JOYSTICK_ROLL_MAX_DEG` | `20.0` | 最大 Roll 目标角 |
| `JOYSTICK_PITCH_MAX_DEG` | `20.0` | 最大 Pitch 目标角 |
| `THROTTLE_RAMP_RATE` | `2.0` | 油门爬坡速率 |

### 3.4 PWM 输出

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `MOTOR_PWM_FREQ_HZ` | `200` | ESC PWM 频率 |
| `MOTOR_PWM_DUTY_MIN` | `2000` | 停转占空比基准 |
| `MOTOR_PWM_DUTY_MAX` | `3200` | 当前最大输出占空比 |

## 4. 电池相关保护参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `BATTERY_ADC_TO_VOLTAGE_SCALE` | `0.00820715` | ADC 到电压换算比例 |
| `BATTERY_SERIES_CELLS` | `3.0` | 按 3S 计算 |
| `BATTERY_LOW_CELL_WARN_V` | `3.55` | 单节低压告警阈值 |
| `BATTERY_LOW_CELL_RECOVER_V` | `3.65` | 单节恢复阈值 |
| `BATTERY_LOW_WARN_THRESHOLD` | `10.65` | 总电压低压告警阈值 |
| `BATTERY_LOW_RECOVER_THRESHOLD` | `10.95` | 总电压恢复阈值 |
| `BATTERY_NOMINAL_VOLTAGE` | `11.1` | 标称电压 |

## 5. VOFA 回传模式切换指令

| 指令 | 含义 |
|---|---|
| `F0!` | 默认飞行状态回传 |
| `F1!` | PID 参数回读 |
| `F2!` | 定位 / 高度观察 |
| `F3!` | 链路测试 |
| `F4!` | 陀螺仪调试 |
| `F5!` | 加速度调试 |
| `F6!` | Roll/Pitch 角度与角速度对比 |
| `F7!` | ADC 原始值、滤波值与目标角 |

## 6. 常用发送指令示例

### 调 Roll 姿态环

```text
R1=3.50!
R2=0.00!
R3=0.00!
```

### 调 Pitch 角速度环

```text
p1=1.60!
p2=0.00!
p3=0.120!
p4=100.0!
```

### 调高度环

```text
H1=5.00!
H2=0.00!
H3=0.00!
```

### 调姿态补偿角

```text
T1=0.00!
T2=0.21!
T3=0.00!
```

### 调 Roll 摇杆 PID

```text
J1=1.00!
J2=0.00!
J3=0.00!
J4=20.0!
J5=10.0!
```

### 调 Pitch 摇杆 PID

```text
K1=1.00!
K2=0.00!
K3=0.00!
K4=20.0!
K5=10.0!
```

### 设置油门

```text
S1=450!
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
```

### 强制失锁

```text
00=0!
```

## 7. 说明

- 本表列的是当前工程源码中的默认值，不代表你板子运行时一定还是这些值。
- 如果启用了菜单调参、VOFA 调参或 Flash 参数存储，运行时值可能已经被改写。
- 当前 VOFA 指令并没有开放所有限幅参数；有些限幅虽然代码里有默认值，但暂时不能直接通过 VOFA 单独调。
