# 成品飞控代码可借鉴的改进方向

本文档用于记录当前项目后续可以从 Betaflight 和 PX4 这类成熟飞控中学习的部分。目标不是照搬大型飞控架构，而是在不破坏现有控制链路的前提下，逐步提高控制链路清晰度、安全性、可调试性和扩展能力。

## 当前项目的主控制链路

当前飞控的水平控制链路总体方向是正确的，可以理解为四串级结构：

```text
位置环
  ↓ 输出目标水平速度 vx / vy
速度环
  ↓ 输出目标姿态角 roll / pitch 修正
角度环
  ↓ 输出目标角速度 roll_rate / pitch_rate / yaw_rate
角速度环
  ↓ 输出 roll_output / pitch_output / yaw_output
电机混控
  ↓ 输出 motor_throttle[4]
电机硬件层
  ↓ PWM / DShot
```

高度控制链路与水平姿态控制链路并行，最后在混控处汇合：

```text
高度 / 高度速度控制
  ↓ 输出总推力修正 HeightSpeedPID_output

水平姿态控制
  ↓ 输出 roll_output / pitch_output / yaw_output

二者在 mix_motor_output() 中合成四个电机输出。
```

因此，后续整理代码时应保护这个链路顺序，不要把外环和内环的依赖方向反过来。

## Betaflight 可借鉴的点

Betaflight 更偏竞速和手动飞控，它的重点是内环、混控、电机输出和安全门禁。

典型链路：

```text
遥控输入 / 模式
  ↓
角速度目标
  ↓
gyro PID
  ↓
mixer
  ↓
motor output
```

可借鉴方向：

1. **内环稳定性优先**
   - 角速度环是最终决定手感和稳定性的核心。
   - 后续调试时应优先保证 gyro PID 输出稳定，再逐级打开角度环、速度环、位置环。

2. **混控层独立**
   - Betaflight 把 PID 输出和电机混控分开。
   - 当前项目后续可以把 `mix_motor_output()` 拆成 `motor_mixer.c/.h`，让控制算法和电机几何分配分离。

3. **解锁和 failsafe 门禁清楚**
   - Betaflight 对 arm/disarm、failsafe、runaway takeoff 等状态有明确保护。
   - 当前项目可以继续加强 `flight_safety` 和未来的 `flight_mode_manager`，统一管理是否允许电机输出、是否 reset PID、是否进入急停。

4. **油门和姿态输出保护**
   - Betaflight 有 throttle 相关补偿、airmode、motor saturation 处理。
   - 当前项目可以先做简化版电机饱和检测，用来限制 PID 积分继续增长。

## PX4 可借鉴的点

PX4 更偏完整自动驾驶飞控，它的控制链路分层非常清楚。

典型多旋翼链路：

```text
FlightTask / 模式管理
  ↓
trajectory_setpoint
  ↓
PositionControl
  ↓
vehicle_attitude_setpoint
  ↓
AttitudeControl
  ↓
rate setpoint
  ↓
RateControl
  ↓
thrust / torque setpoint
  ↓
ControlAllocator
  ↓
actuator outputs
```

可借鉴方向：

1. **每一级控制器只输出下一级 setpoint**

   当前项目后续建议整理成：

   ```text
   Position_Loop_Update()
     → 输出 vx_target / vy_target

   Velocity_Loop_Update()
     → 输出 target_roll / target_pitch

   Angle_Loop_Update()
     → 输出 target_roll_rate / target_pitch_rate / target_yaw_rate

   Rate_Loop_Update()
     → 输出 roll_output / pitch_output / yaw_output
   ```

   `PIT_Demo.c` 只负责按顺序调用，而不是承载每一级内部细节。

2. **模块之间通过明确接口传递目标和反馈**

   不建议长期依赖大量全局变量隐式传递状态。后续可以逐步把接口改成：

   ```c
   Position_Loop_Update(..., &vx_target, &vy_target);
   Velocity_Loop_Update(vx_target, vy_target, ..., &target_roll, &target_pitch);
   ```

   这样调试时能更清楚地知道每一级输入和输出。

3. **传感器有效性进入控制链路**

   PX4 会根据位置、速度、高度、姿态等估计状态是否有效来决定控制器是否工作。当前项目可以逐步完善：

   - 红外位置是否有效；
   - 光流速度是否有效；
   - ToF / 高度数据是否有效；
   - 姿态数据是否有效；
   - 当前轴是否允许积分；
   - 当前输出是否饱和。

4. **控制分配和机型几何解耦**

   PX4 使用 Control Allocator。当前项目不需要这么复杂，但可以先把混控独立成 `motor_mixer`：

   ```text
   throttle + roll_output + pitch_output + yaw_output
     ↓
   motor_throttle[4]
   ```

5. **模式管理集中**

   PX4 的 Commander 统一管理解锁、模式、failsafe。当前项目可以做轻量版：

   ```text
   flight_mode_manager.c/.h
   ```

   负责管理：

   ```text
   DISARMED
   MOTOR_TEST
   TAKEOFF
   CRUISE
   LANDING
   MANUAL
   EMERGENCY_STOP
   ```

   它不直接计算 PID，只决定各控制环是否允许工作、电机是否允许输出、是否需要 reset。

## 当前项目优先级建议

### 第一阶段：继续小规模拆分 PIT_Demo

优先拆：

```text
control/position_loop.c/.h
control/velocity_loop.c/.h
```

目标：

```text
position_loop.c 负责：位置误差 → 速度目标
velocity_loop.c 负责：速度误差 → 姿态目标
PIT_Demo.c 负责：按控制链路顺序调度
```

不要让 `velocity_loop.c` 主动调用 `position_loop.c`。当前项目是四串级 PID，应该由上一级输出给下一级，主调度函数按顺序串起来。

### 第二阶段：明确环间接口

拆完后逐步减少隐式全局状态，把控制链路改成更清楚的输入输出关系：

```text
位置反馈 + 位置目标 → 速度目标
速度反馈 + 速度目标 → 姿态目标
姿态反馈 + 姿态目标 → 角速度目标
角速度反馈 + 角速度目标 → 电机差动输出
```

### 第三阶段：完善 valid / active / reset 逻辑

每一级控制器都应能回答：

```text
当前是否 enabled？
当前是否 active？
输入数据是否 valid？
无效时是否 reset PID？
无效时输出应该保持、清零，还是回退到基准目标？
```

### 第四阶段：增加电机饱和检测和积分抗饱和

建议先实现简单版本：

```text
如果任意电机输出接近 0 或 MOTOR_OUTPUT_SCALE_MAX，认为混控接近饱和。
```

后续可用这个状态限制外环或内环积分继续增长，避免积分越积越大。

重点关注：

```text
vel_pid_x / vel_pid_y
angle_pid_roll / angle_pid_pitch
gyro_pid_roll / gyro_pid_pitch
高度相关 PID
```

### 第五阶段：整理模式和安全状态

在 `flight_safety` 已经独立的基础上，可以后续新增：

```text
flight_mode_manager.c/.h
```

让模式切换、解锁、急停、起飞、巡航、降落、电机测试的状态关系更集中。

### 第六阶段：拆出 motor_mixer

在 `motor_hardware` 已经拆出的基础上，后续可以把 `mix_motor_output()` 也独立出来：

```text
control/motor_mixer.c/.h
```

形成：

```text
控制器输出 → motor_mixer → motor_throttle[4] → motor_hardware → PWM/DShot
```

## 不建议当前立即照搬的内容

1. **不要直接照搬 PX4 的 uORB 消息系统**
   - 当前项目规模较小，直接引入消息总线会增加复杂度。

2. **不要马上做复杂控制分配器**
   - 四轴固定布局目前简单 mixer 足够。

3. **不要一次性重命名或移动 PIT_Demo.c**
   - 它仍然承载 ISR 调用入口和 4kHz 主控制任务，应该等内部职责拆薄后再考虑改名。

4. **不要同时大改模式管理和控制环**
   - 控制链路能跑是最重要的，应该一次只动一个边界。

## 推荐的长期目标结构

```text
project/code/
├── common/
│   └── project_headfile.h
├── system/
│   └── system_init.c/.h
├── safety/
│   └── flight_safety.c/.h
├── flight_runtime/
│   └── flight_tasks.c/.h
├── control/
│   ├── position_loop.c/.h
│   ├── velocity_loop.c/.h
│   ├── angle_loop.c/.h
│   ├── rate_loop.c/.h
│   ├── motor_mixer.c/.h
│   └── pid_controller.c/.h
├── motor_hardware/
├── pmw/
├── vision/
├── menu/
├── communication/
├── icm/
└── docs/
```

这个结构不是一次性完成，而是每次只拆一个明确模块，并在每次拆分后完成编译和无桨功能验证。
