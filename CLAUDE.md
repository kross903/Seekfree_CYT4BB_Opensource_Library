# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 仓库概览

这是一个基于 CYT4BB 平台的双核无人机工程仓库。

- `project/`：业务工程代码与 IAR 工程文件。
- `libraries/`：平台 SDK、驱动和第三方底层库，通常作为被依赖层，不在这里放项目业务逻辑。
- `pid_simulation/`：用于离线复现和验证飞控 PID 行为的 Python 仿真目录。
- `pid_simulation\old`:存放01版本的仿真
- `older/`:存放02版本仿真

`project/code/本文件夹作用.txt` 说明了一个重要约束：新增无人机项目代码优先放在 `project/code/` 下，再由 IAR 工程纳入编译，而不是在别的目录随意扩展源码。

## 常用命令

### IAR 工程

仓库包含两个 IAR 工程：

- `project/iar/project_config/cyt4bb7_cm_7_0.ewp`
- `project/iar/project_config/cyt4bb7_cm_7_1.ewp`

对应工作区：

- `project/iar/project_config/cyt4bb7_cm_7_0.eww`
- `project/iar/project_config/cyt4bb7_cm_7_1.eww`

当前在工程文件中可确认的配置名是 `Debug`。

如果本机安装了 IAR Embedded Workbench，可使用类似下面的命令行构建：

```powershell
IarBuild.exe project/iar/project_config/cyt4bb7_cm_7_0.ewp -build Debug
IarBuild.exe project/iar/project_config/cyt4bb7_cm_7_1.ewp -build Debug
```

如果要在 IDE 中操作，直接打开对应 `.eww` 工作区即可。

仓库里还保留了 IAR 生成的 C-SPY 调试批处理模板：

- `project/iar/project_config/settings/cyt4bb7_cm_7_0.Debug.cspy.bat`
- `project/iar/project_config/settings/cyt4bb7_cm_7_1.Debug.cspy.bat`

这些文件更适合作为调试参数参考，因为里面带有生成时机器的绝对路径，直接跨机器复用前需要检查路径是否仍然有效。


## 架构说明

## 1. 大图景：双核分工

这个仓库不是单 MCU 单循环结构，而是围绕两个 CM7 核拆分：

- `project/user/main_cm7_0.c`：控制核心。负责整机初始化、菜单/监视界面、飞控状态管理，以及与姿态/高度/光流/位置相关的控制逻辑入口。
- `project/user/main_cm7_1.c`：相机核心。主循环非常薄，核心工作是 `CrossCoreCamera_CameraCoreInit()` 和 `CrossCoreCamera_CameraCoreTask()`，说明图像采集或相机侧处理被隔离到另一个核。

理解代码时，先判断改动属于哪一核：

- 飞控、PID、菜单、监视、融合、位置/高度闭环，多数在 CM7_0。
- 摄像头采集和跨核相机链路，多数在 CM7_1 及 `communication/cross_core_camera.*` 一带。

## 2. 启动路径与运行模型

控制核主流程在 `project/user/main_cm7_0.c`：

1. `initialize_control_core_modules()`
2. `Device_Init()`
3. `IR_Localization_Init()` / `IPM_Interface_Init()` / `HeightManager_Init()`
4. 主 `while(true)` 中运行 `menu_open()` 和监视页循环

`Device_Init()` 在 `project/code/common/All_Init.c` 中，把启动拆成几个阶段：

- 显示和基础外设初始化
- 默认 PID 参数初始化
- 运行时模块初始化
- 周期中断初始化

关键点是：这个工程的高频控制不靠主循环推进，而是靠定时中断驱动。`project/code/common/PIT_Demo.c` 明确了控制频率和大量运行态全局变量，它是飞控热路径的核心文件之一。

可以把系统分成两层：

- 主循环层：菜单、监视页、遥测、部分非实时服务。
- 中断层：姿态/速率/高度/位置等高频控制闭环与电机输出。

读代码时不要只找 `main()` 的顺序调用链；很多真正的控制逻辑是在 PIT 中断和传感器更新链里发生的。

## 3. 控制链路的核心分层

飞控控制不是单一 PID，而是多层串级：

- 角速度内环：`gyro_pid_roll/pitch/yaw`
- 姿态外环：`angle_pid_roll/pitch/yaw`
- 高度环：`HeightPID`
- 光流速度环：`vel_pid_x/y`
- 真位置环：`pos_pid_x/y`

默认参数集中在 `project/code/common/All_Init.c` 的 `init_default_pid_parameters()`。

`project/code/control/flight_control.c` 不是高频控制器本体，而是“对外统一访问层”：

- 把运行态 PID、目标角、油门、控制模式等收口到统一接口
- 给 VOFA、菜单、监视界面等慢速模块提供稳定读写入口
- 避免外部模块直接到处改热路径全局变量

因此如果需求是“给上层调参界面/串口协议暴露能力”，优先看 `flight_control.*`；如果需求是“改真实控制行为”，更多要看 `PIT_Demo.c`、PID 实现和传感器/融合链路。

## 4. 传感器、融合与控制的关系

控制所用反馈并不直接等于裸传感器输出，而是经过多层处理：

- IMU 相关滤波在 `project/code/kalman_filter/` 和 `project/code/icm/`
- 高度环当前热路径主要在 `project/code/kalman_filter/height_manager.c`，由 `HeightManager_Task_10ms()` 处理 ToF 高度、目标高度斜坡和 `HeightSpeedPID_output`
- 光流速度闭环核心依赖 `project/code/pmw/pmw_flow.c`，传感器选择和更新周期由 `project/code/pmw/flow_sensor_config.h` 决定
- 红外定位与 IPM 相关在 `project/code/vision/`

从 `kalman_filter/README.md` 可以看出：

- 陀螺相关 Kalman 更新最终服务于姿态/角速度控制
- ToF 高度处理结果会被高度 PID 使用，也会回传给上位监视；旧的 `height_fusion.*` 不应再作为高度 PID 当前主路径理解

从 `PIT_Demo.c` 的注释可以看出：

- 光流速度环反馈来自当前选中的光流传感器 + ToF/高度解算的机体水平速度；当前默认是 `UP-FLOW-302`，`PMW3901` 作为可选/历史兼容路径保留
- 当前 `UP-FLOW-302` 的光流更新周期为 25ms / 40Hz；不要再把光流链路默认理解成固定 PMW3901 20ms / 50Hz
- 位置环并不直接输出姿态，而是主要基于视觉/红外定位 `IR_Localization_Process()` 得到的 `ir_pose_estimate.x/y` 输出目标速度，再交给光流速度环

所以“位置控制”在这里其实是一个链式系统：

`位置观测 -> 位置环 -> 目标速度 -> 速度环 -> 目标姿态修正 -> 姿态环 -> 角速度环 -> 电机输出`

调某一级参数时要先确认上游观测和下游执行是否已经稳定，否则容易把问题误判成 PID 参数问题。

## 5. vision 模块在整个系统中的位置

`project/code/vision/README.md` 给出的结构很关键。这个目录不是单一算法文件，而是三层：

- 业务层：例如 beacon 检测、红外定位
- 项目接口层：`ipm_interface.*`
- 底层数学层：`drone_ipm_embedded.*` 与参考实现 `drone_ipm.*`

控制核中，监视页更新时会调用：

- `IPM_Interface_UpdateFromIMU(...)`
- `IR_Localization_Process(...)`

这说明视觉/红外定位并不是一个完全独立的离线模块，而是直接吃当前姿态和高度信息，再把定位结果喂回控制或监视链路。

一个重要认知是：`vision` 不只是“看图像”，它承担了从图像坐标到世界坐标的工程封装，因此改这里经常会影响位置环、监视页和跨核图像处理结果。

## 6. 菜单、监视与遥测不是控制本体，但强依赖运行态快照

`main_cm7_0.c` 和 `flight_control.c` 说明，菜单页、监视页、VOFA/串口调参与真实控制是解耦的：

- 控制本体在中断和运行态全局变量
- 上层页面/协议通过快照和统一接口读取这些状态

这意味着：

- 如果只是显示不对、页面值不刷新，先查快照构建和接口层。
- 如果是飞机行为不对，但页面显示正常，更多应查控制热路径或传感器链。

## 7. `pid_simulation/` 的角色

`pid_simulation/old/README.md` 说明现有仿真不是独立的理论模型，而是显式对齐当前固件 PID 语义的离线验证工具。

它已经覆盖：

- position loop -> velocity loop -> target roll/pitch
- angle PID -> target angular rate
- gyro PID -> roll/pitch/yaw control output
- 简化飞行器模型、光流/高度传感器延迟与噪声

因此当修改：

- PID 数学实现
- 默认 PID 参数
- 速度/位置/高度链路
- 目标姿态到速率的过渡逻辑

应优先考虑是否同步更新 `pid_simulation/`，否则仿真会失去“板端等价近似”的价值。

## 8. 修改代码时的目录判断

遇到需求时，优先按下面方式定位：

- 改默认 PID 参数：`project/code/common/All_Init.c`
- 改 PID 数学行为：`project/code/control/pid_controller.*`
- 改上层 PID 读写/监视暴露：`project/code/control/flight_control.*`
- 改高频控制链、姿态/速度/位置/电机输出：`project/code/common/PIT_Demo.c`
- 改视觉定位/IPM：`project/code/vision/`
- 改滤波/高度融合：`project/code/kalman_filter/`
- 改光流速度估计：`project/code/pmw/`
- 改离线参数验证：`pid_simulation/`
- 改平台或底层驱动依赖：`libraries/`

## 待补充

### 硬件连接与下载流程

- 待补充

### 常见联调步骤

- 待补充

### 参数调试约定

- 待补充

### 版本发布/打包流程

- 待补充
