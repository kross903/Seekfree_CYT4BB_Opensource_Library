# Project Information

本文件用于帮助 AI 快速建立对本项目结构、入口和常改位置的认识，减少每次任务前的重复摸索。

## Project Overview

- 工程根目录：`Seekfree_CYT4BB_Opensource_Library`
- 当前主要业务代码位于：`project/code`
- 用户层入口位于：`project/user`
- 项目层公共总头文件位于：`project/code/common/project_headfile.h`

## Directory Map

- `project/code/common`
  - 放公共初始化、PIT 任务、共享数据以及项目层总头文件。
  - 关键文件：
    - `All_Init.c/.h`
    - `PIT_Demo.c/.h`
    - `shared_data.h`
    - `project_headfile.h`

- `project/code/communication`
  - 放通信和上位机交互相关代码。
  - 典型模块：
    - `vofa.c/.h`：VOFA 调试数据回传
    - `core_link.c/.h`：核心串口链路
    - `cross_core_camera.c/.h`
    - `dshot600.c/.h`

- `project/code/control`
  - 放控制相关逻辑和数学工具。
  - 典型模块：
    - `flight_control.c/.h`
    - `pid_controller.c/.h`
    - `ControlMath.c/.h`
    - `joystick.c/.h`

- `project/code/icm`
  - 放 IMU 相关驱动与姿态/角速度接口。
  - 典型模块：
    - `icm.c/.h`
    - `icm42688.c/.h`

- `project/code/kalman_filter`
  - 放高度、陀螺等滤波与融合相关代码。
  - 典型模块：
    - `height_manager.c/.h`
    - `height_fusion.c/.h`
    - `gyro_kalman.c/.h`

- `project/code/menu`
  - 放菜单系统、监视页、参数页面相关代码。
  - 典型模块：
    - `menu.c/.h`
    - `screen_pages.c/.h`
    - `parameter_flash.c/.h`
    - `parameter_scheme.c/.h`
    - `key.c/.h`

- `project/code/pmw`
  - 放 PMW3901 光流解算相关代码。
  - 典型模块：
    - `pmw_flow.c/.h`

- `project/code/vision`
  - 放视觉、红外定位、IPM、灯塔检测等模块。
  - 典型模块：
    - `beacon_detect.c/.h`
    - `drone_ipm.c/.h`
    - `drone_ipm_embedded.c/.h`
    - `ipm_interface.c/.h`
    - `ir_localization.c/.h`

- `project/code/information_for_ai`
  - 放 AI 约束说明和项目结构说明。
  - 当前文件：
    - `AI_NOTES.md`
    - `information.md`

## Runtime Entry Points

- `project/user/main_cm7_0.c`
  - CM7_0 主入口。
  - 监视页快照构建、主循环调度、图像/状态展示相关逻辑主要从这里往外看。

- `project/user/main_cm7_1.c`
  - CM7_1 主入口。
  - 如果任务涉及双核分工或另一路主循环，需要检查这里。

- `project/user/cm7_0_isr.c`
  - 中断服务相关入口。
  - 如果任务涉及 PIT、串口、DMA、输入捕获等中断触发链路，需要同时检查这里。

- `project/code/common/All_Init.c`
  - 系统初始化汇总位置。
  - 新增模块初始化时，通常需要检查这里。

- `project/code/common/PIT_Demo.c`
  - 控制周期任务、传感器周期任务、PMW3901 采样/解算接线的重要入口。
  - 涉及姿态、油门、电机控制、光流周期调用时，优先查看这里。

## Common Debug / Observation Points

- `project/code/communication/vofa.c`
  - 上位机波形观察入口。
  - 当需要快速看变量波形、验证补偿或控制效果时，通常从这里改回传通道。

- `project/code/menu/screen_pages.c`
  - 屏幕监视页布局和刷新逻辑。
  - 需要在本机屏幕上增加状态观测页面时，从这里改。

- `project/user/main_cm7_0.c`
  - 监视页快照通常在这里组装，再传给 `screen_pages.c` 展示。

## Header Usage Convention

- `project/code/common/project_headfile.h` 是项目层聚合头文件。
- `project/code` 下的 `.c` 文件可以优先通过它拿到项目层公共声明。
- `project/user/main_cm7_0.c` 和 `project/user/main_cm7_1.c` 当前也已按这个方式接入。
- 模块自己的 `.h` 文件仍应尽量保持最小必要包含，不要把 `project_headfile.h` 当作所有头文件的默认依赖。

## Module Notes

- 光流解算：
  - 主要看 `project/code/pmw/pmw_flow.c`
  - 周期接线主要看 `project/code/common/PIT_Demo.c`
  - 观测链路通常同时涉及 `project/code/communication/vofa.c` 和 `project/code/menu/screen_pages.c`

- 高度相关：
  - ToF 原始量、姿态修正高度、融合高度通常分布在 `project/code/kalman_filter/height_manager.c` 及主循环快照中

- 菜单/监视页相关：
  - 页面枚举和快照结构一般在 `project/code/menu/screen_pages.h`
  - 页面绘制和刷新在 `project/code/menu/screen_pages.c`

## Change Checklist

- 改公共接口时，检查是否需要同步更新 `project_headfile.h`
- 改模块目录或文件名时，检查源码 include 路径和 IAR 工程文件路径
- 改调试页或 VOFA 通道时，检查主循环快照和对应显示/发送逻辑
- 改 PMW、姿态、高度链路时，优先核对采样周期、坐标映射和中间变量观测入口

## Future Expansion

- 在此处继续追加更细的模块关系、调用链说明或常见陷阱。
