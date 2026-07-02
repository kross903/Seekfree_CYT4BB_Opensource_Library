# Vision 模块总览

`project/code/vision` 现在按功能职责分成 4 类目录：

```text
vision/
├── pipeline/      每帧视觉流水线
├── detection/     红外目标板和信标识别
├── projection/    图像点到地面坐标的投影链路
└── reference/     参考实现和调用示例，不参与主流程
```

## 主流程

当前飞行主流程只看这一条链路：

```text
CM7_0
  -> CrossCoreCamera_ControlCorePublishInput(roll, pitch, height)

CM7_1
  -> VisionRuntime_ProcessFrameWithPose()
  -> vision_frame_pipeline
     -> 二值化
     -> 八邻域连通域
     -> IPM_Image2World()
     -> IR_Localization_AppendBlob()
     -> BeaconDetect_AppendBlob()
  -> CrossCoreCamera 回传结果

CM7_0
  -> CrossCoreCamera_ControlCoreFetchFrame()
```

## 目录职责

### `pipeline/`

核心文件：

- `vision_frame_pipeline.c/.h`

职责：

- 采集并处理每帧图像
- 统一二值化和连通域提取
- 分发连通域给 detection
- 汇总诊断数据和 beacon slots

### `detection/`

核心文件：

- `ir_target_localization.c/.h`
- `beacon_detection.c/.h`

职责：

- `ir_target_localization`：红外目标板定位，支持 4 点定位和 3 点补全
- `beacon_detection`：从较大亮斑中筛选信标并寻找最近目标

不负责扫描整张图，也不负责 IPM 坐标换算。

### `projection/`

核心文件：

- `ground_projection.c/.h`
- `ground_projection_ipm_core.c/.h`
- `camera_calibration.c/.h`

职责：

- `ground_projection`：项目级投影入口，统一接姿态、高度、标定和 IPM 核心
- `ground_projection_ipm_core`：嵌入式 IPM 数学核心
- `camera_calibration`：相机内参、畸变、安装外参和点去畸变入口

业务代码应优先调用 `IPM_Image2World()`，不要直接依赖 IPM 核心实现。

### `reference/`

文件：

- `drone_ipm.c/.h`
- `ipm_demo.c`

职责：

- 保存双精度/通用 IPM 推导实现
- 保存接口调用示例
- 作为理解和对照资料

这些文件不参与主工程编译。

## 命名说明

本轮只重命名目录和文件，不改公开函数前缀：

- `VisionRuntime_*` 暂时保留在 `vision_frame_pipeline.*`
- `IR_Localization_*` 暂时保留在 `ir_target_localization.*`
- `BeaconDetect_*` 暂时保留在 `beacon_detection.*`
- `IPM_*` 和 `DroneIPM_Embedded_*` 暂时保留在 projection 目录

这样可以先获得清晰结构，同时降低对控制、通信、VOFA 和双核链路的影响。

## 调参入口

- 图像二值化和大小光斑分流：`pipeline/vision_frame_pipeline.h`
- 红外目标板几何和丢失保护：`detection/ir_target_localization.c`
- 信标筛选参数：`detection/beacon_detection.c`
- 相机标定和安装偏差：`projection/camera_calibration.h`
- IPM 投影模型兜底参数：`projection/ground_projection_ipm_core.h`
