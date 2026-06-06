# Vision 模块总览

这个目录现在可以按 3 层去理解，而不是把所有文件都当成一个大模块看：

```text
图像业务层
├── beacon_detect.*        红外信标检测
└── ir_localization.*      三点红外定位

项目适配层
└── ipm_interface.*        把 IMU / 高度 / IPM 数学封成工程接口

底层数学层
├── drone_ipm_embedded.*   当前工程运行时实际使用的 IPM
├── drone_ipm.*            通用/双精度参考实现
└── ipm_demo.c             接口调用示例
```

## 先看什么

如果你的目标是快速理清主流程，建议按这个顺序阅读：

1. `ipm_interface.h`
2. `beacon_detect.h`
3. `ir_localization.h`
4. `ipm_interface.c`
5. `drone_ipm_embedded.c`

如果你的目标是推导逆透视数学，再去看：

1. `drone_ipm.h`
2. `drone_ipm.c`

如果你只是想知道怎么接到主循环里，看：

1. `ipm_demo.c`
2. `user/cpu2_main.c`

## 每个文件到底干什么

| 文件 | 角色 | 是否主流程使用 | 说明 |
|------|------|----------------|------|
| `beacon_detect.c/.h` | 红外信标检测 | 是 | `Init/Process/GetNearestIndex` 都在 `cpu2_main.c` 主路径上 |
| `ir_localization.c/.h` | 三点红外定位 | 是 | `Init/Process` 在主路径上，`FindPoints` 现在被 `Process` 复用，`GetBinaryImage` 保留调试用 |
| `ipm_interface.c/.h` | 项目封装层 | 是 | 统一接 IMU、高度、坐标换算和俯视图显示 |
| `drone_ipm_embedded.c/.h` | 嵌入式 IPM 核心 | 是 | 工程真正跑的逆透视数学实现 |
| `drone_ipm.c/.h` | 通用参考版本 | 否 | 更适合做推导、桌面验证、对照理解 |
| `ipm_demo.c` | 示例代码 | 否 | 展示如何调用 IPM 接口 |

## 工程里的真实调用链

从 `user/cpu2_main.c` 当前代码看，`vision` 主流程是：

```text
每帧拿到原始图像
    -> IPM_Interface_UpdateFromIMU(height_cm)
    -> IR_Localization_Process(image, &ir_pose_estimate)
    -> BeaconDetect_Process(image, g_beacon_results, max)
    -> BeaconDetect_GetNearestIndex(...)
```

也就是说：

- `IPM_Interface_UpdateFromIMU()` 先刷新本帧姿态和高度
- `IR_Localization_Process()` 和 `BeaconDetect_Process()` 共用同一套 IPM 参数
- 两个业务模块都直接吃原始灰度图，不互相依赖

## 这两部分现在还有没有冗余

按当前工程实际调用关系看：

- `BeaconDetect_Init()`、`BeaconDetect_Process()`、`BeaconDetect_GetNearestIndex()` 都在 `cpu2_main.c` 中被调用，没有对外冗余接口。
- `IR_Localization_Init()`、`IR_Localization_Process()` 都在 `cpu2_main.c` 中被调用。
- `IR_FindPoints()` 之前更偏调试接口；现在已经被 `IR_Localization_Process()` 直接复用，避免同一份“二值化 + 亮斑提取”逻辑写两次。
- `IR_GetBinaryImage()` 当前没有被主循环引用，保留为显示二值图的调试接口。

## 你可以把它理解成两条并行业务线

### 1. 红外信标检测线

用途：找出图像里有哪些亮目标，并给出最近目标。

流程：

```text
原始图像
-> binarize()
-> bfs_region()
-> 面积/宽高比/占空比过滤
-> IPM_Image2World()
-> 输出 BeaconResult_t[]
```

入口函数：

- `BeaconDetect_Process()`
- `BeaconDetect_GetNearestIndex()`

适合关注的参数：

- `g_beacon_config.binary_threshold`
- `g_beacon_config.area_threshold`
- `g_beacon_config.min_aspect`
- `g_beacon_config.max_aspect`
- `g_beacon_config.min_occupancy`

### 2. 三点红外定位线

用途：从 3 个红外灯组成的三角形反推目标中心位置和高度。

流程：

```text
原始图像
-> binarize_image()
-> find_ir_points_internal()
-> 3 个亮斑质心
-> IPM_Image2World()
-> 计算三角形中心
-> 根据边长反推 z
-> 输出 IR_Pose_t
```

入口函数：

- `IR_Localization_Process()`
- `IR_FindPoints()` 只看前半段检测，不做逆透视

适合关注的参数：

- `IR_THRESHOLD`
- `IR_TRIANGLE_SIDE_M`

## IPM 这一层怎么分工

### `ipm_interface.*`

这是项目接口层，负责解决“工程怎么用”的问题：

- 从 IMU 全局变量读取 `Roll_a`、`Pitch_a`
- 接收高度
- 把调用统一成 `IPM_Image2World()` / `IPM_World2Image()`
- 提供俯视图显示接口

### `drone_ipm_embedded.*`

这是数学核心层，负责解决“坐标怎么换”的问题：

- 建立旋转矩阵
- 做图像坐标与世界坐标互相映射
- 生成俯视图

### `drone_ipm.*`

这是参考实现，重点是更容易读懂和推导，不是当前主循环的必经路径。

## 坐标系统一约定

### 世界坐标系

- `X`：向前
- `Y`：向右
- `Z`：向下

### 图像坐标系

- 原点在左上角
- `u`：向右，对应世界/机体系左侧的反方向
- `v`：向下，对应世界/机体系前后方向

### 俯视图坐标系

- 中心点：无人机正下方地面点
- 上方：前方 `+X`
- 左方：左侧 `+Y`

## IPM 数学主线

底层逆透视的核心步骤不复杂，可以只记住这 4 步：

1. 像素坐标转归一化坐标
2. 构建机体系光线方向 `ray_body = [v_n, u_n, 1]`
3. 用姿态旋转矩阵转到世界系
4. 求光线与地面 `Z = 0` 的交点

对应到代码：

- 像素转地面：`DroneIPM_Embedded_Image2World()`
- 地面转像素：`DroneIPM_Embedded_World2Image()`
- 俯视图重映射：`DroneIPM_Embedded_BirdView()`

## 哪些文件是“核心”，哪些是“辅助”

核心：

- `beacon_detect.c`
- `ir_localization.c`
- `ipm_interface.c`
- `drone_ipm_embedded.c`

辅助：

- `drone_ipm.c`
- `ipm_demo.c`

这能帮你避免一开始就陷进参考实现或示例代码里。

## 以后改代码时可以怎么下手

如果你想调检测效果，先改：

- `beacon_detect.h/.c` 里的阈值和筛选参数
- `ir_localization.h` 里的 `IR_THRESHOLD`

如果你想调物理坐标精度，先改：

- `drone_ipm_embedded.h` 里的 `CAM_FOV_X / CAM_FOV_Y`
- `IPM_Interface_UpdateFromIMU()` 中姿态符号定义

如果你想看显示效果，先看：

- `IPM_DisplayBirdView()`
- `IR_GetBinaryImage()`

## 当前目录保留这几种文件的原因

看起来文件多，其实是因为这里混了三类东西：

1. 工程真正运行的代码
2. 数学参考实现
3. 调用示例

把这个分层想清楚后，目录就不乱了。主线只需要抓住：

`beacon_detect / ir_localization -> ipm_interface -> drone_ipm_embedded`

---

# 无人机正朝下摄像头逆透视变换 (Drone IPM)

下面保留一份更偏 IPM 原理的说明，方便单独查逆透视部分。

## 功能说明

本模块实现无人机正朝下摄像头的逆透视变换（Inverse Perspective Mapping, IPM），主要功能：

1. **消除透视畸变**：将倾斜拍摄的图像转换为正交俯视图
2. **姿态补偿**：考虑无人机的 roll（滚转）和 pitch（俯仰）角度
3. **正下方居中**：确保无人机正下方的地面点始终在输出图像的中心

## 文件结构

```text
code/vision/
├── drone_ipm.h
├── drone_ipm.c
├── drone_ipm_embedded.h
├── drone_ipm_embedded.c
├── ipm_interface.h
├── ipm_interface.c
└── README.md
```

## 数学原理

### 逆透视变换流程

1. **像素坐标 -> 归一化坐标**

   ```text
   u_n = (img_x - cx) / fx
   v_n = (img_y - cy) / fy
   ```

2. **构建光线方向（机体系）**

   ```text
   ray_body = [v_n, u_n, 1]
   ```

3. **旋转到世界坐标系**

   ```text
   ray_world = R * ray_body
   ```

4. **求与地面交点**

   ```text
   t = -Z_cam / ray_world[2]
   world_x = X_cam + t * ray_world[0]
   world_y = Y_cam + t * ray_world[1]
   ```

### 旋转矩阵

对于当前正朝下摄像头场景，常用的形式是：

```text
R = | cp      0     sp     |
    | sr*sp   cr   -sr*cp  |
    |-cr*sp   sr    cr*cp  |
```

其中：

- `cp = cos(pitch)`
- `sp = sin(pitch)`
- `cr = cos(roll)`
- `sr = sin(roll)`

## 使用方法

### 工程实际用法

```c
#include "ipm_interface.h"

IPM_Interface_Init();

IPM_Interface_UpdateFromIMU(height_cm);

float world_x, world_y;
IPM_Image2World(img_x, img_y, &world_x, &world_y);
```

### 直接调用嵌入式版本

```c
#include "drone_ipm_embedded.h"

DroneIPM_Embedded_Init();
DroneIPM_Embedded_Update(roll_deg, pitch_deg, height_cm);

float result[2];
DroneIPM_Embedded_Image2World(img_x, img_y, result);
```

## 配置参数

在 `drone_ipm_embedded.h` 中可修改：

```c
#define IMAGE_WIDTH   188
#define IMAGE_HEIGHT  120
#define CAM_FOV_X     50.0f
#define CAM_FOV_Y     35.0f
```

## 输出图像说明

生成的俯视图特点：

- **中心**：对应无人机正下方地面点 `(world_x=0, world_y=0)`
- **上方**：对应前进方向 `+X`
- **左方**：对应左侧 `+Y`
- **缩放**：由 `scale` 参数控制，单位为物理单位/像素

例如 `scale = 0.5` 表示俯视图中每个像素代表 `0.5 cm` 的地面距离。

## 与原参考代码的区别

| 特性 | 原代码(IPM.c) | 本代码 |
|------|---------------|--------|
| 摄像头朝向 | 前向+倾角 | 正朝下 |
| 主要用途 | 车载前视 | 无人机俯视 |
| 坐标原点 | 车辆中心 | 正下方地面 |
| yaw 处理 | 需要 | 不需要 |

## 注意事项

1. 确保 IMU 姿态角定义与本代码一致
2. 高度参数单位需与输出坐标单位一致
3. 视场角参数应根据实际摄像头标定
