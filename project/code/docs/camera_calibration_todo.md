# 相机标定与去畸变后续代办

本文档用于提醒后续处理无人机视觉链路中的相机内参、畸变、外参和 IPM 标定问题。当前无人机工程视觉链路可以运行，但相机标定仍较粗略，后续如果要提升视觉定位精度，应优先补齐本文件中的事项。

## 当前状态

当前工程已有视觉处理基础：

```text
MT9V03X 图像
  ↓
IR_Localization_Process() / BeaconDetect_Process()
  ↓
IPM_Image2World()
  ↓
地面相对坐标 world_x / world_y
```

当前主要问题：

1. 相机内参尚未正式标定。
2. 镜头畸变尚未补偿。
3. 相机相对机体的安装外参尚未系统标定。
4. 当前 IPM 主要依赖 FOV、高度、roll、pitch 的近似模型。
5. `ir_localization.c` 和 `beacon_detect.c` 不应直接承担标定职责。

## 推荐目标结构

后续建议新增统一标定模块：

```text
project/code/vision/
├── camera_calibration.c/.h
│   ├── 相机内参 fx / fy / cx / cy
│   ├── 畸变参数 k1 / k2 / p1 / p2 / k3
│   ├── 相机安装外参
│   └── CameraCalibration_UndistortPoint()
│
├── camera_undistort_table.h
│   └── 离线生成的去畸变查表
│
├── ipm_interface.c/.h
│   ├── IPM_Interface_UpdateFromIMU()
│   ├── IPM_Image2World()
│   └── 统一调用标定和 IPM
│
├── drone_ipm_embedded.c/.h
│   └── 像素射线到地面坐标的数学转换
│
├── ir_localization.c/.h
│   └── 只负责红外正方形目标识别
│
└── beacon_detect.c/.h
    └── 只负责多个亮点 / 信标识别
```

职责划分原则：

```text
识别算法：只找图像上的点
标定模块：修正图像点和相机模型
IPM 模块：图像点转地面坐标
控制模块：只使用 world_x / world_y
```

## 第一阶段：集中相机标定参数

建议新增：

```text
project/code/vision/camera_calibration.h
project/code/vision/camera_calibration.c
```

先集中存放以下参数：

```c
#define CAMERA_FX    0.0f
#define CAMERA_FY    0.0f
#define CAMERA_CX    0.0f
#define CAMERA_CY    0.0f

#define CAMERA_K1    0.0f
#define CAMERA_K2    0.0f
#define CAMERA_P1    0.0f
#define CAMERA_P2    0.0f
#define CAMERA_K3    0.0f

#define CAMERA_MOUNT_ROLL_OFFSET_DEG     0.0f
#define CAMERA_MOUNT_PITCH_OFFSET_DEG    0.0f
#define CAMERA_MOUNT_YAW_OFFSET_DEG      0.0f

#define CAMERA_OFFSET_X_CM               0.0f
#define CAMERA_OFFSET_Y_CM               0.0f
#define CAMERA_OFFSET_Z_CM               0.0f
```

说明：

- 一开始可以先填近似值或 0 值。
- 不要把这些参数散落到 `ir_localization.c`、`beacon_detect.c`、`mt9v03x.c` 中。
- 摄像头驱动只负责采图，不负责视觉标定。

## 第二阶段：完成内参标定

使用当前无人机实际安装的 MT9V03X 摄像头采集棋盘格图像。

目标得到：

```text
fx
fy
cx
cy
k1
k2
p1
p2
k3
```

推荐流程：

```text
1. 固定摄像头和镜头配置。
2. 保持与比赛/飞行时一致的分辨率，例如 188 x 120。
3. 关闭或固定曝光、增益，避免标定图像变化过大。
4. 采集多张不同角度、不同位置的棋盘格图像。
5. 用 OpenCV calibrateCamera() 离线标定。
6. 将 camera_matrix 和 dist_coeffs 写入 camera_calibration.h。
```

需要注意：

- 标定必须使用最终实际镜头和分辨率。
- 如果更换镜头、分辨率或裁剪方式，需要重新标定。
- 不要直接使用 `Unicycle_Beacon_2025` 中的 `mapping_table.h`，那是其他相机/镜头/安装条件下的结果。

## 第三阶段：生成去畸变查表

嵌入式上不建议每次运行完整畸变公式，推荐离线生成查表。

可以新增：

```text
project/code/vision/camera_undistort_table.h
```

建议接口：

```c
void CameraCalibration_UndistortPoint(float distorted_x,
                                      float distorted_y,
                                      float *undistorted_x,
                                      float *undistorted_y);
```

推荐运行方式：

```text
原始图像
  ↓
识别算法找亮点中心 distorted_x / distorted_y
  ↓
CameraCalibration_UndistortPoint()
  ↓
undistorted_x / undistorted_y
  ↓
IPM_Image2World()
  ↓
world_x / world_y
```

优先只对目标中心点去畸变，不要一开始就对整张图去畸变。

原因：

- 目标点数量少，计算量低。
- 对 `IR_Localization` 和 `BeaconDetect` 都能复用。
- 不改变原始图像显示和调试链路。

## 第四阶段：标定相机安装外参

外参包括：

```text
相机相对机体的 roll 安装偏差
相机相对机体的 pitch 安装偏差
相机相对机体的 yaw 安装偏差
相机中心相对机体中心的 x/y/z 偏移
ToF 高度与相机光心高度之间的差值
```

建议先用物理测量得到初值：

```c
#define CAMERA_MOUNT_ROLL_OFFSET_DEG     0.0f
#define CAMERA_MOUNT_PITCH_OFFSET_DEG    0.0f
#define CAMERA_MOUNT_YAW_OFFSET_DEG      0.0f

#define CAMERA_OFFSET_X_CM               0.0f
#define CAMERA_OFFSET_Y_CM               0.0f
#define CAMERA_OFFSET_Z_CM               0.0f
```

然后用地面标定板验证：

```text
1. 将无人机固定在已知高度。
2. 在地面放置已知坐标点或红外标志点。
3. 读取 IPM_Image2World() 输出。
4. 对比真实地面坐标。
5. 逐步修正 pitch / roll / yaw / 图像中心 / FOV / 高度偏差。
```

外参应在 `ipm_interface.c` 或 `drone_ipm_embedded.c` 的更新流程中统一使用，而不是写在识别算法里。

## 第五阶段：改造 IPM 接口

当前推荐让所有图像点到地面点的转换统一走：

```c
IPM_Image2World(img_x, img_y, &world_x, &world_y);
```

未来可在 `IPM_Image2World()` 内部加入：

```text
输入原始图像点
  ↓
CameraCalibration_UndistortPoint()
  ↓
使用内参/外参生成相机射线
  ↓
结合 roll / pitch / height 求地面交点
  ↓
输出 world_x / world_y
```

这样 `ir_localization.c` 和 `beacon_detect.c` 不需要关心相机标定细节。

## 第六阶段：验证指标

标定和去畸变完成后，应验证以下指标：

```text
固定高度下，地面已知点的 world_x / world_y 误差
不同高度下，同一个地面点的坐标一致性
不同 roll / pitch 倾角下，IPM 输出是否仍稳定
画面中心点是否接近 world_x = 0, world_y = 0
画面边缘点是否存在明显畸变残差
IR_Localization 的中心点是否更稳定
BeaconDetect 的距离和角度是否更可信
```

建议先只做无桨静态测试，不要直接接入飞控闭环。

## 不建议当前立即做的事

1. 不要直接搬 `Unicycle_Beacon_2025/code/mapping_table.h`。
2. 不要把去畸变逻辑写进 `mt9v03x.c` 摄像头驱动。
3. 不要把内参/外参散落到 `ir_localization.c` 和 `beacon_detect.c`。
4. 不要一开始就整图去畸变，优先只对识别到的点去畸变。
5. 不要在未验证坐标精度前，把新的视觉坐标直接接入位置环。
6. 不要同时大改视觉识别、IPM、位置环控制链路。

## 推荐执行顺序

```text
1. 新增 camera_calibration.c/.h，集中参数。
2. 固定相机曝光、增益、分辨率。
3. 采集棋盘格图像。
4. 用 OpenCV 标定内参和畸变。
5. 生成去畸变点查表。
6. 在 IPM_Image2World() 前统一调用去畸变。
7. 测试静态地面坐标误差。
8. 标定相机安装外参。
9. 复测不同高度和姿态下的坐标稳定性。
10. 只在验证稳定后，再考虑接入飞控闭环。
```

## 未来完成标准

认为本事项完成时，应至少满足：

```text
camera_calibration.c/.h 已建立
内参和畸变参数来自实际摄像头标定
IPM_Image2World() 统一使用标定结果
IR_Localization 和 BeaconDetect 不再各自处理标定细节
固定高度静态坐标误差可接受
不同高度下坐标尺度变化合理
无桨测试中视觉定位稳定
```
