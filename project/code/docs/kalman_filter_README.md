# Kalman Filter 模块说明

## 目录用途

这个目录现在保留两条已经接入主流程的滤波链路：

1. `gyro_kalman.c/.h`
作用：对 `Roll / Pitch / Yaw` 三轴角速度做一维卡尔曼滤波。

2. `height_fusion.c/.h`
作用：把 `ToF` 高度和红外反推高度融合成统一高度结果，供高度控制直接使用。

## 当前保留的对外接口

### gyro_kalman

- `GyroKalman_Init()`
- `GyroKalman_Update()`

### height_fusion

- `g_height_result`
- `HeightFusion_UpdateToF()`
- `HeightFusion_UpdateIR()`
- `HeightFusion_Predict()`

## 已删除的冗余部分

这次只删除了“当前没有任何外部调用点”的内容：

### gyro_kalman

- 二维卡尔曼滤波结构和接口
  - `GyroKalman2D_t`
  - `GyroKalman2D_Init()`
  - `GyroKalman2D_Update()`
  - `GyroKalman2D_GetAccel()`
  - `GyroKalman2D_Reset()`
- 未被主流程使用的复位接口
  - `GyroKalman_Reset()`
- 未被外部读取的内部状态变量
  - `roll_gyro_kalman`
  - `pitch_gyro_kalman`
  - `yaw_gyro_kalman`
  - `Roll_g_Kalman`
  - `Pitch_g_Kalman`
  - `Yaw_g_Kalman`

### height_fusion

- 未被主流程调用的查询/重置接口
  - `HeightFusion_Init()`
  - `HeightFusion_Reset()`
  - `HeightFusion_GetResult()`
  - `HeightFusion_GetHeight()`
  - `HeightFusion_GetVelocity()`
- 只有声明、没有实际实现，也没有调用点的接口
  - `HeightFusion_SetToFNoise()`
  - `HeightFusion_SetIRNoise()`
- 未被外部使用的内部状态对象
  - `g_height_fusion`

## 谁在调用这些代码

### 1. 陀螺仪卡尔曼滤波

调用者：

- `code/icm/icm.c`

调用路径：

1. `Device_Init()`
2. `init_flight_runtime()`
3. `ICM_Calculate_Init()`
4. `Filter_Init()`
5. `GyroKalman_Init()`

运行时路径：

1. `cc61_pit_ch0_isr()` 进入 4kHz 控制中断
2. `Get_Angular_Rate()`
3. `GyroKalman_Update()`
4. 得到滤波后的三轴角速度
5. 角速度继续送入姿态控制内环 PID

### 2. 高度融合卡尔曼滤波

调用者：

- `code/PIT_Demo.c`
- `code/vofa.c`（只读取结果，不更新滤波器）

运行时路径：

1. `cc61_pit_ch0_isr()` 进入 4kHz 控制中断
2. 每隔一段时间执行 ToF 高度更新
3. 如果 ToF 有效，调用 `HeightFusion_UpdateToF()`
4. 如果红外定位有效，继续调用 `HeightFusion_UpdateIR()`
5. 如果本周期没有有效高度测量，调用 `HeightFusion_Predict()`
6. 融合结果写入 `g_height_result`
7. `PIT_Demo.c` 里的高度 PID 使用 `g_height_result.height_cm`
8. `vofa.c` 可把 `g_height_result.height_cm` 回传到上位机

## 关于“主函数怎样执行”

这个工程不是一个简单的 `main()` 串行调用结构。

实际执行方式是：

1. 上电后先走初始化流程
   - `Device_Init()` 完成外设、姿态解算、电调和中断初始化
2. 初始化完成后，核心控制逻辑不是在一个普通函数里循环跑
3. 真正的主流程由周期中断驱动
   - `cc61_pit_ch0_isr()`：姿态解算、角速度滤波、高度融合、PID 控制
   - `cc60_pit_ch1_isr()`：DShot 输出发送

所以，这个目录里的代码也是被“初始化流程 + 周期中断流程”驱动的，不是被一个单独的 `main()` 直接顺序调用。