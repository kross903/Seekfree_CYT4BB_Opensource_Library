# VOFA 参数与回传指南

本文按当前 `communication/vofa.c` 的实际命令和回传模式整理。重点原则：

- 调参先看推荐入口，不优先打开底层诊断。
- 光流速度链只保留 `F8` 和 `FU` 两个推荐入口。
- `FJ`、`FK`、`FT` 已从固件回传中删除，发送这些命令不会切换当前回传模式。

## 串口配置

| 项目 | 当前值 |
|---|---|
| 串口 | `UART_1` |
| 波特率 | `115200` |
| TX | `UART1_TX_P04_1` |
| RX | `UART1_RX_P04_0` |

## 命令格式

```text
<命令><项目>=<数值>!
```

示例：

```text
R1=5.50!
V4=120.0!
Gx=0.82!
F8!
FU!
```

说明：

- 第 1 个字符是命令类别。
- 第 2 个字符是项目编号或回传模式编号。
- `=` 后是数值；只切换回传模式时可以省略数值，例如 `F8!`。
- 每条命令必须以 `!` 结束。

## 参数命令总览

| 命令 | 用途 | 项目 |
|---|---|---|
| `R` | Roll 姿态环 PID | `R1 Kp`, `R2 Ki`, `R3 Kd` |
| `r` | Roll 角速度环 PID | `r1 Kp`, `r2 Ki`, `r3 Kd`, `r4 integral_limit` |
| `P` | Pitch 姿态环 PID | `P1 Kp`, `P2 Ki`, `P3 Kd` |
| `p` | Pitch 角速度环 PID | `p1 Kp`, `p2 Ki`, `p3 Kd`, `p4 integral_limit` |
| `Y` | Yaw 姿态环 PID | `Y1 Kp`, `Y2 Ki`, `Y3 Kd` |
| `y` | Yaw 角速度环 PID | `y1 Kp`, `y2 Ki`, `y3 Kd`, `y4 integral_limit` |
| `H` | 高度环 PID | `H1 Kp`, `H2 Ki`, `H3 Kd` |
| `h` | 高度速度内环 PID | `h1 Kp`, `h2 Ki`, `h3 Kd` |
| `S` | 目标油门 | `S1 throttle` |
| `T` | IMU 安装补偿角 | `T1 roll_acc_offset`, `T2 pitch_acc_offset`, `T3 reserved_yaw_offset` |
| `B` | 悬停基准角补偿 | `B1 roll_base_offset`, `B2 pitch_base_offset` |
| `V` | X 方向速度环 PID | `V1 Kp`, `V2 Ki`, `V3 Kd`, `V4 output_limit`, `V5 integral_limit` |
| `W` | Y 方向速度环 PID | `W1 Kp`, `W2 Ki`, `W3 Kd`, `W4 output_limit`, `W5 integral_limit` |
| `I` | 红外 X 位置环 PID | `I0 enable`, `I1 Kp`, `I2 Ki`, `I3 Kd`, `I4 output_limit`, `I5 integral_limit` |
| `J` | 红外 Y 位置环 PID | `J0 enable`, `J1 Kp`, `J2 Ki`, `J3 Kd`, `J4 output_limit`, `J5 integral_limit` |
| `G` | 光流陀螺去旋转增益 | `Gx x_gain`, `Gy y_gain` |

参数回读统一使用 `F1!`。先发送某组参数命令，再切到 `F1!`，即可回读该组当前值。

## 推荐回传入口

| 场景 | 推荐命令 | 用途 |
|---|---|---|
| 光流速度环调参 | `FU!` -> `F8!` | 先确认融合状态和 ToF，再观察速度环目标、反馈、积分项和输出 |
| 高度环调参 | `FD!` 或 `FS!` | `FD` 看 8 路摘要，`FS` 看高度估计和串级 PID 全链路 |
| 红外位置闭环 | `FI!` 或 `FP!` | `FI` 看位置环摘要，`FP` 看 X/Y/高度闭环总览 |
| 底层光流排障 | `F0!` 或 `FE!` | `F0` 看后端光流补偿，`FE` 看传感器原始输入 |
| 红外识别排障 | `FG!`, `FH!`, `FQ!` | 分别看中心点、丢失保护、连通域/筛选统计 |
| 主循环耗时 | `FR!` | 看 VOFA/监视页相关主循环耗时 |

## 支持的回传模式

| 命令 | 通道数 | 用途 |
|---|---:|---|
| `F0!` | 8 | 光流后端原始角位移、陀螺去旋转和原始速度 |
| `F1!` | 8 | PID/参数回读 |
| `F2!` | 8 | 红外位置、高度、姿态摘要 |
| `F3!` | 8 | VOFA 链路测试 |
| `F4!` | 8 | 陀螺仪原始值和校准值 |
| `F5!` | 8 | 加速度原始值和校准值 |
| `F6!` | 8 | Roll/Pitch 角度与角速度目标对比 |
| `F7!` | 8 | ADC、摇杆滤波值和目标角 |
| `F8!` | 8 | 光流速度环调参摘要 |
| `F9!` | 16 | 姿态解算总览 |
| `FA!` | 8 | 加速度水平化残差 |
| `FB!` | 8 | 光流速度旋转到世界系后的速度和积分位置 |
| `FC!` | 8 | 速度积分位置与光流单帧位移积分位置对比 |
| `FD!` | 8 | 高度环摘要 |
| `FE!` | 8 | 光流传感器原始输入诊断 |
| `FF!` | 8 | Roll/Pitch 角度环监视 |
| `FG!` | 8 | 红外正方形中心点 |
| `FH!` | 8 | 红外丢失保护状态机 |
| `FI!` | 8 | 红外位置闭环摘要 |
| `FL!` | 8 | 红外位置环 PID 调参 |
| `FM!` | 8 | X 轴位置-速度-Pitch-角速度链路 |
| `FN!` | 8 | Y 轴位置-速度-Roll-角速度链路 |
| `FP!` | 16 | X/Y/高度闭环总览诊断 |
| `FQ!` | 16 | 红外灯识别和筛选统计 |
| `FR!` | 8 | 0 核主循环耗时 |
| `FS!` | 16 | 高度估计和高度串级 PID |
| `FU!` | 16 | 光流融合状态深度诊断 |
| `FV!` | 16 | beacon 下发槽位和坐标 |
| `FW!` | 16 | 当前帧连通域面积降序排列 |

已删除模式：`FJ!`, `FK!`, `FT!`。这些命令不再映射到 `send_back_flag`，发送后保持当前回传模式不变。

## 光流速度链

### 调参流程

1. 发送 `FU!`，确认 `I3 fused_velocity_valid`、`I1 flow_used`、`I7 confidence` 和 `I4 tof_source` 是否正常。
2. 若 `FU` 正常，再发送 `F8!` 调速度环 PID，观察目标速度、速度环直接反馈、积分项和姿态修正。
3. 若 `FU` 异常，先不要调 PID。优先看 `tof_source/range/dt/confidence`，必要时再用 `F0!` 或 `FE!` 排查光流传感器输入。

### `F8`: 速度环调参摘要

| 通道 | 含义 |
|---|---|
| `I0` | X 方向速度环直接反馈 `fb_vx_cms` |
| `I1` | X 方向目标速度 `vx_target_cms` |
| `I2` | X 速度环积分项输出 `x_i_out` |
| `I3` | Pitch 修正输出 `pitch_corr_deg` |
| `I4` | Y 方向速度环直接反馈 `fb_vy_cms` |
| `I5` | Y 方向目标速度 `vy_target_cms` |
| `I6` | Y 速度环积分项输出 `y_i_out` |
| `I7` | Roll 修正输出 `roll_corr_deg` |

`F8 I0/I4` 是速度 PID 实际使用的直接反馈；该值应与融合速度 `FU I14/I15` 基本同步。

### `F9`: 姿态解算总览

| 通道 | 含义 |
|---|---|
| `I0` | `current_roll` / `Roll_a`，Mahony Roll，单位 deg |
| `I1` | `current_pitch` / `Pitch_a`，Mahony Pitch，单位 deg |
| `I2` | `current_yaw` / `Yaw_a`，Mahony Yaw，单位 deg |
| `I3` | `Yaw_a_Pi_genrl * Rad2Ang`，相对初始 Yaw，单位 deg |
| `I4` | `q0`，Mahony 四元数 |
| `I5` | `q1`，Mahony 四元数 |
| `I6` | `q2`，Mahony 四元数 |
| `I7` | `q3`，Mahony 四元数 |
| `I8` | `Roll_g_F`，滤波后 Roll 角速度，单位 deg/s |
| `I9` | `Pitch_g_F`，滤波后 Pitch 角速度，单位 deg/s |
| `I10` | `Yaw_g_F`，滤波后 Yaw 角速度，单位 deg/s |
| `I11` | `acc_norm_filtered`，姿态解算使用的滤波加速度模值，单位 mg |
| `I12` | `Roll_acc_F`，滤波后 Roll/X 加速度，单位 mg |
| `I13` | `Pitch_acc_F`，滤波后 Pitch/Y 加速度，单位 mg |
| `I14` | `Yaw_acc_F`，滤波后 Z 加速度，单位 mg |
| `I15` | `Check_IMU_Data_Quality()`，`0` 正常，`-1` 异常 |

Mahony 加速度修正强度由加速度模值偏差和当前最大轴角速度共同决定：静止/低速时快速收敛，中等运动时降低修正，明显转动或加速度异常时仅保留弱修正，避免滞后的加速度方向拖慢 Roll/Pitch 姿态响应。

### `FU`: 光流融合状态深度诊断

| 通道 | 含义 |
|---|---|
| `I0` | `pmw_flow_state.valid`，光流速度样本有效 |
| `I1` | `pmw_flow_state.flow_used`，本帧光流观测进入融合链路 |
| `I2` | `pmw_flow_state.acc_used`，ICM 预测参与融合 |
| `I3` | `pmw_flow_state.fused_velocity_valid`，速度环反馈可用 |
| `I4` | `tof_source`，`0 NONE`, `1 LIVE`, `2 HOLD` |
| `I5` | `pmw_flow_state.range_cm`，光流换算使用高度 |
| `I6` | `pmw_flow_state.dt_s`，光流样本周期 |
| `I7` | `pmw_flow_state.flow_confidence`，光流观测可信度 |
| `I8` | `pmw_flow_state.flow_fusion_alpha`，动态融合系数 |
| `I9` | `pmw_flow_state.flow_residual_norm_cms`，光流速度与融合速度残差模长 |
| `I10` | `fb_vx_cms`，X 方向速度环直接输入速度 |
| `I11` | `fb_vy_cms`，Y 方向速度环直接输入速度 |
| `I12` | `pmw_flow_state.velocity_x_cms`，X 滤波后光流速度 |
| `I13` | `pmw_flow_state.velocity_y_cms`，Y 滤波后光流速度 |
| `I14` | `pmw_flow_state.velocity_x_fused_cms`，X 融合速度 |
| `I15` | `pmw_flow_state.velocity_y_fused_cms`，Y 融合速度 |

状态解释：

- `valid` 只说明光流后端速度样本当前可用。
- `flow_used` 说明本帧真实光流观测参与了融合修正。
- `acc_used` 说明 ICM 预测链路正在参与融合。
- `fused_velocity_valid` 说明速度环可以继续使用融合速度；短时光流/ToF 失效时可在预测保持窗口内保持为 1。
- `flow_confidence` 对有效光流观测设置下限 `0.30`；无真实观测时仍为 `0`。
- `flow_fusion_alpha` 当前范围为 `0.30 ~ 0.80`；`confidence=0.30` 对应 alpha 下限，`confidence=1.0` 对应 alpha 上限。

### `F0`: 光流后端原始/补偿量

| 通道 | 含义 |
|---|---|
| `I0` | `flow_x_rad` |
| `I1` | `gyro_flow_y_delta_rad` |
| `I2` | `flow_comp_x_rad` |
| `I3` | `velocity_x_raw_cms` |
| `I4` | `flow_y_rad` |
| `I5` | `gyro_flow_x_delta_rad` |
| `I6` | `flow_comp_y_rad` |
| `I7` | `velocity_y_raw_cms` |

### `FE`: 光流传感器原始输入

UP-FLOW-302 模式：

| 通道 | 含义 |
|---|---|
| `I0` | `FLOW_SENSOR_SELECTED` |
| `I1` | `upflow302_x` |
| `I2` | `upflow302_y` |
| `I3` | `upflow302_valid` |
| `I4` | `upflow302_state_flag` |
| `I5` | `upflow302_finsh_flag` |
| `I6` | `upflow302_us` |
| `I7` | `pmw_flow_state.dt_s` |

PMW3901 模式：

| 通道 | 含义 |
|---|---|
| `I0` | `FLOW_SENSOR_SELECTED` |
| `I1` | `flow_delta_x_count` |
| `I2` | `flow_delta_y_count` |
| `I3` | `pmw_flow_state.valid` |
| `I4` | `pmw_flow_state.dt_s` |
| `I5` | `flow_x_rad` |
| `I6` | `flow_y_rad` |
| `I7` | `tof_los_cm` |

## 高度链路

### `FD`: 高度环摘要

| 通道 | 含义 |
|---|---|
| `I0` | 实际高度 `actual_cm` |
| `I1` | 目标高度 `target_cm` |
| `I2` | 高度误差 |
| `I3` | 高度速度 PID 油门修正 |
| `I4` | 目标垂直速度 |
| `I5` | 融合垂直速度 |
| `I6` | 垂直速度误差 |
| `I7` | `base_throttle` |

### `FS`: 高度估计和串级 PID

| 通道 | 含义 |
|---|---|
| `I0` | 融合高度 cm |
| `I1` | 当前目标高度 cm |
| `I2` | ToF 中值高度 cm |
| `I3` | ToF 滤波高度 cm |
| `I4` | ToF 微分速度 cm/s |
| `I5` | 加速度积分速度 cm/s |
| `I6` | 融合垂直速度 cm/s |
| `I7` | 外环目标垂直速度 cm/s |
| `I8` | 垂直速度误差 cm/s |
| `I9` | 速度 PID 油门修正 |
| `I10` | 水平化 Z 加速度 mg |
| `I11` | 重力偏置 mg |
| `I12` | 去重力 Z 加速度 cm/s^2 |
| `I13` | ToF 有效标志 |
| `I14` | 加速度速度参与标志 |
| `I15` | 串级控制使能标志 |

## 红外位置链路

### `FG!`: 红外正方形中心点

| 通道 | 含义 |
|---|---|
| `I0` | `ir_pose_estimate.x`，前向位置 cm |
| `I1` | `ir_pose_estimate.y`，右向位置 cm |
| `I2` | `ir_pose_estimate.z`，视觉估计高度 cm |
| `I3` | `ir_pose_estimate.square_side_cm`，正方形边长估计 cm |
| `I4` | `ir_track_valid`，`1` 表示当前可追踪 |
| `I5` | `ir_position_lost_flag`，`1` 表示已进入丢失保护 |
| `I6` | `ir_position_lost_elapsed_ms`，丢失持续时间 ms |
| `I7` | `ir_pose_estimate.detected_point_count`，当前检测到的点数 |

### `FH!`: 红外丢失保护状态机

| 通道 | 含义 |
|---|---|
| `I0` | `ir_pose_estimate.x`，当前前向位置 cm |
| `I1` | `ir_pose_estimate.y`，当前右向位置 cm |
| `I2` | `ir_loss_debug.raw_detect_valid`，原始检测是否成功 |
| `I3` | `ir_loss_debug.lost_flag`，丢失保护标志 |
| `I4` | `ir_loss_debug.lost_elapsed_ms`，丢失累计时间 ms |
| `I5` | `ir_loss_debug.has_trackable_pose`，是否存在可继续跟踪的历史位姿 |
| `I6` | `ir_loss_debug.task_5ms_count`，5ms 丢失保护任务累计次数 |
| `I7` | `ir_loss_debug.timeout_count`，进入超时分支次数 |

### `FI!`: 红外位置闭环摘要

| 通道 | 含义 |
|---|---|
| `I0` | `pos_diag.fb_x_cm`，X 方向位置环输入 cm |
| `I1` | `pos_diag.fb_y_cm`，Y 方向位置环输入 cm |
| `I2` | `pos_diag.vx_cmd_cms`，X 方向目标速度 cm/s |
| `I3` | `pos_diag.vy_cmd_cms`，Y 方向目标速度 cm/s |
| `I4` | `pos_diag.enabled`，位置环使能标志 |
| `I5` | `pos_diag.active`，位置环当前是否真正接入控制 |
| `I6` | `pos_diag.ir_track_valid`，红外位置输入当前是否有效 |
| `I7` | `pos_diag.velocity_ready`，速度反馈链是否可用 |

### `FL!`: 红外位置环 PID 调参摘要

| 通道 | 含义 |
|---|---|
| `I0` | `pos_diag.error_x_cm`，X 方向位置误差 cm |
| `I1` | `x_p_out`，X 方向位置环 P 项输出 |
| `I2` | `pos_diag.x_i_out`，X 方向位置环 I 项输出 |
| `I3` | `pos_diag.vx_cmd_cms`，X 方向目标速度 cm/s |
| `I4` | `pos_diag.error_y_cm`，Y 方向位置误差 cm |
| `I5` | `y_p_out`，Y 方向位置环 P 项输出 |
| `I6` | `pos_diag.y_i_out`，Y 方向位置环 I 项输出 |
| `I7` | `pos_diag.vy_cmd_cms`，Y 方向目标速度 cm/s |

### `FM!`: X 轴位置-速度-Pitch-角速度链路

| 通道 | 含义 |
|---|---|
| `I0` | `pos_diag.fb_x_cm`，X 方向位置反馈 cm |
| `I1` | `pos_diag.vx_cmd_cms`，X 方向目标速度 cm/s |
| `I2` | `pmw_flow_state.velocity_x_fused_cms`，X 融合速度 cm/s |
| `I3` | `target_angle_pitch`，X 方向对应目标 Pitch 角 deg |
| `I4` | `current_pitch`，实际 Pitch 角 deg |
| `I5` | `target_pitch_rate`，目标 Pitch 角速度 deg/s |
| `I6` | `current_gyro_pitch`，实际 Pitch 角速度 deg/s |
| `I7` | `pos_diag.active`，该链路当前是否处于位置闭环激活态 |

### `FN!`: Y 轴位置-速度-Roll-角速度链路

| 通道 | 含义 |
|---|---|
| `I0` | `pos_diag.fb_y_cm`，Y 方向位置反馈 cm |
| `I1` | `pos_diag.vy_cmd_cms`，Y 方向目标速度 cm/s |
| `I2` | `pmw_flow_state.velocity_y_fused_cms`，Y 融合速度 cm/s |
| `I3` | `target_angle_roll`，Y 方向对应目标 Roll 角 deg |
| `I4` | `current_roll`，实际 Roll 角 deg |
| `I5` | `target_roll_rate`，目标 Roll 角速度 deg/s |
| `I6` | `current_gyro_roll`，实际 Roll 角速度 deg/s |
| `I7` | `pos_diag.active`，该链路当前是否处于位置闭环激活态 |

### `FP!`: X/Y/高度闭环总览

| 通道 | 含义 |
|---|---|
| `I0` | X 目标位置 `pos_diag.target_x_cm` |
| `I1` | Y 目标位置 `pos_diag.target_y_cm` |
| `I2` | X 目标速度 `pos_diag.vx_cmd_cms` |
| `I3` | Y 目标速度 `pos_diag.vy_cmd_cms` |
| `I4` | X 实际速度，速度环直接反馈 `fb_vx_cms` |
| `I5` | Y 实际速度，速度环直接反馈 `fb_vy_cms` |
| `I6` | X 对应目标角度 `target_angle_pitch` |
| `I7` | Y 对应目标角度 `target_angle_roll` |
| `I8` | X 对应实际角度 `current_pitch` |
| `I9` | Y 对应实际角度 `current_roll` |
| `I10` | X 对应目标角速度 `target_pitch_rate` |
| `I11` | Y 对应目标角速度 `target_roll_rate` |
| `I12` | X 对应实际角速度 `current_gyro_pitch` |
| `I13` | Y 对应实际角速度 `current_gyro_roll` |
| `I14` | 目标高度 `HeightManager_Get_Active_Target()` |
| `I15` | 实际高度 `g_height_result.height_cm` |

X 轴速度闭环输出对应 Pitch，Y 轴速度闭环输出对应 Roll。`I4/I5` 是速度 PID 实际使用的直接反馈；该值应与融合速度 `FU I14/I15` 基本同步。

### `FQ!`: 红外灯识别和筛选统计

| 通道 | 含义 |
|---|---|
| `I0` | `vision_diag.blob_count`，当前帧总连通域数量 |
| `I1` | `vision_diag.ir_split_count`，分到红外定位管线的连通域数量 |
| `I2` | `vision_diag.beacon_split_count`，分到 beacon 管线的连通域数量 |
| `I3` | `vision_diag.ir_blob_count`，最终送入红外定位候选池的数量 |
| `I4` | `vision_diag.ir_reject_area_low_count`，因面积过小被拒绝次数 |
| `I5` | `vision_diag.ir_reject_area_high_count`，因面积过大被拒绝次数 |
| `I6` | `vision_diag.ir_reject_size_count`，因尺寸条件不满足被拒绝次数 |
| `I7` | `vision_diag.ir_reject_aspect_count`，因长宽比不满足被拒绝次数 |
| `I8` | `vision_diag.ir_reject_insert_count`，因候选插入失败被拒绝次数 |
| `I9` | `vision_diag.ir_track_valid`，红外定位当前是否有效 |
| `I10` | `vision_diag.ir_detected_point_count`，当前用于定位的点数 |
| `I11` | `vision_diag.ir_x_cm`，红外中心前向位置 cm |
| `I12` | `vision_diag.ir_y_cm`，红外中心右向位置 cm |
| `I13` | `vision_diag.ir_square_side_cm`，红外正方形边长估计 cm |
| `I14` | `vision_diag.ir_height_cm`，红外估计高度 cm |
| `I15` | `vision_diag.beacon_blob_count`，最终送入 beacon 候选池的数量 |

### `FV!`: beacon 下发槽位

| 通道 | 含义 |
|---|---|
| `I0` | `vision_diag.beacon_count`，当前帧 beacon 识别结果总数 |
| `I1` | `beacon_slots[0].valid`，slot0 是否有效 |
| `I2` | `beacon_slots[0].x_cm`，slot0 前向坐标 cm |
| `I3` | `beacon_slots[0].y_cm`，slot0 右向坐标 cm |
| `I4` | `beacon_slots[1].valid`，slot1 是否有效 |
| `I5` | `beacon_slots[1].x_cm`，slot1 前向坐标 cm |
| `I6` | `beacon_slots[1].y_cm`，slot1 右向坐标 cm |
| `I7` | `beacon_slots[2].valid`，slot2 是否有效 |
| `I8` | `beacon_slots[2].x_cm`，slot2 前向坐标 cm |
| `I9` | `beacon_slots[2].y_cm`，slot2 右向坐标 cm |
| `I10` | `beacon_slots[3].valid`，slot3 是否有效 |
| `I11` | `beacon_slots[3].x_cm`，slot3 前向坐标 cm |
| `I12` | `beacon_slots[3].y_cm`，slot3 右向坐标 cm |
| `I13` | `beacon_slots[4].valid`，slot4 是否有效 |
| `I14` | `beacon_slots[4].x_cm`，slot4 前向坐标 cm |
| `I15` | `beacon_slots[4].y_cm`，slot4 右向坐标 cm |

### `FW!`: 当前帧连通域面积

| 通道 | 含义 |
|---|---|
| `I0` | 当前帧面积第 1 大的连通域，单位 px |
| `I1` | 当前帧面积第 2 大的连通域，单位 px |
| `I2` | 当前帧面积第 3 大的连通域，单位 px |
| `I3` | 当前帧面积第 4 大的连通域，单位 px |
| `I4` | 当前帧面积第 5 大的连通域，单位 px |
| `I5` | 当前帧面积第 6 大的连通域，单位 px |
| `I6` | 当前帧面积第 7 大的连通域，单位 px |
| `I7` | 当前帧面积第 8 大的连通域，单位 px |
| `I8` | 当前帧面积第 9 大的连通域，单位 px |
| `I9` | 当前帧面积第 10 大的连通域，单位 px |
| `I10` | 当前帧面积第 11 大的连通域，单位 px |
| `I11` | 当前帧面积第 12 大的连通域，单位 px |
| `I12` | 当前帧面积第 13 大的连通域，单位 px |
| `I13` | 当前帧面积第 14 大的连通域，单位 px |
| `I14` | 当前帧面积第 15 大的连通域，单位 px |
| `I15` | 当前帧面积第 16 大的连通域，单位 px |

## 其他常用回传

| 命令 | 通道摘要 |
|---|---|
| `F2!` | 红外位置、高度、姿态摘要 |
| `F3!` | VOFA 测试波形、姿态、高度、ToF 原始距离 |
| `F4!` | 陀螺仪原始值和校准值 |
| `F5!` | 加速度原始值、校准值、模长 |
| `F6!` | Roll/Pitch 目标角、实际角、目标角速度、实际角速度 |
| `F7!` | ADC 原始值、滤波值、归一化值和目标角 |
| `F9!` | Mahony 姿态角、四元数、滤波角速度、滤波加速度和 IMU 健康状态 |
| `FA!` | 当前 Roll/Pitch、水平化加速度、加速度零偏残差 |
| `FB!` | 光流速度旋转到世界系后的速度和积分位置 |
| `FC!` | 速度积分位置与单帧光流位移积分位置对比 |
| `FF!` | Roll/Pitch 角度环目标、反馈、I 项和目标角速度 |
| `FR!` | 最近耗时、最大耗时、平均耗时、频率、超时次数和系统时钟 |
| `FV!` | beacon 总数和 5 个下发槽位的 valid/x/y |
| `FW!` | 当前帧前 16 大连通域面积 |

## 常用发送示例

```text
F8!
F9!
FU!
FD!
FS!
FI!
FP!
F0!
FE!
FR!
FV!
FW!
```

速度环调参：

```text
FU!
V1=0.80!
V2=0!
V3=0!
V4=120.0!
V5=30.0!
W1=0.80!
W2=0!
W3=0!
W4=120.0!
W5=30.0!
F8!
```

高度环调参：

```text
FD!
H1=4.00!
FS!
```

位置环调参：

```text
FI!
I0=1!
J0=1!
I1=0.60!
J1=0.60!
FP!
```

光流去旋转增益：

```text
F0!
Gx=0.82!
Gy=0.94!
FU!
```

Height cascade PID tuning:

```text
H1=2.0!       // outer height-loop HeightPID.Kp
H2=0.0001!    // outer height-loop HeightPID.Ki
H3=0.0!       // outer height-loop HeightPID.Kd

h1=1.0!       // inner height-velocity-loop HeightVelPID.Kp
h2=0.0001!    // inner height-velocity-loop HeightVelPID.Ki
h3=0.0!       // inner height-velocity-loop HeightVelPID.Kd

F1!           // read back the last selected PID command group
FD!           // 8-channel height-loop summary
FS!           // 16-channel height estimator/cascade PID diagnostic
```

强制失锁：

```text
00=0!
```

## 方向关系速查

| 模块 | 当前约定 |
|---|---|
| 机体系/世界系 | X 向前，Y 向右，Z 向下 |
| 光流速度后端 | `flow_comp_x` 映射机体 X；`flow_comp_y` 映射机体 Y；速度输出 X 符号为 `+1`，Y 符号为 `-1` |
| UP-FLOW-302 | 原始 X/Y 先交换，当前 `X_SIGN = -1`，`Y_SIGN = +1` |
| PMW3901 | 原始 X/Y 不交换，X/Y 符号均为 `+1` |
| 红外定位 | `ir_pose_estimate.x` 为前向位置，`ir_pose_estimate.y` 为右向位置 |
| ToF | 作为下向高度使用，并在高度管理器中做安装补偿 |
