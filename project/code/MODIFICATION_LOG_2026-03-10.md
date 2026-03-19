# 2026-03-10 修改日志

## 修改目标

本轮修改遵循两个前提:

1. 不打断当前飞控主流程，不改现有 ISR 调用顺序和外部入口名字。
2. 只针对已经讨论过的三个问题做增量优化:
   - 高度融合
   - 低压检测
   - VOFA 串口调参

---

## 1. 高度融合优化

### 修改文件

- `code/PIT_Demo.c`
- `code/Kalman Filter/height_fusion.c`
- `code/Kalman Filter/height_fusion.h`
- `code/vision/ir_localization.c`
- `code/vision/ir_localization.h`

### 具体修改

1. 将高度融合的执行方式明确改为:
   - 每个 40Hz 融合周期先 `HeightFusion_Predict()`
   - ToF 有效时执行 `HeightFusion_UpdateToF()`
   - 视觉高度有新样本时执行 `HeightFusion_UpdateIR()`

2. 在 `code/PIT_Demo.c` 中新增:
   - `consume_new_ir_height_measurement()`
   - `update_height_fusion_step()`

3. 在 `code/vision/ir_localization.c` 中新增视觉样本序号:
   - `ir_pose_update_seq`

4. 在 `code/Kalman Filter/height_fusion.c` 中调整了融合器语义:
   - `HeightFusion_Predict()` 只负责预测
   - `HeightFusion_UpdateToF()` 不再内部重复预测
   - `HeightFusion_UpdateIR()` 不再依赖 ToF 有效才能被调用

### 修改后的三种工况

1. ToF 正常:
   - `Predict + ToF Update + 可选视觉 Update`

2. ToF 失效但视觉正常:
   - `Predict + 视觉 Update`

3. ToF 和视觉都失效:
   - `Predict only`

### 恢复方法

如果你想恢复到旧逻辑:

1. 删除 `consume_new_ir_height_measurement()` 和 `update_height_fusion_step()`。
2. 在 `update_tof_height_state()` 中恢复成:
   - ToF 有效才更新融合
   - ToF 无效时只 `HeightFusion_Predict()`
3. 删除 `ir_pose_update_seq` 及其相关使用。
4. 把 `HeightFusion_UpdateToF()` 恢复成内部自带预测的旧结构。

---

## 2. 低压检测优化

### 修改文件

- `code/PIT_Demo.c`
- `code/PIT_Demo.h`
- `code/All_Init.c`

### 具体修改

1. 运行期低压检测改为:
   - ADC 电压低通滤波
   - 低压连续确认
   - 恢复连续确认
   - 滞回阈值

2. `low_voltage_check()` 改为非阻塞提示:
   - 不再进入死循环
   - 只刷新屏幕提示
   - 是否停机继续由人工判断

3. 启动阶段 `check_boot_voltage()` 改为:
   - 开机采样一次电压
   - 低压时短暂显示告警
   - 告警后继续启动

### 当前行为

1. 开机低压:
   - 屏幕提示
   - 系统继续初始化

2. 运行期低压:
   - `low_voltage_protect_flag` 会置位
   - 菜单循环内会刷新低压提示
   - 不会强制停机

### 恢复方法

如果你想恢复成旧的“启动拦截”方式:

1. 在 `code/All_Init.c` 中把 `check_boot_voltage()` 改回:
   - 低压时 `while(1)` 等按键

如果你想恢复成更简单的运行期检测:

1. 删除 `LOW_VOLTAGE_RECOVER_THRESHOLD`
2. 删除 `LOW_VOLTAGE_FILTER_ALPHA`
3. 删除连续计数逻辑
4. 改回单次阈值判断直接置位/清零

---

## 3. VOFA 优化

### 修改文件

- `code/vofa.c`
- `code/vofa.h`

### 具体修改

1. 将旧的“直接读 FIFO 并立即重用 `fifo_get_data`”改为:
   - 中断只收字节
   - `Send_test()` 负责组包
   - 收到 `!` 才算一条完整命令
   - 完整命令进入一个轻量队列
   - `USART_PID_Adjust()` 每次只消费一条命令

2. `Get_Data()` 改为:
   - 返回 `uint8` 成功/失败
   - 通过输出参数返回解析值
   - 解析失败时不再默认返回 `0.0f`

3. 新增 `send_back_command`
   - 解决 PID 回读模式依赖旧包首字符的问题

4. 移除了原来 `Send_test()` 中把接收到的原始命令再次回写串口的行为
   - 避免干扰 VOFA 正常数据流

### 当前行为

1. 只有完整包才会执行
2. 同一包只会执行一次
3. 坏包不会被当成数值 0
4. 队列满时保留较新的调参包，丢弃最旧包

### 恢复方法

如果你想恢复旧版 VOFA 行为:

1. 删除包队列相关变量:
   - `vofa_partial_packet`
   - `vofa_packet_queue`
   - `vofa_packet_queue_head`
   - `vofa_packet_queue_tail`
   - `vofa_packet_queue_count`
   - `vofa_packet_ready`

2. 把 `Send_test()` 改回:
   - 直接读取 FIFO 到 `fifo_get_data`
   - 不做 `!` 终止符组包

3. 把 `Get_Data()` 改回 `float Get_Data(void)` 风格

4. 把 `USART_PID_Adjust()` 改回直接依赖 `fifo_get_data[0]` 和 `Get_Data()` 返回值

5. 把 `send_date_back()` 的 PID 回读改回使用 `fifo_get_data[0]`

---

## 4. 兼容性修正

### 修改文件

- `code/PIT_Demo.h`
- `code/vofa.h`
- `code/vision/ir_localization.h`
- `code/Kalman Filter/height_fusion.h`
- `code/All_Init.c`
- `code/Kalman Filter/height_fusion.c`
- `code/vision/ir_localization.c`

### 具体修改

这几个文件在本轮整理前后做过编码转换。为了避免注释与代码行粘连导致的编译风险，本次顺手重新整理了这些文件的格式和声明布局。

### 恢复方法

如果你不需要本轮修改，最稳妥的恢复方式不是手工逐行删改，而是直接用版本管理回退这些文件。

---

## 建议的回退顺序

如果你后面只想回退某一部分，建议按下面顺序操作:

1. 先回退 `code/vofa.c` 和 `code/vofa.h`
2. 再回退 `code/All_Init.c`、`code/PIT_Demo.c`、`code/PIT_Demo.h`
3. 最后回退 `code/Kalman Filter/height_fusion.*` 和 `code/vision/ir_localization.*`

原因:

1. VOFA 独立性最高，最容易单独回退。
2. 低压检测改动只影响提示和初始化，不影响视觉模块。
3. 高度融合和视觉样本新鲜度是联动改动，最好一起回退。

---

## 5. 菜单系统优化

### 修改文件

- `code/menu/menu.h`
- `code/menu/menu.c`
- `code/menu/menu_list.c`
- `user/cpu2_main.c`

### 修改目的

1. 保留你现有的主流程入口:
   - `menu_open()`
   - `menu_open_init()`
   - `core2_main()`

2. 解决旧菜单把“进入飞行监视页”和“退出菜单系统”混用 `g_menu_exit_flag` 的问题。

3. 在不改飞控主逻辑的前提下，把已经存在的参数页和方案页正式挂回主菜单。

### 具体修改

1. 在 `code/menu/menu.h` 和 `code/menu/menu.c` 中新增了显式界面状态:
   - `MENU_SCREEN_MENU`
   - `MENU_SCREEN_MONITOR`

2. 新增菜单状态接口:
   - `menu_enter_monitor_view()`
   - `menu_enter_main_menu_view()`
   - `menu_is_monitor_view()`

3. `menu_list.c` 中的 `exit_menu_system()` 不再做:
   - `g_menu_exit_flag = 1`
   - `key_flag = Button_4`

   现在改为:
   - 先保留原来的飞行模式选择
   - 保留退出动画
   - 最后切到 `MENU_SCREEN_MONITOR`

4. 主菜单入口补齐为:
   - `Parameters`
   - `Schemes`
   - `ICM Init`
   - `Flight View`

5. `menu_open_init()` 现在会:
   - 复位退出标志
   - 强制回到主菜单界面状态
   - 启动时检查一次参数索引冲突，并弹出短提示

6. `menu_open()` 现在只在“当前确实处于菜单页”时运行菜单循环。
   如果当前已经在飞行监视页，它会直接返回，不重复刷新菜单。

7. `cpu2_main.c` 中原来的:
   - `while(g_menu_exit_flag)`

   改成了:
   - `while(menu_is_monitor_view())`

8. `cpu2_main.c` 新增 `update_monitor_view()`:
   - 里面保留了你原来的监视页显示内容
   - 只是把这部分代码从 `core2_main()` 主循环里收口出来
   - 没改原来的图像处理、VOFA、数据显示主逻辑

9. 监视页下长按 `Button_4` 返回菜单时，不再直接写 `g_menu_exit_flag`，
   而是调用 `menu_enter_main_menu_view()` 回到主菜单。

### 当前行为

1. 上电后:
   - 初始化菜单状态
   - 进入主菜单

2. 在主菜单中选择 `Flight View` 后:
   - 仍然会先走你原来的飞行模式选择和退出动画
   - 之后进入飞行监视页

3. 在飞行监视页中:
   - 原有按键调油门、标定、解锁逻辑保留
   - 长按 `Button_4` 会回到主菜单

4. 参数菜单和参数方案菜单现在能从主菜单直接进入

### 恢复方法

如果你想恢复到旧版菜单行为:

1. 删除 `menu.h` 里新增的:
   - `MenuScreenState`
   - `g_menu_screen_state`
   - `menu_enter_monitor_view()`
   - `menu_enter_main_menu_view()`
   - `menu_is_monitor_view()`

2. 把 `menu.c` 里菜单界面状态相关逻辑删除，恢复成仅依赖 `g_menu_exit_flag` 控制菜单循环。

3. 把 `menu_list.c` 中的 `exit_menu_system()` 改回:
   - `g_menu_exit_flag = 1`
   - `key_flag = Button_4`

4. 把 `menu_open()` 改回无条件进入菜单循环。

5. 把 `cpu2_main.c` 中的:
   - `while(menu_is_monitor_view())`

   改回:
   - `while(g_menu_exit_flag)`

6. 如果你不想保留新接入的页面，再把主菜单项恢复成旧版本，只保留:
   - `ICM Init`
   - `Exit Menu`

### 2026-03-10 补充修正: 控制模式文案与命名统一

补充修改文件:

- `code/menu/menu_list.c`
- `code/menu/MENU_USAGE_README.md`
- `code/flight_control.h`
- `code/flight_control.c`
- `code/PIT_Demo.c`

补充修改内容:

1. `Flight View` 进入前的确认页文案从:
   - `OPEN FLY MODEL?`
   - `0. CLOSE`
   - `1. OPEN`

   改成:
   - `SELECT CTRL MODE`
   - `0. MANUAL`
   - `1. ANGLE`

2. `CONTROL_MODE_GYRO` 的对外语义改成 `CONTROL_MODE_MANUAL`
   - 仍然保持数值为 `0`
   - 保留 `CONTROL_MODE_GYRO` 兼容别名，避免影响旧代码

3. `PIT_Demo.c` 中原先的 `apply_manual_mode_output()` 改名为:
   - `apply_manual_throttle_mode_output()`

补充恢复方法:

1. 如果你只想恢复旧文案，改回 `menu_list.c` 里的 3 行显示字符串即可。
2. 如果你只想恢复旧命名，删掉 `CONTROL_MODE_MANUAL` 并把代码引用改回 `CONTROL_MODE_GYRO`。
3. 这次补充修正没有改模式编号，也没有改主流程切换逻辑。

### 2026-03-10 补充修正: 统一急停接口

补充修改文件:

- `code/PIT_Demo.h`
- `code/PIT_Demo.c`
- `user/cpu2_main.c`
- `code/menu/key.h`
- `code/menu/key.c`
- `code/menu/MENU_USAGE_README.md`

补充修改内容:

1. 在 `PIT_Demo` 中新增统一急停接口:
   - `Flight_Runtime_Emergency_Stop(uint8 reason)`
   - `Flight_Runtime_Clear_Emergency_Stop_Latch(void)`

2. 当前接入统一急停的来源:
   - 姿态超限急停
   - 飞行监视页 `KEY_4` 人工急停

3. `Button_4` 在监视页中不再直接写 `armed = 0`，
   而是统一调用 `Flight_Runtime_Emergency_Stop(EMERGENCY_STOP_REASON_MANUAL_BUTTON)`。

4. 重新解锁前会调用 `Flight_Runtime_Clear_Emergency_Stop_Latch()`，
   用于清掉上一轮急停锁存状态和 LED3 告警。

5. 删除了过时的急停残留:
   - `nec_stop_flag`
   - `PIT_Demo.c` 中旧的红外急停注释块

补充恢复方法:

1. 如果你想恢复旧的按键停机写法，把 `cpu2_main.c` 中 `Button_4` 分支改回直接写 `armed = 0`。
2. 如果你不想保留统一急停接口，删除 `Flight_Runtime_Emergency_Stop` 和 `Flight_Runtime_Clear_Emergency_Stop_Latch`，再把姿态超限逻辑改回原来的内部静态函数。
3. `nec_stop_flag` 当前已经无实际使用，如果你以后要恢复旧红外急停方案，需要连同新的触发逻辑一起重新设计，不建议直接把老注释拷回去。

### 2026-03-10 补充修正: Flight View 按键与监视页重排

补充修改文件:

- `user/cpu2_main.c`
- `code/PIT_Demo.h`
- `code/PIT_Demo.c`
- `code/All_Init.c`
- `code/menu/MENU_USAGE_README.md`

补充修改内容:

1. 监视页输入不再依赖菜单层 `key_flag` 的累计效果:
   - `KEY_1` / `KEY_2` 改为直接消费底层短按事件
   - `KEY_4` 长按直接使用 `KEY_LONG_PRESS` 判定返回主菜单
   - 修正了旧监视页里“长按返回依赖 `key_tick > 50`”在高刷新/空轮次下可能永远回不去的问题

2. `cpu2_main.c` 里的手动油门调整改成统一调用:
   - `Flight_Control_Set_Throttle()`
   - 修正了旧代码里减油门分支比较条件写反的问题

3. IPS200S 监视页排版改成固定分区:
   - 上部为相机灰度图
   - 下部为模式、解锁、电压、油门、姿态、信标、ToF/融合结果
   - 监视页只清信息区，不再每次把整页告警文案叠到图像上

4. 低压提示改成统一参数来源:
   - 新增 `BATTERY_ADC_TO_VOLTAGE_SCALE`
   - 新增 `BATTERY_LOW_WARN_THRESHOLD`
   - 新增 `BATTERY_LOW_RECOVER_THRESHOLD`
   - 启动检测和运行期检测都改为走同一组电池参数

5. `low_voltage_check()` 保留函数壳子，但不再单独往屏幕叠加 “LOW BATTERY WARNING”
   - 目的是保留旧接口，避免打断主流程调用点
   - 真实显示已收口到新的监视页 HUD

6. 监视页手动油门步进从 `50` 调整为 `20`
   - 位置: `user/cpu2_main.c`
   - 新增常量:
     - `MONITOR_THROTTLE_STEP`
     - `MONITOR_ARM_START_THROTTLE`
   - 目的: 减小手动模式调试时的单次油门跳变量，提升可控性

补充恢复方法:

1. 如果你想恢复旧的监视页按键逻辑，把 `user/cpu2_main.c` 改回:
   - `checker(key_flag)`
   - `stopper()`
   - `key_tick > 50` 返回菜单

2. 如果你想恢复旧的低压提示叠加方式，把 `PIT_Demo.c` 中 `low_voltage_check()` 改回原来直接画:
   - `LOW BATTERY WARNING`
   - `CHECK VOLTAGE MANUALLY`

3. 如果你想恢复原来的电池阈值常量，删除 `PIT_Demo.h` 中新增的:
   - `BATTERY_ADC_TO_VOLTAGE_SCALE`
   - `BATTERY_LOW_WARN_THRESHOLD`
   - `BATTERY_LOW_RECOVER_THRESHOLD`
   然后把 `All_Init.c` 和 `PIT_Demo.c` 改回各自使用硬编码常量。

4. 如果你想恢复原来的监视页油门步进，把 `user/cpu2_main.c` 中:
   - `MONITOR_THROTTLE_STEP 20.0f`
   改回:
   - `MONITOR_THROTTLE_STEP 50.0f`

### 2026-03-10 补充修正: MANUAL / ANGLE 油门状态分离

修改时间: 2026-03-10

修改目的:

1. 手动模式和角度模式不再共用同一组 `target_throttle / current_throttle / base_throttle`。
2. 保持 `Flight_Control_*` 入口不变，避免对菜单、监视页、VOFA 和主控制 ISR 造成额外破坏。
3. 修正旧逻辑里 `Flight_Control_Arm()` 总是把模式强行改回 `MANUAL` 的问题。

本次修改:

1. 在 `code/PIT_Demo.c` 中新增两套独立运行时油门状态:
   - `manual_target/current/base_throttle`
   - `angle_target/current/base_throttle`

2. 保留原有公开变量:
   - `target_throttle`
   - `current_throttle`
   - `base_throttle`
   现在这 3 个量只作为“当前控制模式的镜像值”对外显示和兼容旧接口。

3. 新增运行时辅助接口:
   - `Flight_Runtime_Reset_All_Throttle_States()`
   - `Flight_Runtime_Set_Throttle_For_Mode()`
   - `Flight_Runtime_Get_Target_Throttle_For_Mode()`
   - `Flight_Runtime_Get_Base_Throttle_For_Mode()`
   - `Flight_Runtime_Sync_Public_Throttle_State()`

4. `update_runtime_throttle_state()` 现在只会更新当前模式自己的油门链路:
   - `MANUAL` 只更新手动模式油门状态
   - `ANGLE` 只更新角度模式油门状态

5. `Flight_Control_Arm()` 不再强制切回 `CONTROL_MODE_MANUAL`:
   - 现在会保留你在菜单中已经选好的模式
   - 同时清零两套模式的油门状态，保证重新解锁时从干净状态开始

6. `Flight_Control_Set_Throttle()` / `Get_Target_Throttle()` / `Get_Throttle()`:
   - 现在都按当前模式读写对应的独立油门状态

7. `user/cpu2_main.c` 监视页解锁分支:
   - 删除了解锁后直接写 `current_throttle = 0` 和 `base_throttle = 0` 的旧做法
   - 改成只通过 `Flight_Control_Arm()` 和 `Flight_Control_Set_Throttle()` 操作当前模式

恢复方法:

1. 如果你想恢复“手动和角度模式共用一套油门状态”的旧逻辑:
   - 删除 `code/PIT_Demo.c` 中新增的两套模式油门变量和 5 个 `Flight_Runtime_*` 辅助接口
   - 把 `update_runtime_throttle_state()` 改回直接对公开的
     `target_throttle / current_throttle / base_throttle`
     做 `throttle_ramp()`

2. 如果你想恢复 `Flight_Control_Arm()` 一解锁就强制切回手动模式:
   - 把 `code/flight_control.c` 中 `Flight_Control_Arm()` 改回:
     `control_mode = CONTROL_MODE_MANUAL;`

3. 如果你想恢复监视页解锁时直接清共享油门缓存:
   - 把 `user/cpu2_main.c` 中 `KEY_3` 分支改回直接写
     `current_throttle = 0.0f;`
     `base_throttle = 0.0f;`

### 2026-03-10 补充修正: 未解锁状态下的油门调节被主 ISR 清零

修改时间: 2026-03-10

修改目的:

1. 修正监视页中 `KEY_1 / KEY_2` 在未解锁状态下刚写入目标油门、下一拍就被 4kHz 主 ISR 清零的问题。
2. 保持电机在 `armed == 0` 时仍然严格输出 0，不放宽安全边界。

本次修改:

1. 在 `code/PIT_Demo.c` 中新增 `clear_runtime_output_state_keep_targets()`
   - 未解锁时只清 `current/base throttle`
   - 不再把用户刚设置的 `target throttle` 一起清零

2. `update_runtime_throttle_state()` 的未解锁分支现在:
   - 保留当前模式的目标油门
   - 清空当前输出链路和电机输出

3. `user/cpu2_main.c` 中 `KEY_3` 解锁分支现在:
   - 先读取监视页当前已经设好的目标油门
   - 解锁后优先沿用这个目标值
   - 但仍保证不低于 `MONITOR_ARM_START_THROTTLE`

恢复方法:

1. 如果你想恢复“未解锁时目标油门也每拍清零”的旧逻辑:
   - 把 `code/PIT_Demo.c` 中 `update_runtime_throttle_state()` 的未解锁分支改回
     `apply_disarmed_output_state();`

2. 如果你想恢复 `KEY_3` 一解锁就固定写死 `100`:
   - 把 `user/cpu2_main.c` 中 `KEY_3` 分支改回直接调用
     `Flight_Control_Set_Throttle(MONITOR_ARM_START_THROTTLE);`

### 2026-03-10 补充修正: 取消手动/姿态目标油门分家

修改时间: 2026-03-10

修改目的:

1. 按当前调试需求，`MANUAL` 和 `ANGLE` 只需要共用一套目标油门。
2. 两个模式的区别只保留在“是否参与姿态 PID 控制”，不再拆成两套独立目标油门。
3. 进入模式后初始油门为 `0`，之后完全由 `KEY_1 / KEY_2` 手动增减。

本次修改:

1. `code/PIT_Demo.c`
   - 去掉了手动模式/姿态模式各自独立的目标油门状态
   - 恢复成一套共享的:
     `target_throttle / current_throttle / base_throttle`

2. `update_runtime_throttle_state()`
   - 恢复为直接对共享油门链路做 `throttle_ramp()`
   - 不再按模式分别维护不同的基础油门

3. `user/cpu2_main.c`
   - `KEY_3` 解锁时不再强制把目标油门抬到 `100`
   - 现在会直接沿用你当前已经调好的目标油门
   - 如果当前目标值是 `0`，解锁后电机仍保持不转，直到你按 `KEY_1`

4. 删除了已经过时的说明文件:
   - `code/menu/MANUAL_ANGLE_THROTTLE_UPDATE_2026-03-10.md`

恢复方法:

1. 如果你想恢复“手动和姿态模式各自一套目标油门”的旧改法:
   - 重新加回 `code/PIT_Demo.c` 里两套独立的模式油门变量
   - 把 `update_runtime_throttle_state()` 改回按模式分别更新

2. 如果你想恢复 `KEY_3` 解锁时最低起步 `100`:
   - 把 `user/cpu2_main.c` 中 `KEY_3` 分支改回:
     `Flight_Control_Set_Throttle(MONITOR_ARM_START_THROTTLE);`

### 2026-03-10 补充修正: ICM Init 等待 3 秒界面重排
修改时间: 2026-03-10

修改目的:

1. 解决进入 `ICM Init` 后，陀螺仪零漂校准提示和等待 3 秒提示在 IPS200S 上堆叠的问题。
2. 让 `ICM_Calculate_Init()` 和 `IMU_calibration()` 都使用固定行布局，避免旧文字残留在屏幕上。
3. 在不改动零漂采样、Flash 写入和后续原点校准流程的前提下，只优化 UI 可读性。

本次修改:

1. `code/icm/icm.c`
   - 新增 `icm_init_draw_stage_page()`，统一绘制 `ICM Init` 各阶段页面。
   - 新增 `icm_init_show_progress()`，在固定位置显示倒计时和采样进度。
   - `ICM_Calculate_Init()` 入口页改为固定布局的阶段页，不再沿用旧的单行叠写方式。
   - `IMU_calibration()` 的 `Don't move me` 提示改为固定布局的倒计时页和采样进度页。
   - 零漂完成后改为显示 `Stage: Saved` 页面，再继续后续流程。

2. `code/icm/icm42688.c`
   - 去掉 `Init_ICM42688()` 里原本写在左上角的 `ICM42688 Ready 1/2/Ready!` 成功提示。
   - 保留初始化失败时的错误提示，避免主 `ICM Init` 页面被底层驱动的调试文字覆盖。

恢复方法:

1. 如果你想恢复成旧的简易提示方式:
   - 删除 `code/icm/icm.c` 中 `icm_init_draw_stage_page()` 和 `icm_init_show_progress()`
   - 把 `ICM_Calculate_Init()` 中的阶段页调用改回原来的 `ips200_show_string()` 单行提示
   - 把 `IMU_calibration()` 中的倒计时和完成提示改回原来的:
     `IMU calibration`
     `Don't move me!(2)`
     `Don't move me!(1)`
     `IMU Calibration OK`

### 2026-03-10 补充修正: 姿态模式油门链路去掉隐藏定高干预
修改时间: 2026-03-10

修改目的:

1. 解决 `ANGLE` 模式下 `TTHR` 改了但电机仍像固定小油门启动的问题。
2. 让当前工程里的 `ANGLE` 模式只比 `MANUAL` 多姿态 PID，不再偷偷叠加固定 `100cm` 的高度环控制。
3. 让姿态模式的低油门手感和手动模式保持一致，避免小油门区被强行抬到固定最小值。

本次修改:

1. `code/PIT_Demo.c`
   - `calculate_angle_mode_output()` 中不再调用固定目标 `100.0f` 的高度环 PID。
   - `HeightSpeedPID_output` 现在在 `ANGLE` 模式下直接保持为 `0.0f`。
   - `clamp_motor_mix_output()` 不再把 `0~100` 之间的正油门强行抬到 `SAFE_MIN_THROTTLE`。
   - 小于 `48` 的正油门仍然会在后面的 `motor_output_safe_limit()` 里被映射到 DShot 最小有效值，主流程不变。

恢复方法:

1. 如果你想恢复旧的姿态模式逻辑:
   - 把 `code/PIT_Demo.c` 中 `calculate_angle_mode_output()` 里的
     `HeightSpeedPID_output = 0.0f;`
     改回原来的 `PID_Calculate(&HeightPID, 100.0f, g_height_result.height_cm, ISR_DT);`
   - 把 `clamp_motor_mix_output()` 恢复成原来会把正油门下限抬到 `SAFE_MIN_THROTTLE` 的写法
