# RotorPy PID 仿真

这个目录用于把当前无人机固件的级联 PID 链路接入 `D:\Aa-app\rotorpy-main`。

仿真链路：

```text
位置环 -> 速度环 -> 角度环 -> 角速度环 -> 混控 -> RotorPy Multirotor
```

位置环默认开启。仿真中使用 RotorPy 真值相对位置替代 `ir_pose_estimate.x/y`，目标位置为 `(0cm, 0cm)`，用于模拟无人机追踪红外正方形中心。

## 常规使用

最常用的启动方式是运行脚本，不需要手动输入完整命令：

```powershell
.\pid_simulation\rotorpy_pid\run_common.ps1
```

如果 PowerShell 策略不允许运行脚本，可以用 bat：

```powershell
.\pid_simulation\rotorpy_pid\run_common.bat
```

常规模式会自动使用 `D:\Aa-app\rotorpy-main`，并执行：

- 运行位置环、速度环、角度环、角速度环、混控和 RotorPy 动力学
- 保存 `pid_simulation/rotorpy_pid/outputs/rotorpy_pid_log.csv`
- 保存 `pid_simulation/rotorpy_pid/outputs/rotorpy_pid_plot.png`
- 保存 RotorPy examples 风格图像到 `pid_simulation/rotorpy_pid/outputs/`
- 显示 RotorPy 动画，并打开风向量箭头

动画中的红色箭头对应 RotorPy `animate_wind=True` 的风向量箭头，和 `examples.basic_usage` 中的箭头含义一致。

## 运行前先进入项目根目录

如果当前终端显示的是：

```powershell
PS C:\Users\25914>
```

不能直接运行 `python pid_simulation/...`，因为这个相对路径只在无人机项目根目录下存在。先执行：

```powershell
cd "D:\DAS\drone\work\cyt4bb7 case01\Seekfree_CYT4BB_Opensource_Library"
```

确认终端变成类似：

```powershell
PS D:\DAS\drone\work\cyt4bb7 case01\Seekfree_CYT4BB_Opensource_Library>
```

再运行仿真。

## 基础仿真

```powershell
python .\pid_simulation\rotorpy_pid\run_rotorpy_pid_sim.py --duration 8 --rotorpy-root "D:\Aa-app\rotorpy-main"
```

如果你不想切换目录，也可以在任意目录运行绝对路径版本：

```powershell
python "D:\DAS\drone\work\cyt4bb7 case01\Seekfree_CYT4BB_Opensource_Library\pid_simulation\rotorpy_pid\run_rotorpy_pid_sim.py" --duration 8 --rotorpy-root "D:\Aa-app\rotorpy-main"
```

运行后生成：

- `pid_simulation/rotorpy_pid/outputs/rotorpy_pid_log.csv`
- `pid_simulation/rotorpy_pid/outputs/rotorpy_pid_plot.png`

## RotorPy examples 风格图像

显示 RotorPy 自带 `Plotter.plot_results()` 的多窗口图，包括 3D path、position/velocity、attitude/rate、commands、wind、IMU 等：

```powershell
python .\pid_simulation\rotorpy_pid\run_rotorpy_pid_sim.py --duration 8 --rotorpy-plot --rotorpy-root "D:\Aa-app\rotorpy-main"
```

保存 RotorPy Plotter 图像到本项目 `outputs/`：

```powershell
python .\pid_simulation\rotorpy_pid\run_rotorpy_pid_sim.py --duration 8 --rotorpy-plot --save-rotorpy-prefix firmware_pid --rotorpy-root "D:\Aa-app\rotorpy-main"
```

## RotorPy 动画

显示 RotorPy 自带飞行动画，并显示和 `examples.basic_usage` 一样的风向量箭头：

```powershell
python .\pid_simulation\rotorpy_pid\run_rotorpy_pid_sim.py --duration 8 --rotorpy-animate --animate-wind --rotorpy-root "D:\Aa-app\rotorpy-main"
```

如果要保存动画，添加 `--save-rotorpy-prefix`：

```powershell
python .\pid_simulation\rotorpy_pid\run_rotorpy_pid_sim.py --duration 8 --rotorpy-animate --animate-wind --save-rotorpy-prefix firmware_pid --rotorpy-root "D:\Aa-app\rotorpy-main"
```

保存的 RotorPy 图像/动画会复制到：

```text
pid_simulation/rotorpy_pid/outputs/
```

## 高度阶跃测试

默认初始高度等于目标高度，便于先观察位置环、速度环和姿态环。如果要测试从地面起飞到 110cm：

```powershell
python .\pid_simulation\rotorpy_pid\run_rotorpy_pid_sim.py --z0-cm 0 --height-cm 110 --duration 8 --rotorpy-root "D:\Aa-app\rotorpy-main"
```

## 输入式调参

启动输入式调参：

```powershell
.\pid_simulation\rotorpy_pid\tune_common.ps1
```

如果 PowerShell 策略不允许运行脚本，可以用 bat：

```powershell
.\pid_simulation\rotorpy_pid\tune_common.bat
```

或使用完整命令：

```powershell
python .\pid_simulation\rotorpy_pid\interactive_rotorpy_pid.py --duration 5 --rotorpy-root "D:\Aa-app\rotorpy-main"
```

进入后可输入：

```text
set pos_x.kp 1.5
set vel_x.kp 0.8
set angle_roll.kp 7.0
set gyro_pitch.kp 2.0
setcfg initial_x_cm 80
run
show
rotorpy_plot
rotorpy_animate
common
save
quit
```

命令说明：

- `set <pid>.<field> <value>`：设置 PID 参数，例如 `set pos_x.kp 1.5`
- `setcfg <name> <value>`：设置仿真配置，例如 `setcfg initial_x_cm 80`
- `run`：用当前参数重新运行仿真
- `show`：显示自定义 PID 总览图
- `rotorpy_plot`：显示 RotorPy examples 风格图像
- `rotorpy_animate`：显示 RotorPy 飞行动画，默认带风向量箭头
- `common` / `run_common`：一键运行、保存、显示 RotorPy 图像和带风箭头动画
- `save`：保存 CSV 和自定义 PID 总览图
- `list`：列出当前 PID 参数
- `help`：显示帮助
- `quit`：退出

可用 PID 名称：

```text
pos_x pos_y vel_x vel_y angle_roll angle_pitch angle_yaw gyro_roll gyro_pitch gyro_yaw height
```

可用 PID 字段：

```text
kp ki kd output_limit integral_limit integral_threshold alpha
```

## 当前近似

- 使用 RotorPy `hummingbird_params` 作为默认机体参数。
- 油门到电机转速使用线性近似，并让 `base_throttle=770` 对应 RotorPy 悬停转速。
- 默认最大有效转速按悬停转速的 1.8 倍限制，避免 RotorPy 默认电机上限导致高度响应过激。
- 默认使用 `SinusoidWind` 生成动画风向量箭头，但 `aero=False`，所以风主要用于可视化。
- 默认不加入位置观测噪声、延迟和丢帧；可以在 `config.py` 中调整。
- 默认关闭 RotorPy 气动项，便于先观察 PID 链路本身。

后续要提高真实性，应优先替换：质量、轴距、惯量、电机推力系数、油门-推力曲线、传感器延迟/噪声模型。

更完整的真实化路线见：

```text
pid_simulation/rotorpy_pid/improve-guide.md
```
