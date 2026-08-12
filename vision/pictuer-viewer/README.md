# MT9V03 图像特征查看器

独立离线工具，用于查看 `vision/capture` 中保存的图像并分析灰度和二值连通域。该工具不连接串口、不运行固件仿真，也不修改原始图片。

## 启动

在仓库根目录执行：

```powershell
python vision/pictuer-viewer/pictuer_viewer.py
```

依赖与现有视觉上位机相同，只需要 Pillow 和 Python 自带的 Tkinter：

```powershell
python -m pip install -r vision/requirements.txt
```

## 功能

- 递归浏览 `vision/capture`，PNG/RAW 同名时优先显示 PNG。
- 原始 `188x120` 灰度图和去畸变 `376x240` 灰度图切换。
- 去畸变直接读取固件的 `camera_calibration_params.h`，使用同一 fisheye 模型和双线性采样。
- 鼠标滚轮局部放大，右键拖动画面，悬停查看坐标、灰度和二值结果。
- 单一固定阈值实时二值化，规则为 `灰度值 >= 阈值`。
- 连续前景段连通域分析，默认使用与固件一致的 8 邻域。
- 输出面积、质心、外接框、边界像素、圆度和占空比。
- 双击候选或点击“打开候选”，独立查看候选灰度图和二值图。
- 自动读取同名 JSON 中的 Roll、Pitch、Yaw 和 ToF 高度。

工具会在退出时将目录、阈值、最小面积和邻域设置保存到本目录的 `viewer_settings.json`。

## 自检

```powershell
python vision/pictuer-viewer/pictuer_viewer.py --self-test
```

