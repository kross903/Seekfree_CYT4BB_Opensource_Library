# 210 度广角相机离线标定工具

这个目录只放电脑端 OpenCV / Python 标定工具，不加入 IAR 固件工程。

## 目录

```text
frames/   从板子导出的 188x120 原始灰度图
scripts/  Python/OpenCV 标定与生成头文件脚本
output/   标定 JSON、报告、生成的 C 头文件
```

## 基本流程

可以直接启动桌面界面：

```powershell
cd C:\Users\saileach\Desktop\飞跃雷区资料\摄像头标定
py scripts\camera_tool_ui.py
```

如果当前终端还停在 `CYT4BB7_Drone` 工程根目录，也可以直接运行：

```powershell
py ..\摄像头标定\scripts\camera_tool_ui.py
```

1. 固定摄像头镜头、分辨率、曝光、增益和安装位置。
2. 把板端导出的原始图像放入 `frames/` 或其子目录，建议不少于 30 张。
3. 运行标定：

```powershell
py scripts\calibrate_fisheye.py --frames frames\uart_capture --pattern-cols 11 --pattern-rows 8 --square-size-cm 2.0
```

4. 生成固件头文件：

```powershell
py scripts\generate_firmware_headers.py --calibration output\camera_calibration.json --output output
```

5. 把 `output\*.h` 覆盖到固件工程：

```text
CYT4BB7_Drone\Seekfree_CYT4BB_Opensource_Library\project\code\vision\
```

## 注意

- 标定图必须是板端实际参与算法的 `188x120` 原图，不能用屏幕截图或缩放图替代。
- 当前棋盘为 12 列 x 9 行方格，因此应填写 11 列 x 8 行内角点。
- 标定器会递归查找图像，并使用 2 倍插值后的 `findChessboardCornersSB` 提高低分辨率角点识别率；求得的角点会换算回原始 188x120 坐标。
- `output/corner_detection_report.json` 会记录发现、读取、接受和拒绝的每张图片，排查时先看这个文件。
- 默认使用完整的 `188x120` 传感器矩形，`valid_radius_px=0` 表示不做圆形裁剪。
- 如果实测确认镜头边缘存在无成像区或误差不可接受，再通过 `--valid-radius-px` 传入正数限制有效半径。
- Python/OpenCV 只用于离线生成参数和表，固件端只使用 C 头文件。
- 去畸变预览输出为 `376x240`，保持标定焦距对应的原像素尺度，并将主点置于画布中心；有限画布会舍弃针孔投影中无限拉伸的四角极端区域。
- `camera_calibration.json` 中的 `undistort_width`、`undistort_height` 会同步生成到固件参数头文件，供上位机 C 仿真使用。
