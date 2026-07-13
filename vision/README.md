# 视觉代码仿真上位机

这是一个电脑端 Tkinter 工具，用真实的固件视觉 C 源码处理 `188x120` 灰度帧。

## 启动

```powershell
python vision_tool.py
```

首次启动前需要安装 GCC/Clang。Windows 推荐 MinGW-w64 GCC 或 LLVM Clang，并确保 `gcc`/`clang` 在 `PATH` 中；也可以设置 `VISION_CC` 指向编译器可执行文件。

## 功能

- 选择固件 `project/code/vision` 源码目录。
- 加载标定工具导出的 PNG、RAW 或 PGM 帧。
- 使用 PC 硬件适配层编译并运行真实 C 视觉代码。
- 调整 Roll、Pitch、高度，查看二值图、连通域和信标框选结果。
- 在左侧编辑并保存 C/H 文件，下一次构建或运行会使用最新源码。
- 构建日志、运行结果和中间二值图保存在 `runs/`。

## 自检

```powershell
python vision_tool.py --self-test
```

仿真适配层位于 `sim/`，不改写固件工程中的硬件头文件。当前工具不会自动覆盖固件文件，源码保存操作只写入你在 UI 中选择的源码目录。
