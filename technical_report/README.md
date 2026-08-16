# 博世·飞驰人生队技术报告 LaTeX 框架

本目录面向第二十一届全国大学生智能汽车竞赛飞跃雷区组，队伍信息已经写入：

- 学校：江南大学
- 队伍：博世·飞驰人生队
- 指导教师：吴定会、王建华
- 队员：田进、何昱岷、玄嘉澍、谢欣冉、叶泽华

## 构建

在仓库根目录执行：

```powershell
.\technical_report\build.ps1
```

最终 PDF 输出到：

```text
output/pdf/jiangnan_bosch_flying_life_technical_report_framework.pdf
```

工程使用 XeLaTeX 与 Biber。主要入口为 `main.tex`，队伍信息集中在
`config/team_info.tex`，章节位于 `chapters/`，附录位于 `appendices/`。

## 使用约定

1. 黄色“待补充/核验”框是定稿任务，不是正文；提交前必须全部处理并删除。
2. 所有图片统一放入 `figures/`，推荐使用 PDF/SVG 转 PDF 的矢量图或高分辨率 PNG。
3. 图表必须注明单位、样本数和测试条件；正文中的定量结论必须能追溯到原始数据。
4. 代码、标定参数和实验数据都应标注 Git 提交哈希。
5. 参考文献通过 `references.bib` 管理，正文使用 `\supercite{key}` 以上标形式引用。
6. 完整原理图、核心源码和测试记录分别放在附录 A、B、C。

## 当前已按仓库填入的内容

- CYT4BB7 双核分工与实时调度路径；
- ICM42688、双 ToF、UP-FLOW-302/PMW3901、双摄像头；
- 位置—速度—姿态—角速度级联控制；
- P/Sqrt 位置纠偏、A/J 轨迹整形和动态制动；
- DShot600 与电调串口遥测；
- 43 字节地面车速度协议及 200 ms 超时；
- 摄像头当前鱼眼标定参数；
- VOFA、Python 视觉上位机与 RotorPy 验证工具链。
