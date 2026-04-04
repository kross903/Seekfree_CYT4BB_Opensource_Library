# AI Notes

本文件用于记录 AI 在本项目中需要优先遵守的约束和提醒。

## Current Rules

1. 完成任务后不需要做 IAR 实际编译验证。
2. 当前机器不具备 `iarbuild`、`iccarm`、`clang` 等可直接完成工程编译验证的工具。
3. 当 `project/code` 下新增或更新项目层公开头文件声明时，需要同步维护 [`project_headfile.h`](../common/project_headfile.h)。
4. `project/code` 下的 `.c` 文件默认优先通过 `common/project_headfile.h` 获取项目层公共声明；如有模块私有声明，再补模块自己的头文件。
5. `project/code` 下的 `.h` 文件不要无脑依赖 `project_headfile.h`，应尽量保持最小必要包含。

## Working Notes

- `information.md` 用于帮助 AI 快速理解项目结构、入口和常改位置。
- 若本次任务涉及目录调整、模块迁移或公共接口变动，应同步检查 `information.md` 是否需要更新。
- 若只做静态检查，请在结果中明确说明“未做 IAR 实际编译验证”。

## Future Notes

- 在此处继续追加新的 AI 约束。
- 尽量把长期有效的规则放在 `Current Rules`，把阶段性说明放在本节后续位置。
