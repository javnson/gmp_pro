# GMP Controller Template Library（CTL）

[English](readme.md) | **简体中文**

CTL 提供跨平台数字控制算法、数学类型、应用框架和完整控制 suite，可用于 PC 仿真、PIL/HIL 和实时硬件控制器。

| 目录 | 作用 | 文档 |
| --- | --- | --- |
| `math_block` | 控制类型、标幺运算、矩阵和坐标变换 | [中文](math_block/readme.md) |
| `component` | 可复用控制器、接口、保护和预设 | [中文](component/readme_cn.md) / [English](component/readme.md) |
| `framework` | 控制器生命周期、状态机和调度框架 | [中文](framework/doc/readme_cn.md) / [English](framework/doc/readme.md) |
| `suite` | 可运行的跨平台控制工程 | [中文](suite/readme_cn.md) / [English](suite/readme.md) |

实时计算优先使用 `ctrl_gt` 和 CTL 数学辅助函数；原始物理参数使用 `parameter_gt`。公共算法必须与芯片寄存器解耦，具体 ADC、PWM 和通信实现放入目标工程 `xplt`。
