# GMP Controller Template Library（CTL）

[English](readme.md) | **简体中文**

CTL 提供跨平台数字控制算法、数学类型、应用框架和完整控制 suite，可用于 PC 仿真、PIL/HIL 和实时硬件控制器。

| 目录 | 作用 | 文档 |
| --- | --- | --- |
| `math_block` | CTL 唯一数值入口；不引入 CSP、设备层和 GMP runtime | [中文](math_block/readme_cn.md) / [English](math_block/readme.md) |
| `component` | 可复用控制器、接口、保护和预设 | [中文](component/readme_cn.md) / [English](component/readme.md) |
| `framework` | 控制器生命周期、状态机和调度框架 | [中文](framework/doc/readme_cn.md) / [English](framework/doc/readme.md) |
| `suite` | 可运行的跨平台控制工程 | [中文](suite/readme_cn.md) / [English](suite/readme.md) |
| `portable` | 不使用 CSP/GMP 运行框架时的 TI DSP、STM32 最小接入契约 | [中文](portable/README_CN.md) / [English](portable/README.md) |
| `unit_test` | Visual Studio/CTest 回归测试入口 | [测试说明](unit_test/README.md) |

实时计算优先使用 `ctrl_gt` 和 CTL 数学辅助函数；原始物理参数使用 `parameter_gt`。公共算法必须与芯片寄存器解耦，具体 ADC、PWM 和通信实现放入目标工程 `xplt`。

组件的 `sys_compile`、`sys_sim`、`sys_hw` 是不同验证范围；编译通过不能替代仿真或实物证据。模块选择和同步以 `tools/facilities_generator/src_mgr/gmp_framework_dic.json` 为准。
