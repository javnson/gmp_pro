# GMP Core

[English](readme.md) | **简体中文**

`core` 按职责分层，依赖方向固定为：

`std -> base -> mm / pm / dev / protocol -> rt`

| 层级 | 职责 | 文档 |
| --- | --- | --- |
| `std` | 架构、编译器、错误码、类型和配置契约 | [std](std/README.md) |
| `base` | 可移植基础服务与通用数据结构 | [base](base/README.md) |
| `mm` | 内存管理 | [mm](mm/readme.md) |
| `pm` | 任务、调度和状态管理 | [pm](pm/readme.md) |
| `dev` | 外设契约、驱动与数据链路服务 | [dev](dev/readme.md) |
| `protocol` | 与硬件传输层解耦的协议引擎和数据模型 | [CANopen](protocol/canopen/README_CN.md) |
| `rt` | 完整 GMP 启动和运行时装配 | [rt](rt/README.md) |

只需要配置、标准类型和宏时包含 `<gmp_type.h>`；只有需要完整 GMP 运行
框架时才包含 `<gmp_core.h>`。项目中的源代码管理器输出是生成物，不作为
这些模块的权威副本。

模块归属、依赖和 README 维护规则见 [Core 架构与维护指南](CORE_GUIDE.md)。
