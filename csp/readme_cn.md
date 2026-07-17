# GMP Chip Support Package（CSP）

[English](readme.md) | **简体中文**

CSP 负责把 `core` 和 `ctl` 的平台无关逻辑连接到具体芯片、操作系统或仿真环境。每个平台至少应提供：

| 文件 | 作用 |
| --- | --- |
| `csp.config.h` | 启用、关闭并验证平台功能 |
| `csp.general.h` | C 语言平台公共入口 |
| `csp.general.hpp` | C++ 平台公共入口 |
| `csp.typedef.h` | 平台相关基础类型定义 |

CSP 还必须实现 `core` 声明的 `gmp_hal_*` 平台接口。芯片外设、编译器差异和中断适配应留在 CSP 或工程 `xplt` 中，不能进入公共控制算法。

支持平台和新增 CSP 的完整流程见 [CSP 使用与扩展指南](CSP_GUIDE.md)。
