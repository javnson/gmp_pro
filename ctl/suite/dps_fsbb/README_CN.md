# 四开关 Buck-Boost 变换器

[English](README.md) | **简体中文**

本工程实现四开关 Buck-Boost DC-DC 控制器，在 C2000 硬件工程和 PC 仿真工程之间共享 GMP 控制代码。它既是可复用的数字电源模板，也展示了基于 SDPE 的完整部署与调试流程。

## 目录

- `src/`：各平台共用的控制算法。
- `sdpe_general/`：工程公共 SDPE 配置。
- `project/f280039c_Iris_node/`：C2000 硬件工程。
- `project/simulate/`：Visual Studio 仿真工程。
- `doc/`：UDP/SIL 实验过程和结果。
- `tunable_variable_list.json`：调试工具可访问的变量清单。

## 建议流程

应从最低 `BUILD_LEVEL` 开始，依次验证采样、PWM 极性、保护和功率级开环行为，再启用闭环。公共控制参数放在 `sdpe_general/` 中，平台相关的外设映射放在各工程自己的 SDPE 配置中。

仿真工程会优先使用仓库管理的 Python 与 vcpkg 环境。编译前请先运行 GMP 安装入口，以生成源文件和头文件、分发 SDPE 工具并准备本地依赖。当前验证记录见 [UDP/SIL 仿真实验报告](doc/README.md)。
