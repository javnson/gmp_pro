# General Motor Platform（GMP）

**简体中文** | [English](README.md)

GMP 是一套面向实时控制、电机驱动和电力电子系统的跨平台开发框架。仓库同时提供控制算法组件、硬件抽象、可运行控制套件、参数工程工具，以及 MATLAB/Simulink SIL 联合仿真支持。

![GMP Logo](manual/img/GMP_LOGO.png)

GMP 的目标不是仅提供若干独立算法，而是让同一套控制逻辑能够在 PC 仿真、TI C2000、STM32 等平台之间复用，并把以下内容清晰分离：

- 平台无关的控制算法与状态机；
- ADC、PWM、编码器、UART 等平台外设；
- 电路、传感器、控制器及保护参数；
- 工程实际需要的 GMP 源文件；
- Simulink 被控对象与控制器程序。

> 当前仓库主要面向 Windows 开发环境。硬件工程还需要相应厂商工具链，例如 Code Composer Studio、SysConfig、STM32CubeMX/CubeIDE 或 Keil。仿真工程需要 MATLAB/Simulink 以及 Simscape Electrical Specialized Power Systems。

## 1. 快速开始

### 1.1 获取仓库

仓库包含子模块时，请使用递归方式克隆：

```bat
git clone --recursive <repository-url> gmp_pro
cd gmp_pro
```

如果已经完成普通克隆，可以补充执行：

```bat
git submodule update --init --recursive
```

仓库路径中应避免中文字符和过长路径。

### 1.2 安装开发环境

GMP 提供两种安装方式。

推荐使用仓库私有环境，它会把 Python、Git、CMake、Ninja、Doxygen、Graphviz 和 vcpkg 等工具安装到本仓库的 `bin` 目录：

```bat
install_gmp_virtual_env.bat
```

如果希望沿用系统级工具和 Scoop 环境，可以运行经典安装：

```bat
install_gmp.bat
```

两种方式都会注册用户环境变量：

```text
GMP_PRO_LOCATION=<当前 gmp_pro 根目录>
```

安装完成后，建议通过 GMP 开发命令行运行工具：

```bat
gmp_env.bat
```

也可以直接执行单条命令：

```bat
gmp_env.bat python --version
gmp_env.bat cmake --version
```

完整的环境安装、离线复制部署、代理配置和修复方法见 [GMP 环境安装说明](tools/gmp_installer/README_CN.md)。

### 1.3 安装 Simulink 库

在 MATLAB 中运行：

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'install_gmp_simulink_lib.m'));
```

安装程序会根据 MATLAB Release 部署并注册相应版本的 GMP Simulink 库。当前模型依赖 Specialized Power Systems；MATLAB R2026a 及之后的版本已不再提供该库，因此应使用 MATLAB R2025b 或更早版本，并确保已安装对应产品。

### 1.4 从已验证套件开始

新项目优先从 `ctl/suite` 中与目标拓扑最接近的工程开始，而不是从空白工程重新拼装：

| 目标应用 | 推荐套件 | 说明 |
| --- | --- | --- |
| 四开关全桥 DC-DC | [`dps_fsbb`](ctl/suite/dps_fsbb/README_CN.md) | 包含 C2000 工程、SDPE 配置和 UDP/SIL 仿真实验报告。 |
| 双向隔离 CLLLC / DAB | [`dps_clllc`](ctl/suite/dps_clllc/README_CN.md) | 面向 F280025C，支持变频、移相与高级 PWM 接口。 |
| PMSM 矢量控制 | [`mcs_pmsm_nt`](ctl/suite/mcs_pmsm_nt/README_CN.md) | 当前 PMSM 新模板，支持 C2000、STM32 和仿真平台。 |
| PMSM 参数识别 | [`mcs_pmsm_id`](ctl/suite/mcs_pmsm_id/README_CN.md) | 用于电机参数识别。 |
| 三相跟网型变流器 | [`pgs_inv_GFL_inverter`](ctl/suite/pgs_inv_GFL_inverter/README_CN.md) | GFL 并网变流器控制工程。 |
| 单相整流/逆变器 | [`pgs_sinv_rc`](ctl/suite/pgs_sinv_rc/readme_cn.md) | 包含 BUILD_LEVEL 1–5、重复控制及完整 SIL 实验报告。 |

`mcs_pmsm` 和部分 `mcs_acm` 内容属于较早的工程结构，适合兼容维护，不建议作为新项目的首选模板。完整套件说明见 [CTL Suite 指南](ctl/suite/readme_cn.md)。

## 2. 仓库结构

| 目录 | 作用 |
| --- | --- |
| [`core`](core/readme_cn.md) | GMP 核心运行时：调度、设备、内存、通信及基础框架。 |
| [`csp`](csp/readme_cn.md) | Chip Support Package，芯片和运行平台支持。 |
| [`ctl`](ctl/readme_cn.md) | 控制模板库，包括数学模块、控制组件、框架和完整 suite。 |
| [`cctl`](cctl/readme_cn.md) | C++ 类形式的控制与电力电子对象。 |
| [`vcore`](vcore/readme_cn.md) | HDL/Verilog 与相关实验性平台支持。 |
| [`slib`](slib/readme_cn.md) | GMP MATLAB/Simulink 库的源文件、安装脚本和版本化安装目录。 |
| [`tools`](tools) | SDPE、源代码管理、SIL/PIL、调试器、安装器等开发工具。 |
| [`quick_start`](quick_start/readme_cn.md) | 轻量级工程和源文件生成示例。 |
| [`manual`](manual/README_CN.md) | 使用指南、编码规范及平台操作文档。 |
| [`third_party`](third_party) | 仓库直接维护的第三方依赖。 |

常用 C/C++ 总入口为：

```c
#include <gmp_core.h>
```

或：

```cpp
#include <gmp_core.hpp>
```

具体应用通常不需要手工包含所有 GMP 源文件，应使用工程内的 `gmp_src_mgr` 生成实际依赖。

## 3. Suite 工程模型

当前推荐的控制套件采用以下结构：

```text
ctl/suite/<suite>/
├── src/                         公共控制算法与用户逻辑
├── sdpe_general/                全平台公共 SDPE 参数
├── project/
│   ├── simulate/                PC + Simulink SIL 工程
│   │   ├── sdpe_mgr/            仿真平台特有参数
│   │   ├── gmp_src_mgr/         本工程 GMP 源文件依赖
│   │   └── xplt/                仿真平台适配层
│   └── <hardware>/              目标硬件工程
│       ├── sdpe_mgr/            板卡、芯片和外设特有参数
│       ├── gmp_src_mgr/         本工程 GMP 源文件依赖
│       └── xplt/                硬件平台适配层
└── doc/                         实验报告、波形和结果归档
```

各层职责如下：

- `src`：控制对象、状态机、控制中断逻辑和 Datalink 用户任务，不直接访问芯片寄存器。
- `xplt`：实现 ADC 输入、PWM 输出、外设初始化、快速使能/关断和 UART 等平台接口。
- `sdpe_general`：拓扑、额定值、标幺基值、控制带宽、保护阈值等跨平台公共配置。
- `project/<target>/sdpe_mgr`：时钟、ADC/PWM 资源、板卡连接和平台开关等工程特有配置。
- `gmp_src_mgr`：选择并同步该工程实际使用的 GMP 组件。

典型实时调用链为：

```text
主控制中断
  -> ctl_input_callback()       采样并转换为控制量
  -> gmp_base_ctl_step()
       -> ctl_dispatch()        执行平台无关控制算法
  -> ctl_output_callback()      写入 PWM 等执行器
```

硬件移植时，应优先修改目标工程的 `xplt` 和平台 SDPE 配置；只有控制策略本身发生变化时才修改公共 `src`。

## 4. SDPE 参数工程

SDPE（System Design and Parameter Engineering）用于统一描述电源拓扑、逆变器、传感器、电机、板卡资源、控制器和工程参数，并生成 C 头文件与 MATLAB 初始化脚本。

在采用双层 SDPE 的 suite 中：

1. 运行 `sdpe_general\sdpe_edit.bat`，会打开公共配置以及该 suite 下所有工程配置。
2. 运行某个 `project\<target>\sdpe_mgr\sdpe_edit.bat`，会同时打开公共配置和当前目标配置。
3. 修改后分别运行两层目录中的 `sdpe_generate.bat`。
4. 运行 `sdpe_validate.bat` 检查 requirement 和生成结果。

Simulink 模型通过模型位置计算相对路径，运行时依次执行：

```text
../../sdpe_general/*_matlab_init.m
./sdpe_mgr/*_matlab_init.m
```

每一层必须恰好包含一个生成的 MATLAB 初始化脚本。工程不应在模型回调或 SDPE 文件中保存开发者机器的绝对路径。

若升级了 SDPE 工具模板，可将工具链部署到仓库内所有 `sdpe_general` 和 `sdpe_mgr`：

```bat
tools\SDPE_v2\gmp_sdpe_deploy_project_mgr.bat
```

SDPE 的 schema、命令行、GUI 和双层工程约定见 [SDPE v2 文档](tools/SDPE_v2/README_CN.md)。

## 5. GMP 源代码管理器

每个可独立编译的工程通常包含 `gmp_src_mgr`。其配置文件 `gmp_framework_config.json` 描述实际使用的 GMP 模块；生成结果包括保持目录结构的头文件、平铺后的 C/C++ 源文件以及编译器 include 路径清单。

常用流程：

```bat
project\<target>\gmp_src_mgr\gmp_config.bat
project\<target>\gmp_src_mgr\gmp_generate_inc.bat
project\<target>\gmp_src_mgr\gmp_generate_src.bat
```

有些 IDE 工程把 `gmp_src_mgr` 放在更深一层，例如 `project/<target>/src/gmp_src_mgr`；以目标工程已经提供的脚本位置为准。修改 GMP 库组件后，必须重新生成并同步目标工程，不能只修改旧的生成副本。

## 6. SIL 联合仿真

GMP 的标准 SIL 方式让控制器作为 PC 原生程序运行，Simulink 负责功率电路和传感器模型，双方通过 UDP 交换 ADC、PWM、状态和调试数据：

```text
Simulink power model
  <-> GMP SIL block / UDP helper
  <-> project/simulate controller executable
  <-> suite/src control algorithm
```

推荐启动顺序：

1. 在 `sdpe_general` 和仿真工程 `sdpe_mgr` 中配置、生成参数。
2. 使用 `gmp_src_mgr` 同步依赖并编译 `project/simulate` 控制器工程。
3. 打开 suite 自带的 `.slx` 模型，确认模型初始化已执行两层 SDPE 脚本。
4. 启动控制器程序，再启动 Simulink 仿真。
5. 按 suite 文档规定的 `BUILD_LEVEL` 从开环逐级验证到完整闭环。

可直接参考两套已经归档的实验流程：

- [FSBB UDP/SIL 仿真实验文档](ctl/suite/dps_fsbb/doc/README.md)
- [SINV UDP/SIL 仿真实验文档](ctl/suite/pgs_sinv_rc/doc/README.md)

各 suite 的参数含义、模型接线、验证判据和启动脚本以其自身 `README.md` 与 `doc` 为准。

## 7. PIL、Datalink 与在线调试

GMP 还提供 Processor-in-the-Loop 和 Datalink 调试机制：

- PIL 在真实 MCU 上执行控制程序，并把仿真输入输出与上位机交换。
- Datalink 提供变量观察、参数在线调整、Memory Perspective、PIL 数据交换等服务。
- 调试器入口位于 `tools/gmp_pil_server/gmp_debugger_v2`。
- 通信和后台任务通常在 suite 的 `user_main.c` 中组织，UART 等物理接口在目标 `xplt` 中实现。

高频控制中断中不应执行阻塞式串口或界面通信；调试数据应交给后台调度任务处理。

## 8. 开发与维护约定

- 公共算法放在 suite 的 `src`，芯片与板卡差异放在 `project/<target>/xplt`。
- 可配置物理量应进入 SDPE，避免在 `ctl_main.c` 中散落无来源的常数。
- 仿真和硬件工程应共享同一套控制源代码和公共参数定义。
- 新增依赖后更新 `gmp_framework_config.json`，并重新生成工程源文件。
- 不在代码、模型回调、JSON 或脚本中提交盘符相关的绝对路径。
- 不直接修改 `slib/install_path` 中的已安装 Simulink 库；应修改 `slib/simulink_lib_src` 后重新安装。
- SDPE 与 `gmp_src_mgr` 的工程内脚本是部署副本；工具本身应在 `tools` 下的模板中修改后统一分发。
- 控制器应按 `BUILD_LEVEL` 从采样、PWM 和开环开始逐级验证，不能跳过保护与极性检查直接闭环上电。
- 提交前至少运行相关生成器、目标编译、仿真 smoke test 以及 `git diff --check`。

中文编码说明与详细手册：

- [代码规范](manual/code-standard.md)
- [MATLAB/Simulink 库与 SIL 说明](slib/readme_cn.md)
- [CTL 组件开发说明](ctl/component/readme_cn.md)
- [CTL Suite 指南](ctl/suite/readme_cn.md)
- [SDPE v2 文档](tools/SDPE_v2/README_CN.md)
- [C28x SysConfig 工程说明](csp/c28x_syscfg/doc/readme.md)
- [面向 AI 的仓库维护技能](.agents/skills/maintain-gmp-repository/SKILL.md)

英文资料和英文工程指南统一收录在 [English README](README.md) 中。

## 9. 许可证

GMP 使用 [Apache License 2.0](LICENSE.txt)。第三方组件的版权和许可证信息见 [NOTICE](NOTICE) 及各组件目录中的声明。
