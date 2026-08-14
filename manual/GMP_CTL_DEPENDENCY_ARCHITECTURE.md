# GMP 与 CTL 依赖架构及维护规范

## 1. 目标

本文规定 GMP 基础平台、CSP、CTL、应用工程和 Facility 源码管理器之间的依赖方向。目标是让 CTL 算法既能在完整 GMP 运行时中使用，也能在不引入外设、任务框架和平台生命周期的场景中独立使用。

依赖设计遵循三条原则：

1. 算法只依赖类型、数学和显式声明的算法组件。
2. 平台配置按“用户配置 > CSP 默认配置 > 仓库默认配置”的优先级装配。
3. `gmp_src`、`gmp_inc` 等目录是生成结果，仓库源码和 Facility 注册表才是权威来源。

## 2. 系统分层与职责

| 层 | 目录或入口 | 职责 | 允许依赖 |
| --- | --- | --- | --- |
| 用户工程 | 固定名称的 `xplt.config.h`、工程设置和 `ctl_main.*` | 最终类型、功能开关、硬件参数和应用组合 | CSP、core、CTL |
| CSP | `csp/<platform>` | 芯片/编译器默认值、系统 tick、临界区和平台实现 | core 标准与端口契约；不得反向依赖应用算法 |
| 标准层 | `core/std`、`gmp_type.h` | 配置选项、编译器属性、基础类型、错误码、端序 | C 标准库及用户/CSP 配置 |
| 基础层 | `core/base` | 时间、断言、打印、基础服务契约 | `core/std` |
| 内存层 | `core/mm` | 内存管理 | `core/std`/`core/base` 中明确需要的服务 |
| 任务层 | `core/pm` | 调度器、任务和进程管理 | `core/base`，不得依赖 CTL 算法 |
| 外设层 | `core/dev` | 外设端口、驱动、数据链路 | `core/std`/`core/base` 和 CSP 端口 |
| 运行时层 | `core/rt`、`gmp_core.h` | 完整 GMP 生命周期、CSP 和应用装配 | base、dev、mm、pm、可选 CTL |
| CTL 数学契约 | `ctl/math_block/gmp_math.h` | `ctrl_gt`、数学后端、常量、向量/矩阵/坐标变换 | 正常模式只依赖 `gmp_type.h`；便携模式依赖 portable 契约 |
| CTL 组件 | `ctl/component` | 控制器、滤波器、观测器、接口模型 | `gmp_math.h` 和 Facility 中显式声明的其他 CTL 模块 |
| CTL 框架 | `ctl/framework` | 可选状态机和调度框架 | `core/base` 与显式 CTL 组件；不得依赖应用 `ctl_main.h` |

依赖方向应保持自上而下，算法层不得通过 `gmp_core.h` 间接获得外设、CSP 或运行时服务。

### 2.1 仓库级模块地图

除 core/CSP/CTL 主链外，仓库还包含以下边界。它们不应被嵌入式 CTL 组件隐式引入：

| 目录 | 定位 | 依赖与维护边界 |
| --- | --- | --- |
| `ctl/suite` | 完整控制应用与目标工程 | 组合 core、CSP、CTL component、SDPE 参数和 `xplt` 适配；应用依赖在这里闭合 |
| `cctl` | 实验性 C++ 控制与电力电子对象 | 面向 C++11/主机数值模型；不是当前嵌入式 suite 的默认运行时，不得成为 C 语言 CTL 的反向依赖 |
| `vcore` | HDL/Verilog 与硬件控制实验 | 与 C 运行时分离；每个设计单独记录综合工具、器件、时序约束和验证方法 |
| `slib` | MATLAB/Simulink 模型、脚本和 SIL 发布物 | 协议 C++ 源只在 `tools/gmp_sil/sil_helper` 维护；模型和 MEX 按 MATLAB Release 生成，不复制为第二份源码 |
| `tools/facilities_generator` | 模块注册、依赖选择与源码分发 | 管理“选择哪些代码”；权威数据是 Facility 注册表和工程配置 |
| `tools/SDPE_v2` | 硬件实体、参数模板与工程参数装配 | 管理“代码采用哪些参数”；模板、实体、需求和生成结果分层维护 |
| `third_party` | 外部依赖 | 由环境/包管理流程恢复；业务模块不得修改其源码来掩盖本仓库接口问题 |
| `manual` | 跨模块架构与维护规范 | 记录稳定的系统规则；目录内 README 记录局部用法，二者应互相链接而不重复维护实现细节 |

Facility 当前可执行模块树覆盖 `core`、`csp` 和 `ctl`。`cctl` 与 `vcore` 在形成稳定、可选择的发布单元之前保持独立验证；若将来纳入 Facility，必须先定义模块所有权、直接依赖和对应生成/验证链，不能仅在根列表中声明名称。

## 3. 配置与类型装配顺序

正常 GMP 工程使用以下链路：

```text
应用/用户的 xplt.config.h
        ↓ 覆盖
CSP 的 csp.config.h
        ↓ 补充
core/std/cfg/gmp.cfg.h 仓库功能默认值
        ↓
CSP 的 csp.typedef.h 真实平台例外
        ↓ 补齐
core/std/cfg/arch.cfg.h 架构类型默认值
        ↓ 回退
core/std/cfg/types.cfg.h 通用类型默认值
        ↓ 校验并形成
gmp_type.h 基础类型与编译器契约
```

`gmp_type.h` 是库组件获取 GMP 统一类型的最小入口。它不装配外设管理、任务框架或完整运行时。保存 tick 的 `time_gt` 由 `GMP_PORT_TIME_T` 决定，因此可由用户或 CSP 选择 32 位或 64 位实现；算法不得假设其固定宽度。

`core/std/arch` 维护处理器数据模型。目前自动识别 Cortex-M、C28x、C29x、x86、x86-64 和 32-bit RISC-V。架构头只定义尚未被用户或 CSP 定义的 `GMP_PORT_*` 宏，因此 CSP 不得复制整张通用类型表，只保留外设句柄、特殊 tick 宽度等真实例外。C28x 的 C byte 为 16 bit，不存在 `int8_t/uint8_t`；可复用状态、协议字节容器和小整数应分别采用 `data_gt`、`fast_gt` 或明确的序列化逻辑。

临界区进入/退出必须由 CSP 以 `GMP_STATIC_INLINE` 提供，使关中断/开中断指令在调用点展开；不得为解决链接问题改成外部函数。只使用 `gmp_type.h`/`gmp_base.h` 的平台桥接模块若需要临界区，必须显式取得 CSP 内联定义并在 Facility 中登记该装配依赖。`gmp_base_print` 的声明则由 `GMP_USER_PRINT_FUNCTION_DECLARATION` 从固定 CSP 配置送入 `gmp_base.h`，调度器等基础模块可以保留诊断输出；是否关闭输出由用户配置决定，不能通过删除调用规避依赖。

配置头在基础类型尚未完成时可能包含 SDPE 生成的硬件参数头。硬件参数头必须保持“宏与常量资产”的性质，不得引入 `gmp_math.h` 或控制器实现头，否则会形成 `gmp_type.h → 配置 → 预设 → gmp_math.h` 的递归装配。

## 4. CTL 数学入口

CTL 公共算法头统一包含：

```c
#include <ctl/math_block/gmp_math.h>
```

禁止在可复用 CTL 组件中包含 `gmp_core.h`。完整应用仍可在应用入口包含 `gmp_core.h`。

`gmp_math.h` 提供：

- `ctrl_gt` 与 `parameter_gt`；
- 浮点、双精度、定点和 CSP 数学后端选择；
- CTL 数学常量和补丁；
- 基础向量、矩阵、复数和坐标变换；
- 独立的 `gmp_ctl_assert` 断言契约。

资源受限或不使用 GMP/CSP 的场景可定义 `GMP_CTL_PORTABLE`，由 `ctl/portable/gmp_ctl_portable.h` 提供最小类型与时间契约。

## 5. Facility 依赖规则

权威注册表为 `tools/facilities_generator/src_mgr/gmp_framework_dic.json`。

每个模块的字段含义如下：

- `src_patterns`：该模块拥有的实现文件；
- `inc_patterns`：该模块拥有并发布的头文件；
- `depends_on`：编译或链接该模块必须同时选择的直接模块；
- `help_docs`：模块说明；
- `tasks`：编译、原理仿真和硬件验证状态。

维护 `depends_on` 时应遵守：

1. 源/头文件直接包含另一个 Facility 模块的头时，必须存在一条可达依赖路径。
2. 控制器使用 PID、滤波器、PLL、顾问或接口类型时，应声明对应的具体模块，不能只依赖 `math_block|_internal`。
3. 公共契约和实现聚合器要分开。例如 `component|interface|base` 发布接口基础类型，`component|interface|_internal` 聚合 ADC/PWM 实现，避免基础头与子模块形成环。
4. 一个头原则上只由一个模块拥有。共享代码通过 `depends_on` 复用，不应复制到多个模块的 `inc_patterns`。
5. 条件编译的替代实现不等于无条件依赖。例如 `GMP_CTL_PORTABLE` 是正常数学类型入口的替代路径。
6. 依赖图不得有环。

典型依赖包括：

| 模块类别 | 常见直接依赖 |
| --- | --- |
| 电流环/机械环 | PID 或 dq-PI、离散滤波器、编码器接口、坐标变换 |
| PMSM/ACIM 观测器 | PID、离散滤波器、ATO-PLL、相应电机顾问和 PU 顾问 |
| 数字电源控制器 | ADC/PWM 接口、PID、PR、SOGI、双二阶或通用离散滤波器 |
| DSA | intrinsic divider、math utilities；数据链路版本另依赖 core base/dev |
| 硬件预设 | 仅参数/选项资产；不得依赖运行时或数学实现 |

## 6. 源码生成与工程文件

Facility 选择完成后，Visual Studio 公共属性表按以下优先级生成扁平化源码：

1. 工程提供 `gmp_src_mgr/gmp_generate_src.bat` 时，调用工程定制脚本；
2. 只有 `gmp_src_mgr/gmp_framework_config.json` 时，直接调用 Facility v3 同步器；
3. 旧工程仍可由工程根目录的 `gmp_fac_generate_src.bat` 兼容生成。

三个入口都必须读取权威 Facility 注册表和工程选择配置，不得把仓库源码复制为新的权威副本。

工程文件必须引用当前生成名称。核心重构后，旧的 `gmp_dev_util.c`、`gmp_process_mgr.c`、`gmp_std_error_code.c` 和 `gmp_std_port.c` 已由 `gmp_runtime.c` 等新模块替代；生成出的 `function_scheduler.c`、`ring_buffer.c` 等仍需在工程中登记。

不要直接修改 `gmp_src` 或 `gmp_inc` 中的内容。Keil 等 IDE 的源代码管理器可能重新生成这些目录，手工放入权威源码会造成引用顺序和重复定义问题。

Keil 工程只允许引用工程内的 `gmp_src_mgr/gmp_src` 与 `gmp_src_mgr/gmp_inc` 生成结果。每次 Facility 生成改变文件集合后，必须同步刷新 `.uvprojx` 的生成源码组：删除已退出 Facility 的旧文件，并加入新生成文件；不得用指向仓库权威目录的路径来临时补齐缺项。

### 6.1 DataLink 平台映射

DataLink 核心只负责协议状态机，UART 句柄、实例和接收机制由 SDPE/xplt/CSP 闭合。以 STM32G431 为例：SDPE 将 `MCS_UART_HANDLE` 映射为 `&huart2`、`MCS_UART_INSTANCE` 映射为 `USART2`；CubeMX 提供 USART2 RX 循环 DMA；xplt 的半满/全满回调计算 DMA 增量并调用 `gmp_dev_dl_push_str()`；发送路径通过 CSP 的 `gmp_hal_uart_write()` 输出 DataLink header 与 payload。修改任一端时必须同时核对四层映射，不能只验证宏存在。

DataLink Scope 的线格式触发电平固定为 F32，但目标侧字段不得固定成 `float`。`gmp_scope_parameter_gt` 在启用 CTL 且未定义 `SPECIFY_DISABLE_GMP_MATH` 时绑定 `parameter_gt`，否则回退为 `float`；解码时先读取 F32，再显式转换为目标参数类型。用户还可通过 `GMP_SCOPE_PARAMETER_T` 覆盖局部类型，而不改变协议线格式。

## 7. 维护流程

新增或迁移 CTL 模块时依次执行：

1. 确认模块属于 math、component、framework 或应用边界。
2. 公共算法头包含 `gmp_math.h`，运行时框架只包含所需的 `core/base` 服务。
3. 将源/头文件注册到唯一 Facility 模块。
4. 根据直接 `#include` 补充 `depends_on`，并检查是否形成环。
5. 运行 Facility 路径校验和依赖审计：

   ```powershell
   $env:GMP_PRO_LOCATION = 'E:\lib\gmp_pro'
   python tools\facilities_generator\src_mgr\facility_dependency_audit.py
   ```

6. 用源码管理器重新生成工程输出，不手工同步生成目录。
7. 编译所有注册公共头、实现文件和可用的 SIL 工程。
8. 运行数值回归测试。主机测试通过只代表主机编译与算法结果，不替代 TI、ARM、Keil、MATLAB/Simulink 或实体硬件验证。

### 7.1 维护责任与变更触发

| 变更位置 | 必须同步维护 | 最低验证 |
| --- | --- | --- |
| `core/std`、`gmp_type.h` | 用户/CSP/默认值优先级、类型宽度、编译器契约、options 文档 | C/C++ 类型 smoke test，并至少验证一个 CSP |
| `core/base/dev/mm/pm/rt` | Facility 所有权、直接依赖、相应目录 README | Facility 校验；受影响平台或主机替代实现编译 |
| `csp/<platform>` | 固定配置入口、tick/临界区/驱动实现、平台 README | 对应工具链编译；硬件行为变化必须重新做板级验证 |
| `ctl/math_block` | `gmp_math.h` 最小边界、portable 后端、数学 README | 标准 C/C++ 与受支持 portable smoke test |
| `ctl/component` | 公共头自包含、Facility `depends_on`、算法测试 | 公共头独立编译、依赖审计和数值回归 |
| `ctl/framework` | core/base 契约及所用组件依赖 | 框架编译；不得重新包含应用 `ctl_main.h` |
| `ctl/suite` | Facility 选择、SDPE requirement、`xplt`、工程 README | SIL 构建/回归；目标工程另做工具链与硬件验证 |
| `cctl` | C++ API、求解器/模型测试和局部 README | 对应 C++ 离线测试；不得用其结果替代 CTL 硬件验证 |
| `vcore` | HDL 接口、约束、测试平台与工具版本 | 仿真、综合及时序报告；按设计记录板级验证 |
| `slib` 或 SIL 协议 | 唯一 C++ 协议源、MATLAB 模型、Mask、发布版本说明 | MEX、MATLAB 回归、定步长 SIL/Rapid Accelerator |
| Facility/SDPE 工具 | 数据格式、生成器、分发脚本和工具 README | schema/注册表校验、代表工程重新生成与无绝对路径检查 |

### 7.2 当前目标工具链验证基线

| 架构/工程 | 工具链 | 验证目标 |
| --- | --- | --- |
| C28x / `csp/c28x_syscfg/iris_280039c_board` | TI C2000 CGT 22.6.1 LTS + CCS headless build | Facility 生成、SysConfig、全部 C/ASM 编译和链接 |
| C29x / `ctl/suite/mcs_pmsm_nt/project/f29h85x_lp_3phgan` | TI C29 Clang 2.2.0 LTS + F29H85x SDK 1.02.01.00 | SysConfig、CTL/core/CSP 编译和链接 |
| Cortex-M4 / `ctl/suite/mcs_pmsm_nt/project/stm32g431` | Keil ARMCC 5.06u6；另用 GNU Arm 进行架构 smoke test | 生成副本引用、DataLink UART/DMA 映射、全部编译和链接 |

这些结果是工具链与链接验证，不代表板级时序、UART 电气连接、DMA 运行状态或电机功率级已经通过硬件验证。

## 8. 评审检查清单

- 可复用 CTL 文件中是否仍出现 `#include <gmp_core.h>`？
- 数学/控制类型是否只从 `gmp_math.h` 获取？
- 硬件参数和 SDPE 配置头是否保持纯配置属性？
- Facility 文件所有权是否唯一？
- 每个 PID、滤波器、接口和顾问依赖是否可达？
- 依赖图是否无环？
- 工程引用是否与源码管理器的实际输出一致？
- README、Facility `help_docs` 和本文是否随结构变更同步更新？
- 验证记录是否区分主机编译、原理仿真和硬件验证？
