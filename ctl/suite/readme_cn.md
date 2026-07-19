# GMP CTL Suite 指南

[English](readme.md) | **简体中文**

## 1. Suite 定位

`ctl/suite` 提供不同控制场景的跨平台控制器实现。它不是单个算法模块，而是基于 `ctl/component`、`ctl/math_block`、`core` 和 `csp` 组织出的完整控制工程模板。

Suite 的目标是：

- 复用同一套控制逻辑，在仿真环境和实际硬件平台上运行。
- 将控制算法、用户交互、硬件接口和工程依赖管理分离。
- 允许用户把某个 suite 独立拷贝为自己的工程仓库，通过源代码管理工具生成所需 GMP 头文件和源文件。

## 2. 当前 Suite 清单

Suite 名称通常使用应用类别前缀：

| 前缀 | 全称 | 典型应用场合 |
| --- | --- | --- |
| `dps` | Digital Power Suite | 数字电源类电力电子变换器，例如 DC-DC、四开关全桥、PFC 等。 |
| `mcs` | Motor Control Suite | 电机控制类应用，例如异步电动机、永磁同步电机、伺服驱动和参数识别。 |
| `pgs` | Power Grid Suite | 电网互动类电力电子变换器，例如跟网型变流器、并网逆变器、单相变流器等。 |

| Suite | 含义 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| [`dps_clllc`](dps_clllc/README_CN.md) | Digital Power Suite, CLLLC / DAB | 正式工程 | 双向隔离谐振变换器控制。 |
| [`dps_fsbb`](dps_fsbb/README_CN.md) | Digital Power Suite, four-switch full bridge | 推荐结构 | 四开关全桥电源/变流器控制。 |
| `mcs_acm` | Motor Control Suite, asynchronous motor | 历史结构 | 异步电动机控制，目前仍使用 `implement` 旧结构。 |
| `mcs_pmsm` | Motor Control Suite, PMSM | 已弃用 | 永磁同步电机旧工程，不建议新项目继续基于它开发。 |
| [`mcs_pmsm_nt`](mcs_pmsm_nt/README_CN.md) | Motor Control Suite, PMSM new template | 正式工程 | 永磁同步电机正式工程，新 PMSM 应优先基于它开发。 |
| [`mcs_pmsm_id`](mcs_pmsm_id/README_CN.md) | Motor Control Suite, PMSM identification | 正式工程 | PMSM 参数识别工程，用于识别电机参数。 |
| [`pgs_inv_GFL_inverter`](pgs_inv_GFL_inverter/README_CN.md) | Power Grid Suite, grid-following inverter | 正式工程 | GFL 跟网型变流器/逆变器控制。 |
| [`pgs_sinv_rc`](pgs_sinv_rc/readme_cn.md) | Power Grid Suite, single-phase inverter with repetitive control | 正式工程 | SINV 单相变流器/逆变器控制。 |

术语：

- `FSBB`: four-switch full bridge，四开关全桥。
- `ACM`: asynchronous motor，异步电动机。
- `PMSM`: permanent magnet synchronous motor，永磁同步电机。
- `NT`: new template，当前正式 PMSM 工程。
- `ID`: identification，用于电机参数识别。
- `GFL`: grid-following，跟网型变流器。
- `SINV`: single-phase inverter，单相变流器。

## 3. 推荐目录结构

推荐的新结构如下：

```text
ctl/suite/<suite_name>/
├── src/
│   ├── ctl_main.c
│   ├── ctl_main.h
│   ├── user_main.c
│   └── user_main.h
├── project/
│   ├── simulate/
│   │   ├── *.slx
│   │   ├── gmp_src_mgr/
│   │   └── xplt/
│   └── <hardware_platform>/
│       ├── gmp_src_mgr/
│       └── xplt/
├── doc/
└── README.md
```

其中：

- `src` 是所有平台共享的控制工程通用代码。
- `project/simulate` 是与仿真环境联动的仿真工程。
- `project/<hardware_platform>` 是实际硬件平台工程。
- 每个可运行工程通常包含 `gmp_src_mgr` 和 `xplt`。

历史工程可能仍存在 `implement` 目录。新开发优先遵循 `src + project/<platform>/xplt` 结构；旧结构只作为维护对象。

## 4. `src` 通用控制代码

每个推荐结构的 suite 都有一个 `src` 文件夹。该目录是所有平台共享的控制代码，主要承担两类任务：

1. 用户交互逻辑。
2. 控制逻辑。

典型文件：

| 文件 | 职责 |
| --- | --- |
| `ctl_main.c` | 控制对象定义、控制初始化、控制周期调度、主要控制状态机。 |
| `ctl_main.h` | 控制逻辑对外接口、控制对象声明、控制任务入口。 |
| `user_main.c` | 用户交互、命令处理、非实时任务、上层调度逻辑。 |
| `user_main.h` | 用户交互相关接口声明。 |

约束：

- `src` 不应直接包含芯片寄存器、板级外设驱动或具体硬件连接。
- 所有平台相关输入输出都应通过 `xplt.ctl_interface.h` 隔离。
- 所有硬件参数和连接关系应从对应平台的 `xplt/ctrl_settings.h` 获取。
- 控制周期中的算法计算应遵守 component 规范：使用 `ctrl_gt` 和标幺值系统。

## 5. `project/simulate` 仿真工程

每个正式 suite 的 `project` 文件夹中通常都有 `simulate` 工程。该工程用于和仿真环境联动，通常包含 Simulink 模型和 PC 侧工程文件。

用途：

- 与 Simulink 联合仿真，快速验证控制算法可行性。
- 在不接入真实硬件的情况下验证控制流程、状态机和参数。
- 作为移植到硬件平台前的第一层验证。

典型内容：

- `*.slx`: Simulink 模型。
- `*.slxc`: Simulink 缓存或模型相关文件。
- Visual Studio / CMake / 其他 PC 仿真工程文件。
- `gmp_src_mgr`: 当前仿真工程使用的 GMP 源代码配置管理工具。
- `xplt`: 仿真平台的 cross platform 适配代码。

注意：

- `simulate` 不是硬件工程的替代品，它用于算法和流程验证。
- 仿真工程的 `xplt` 仍然应遵循和硬件工程相同的接口边界。
- 仿真通过后，再进入实际硬件工程做编译和实物验证。

## 6. 硬件平台工程

`project` 下除 `simulate` 之外的目录通常代表实际运行场景的工程。各 suite 支持的平台不同。

当前主要平台清单：

| Suite | project 平台 |
| --- | --- |
| `dps_fsbb` | `f280039c_Iris_node`, `simulate` |
| `mcs_acm` | `f280049c`, `motor_control_simulink`, `stm32g431`, `stm32g474` |
| `mcs_pmsm` | `f280039c_Iris_node`, `f280049c`, `f280049c_new`, `f28p65`, `motor_control_simulink`, `stm32f405`, `stm32f405_new`, `stm32g431`, `stm32g474`, `stm32g474_tim` |
| `mcs_pmsm_id` | `f280039c_Iris_node`, `f280049c`, `simulate`, `stm32f405`, `stm32g431`, `stm32g474_hrtim` |
| `mcs_pmsm_nt` | `f280039c_Iris_node`, `f280049c`, `simulate`, `stm32f405`, `stm32g431`, `stm32g474_hrtim` |
| `pgs_inv_GFL_inverter` | `f280039c_Iris_node`, `f280049c`, `simulate`, `stm32g431` |
| `pgs_sinv_rc` | `f280039c_Iris_node`, `simulate` |

说明：

- `mcs_pmsm` 是弃用旧工程，平台较多但不代表推荐新开发入口。
- `motor_control_simulink` 是历史仿真工程命名；新结构推荐使用 `project/simulate`。
- 某些平台目录中可能存在 IDE 生成的 `Debug` 子目录，其中的 `gmp_src_mgr` 或 `xplt` 是构建产物或工程复制结果，不应作为规范入口。

## 7. `gmp_src_mgr` 源代码配置管理

每个可独立运行的工程下通常都有 `gmp_src_mgr` 文件夹。它负责把 GMP 仓库中的必要头文件和源文件整理到当前工程中，使 suite 可以脱离完整 GMP 仓库独立运行。

典型文件：

| 文件/目录 | 作用 |
| --- | --- |
| `gmp_config.bat` | 启动配置界面，选择当前工程启用或禁用的 GMP 功能。 |
| `gmp_generate_inc.bat` | 根据当前依赖生成需要的头文件。 |
| `gmp_generate_src.bat` | 根据当前依赖生成需要的源文件。 |
| `gmp_framework_config.json` | 当前工程的 GMP 模块配置。 |
| `gmp_compiler_includes.txt` | 生成后的 include 路径信息，供工程配置使用。 |
| `gmp_src/` | 生成的 GMP 源文件平铺目录。 |

生成规则：

- 源文件会被平铺放入 `gmp_src_mgr/gmp_src`。
- 头文件生成时保持原有目录结构。
- 头文件结构完整继承自 `gmp_pro` 项目根目录，因此 include 仍以 GMP 根目录为逻辑根。
- 修改 suite 依赖后，应重新运行 include/source 生成脚本。

推荐流程：

1. 运行 `gmp_config.bat`，选择所需 GMP 模块。
2. 运行 `gmp_generate_inc.bat`，生成头文件。
3. 运行 `gmp_generate_src.bat`，生成源文件。
4. 检查工程 include 路径是否包含生成目录。
5. 编译当前平台工程。

## 8. `xplt` 跨平台适配层

每个工程中的 `xplt` 表示 cross platform。该目录保存全部与当前运行硬件或仿真平台相关的代码。用户移植 suite 时，通常只需要重点关注 `xplt`。

典型文件：

| 文件 | 职责 |
| --- | --- |
| `ctrl_settings.h` | 定义硬件属性、硬件参数、连接关系、控制频率、传感器量程等。所有数据源应从这里获取。 |
| `xplt.config.h` | 定义当前 `gmp_pro` 工作状态的属性配置，例如平台、芯片、编译特性、GMP 功能开关。 |
| `xplt.ctl_interface.h` | 实现控制输入和输出接口。由于很多接口函数需要内联，该文件以头文件形式提供实现。 |
| `xplt.peripheral.c` | 定义当前硬件特有的输入输出外设初始化与驱动逻辑。 |
| `xplt.peripheral.h` | `xplt.peripheral.c` 的配套声明。 |

移植原则：

- 优先修改 `xplt`，不要改 `src` 中的通用控制逻辑。
- 新硬件的 ADC、PWM、GPIO、编码器、通信等外设初始化放入 `xplt.peripheral.c/h`。
- 控制逻辑需要的输入输出函数放入 `xplt.ctl_interface.h`。
- 新增硬件参数、采样比例、极性、连接关系放入 `ctrl_settings.h`。
- 平台特定宏、GMP 工作模式和编译属性放入 `xplt.config.h`。
- 用户需要增加自己特有的硬件功能时，也应优先增加在 `xplt` 文件夹中。

## 9. 跨平台实现方式

Suite 的跨平台方式可以概括为：

```text
src/
  user_main.*     -> 用户交互、非实时任务
  ctl_main.*      -> 控制对象、控制状态机、控制周期

project/<platform>/xplt/
  ctrl_settings.h       -> 硬件参数和连接关系
  xplt.config.h         -> GMP 工作状态和平台属性
  xplt.ctl_interface.h  -> 控制输入输出接口
  xplt.peripheral.*     -> 平台外设

project/<platform>/gmp_src_mgr/
  gmp_config.bat
  gmp_generate_inc.bat
  gmp_generate_src.bat
```

`src` 通过 `xplt.ctl_interface.h` 访问输入输出，不直接理解硬件。`xplt` 通过 `ctrl_settings.h` 描述硬件，不把控制算法写死到外设驱动中。`gmp_src_mgr` 负责把被选中的 GMP 模块生成到当前工程。

## 10. 主中断与控制调用链

Suite 与 GMP CTL 框架协作的核心入口是 `gmp_base_ctl_step()`。该函数定义在 `ctl/framework/ctl_dispatch.h`，通常由硬件主中断调用。

常规硬件工程中的调用链如下：

```text
project/<platform>/xplt/xplt.peripheral.c
  MainISR()
    ├─ 必要的硬件中断前置操作
    ├─ gmp_base_ctl_step()
    │    ├─ ctl_input_callback()
    │    ├─ ctl_dispatch()
    │    └─ ctl_output_callback()
    ├─ 系统 tick、监控、日志等可选任务
    └─ 清除中断标志 / ACK 中断
```

对应文件职责：

| 函数 | 典型位置 | 职责 |
| --- | --- | --- |
| `MainISR()` | `project/<platform>/xplt/xplt.peripheral.c` | 平台主中断。完成必要中断操作，并调用 `gmp_base_ctl_step()`。 |
| `gmp_base_ctl_step()` | `ctl/framework/ctl_dispatch.h` | GMP CTL 框架入口，组织输入、控制分发和输出。 |
| `ctl_input_callback()` | `project/<platform>/xplt/xplt.ctl_interface.h` | 从 ADC、编码器、GPIO、仿真输入等数据源读取并转换到控制对象。 |
| `ctl_dispatch()` | `src/ctl_main.h` 或 suite 控制源文件 | 执行 suite 的核心控制逻辑。 |
| `ctl_output_callback()` | `project/<platform>/xplt/xplt.ctl_interface.h` | 将控制结果写入 PWM、DAC、GPIO、仿真输出等目标。 |

用户移植时应优先理解这条链路。通常只需要改 `xplt` 中的输入输出和外设代码，不需要改变 `src` 中的控制调度。只有当控制策略本身改变时，才修改 `ctl_dispatch()` 及其调用的控制逻辑。

注意：

- 主中断应先处理平台必要的采样、中断状态或同步操作，再调用 `gmp_base_ctl_step()`。
- `ctl_input_callback()` 和 `ctl_output_callback()` 是内联函数，所以定义在 `xplt.ctl_interface.h`。
- `ctl_dispatch()` 是 suite 的控制核心，不应包含芯片寄存器操作。
- 某些工程可能启用 `SPECIFY_ENABLE_CTL_FRAMEWORK_NANO`，此时 `gmp_base_ctl_step()` 会进入 Nano Framework 分支；新用户仍应先理解标准 callback 模式。

## 11. SIL 与 PIL 联合仿真

GMP Suite 支持两类与电脑仿真环境联动的方式：SIL 和 PIL。

### 11.1 SIL 仿真

SIL 是 Software-in-the-Loop。控制程序运行在电脑上，通常以 Visual Studio 工程形式保存在 `project/simulate` 中，并与 Simulink 模型联合仿真。

SIL 的关键机制：

- 使用专用 CSP 将电脑抽象成一个芯片。
- 相关 CSP 位于 `csp/windows_simulink`。
- 控制程序按照 Simulink 时间感知物理时间，从而与模型同步运行。
- Simulink 与 SIL 程序之间的通信辅助工具位于 `tools/gmp_sil/udp_helper_v2`。
- 当前默认使用 UDP 协议传输仿真数据。

SIL 典型结构：

```text
Simulink model
  ⇄ UDP helper
  ⇄ project/simulate Visual Studio program
      ⇄ csp/windows_simulink
      ⇄ suite src control logic
```

使用建议：

- 推荐在同一台电脑或稳定局域网内运行 Simulink 和 SIL 程序。
- 虽然 UDP 可以跨电脑通信，但实际网络环境可能丢包。
- UDP 没有内建包校验和重传机制；一旦丢包，联合仿真可能卡住。
- Simulink 的仿真逻辑和 UI 耦合较紧，通信卡住时 UI 也可能无法结束仿真或关闭窗口。
- 如果 Simulink 卡住，可以尝试再次运行控制器程序，重新建立通信；通信恢复后 Simulink 通常会恢复响应并停止仿真。

### 11.2 PIL 仿真

PIL 是 Processor-in-the-Loop。控制程序真实运行在微控制器中，芯片通过串口、网络等方式将数据传输到电脑，再由电脑路由到 Simulink。

PIL 与普通硬件运行的区别在于：实际功率硬件的 `input/output` 不适合作为仿真接口，因此 PIL 有独立的输入输出 callback。

PIL callback 原型定义在各平台的 `xplt/xplt.ctl_interface.h`：

```c
GMP_STATIC_INLINE void ctl_output_callback_pil(gmp_sim_tx_buf_t* tx);
GMP_STATIC_INLINE void ctl_input_callback_pil(const gmp_sim_rx_buf_t* rx);
```

正式 suite 中通常在 `src/ctl_main.c` 实现 `gmp_pil_sim_step()`，由 PIL 内核调用：

```text
core/dev/pil_core.h / core/dev/src/gmp_pil_core.c
  gmp_pil_sim_rx_cb()
    └─ gmp_pil_sim_step(rx, tx)
         ├─ ctl_input_callback_pil(rx)
         ├─ ctl_dispatch()
         └─ ctl_output_callback_pil(tx)
```

PIL 关键文件：

| 文件 | 作用 |
| --- | --- |
| `core/dev/pil_core.h` | PIL 仿真模块接口，定义 `gmp_sim_rx_buf_t`、`gmp_sim_tx_buf_t`、`gmp_pil_sim_t` 等。 |
| `core/dev/src/gmp_pil_core.c` | PIL 模块实现，接收仿真请求并调用 `gmp_pil_sim_step()`。 |
| `project/<platform>/xplt/xplt.ctl_interface.h` | 平台 PIL 输入输出 callback。 |
| `src/ctl_main.c` | suite 侧 `gmp_pil_sim_step()` 桥接实现。 |

当前 PIL 主要支持串口通信，通信协议使用 Datalink。

## 12. Datalink 调试机制

Datalink，也常简称 DL，是 GMP Suite 当前采用的重要调试机制。功率电路调试时，传统调试器常遇到连接不稳定、断点影响实时性、变量观察不可靠等问题。Datalink 提供了一套面向上位机的通信协议，使用户可以像 Watch、曲线观察、在线参数调整一样查看和修改控制器中的变量。

核心文件：

| 文件 | 作用 |
| --- | --- |
| `core/dev/datalink.h` | Datalink 协议核心，提供帧解析、事件循环、收发状态机和命令机制。 |
| `core/dev/mem_presp.h` | Memory perspective 功能，用于按内存区域观察数据。 |
| `core/dev/tunable.h` | Tunable 参数功能，用于在线读写参数字典。 |
| `core/dev/readme_dl_protocol.md` | Datalink 协议说明文档。 |

在每个正式 suite 的 `user_main.c` 中，通常可以找到一组与 DL 调试对接的对象和任务，例如：

- `gmp_datalink_t dl`
- `gmp_pil_sim_t pil`
- `gmp_param_tunable_t tunable`
- `gmp_mem_persp_t mem_persp_server`
- `tsk_dl_debug_device()`

典型任务逻辑：

```text
user_main.c
  tsk_dl_debug_device()
    ├─ flush_dl_rx_buffer()
    ├─ gmp_dev_dl_loop_cb(&dl)
    ├─ GMP_DL_EVENT_TX_RDY -> flush_dl_tx_buffer()
    └─ GMP_DL_EVENT_RX_OK
         ├─ gmp_pil_sim_rx_cb(&pil)
         ├─ gmp_param_tunable_rx_cb(&tunable)
         ├─ gmp_mem_persp_rx_cb(&mem_persp_server)
         └─ default / echo handler
```

上位机调试器位于：

```text
tools/gmp_pil_server/gmp_debugger_v2/run.bat
```

这是一个基于 PyQt 的调试程序。相关实现可参考 `tools/gmp_pil_server/gmp_debugger_v2`，其中包含 Datalink 核心、PIL、tunable、memory perspective、曲线观察等页面或功能。

使用建议：

- 调试变量应通过 `user_main.c` 中的 tunable 字典或 memory region 明确登记。
- 通信收发的硬件适配通常放在 `xplt.peripheral.c/h`。
- DL 任务不应放入高频控制中断中长时间阻塞，应通过任务调度或后台循环处理。
- 调试功率硬件时，优先使用 Datalink 观察和调整变量，减少对传统断点调试的依赖。

## 13. 基于 Suite 建立新工程

推荐用户基于 suite 建立自己的独立仓库：

1. 选择最接近目标应用的 suite。
2. 复制该 suite 到新的工程仓库。
3. 保留 `src` 作为控制逻辑起点。
4. 新增或复制一个 `project/<your_platform>`。
5. 修改 `project/<your_platform>/xplt` 适配硬件。
6. 使用 `gmp_src_mgr` 选择依赖并生成头文件、源文件。
7. 先运行 `project/simulate` 做联合仿真，再进入硬件工程验证。

选择建议：

| 目标 | 推荐起点 |
| --- | --- |
| PMSM 正式控制工程 | `mcs_pmsm_nt` |
| PMSM 参数识别 | `mcs_pmsm_id` |
| 异步电动机控制 | `mcs_acm`，但需注意其旧结构 |
| 四开关全桥 | `dps_fsbb` |
| 跟网型三相变流器 | `pgs_inv_GFL_inverter` |
| 单相变流器 | `pgs_sinv_rc` |

## 14. 开发约束

- 新 suite 优先采用 `src + project/<platform>/xplt + gmp_src_mgr` 结构。
- 控制逻辑应尽量平台无关。
- 硬件适配只放在 `xplt`。
- 仿真工程和硬件工程应共享同一套 `src` 控制逻辑。
- 所有控制周期计算遵守 component 开发规范，使用 `ctrl_gt` 和标幺值系统。
- 修改依赖后必须重新生成 GMP 头文件和源文件。
- 文档、平台支持表和工程状态应随代码同步更新。

## 15. 版本记录

| 版本 | 日期 | 说明 |
| --- | --- | --- |
| 2.0 | 2026-07-08 | 重写 suite 指南，按当前 `src/project/xplt/gmp_src_mgr` 结构整理。 |
