# CCTL 主机仿真 CSP 接口说明

[English](README.md) | **简体中文**

本文档定义 `csp/cctl` 的维护边界。CSP 负责宿主进程入口、GMP 生命周期、仿真
实数类型、主线程仿真运行时、控制台进度、异步文件输出和通用输出使能状态；具体工程
负责控制器、`setup_peripheral()`、主电路和电机、MCU 寄存器映射、SDPE 参数、
记录格式以及结果校验。

## 头文件组成

C 程序包含 `gmp_core.h`，C++ 程序包含 `gmp_core.hpp`。GMP 会通过标准配置链
自动引入：

- `csp.config.h`：声明有限步 PC 仿真环境，启用 CSP 主动退出钩子，并默认以
  `USING_FLOAT_FPU` 选择 `ctrl_gt=float`；工程可在更早加载的
  `xplt.config.h` 中覆盖。
- `csp.typedef.h`：仅包含 C 兼容的类型定制，保留受保护的 `ctrl_gt` 默认值，
  其他基础类型沿用 GMP portable 定义。
- `csp.typedef.hpp`：定义连续仿真使用的 `sim_real_gt`，默认为 `double`。在包含
  GMP 头文件前定义 `GMP_CCTL_SIM_REAL_TYPE` 可以换用其他 C++ 类型。
- `csp.general.h`：声明控制器 C 代码可调用的 CSP 接口。
- `csp.general.hpp`：按规范包装 C 头，并额外包含 `csp.typedef.hpp` 和
  `csp_cctl.hpp`。工程 C++ 代码通常只需包含 `gmp_core.hpp`。

`ctrl_gt` 用来复现真实控制器的数值行为，因此默认保持 `float`；`sim_real_gt`
用于主电路、电机和外设的连续量仿真，因此默认使用 `double`。

替换后的 `sim_real_gt` 必须可默认构造、可复制、能从整数和浮点常量构造，并
支持赋值、加减乘除、单目负号和普通比较。它还必须兼容 CCTL 模型使用的
`abs/min/max/isfinite/floor/fmod/sqrt/sin/cos/remainder` 等数学操作。
内建的 `float`、`double`、`long double` 均满足要求；自定义类需要提供匹配的
数学重载和转换。若生成的主电路固定使用 `double`，仅修改该别名不会自动改变
生成矩阵的标量类型。

## C 接口

- `csp_sl_enable_output()`：设置 CSP 的功率级输出使能。应在控制状态机进入 PWM
  有效状态时调用；PMSM 工程的 `ctl_fast_enable_output()` 已连接到此函数。
- `csp_sl_disable_output()`：清除输出使能。初始化、停机、故障及禁用状态都应
  先调用它，再处理比较值。
- `csp_cctl_output_is_enabled()`：供工程的 MCU 仿真读取当前使能状态，非零时
  ePWM 才允许驱动开关管。
- `gmp_hal_wd_feed()`、`gmp_hal_wd_enable()`、`gmp_hal_wd_disable()`：宿主环境
  的空操作实现，用于满足通用 CSP 看门狗接口。

## 进程入口和 GMP 生命周期

选择 `csp|cctl` 后，`src/csp_cctl_main.cpp` 是唯一的进程 `main()`，工程不能再
定义另一个入口。CSP 保存命令行参数后调用 `gmp_base_entry()`，顺序为：

```text
gmp_csp_startup
  -> setup_peripheral
  -> ctl_init
  -> init
  -> gmp_csp_post_process
  -> gmp_csp_loop（重复：芯片 -> 外设输出 -> 电路 -> 外设采样/中断）
  -> gmp_csp_exit
```

`gmp_csp_startup()` 统一解析 `--no-pause`、`--realtime-priority`、
`--normal-priority`、`--no-realtime-priority`、`--profile`、`--build-info`
和 `--output <路径>`。工程在标准 C 链接的 `init()` 中通过 `command_line()`
读取结果，并注册构建信息和仿真配置。`gmp_csp_post_process()` 初始化被控对象并
启动文件、控制台两个服务线程；每次 `gmp_csp_loop()` 只推进一个仿真步。每个
完整仿真周期之后，
核心框架通过 `gmp_csp_should_exit()` 判断是否结束。`gmp_csp_exit()` 负责校验、
汇合线程、打印摘要、恢复优先级和按配置暂停。

CCTL CSP 同时定义 `SPECIFY_CSP_MANAGES_USER_MAINLOOP` 和
`SPECIFY_CSP_MANAGES_CTL_MAINLOOP`。被控对象步长通常远小于 MCU 后台任务周期，
因此 GMP 核心不会按被控对象步频自动调用两个 mainloop。工程的芯片模型应通过
`compute_budget_scheduler` 按 SDPE 配置的独立频率调用 `mainloop()` 和
`ctl_mainloop()`；它采用相位累加器，也支持 33 kHz 这类非整数分频。实时控制
入口 `gmp_base_ctl_step()` 不属于该预算，只能由外设模型在 ADC 转换完成并锁存
寄存器后，通过模拟 ADC ISR 同步调用。

CSP 自己实现 `gmp_csp_startup()`、`gmp_csp_post_process()`、`gmp_csp_loop()`、
`gmp_csp_exit()`、`gmp_csp_stuck_routine()` 和 `gmp_csp_not_implement()`，工程
不应覆盖这些函数。

## 标准仿真载体

`embedded_chip_simulation`、`peripheral_simulation` 和
`circuit_simulation` 分别描述芯片算力/后台任务、ADC/PWM/编码器等外设，以及
电气和机械被控对象。`simulation_system` 将它们按固定顺序组合：

```text
芯片后台任务预算
  -> 外设输出施加到电路
  -> 电路与机械模型步进
  -> 外设采样电路输出
  -> 必要时同步触发 ADC ISR/控制计算
```

初始化顺序为芯片、外设、电路，注销顺序相反。中断由外设事件产生，不由芯片
后台调度器轮询，也不能在主电路步进函数中直接调用控制器。

## 仿真运行时

`simulation_runtime::initialize(config, callbacks)` 校验并保存一次运行。配置必须
提供正数 `total_steps`、有限且大于零的 `plant_step_s`、非零 POD 记录大小、
输出文件名和记录格式化函数；环容量、批量长度、刷新周期和暂停策略应来自
SDPE，输出路径和优先级可由 CSP 命令行覆盖。

回调职责如下：

- `initialize`：GMP 已完成控制器和平台初始化后，再初始化被控对象；
- `step`：由每次 `gmp_csp_loop()` 推进一个数值状态；`step_range` 仅保留给阻塞
  便利接口；
- `finalize`：执行结束状态校验；
- `write_record`：在文件线程中格式化一个已经复制的 POD 记录；
- `print_summary`：由 `gmp_csp_exit()` 打印工程专属结果。

标准框架路径使用 `start()`、重复 `step()` 和 `finalize()`，不会在工程入口中
阻塞完成整段仿真。`run()` 是在调用线程执行同样序列的独立便利接口；只有文件
输出和控制台进度是工作线程。`finalize()` 负责汇合二者，并且可重复调用。

`interface_transfer(record, size)` 在热路径中把记录非阻塞复制到 SPSC 环。
记录必须可平凡复制，大小必须与配置完全一致。环满时返回 `false` 并丢弃新记录，
不会阻塞求解器。

`completed_steps()`、`buffered_records()`、`config()` 和 `summary()` 提供只读
状态；`print_summary(stream)` 打印通用性能/I/O 信息；工程打印完拓扑摘要后可
调用 `pause_if_requested(suppress)` 执行手动运行的退出暂停。

工程必须通过 `gmp_src_mgr` 选择 `csp|cctl`，不能在 CMake 中手工枚举 CSP 源文件。
