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
  -> gmp_base_show_label（Logo 未禁用且平台已绑定输出设备时）
  -> ctl_init
  -> init（用户 user_main）
  -> gmp_csp_post_process -> csp_cctl_project_configure（工程仿真装配）
  -> gmp_csp_loop（重复：芯片 -> 外设输出 -> 电路 -> 外设采样/中断）
  -> gmp_csp_exit
```

`gmp_csp_startup()` 统一解析 `--no-pause`、`--realtime-priority`、
`--normal-priority`、`--no-realtime-priority`、`--profile`、`--build-info`、
`--viewer`、`--continuous`、`--duration <秒>`、`--supervised`、
`--wait-for-start`、`--headless` 和 `--output <路径>`。标准 `init()` 和
`mainloop()` 始终属于用户的
`user_main.c`，CCTL 工程不得覆盖。工程应实现固定 C 链接钩子
`csp_cctl_project_configure()`，CSP 在用户 `init()` 完成后的
`gmp_csp_post_process()` 中调用它；该钩子通过 `command_line()` 读取结果，
并注册构建信息和仿真配置。随后 `gmp_csp_post_process()` 初始化被控对象并
启动文件、控制台两个服务线程；每次 `gmp_csp_loop()` 只推进一个仿真步。每个
完整仿真周期之后，
核心框架通过 `gmp_csp_should_exit()` 判断是否结束。`gmp_csp_exit()` 负责校验、
汇合线程、打印摘要、恢复优先级和按配置暂停。

完全不带命令行参数时，工程只初始化到能够确定默认时长和输出文件的位置，随后
把进程控制权交给 `%GMP_PRO_LOCATION%` 下的 Viewer Manager。第一个进程不执行
数值步进便退出；Viewer 使用 `--supervised --wait-for-start --no-pause` 重启同一
可执行文件，因此不会递归启动 Viewer。`--headless` 可显式保留传统直接执行。

受管模式下 stdout 专用于逐行 JSON，GMP Logo 和用户日志改发 stderr。协议版本
1 输出 `ready`、周期 `status`、命令确认、`datalink` 字节消息和最终 `summary`，
接受 `start`、`pause`、`resume`、`stop`、`set_duration` 和 `datalink`。Data Link
消息的 `data` 字段使用 Base64，解码后是未经改写的标准 GMP 线协议字节；收发队列
各限制为 64 KiB，避免失去 Viewer 时无限增长。暂停只在完整电路步边界生效，时长
为 0 表示无限运行。C++ 端统一用 `nlohmann::json` 构建和解析消息，因此选择该
CSP 的工程必须使用 GMP vcpkg 工具链，执行
`find_package(nlohmann_json CONFIG REQUIRED)` 并链接
`nlohmann_json::nlohmann_json`。

`--continuous` 会忽略 `total_steps`，持续推进仿真，直到控制台输入 `q` 或 `Q`。
按键由控制台服务线程每 25 ms 非阻塞检测，不进入数值热路径。退出仍走标准
`gmp_csp_exit()`：调用模型 finalize、排空全部 CSV 队列、汇合服务线程并打印
摘要。交互式 Viewer 初始保持 Stop，只有用户在窗口中点击 Start 后才会启动受管
仿真。Stop 状态下受管进程已经完成初始化并写出各 CSV 表头，Viewer 因而可以在
第一个数值步之前列出并配置所有曲线。与 `--viewer` 同时使用时，Viewer 自动启用
20 Hz 刷新和默认 0.1 s
滚动时间窗。

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

Logo 是否编译由 `SPECIFY_GMP_LOGO_MODE`（或兼容宏
`SPECIFY_DISABLE_GMP_LOGO`）决定，但能否看到还取决于平台是否给
`default_debug_dev` 绑定了非空调试设备，以及 `gmp_hal_uart_send()` 是否真正
输出数据。CCTL CSP 在 `gmp_csp_startup()` 中统一绑定宿主控制台并将 GMP 打印
缓冲写到标准输出，工程不需要重复实现调试 UART。

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
提供正数 `total_steps`、有限且大于零的 `plant_step_s`，以及至少一个
`simulation_output_config`。每个输出独立定义 POD 记录大小、SPSC 环容量、批量
长度、CSV 路径、表头和格式化函数；原单记录配置保留为兼容入口。刷新周期和暂停策略应来自
SDPE，输出路径和优先级可由 CSP 命令行覆盖。

回调职责如下：

- `initialize`：GMP 已完成控制器和平台初始化后，再初始化被控对象；
- `step`：由每次 `gmp_csp_loop()` 推进一个数值状态；`step_range` 仅保留给阻塞
  便利接口；
- `service`：数值步进处于 Stop/Pause 时，在仿真主线程服务通信等外设工作；
- `finalize`：执行结束状态校验；
- `write_record`：在文件线程中格式化一个已经复制的 POD 记录；
- `print_summary`：由 `gmp_csp_exit()` 打印工程专属结果。

标准框架路径使用 `start()`、重复 `step()` 和 `finalize()`，不会在工程入口中
阻塞完成整段仿真。`run()` 是在调用线程执行同样序列的独立便利接口；只有文件
输出和控制台进度是工作线程。`finalize()` 负责汇合二者，并且可重复调用。

`interface_transfer(stream_index, record, size)` 在热路径中把记录非阻塞复制到
指定输出的 SPSC 环；不带 stream index 的重载写入第 0 个输出。
记录必须可平凡复制，大小必须与配置完全一致。环满时返回 `false` 并丢弃新记录，
不会阻塞求解器。所有输出仍由同一个文件线程轮询和批量写入；各流独立统计
queued/written/dropped/bytes。`--viewer` 会通过 `GMP_PRO_LOCATION` 找到 Viewer，
传入全部输出路径并启用 20 Hz 动态刷新。

`datalink_read()`/`datalink_write()` 是受管字节流的 C++ 接口，C 外设映射通常使用
`csp_cctl_datalink_read()`/`csp_cctl_datalink_write()`。CSP 不解析 Data Link 帧；
工程必须把接收字节送入 `gmp_dev_dl_push_str()`，并返回
`gmp_dev_dl_get_tx_hw_hdr/pld()` 给出的标准帧。这样 UART 目标、CCTL 和 Viewer 共用
同一个 Data Link 状态机、CRC、转义和设施分派器。

控制中断计数和仿真毫秒时基属于 CSP。ADC ISR 调用
`csp_cctl_notify_controller_interrupt()` 后再分派 `gmp_base_ctl_step()`；工程外设层
不得自行维护 controller tick 或实现 `gmp_base_get_system_tick()`。CSP 还提供
16 通道控制速率软件示波器 `csp_cctl_scope_write/read()`，供控制代码发布要写入
控制 CSV 的 `float` 变量。

`completed_steps()`、`buffered_records()`、`config()` 和 `summary()` 提供只读
状态；`print_summary(stream)` 打印通用性能/I/O 信息；工程打印完拓扑摘要后可
调用 `pause_if_requested(suppress)` 执行手动运行的退出暂停。

工程必须通过 `gmp_src_mgr` 选择 `csp|cctl`，不能在 CMake 中手工枚举 CSP 源文件。
