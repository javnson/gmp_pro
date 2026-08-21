# CCTL 主机仿真 CSP

[English](README.md) | **简体中文** | [完整接口说明](doc/README_CN.md)

该 CSP 提供可复用的直接 CCTL 主机仿真运行时。工程只负责控制器、外设映射、生成的主电路以及电机等具体拓扑；CSP 负责调度与数据观测基础设施。

`simulation_runtime` 对外提供 `initialize()`、`start()`、单步 `step()`、非阻塞 `interface_transfer()`、`run()` 和 `finalize()`。标准 GMP 生命周期由主线程在每次 `gmp_csp_loop()` 中推进一个仿真步，批量文件输出和控制台进度使用两个工作线程。仿真与文件线程之间使用 `cctl/dsa` 提供的预分配 SPSC 无锁记录环形缓冲区；缓冲区满时丢弃新观测记录而不阻塞数值求解器，最终 summary 会报告丢弃数量。

工程应通过 SDPE 管理时间步长、环形缓冲区容量、批量写入长度和退出暂停策略，再用生成宏填写 `simulation_config`。通过 `gmp_src_mgr` 的 `csp|cctl` 模块引入该 CSP，不应在 CMake 中手工枚举其源文件。

文件输出回调在文件线程执行，因此工程可以把 POD 仿真记录直接通过`interface_transfer()` 送入环形缓冲区，避免在高频仿真线程中执行文本格式化。`run()` 返回的 summary 包含步数、模拟/墙钟时间、实时倍率、写入字节数和丢弃记录数；`pause_if_requested()` 用于在工程打印完拓扑结果后执行退出暂停。

目前 CSP 已补齐标准的 `csp.config.h`、`csp.typedef.h`、`csp.typedef.hpp`、`csp.general.h` 和 `csp.general.hpp` 组合，并由 CSP 自己提供 `main()`，通过 `gmp_base_entry()` 进入工程。生命周期、仿真类型约束及全部公开函数见完整接口说明。

通用 `simulation_system` 将工程标准化为芯片、外设和电路三个载体。芯片载体可用
`compute_budget_scheduler` 按独立频率分配用户/后台代码算力；ADC 控制计算则仅由
外设载体的转换完成中断触发，不与用户代码频率绑定。
