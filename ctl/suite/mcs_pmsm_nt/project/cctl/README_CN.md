# mcs_pmsm_nt 直接 CCTL 联合仿真

本工程把现有 `mcs_pmsm_nt` 控制器、MNA 生成的三相主电路和
`cctl::pmsm_cs` 电流源型永磁同步电机直接链接到同一个进程中。它不经过
Windows/Simulink 网络通信层，因此每一个控制周期都严格执行
`ADC/eQEP -> ctl_input_callback -> ctl_dispatch -> ctl_output_callback -> ePWM`。

运行 `build_test.bat` 即可完成 SDPE、GMP 源文件、CMake 接口、主电路代码、
编译和闭环回归这七个阶段。脚本只从环境变量 `GMP_PRO_LOCATION` 定位 GMP；
生成物位于系统临时目录 `%TEMP%\gmp_mcs_pmsm_nt_cctl_build`。

工程配置遵循两套 GMP 管理机制：

- `sdpe_mgr/sdpe_requirement.json` 是硬件与仿真参数的唯一配置源，生成
  `sdpe_mgr/ctrl_settings.h`。ADC 分辨率/参考电压、eQEP 线数、ePWM 时钟/周期/
  死区、仿真时长、负载、输出缓冲区和暂停策略均由这里管理；不要手工修改生成头。
- `gmp_src_mgr/gmp_framework_config.json` 选择控制器依赖、`cctl|dsa` 和
  `csp|cctl`。源管理器生成本地扁平源以及 `gmp_config.cmake`，项目 CMake 只
  `include` 该文件，不再手工枚举 GMP 库源文件，也不再借用 `simulate` 工程。
  CMake 生成器直接从已选模块及其依赖闭包汇总 `inc_dirs`，`src_only` 模式不
  依赖可能过期的 `gmp_compiler_includes.txt`。
- `hw/PMSM.CIR` 是项目主电路。`hw/generate_code.bat` 只解析一次网表，将 JSON
  写入 `hw/generated`，再从同一数据分别生成
  `hw/generated/fixed/pmsmcircuit.hpp` 和
  `hw/generated/eigen/pmsmcircuit.hpp`。Eigen 由 GMP 安装程序维护的 vcpkg
  环境提供，不引用已弃用的 `third_party` 副本。

仿真使用 100 ns 电路/电机步长和 50 us（20 kHz）控制器步长，
`cctl::fixed_rate_divider` 保证控制器每 500 个快步长运行一次。TI 风格外设模型
位于 `cctl/peripheral_if`：12/16 位右对齐 ADC、中心对齐互补 ePWM（含
DBRED/DBFED 死区）以及带圈数信息的 eQEP。负载转矩通过
`pmsm_cs_input::load_torque_nm` 输入。4 s 回归测试在 0.5 s 后施加 0.02 N·m，
留出足够时间检查现有速度 PI 对 300 rpm 指令的稳态响应。

两种后端的结果 CSV 分别为 `mcs_pmsm_nt_cctl_fixed.csv` 和
`mcs_pmsm_nt_cctl_eigen.csv`，包含 PWM 比较值、三相电压电流、dq 电流、
转矩、负载和编码器计数。测试同时检查 ADC/ePWM/eQEP、500:1 分频、桥臂
无直通、数值有限性、限流和 300 rpm 速度闭环。

## 运行时分层

`cctl/dsa/spsc_record_ring.hpp` 是与平台无关的预分配 SPSC 无锁定长记录环。
`csp/cctl` 在其上实现 `initialize`、`step`、`interface_transfer`、`run` 和
`finalize`，并启动仿真、文件输出、控制台进度三个线程。仿真线程只进行一次
非阻塞记录拷贝；32 MB 环满时丢弃新记录而不阻塞求解器。文件线程按 1 MB
批量格式化和写入，控制台线程每 1 s 更新进度、ETA、队列占用和丢弃数。
控制台先显示 `GMP CCTL Motor Simulation Kit`，随后显示总仿真时间、步长、
总步数和矩阵后端，再使用 64 字符的 `=`/`>` 进度条显示运行状态。

仿真结束会打印模拟时间/墙钟时间、实时倍率、步数、写入量、丢弃量以及电机
稳态结果。直接运行可执行文件时按 SDPE 默认执行 `system("@pause")`；自动化
测试使用 `--no-pause`。也可用 `--output <文件>` 覆盖 CSV 路径。

## fixed 与 Eigen 性能对比

`build_test.bat` 会构建并回归两个独立可执行文件，避免同一进程中的 GMP C
全局状态相互影响。两个目标共享手写 testbench、SDPE 参数、网表 JSON、输入、
4000 万个求解步和物理结果检查，仅矩阵后端不同。CTest 结果分别写入
`mcs_pmsm_nt_cctl_fixed.csv` 和 `mcs_pmsm_nt_cctl_eigen.csv`，第三项测试再以
`1e-8` 容差逐元素比较全部 80,000 行。当前最大差为 `4.20e-10`。

构建后运行 `benchmark_backends.bat [重复次数]` 可进行交替顺序的全系统基准；
默认重复 3 次，并保留 CSV 格式化开销但将数据发送到 `NUL`，最后报告均值、
中位数、最小值、fixed/Eigen 比值以及最终物理量漂移。当前开发机三轮结果为
fixed 9.067892 s、Eigen 8.059164 s，Eigen 快 1.1252 倍。因此桌面联合仿真
推荐 Eigen；fixed 仍保留为静态存储、无 Eigen 运行依赖以及后续嵌入式/FPGA
代码演进的基线。具体数值需在目标计算机重新运行基准确认。

主电路和电机并不是可并行的独立任务：第 `n` 步主电路使用第 `n-1` 步电机
电流得到电压，第 `n` 步电机随即使用该电压得到下一步所需电流。控制器还要
在每 500 步的边界先读取反馈再更新门极。保持当前离散语义时，依赖链为
`controller -> circuit[n] -> motor[n] -> circuit[n+1]`；拆成三个计算线程只会
把同样的串行链改成每 100 ns 跨线程握手。若允许一拍延迟可采用并行 Jacobi
协同仿真，但那是另一种数值模型，不能作为当前回归的透明加速。现阶段保留
单一数值线程，同时让文件线程和控制台线程与它真正并行。
