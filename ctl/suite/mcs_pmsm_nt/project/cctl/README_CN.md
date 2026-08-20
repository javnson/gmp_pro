# mcs_pmsm_nt 直接 CCTL 联合仿真

本工程把现有 `mcs_pmsm_nt` 控制器、MNA 生成的三相主电路和
`cctl::pmsm_cs` 电流源型永磁同步电机直接链接到同一个进程中。它不经过
Windows/Simulink 网络通信层，因此每一个控制周期都严格执行
`ePWM SOC -> ADC 锁存/中断 -> ctl_input_callback -> ctl_dispatch ->
ctl_output_callback -> ePWM 更新`。

运行 `build_test.bat` 即可完成 SDPE、GMP 源文件、CMake 接口、主电路代码、
编译和闭环回归这七个阶段。脚本只从环境变量 `GMP_PRO_LOCATION` 定位 GMP；
生成物位于系统临时目录 `%TEMP%\gmp_mcs_pmsm_nt_cctl_build`。

工程配置遵循两套 GMP 管理机制：

- `sdpe_mgr/sdpe_requirement.json` 是硬件与仿真参数的唯一配置源，生成
  `sdpe_mgr/ctrl_settings.h`。ADC 分辨率/参考电压、eQEP 线数、ePWM 时钟/周期/
  死区、ADC 触发比较值、仿真时长、负载、输出缓冲区和暂停策略均由这里管理；
  不要手工修改生成头。
- `gmp_src_mgr/gmp_framework_config.json` 选择控制器依赖、`cctl|dsa` 和
  `csp|cctl`。源管理器生成本地扁平源以及 `gmp_config.cmake`，项目 CMake 只
  `include` 该文件，不再手工枚举 GMP 库源文件，也不再借用 `simulate` 工程。
  CMake 生成器直接从已选模块及其依赖闭包汇总 `inc_dirs`，`src_only` 模式不
  依赖可能过期的 `gmp_compiler_includes.txt`。
- `hw/PMSM.CIR` 是项目主电路。`hw/generate_code.bat` 只解析一次网表，将 JSON
  写入 `hw/generated`，默认生成 `hw/generated/eigen/pmsmcircuit.hpp` 和
  `pmsmcircuit.archive`。头文件保存固定维结构与加载逻辑，归档保存去重后的
  Eigen 矩阵池；CMake 会把归档复制到可执行文件目录。
  将环境变量 `MATRIX_BACKEND` 显式设为 `fixed` 或 `all` 时，仍可生成
  `hw/generated/fixed/pmsmcircuit.hpp`。Eigen 由 GMP 安装程序维护的 vcpkg
  环境提供，不引用已弃用的 `third_party` 副本。

仿真使用 100 ns 电路/电机步长。主 ePWM 在每个 50 us（20 kHz）中心对齐载波
的 CMPB 上数事件产生一次 ADC SOC；当前 SDPE 值为 250 TBCLK，testbench 会
断言该事件落在三个下桥同时导通的 low-side 采样窗口。ADC 在 SOC 到来前只
更新模拟输入，触发时同时锁存七路结果、置中断 pending，并立即执行控制主中断。
TI 风格外设模型位于 `cctl/peripheral_if`：带 SOC/中断握手的 12/16 位右对齐
ADC、带比较事件触发输出的中心对齐互补 ePWM（含 DBRED/DBFED 死区），以及
带圈数信息的 eQEP。负载转矩通过
`pmsm_cs_input::load_torque_nm` 输入。4 s 回归测试在 0.5 s 后施加 0.02 N·m，
留出足够时间检查现有速度 PI 对 300 rpm 指令的稳态响应。

主电路网表公开 `VADC_VDC/VADC_VA/VADC_VB/VADC_VC/VADC_IA/VADC_IB/VADC_IC`
七个最终调理电压。MNA JSON 为这些探针增加 `role=adc_sample_voltage` 和
`adc_channel` 元数据。ADC 量化为
`floor(clamp(VADC, 0, Vref) / Vref * 2^N)`，并在满量程饱和到 `2^N-1`；
本工程使用 `Vref=3.3 V`、`N=12`。SDPE 的控制器标定与网表实际模拟前端一致：
电压分压为 `1/48`，电流灵敏度为 `5 mOhm * 11 = 0.055 V/A`，零电流偏置为
1.65 V。该网表的物理 A/B/C 下桥采样网络分别标为 `VADC_IC/IB/IA`，因此手写
testbench 按实际接线重新排列后再写入控制器 IA/IB/IC 通道。

默认 Eigen 结果写入 `mcs_pmsm_nt_cctl.csv`，包含 PWM 比较值、三相电压电流、dq 电流、
转矩、负载、编码器计数和七路原始 ADC code。测试同时检查 ADC/ePWM/eQEP、
每载波一次 SOC、ADC 中断确认、三下桥采样窗口、桥臂无直通、数值有限性、
限流和 300 rpm 速度闭环。

## 运行时分层

`cctl/dsa/spsc_record_ring.hpp` 是与平台无关的预分配 SPSC 无锁定长记录环。
`csp/cctl` 在其上实现 `initialize`、`step`、`interface_transfer`、`run` 和
`finalize`，并启动仿真、文件输出、控制台进度三个线程。仿真线程只进行一次
非阻塞记录拷贝；32 MB 环满时丢弃新记录而不阻塞求解器。文件线程按 1 MB
批量格式化和写入，控制台线程每 1 s 更新进度、ETA、已完成仿真时间、瞬时
求解吞吐率（Mstep/s）、队列占用和丢弃数。交互终端中的状态行和进度条会在
固定光标锚点原位刷新，不会不断追加新行；进度条会读取当前控制台可视宽度，
在扣除百分比后尽量撑满整行，并在窗口缩放后自动调整。输出重定向到文件或
CTest 时只打印最终状态，避免日志中出现控制序列和逐秒输出。

Windows 下由 SDPE 的 `CCTL_SIM_REALTIME_PRIORITY` 决定是否在仿真期间申请
`REALTIME_PRIORITY_CLASS`，当前默认开启。命令行 `--realtime-priority` 可强制
开启，`--normal-priority` 或 `--no-realtime-priority` 可关闭。申请结果会显示
在启动信息和最终摘要中；权限不足时自动使用普通优先级，仿真结束后恢复原
优先级。CTest 显式使用普通优先级，避免自动化任务影响同机其他进程。

仿真结束会打印模拟时间/墙钟时间、实时倍率、步数、写入量、丢弃量以及电机
稳态结果。直接运行可执行文件时按 SDPE 默认执行 `system("@pause")`；自动化
测试使用 `--no-pause`。也可用 `--output <文件>` 覆盖 CSV 路径。

## 矩阵后端选择

无后缀目标 `mcs_pmsm_nt_cctl` 和 `build_test.bat` 均默认使用 Eigen。此前对新
23 状态、729 拓扑网表的三轮普通优先级测量为 Eigen 16.141034 s、fixed
22.135849 s，因此不再让日常生成、编译和回归承担 fixed 的额外成本。

fixed 能力没有删除：运行 `build_test.bat --with-fixed` 会额外生成 fixed 头、
以 `CCTL_BUILD_FIXED_BACKEND=ON` 构建 `mcs_pmsm_nt_cctl_fixed`，并执行独立
闭环测试。fixed 系数池继续使用 C++17 `constexpr` 常量初始化和可选 AVX2，
适合作为静态存储、无 Eigen 运行依赖以及后续嵌入式/FPGA 演进的显式后端。

Eigen 归档只在 `PmsmCircuit` 构造时读取和校验，仿真步进期间没有文件 I/O。
当前 23 状态、729 拓扑模型的 JSON 为 181,466,029 字节；原内嵌 Eigen 头约
24.66 MB，拆分后头文件约 23 KB、归档约 8.33 MB。归档与 JSON 都是可再生文件，
不纳入 Git；发布可执行程序时必须把 `pmsmcircuit.archive` 放在工作目录，或向
`PmsmCircuit` 构造函数传入明确路径。

主电路和电机并不是可并行的独立任务：第 `n` 步主电路使用第 `n-1` 步电机
电流得到电压，第 `n` 步电机随即使用该电压得到下一步所需电流。控制器还要
在 ePWM SOC/ADC 中断边界先读取反馈再更新门极。保持当前离散语义时，依赖链为
`controller -> circuit[n] -> motor[n] -> circuit[n+1]`；拆成三个计算线程只会
把同样的串行链改成每 100 ns 跨线程握手。若允许一拍延迟可采用并行 Jacobi
协同仿真，但那是另一种数值模型，不能作为当前回归的透明加速。现阶段保留
单一数值线程，同时让文件线程和控制台线程与它真正并行。
