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
本工程使用 `Vref=3.3 V`、`N=12`。工程通过
`sdpe_mgr/private_hardware/inverter_3ph/mcs_pmsm_nt_cctl_inverter.json` 绑定
2136SINV 的私有仿真标定：电压分压为 `1/48`，电流灵敏度为
`5 mOhm * 15 = 0.075 V/A`，零电流偏置为 1.65 V。网表的六个运放反馈电阻为
30 kΩ，与全局 `gmp_3ph_2136sinv_dual` 的 15 倍参数一致；
`hw/validate_generated_model.py` 会在编译前从生成矩阵反算三相电流和母线电压
DC 增益，并与私有 SDPE 参数比较。该网表的物理 A/B/C 下桥采样网络分别标为 `VADC_IC/IB/IA`，因此手写
testbench 按实际接线重新排列后再写入控制器 IA/IB/IC 通道。

默认 Eigen 结果写入 `mcs_pmsm_nt_cctl.csv`，包含 PWM 比较值、三相电压电流、dq 电流、
转矩、负载、编码器计数和七路原始 ADC code。测试同时检查 ADC/ePWM/eQEP、
每载波一次 SOC、ADC 中断确认、三下桥采样窗口、桥臂无直通、数值有限性、
限流和 300 rpm 速度闭环。
大文件可直接用 `tools/cctl_studio/result_viewer/run_result_viewer.bat` 打开；该工具
按需后台加载列、保留极值降采样，并支持多张图分别选曲线及联动缩放。

## 运行时分层

`cctl/dsa/spsc_record_ring.hpp` 是与平台无关的预分配 SPSC 无锁定长记录环。
`csp/cctl` 在其上实现 `initialize`、`step`、`interface_transfer`、`run` 和
`finalize`，并启动仿真、文件输出、控制台进度三个线程。仿真线程只进行一次
非阻塞记录拷贝；32 MB 环满时丢弃新记录而不阻塞求解器。文件线程按 1 MB
批量格式化和写入，控制台线程每 1 s 更新进度、ETA、已完成仿真时间、瞬时
求解吞吐率（Mstep/s）、队列占用和丢弃数。最终摘要还会给出环形队列峰值及
文件线程实际用于格式化/写入的 `writer_busy` 时间；该时间发生在独立线程，
不会直接累加到求解器热路径。交互终端中的状态行和进度条会在
固定光标锚点原位刷新，不会不断追加新行；进度条会读取当前控制台可视宽度，
在扣除百分比后尽量撑满整行，并在窗口缩放后自动调整。输出重定向到文件或
CTest 时只打印最终状态，避免日志中出现控制序列和逐秒输出。

Windows 下由 SDPE 的 `CCTL_SIM_REALTIME_PRIORITY` 决定是否在仿真期间申请
`REALTIME_PRIORITY_CLASS`，当前默认开启。命令行 `--realtime-priority` 可强制
开启，`--normal-priority` 或 `--no-realtime-priority` 可关闭。申请结果会显示
在启动信息和最终摘要中；权限不足时自动使用普通优先级，仿真结束后恢复原
优先级。CTest 显式使用普通优先级，避免自动化任务影响同机其他进程。

`queue=0` 只表示打印瞬间 SPSC 环已经被文件线程取空；旁边的 `staged` 显示
文件线程私有批缓冲中的记录数。二者都不表示仿真线程同步写文件。仿真结束会
打印模拟时间/墙钟时间、实时倍率、步数、写入量、丢弃量以及电机
稳态结果。直接运行可执行文件时按 SDPE 默认执行 `system("@pause")`；自动化
测试使用 `--no-pause`。也可用 `--output <文件>` 覆盖 CSV 路径。传入
`--profile` 会稀疏采样外设、主电路、电机和维护逻辑的热路径耗时，并统计每次
控制 ISR 的平均耗时。

## 矩阵后端选择

无后缀目标 `mcs_pmsm_nt_cctl` 和 `build_test.bat` 均默认使用 Eigen。启动信息
会明确显示 `build=Release optimized=yes`；若从 IDE 误运行 Debug/`/Od` 版本，
程序会立即打印性能警告。Eigen 的小型固定维表达式在未优化构建下可能慢数十倍，
性能回归必须通过 `build_test.bat` 或显式的 CMake Release 配置运行。

Visual Studio 打开本目录时会读取 `CMakePresets.json`。配置选择器中
`Windows MSVC Release (recommended)` 是默认推荐的性能配置，`Windows MSVC
Debug` 保留真正的 `/Od` 调试语义；选择配置后再构建/启动
`mcs_pmsm_nt_cctl.exe`。Visual Studio 会记住上次选择，因此已经打开过本工程的
工作区可能仍显示旧的 `x64-Debug`，此时只需在配置选择器中改选一次 Release。
命令行可用 `mcs_pmsm_nt_cctl.exe --build-info` 瞬间确认配置，而不运行 4000 万步：
Release 应显示 `build=Release optimized=yes`，Debug 应显示
`build=Debug optimized=no`。对于 Visual Studio 多配置生成器，工具栏配置才是
有效选择，`CMAKE_BUILD_TYPE` 不参与选择。CMake 还会把 Eigen archive 复制到
所选配置的可执行文件目录，并将该目录设为 VS 调试工作目录。

电机模型仍支持 Euler、二阶中点 RK 和经典 RK4，通用模型默认 RK4。本工程的
`CCTL_SIM_PMSM_INTEGRATION_ORDER` 由 SDPE 管理；100 ns 步长相对于约毫秒级
电气时间常数足够小，因此默认取 1 阶 Euler，仍保持每个电路步更新一次电机，
没有引入多速率保持或额外一拍延迟。设为 2 或 4 可进行精度对照。

当前机器上 4 s、40,000,000 步、23 状态/729 拓扑的 Release Eigen 回归约为
9.4 s；直接输出到 `NUL` 的剖析运行约 9.11 s、4.39 Mstep/s。此前同一离散
耦合使用 RK4 时为 12.15 s。Euler 与 RK4 的末 50 ms 平均转速差约 0.003 rpm，
闭环回归全部通过。80,000 条、约 22.45 MB CSV 的文件线程忙碌时间约 0.57 s，
且与 9.11 s 求解并行，因此不是数百秒运行时间的原因。

fixed 能力没有删除：运行 `build_test.bat --with-fixed` 会额外生成 fixed 头、
以 `CCTL_BUILD_FIXED_BACKEND=ON` 构建 `mcs_pmsm_nt_cctl_fixed`，并执行独立
闭环测试。fixed 系数池继续使用 C++17 `constexpr` 常量初始化和可选 AVX2，
适合作为静态存储、无 Eigen 运行依赖以及后续嵌入式/FPGA 演进的显式后端；
本次同配置回归约为 12.6 s，因此默认仍使用 Eigen。

Eigen 归档只在 `PmsmCircuit` 构造时读取和校验，仿真步进期间没有文件 I/O。
schema v2 JSON 在写盘时已保存去重矩阵池并采用紧凑序列化；当前 23 状态、
729 拓扑模型约为 77.2 MB，运行矩阵池从 1,604,529 个逻辑系数压缩到 1,037,408 个，减少
35.35%。原内嵌 Eigen 头约
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

主电路生成脚本支持环境变量 `DISCRETIZATION_METHOD=forward_euler`、
`backward_euler` 或 `rk4`。RK4 对线性仿射模型预计算为单次矩阵推进，不增加
运行时阶段数；但本电路的 pF 级调理/寄生电容使系统高度刚性，100 ns 步长的
RK4 回归会在首步产生非有限值。因此工程默认保持后向欧拉，RK4 选项用于时间
常数较温和的拓扑或进一步减小步长后的精度对照。
