# mcs_pmsm_nt 直接 CCTL 联合仿真

本工程把现有 `mcs_pmsm_nt` 控制器、MNA 生成的三相主电路和
`cctl::pmsm_cs` 电流源型永磁同步电机直接链接到同一个进程中。它不经过
Windows/Simulink 网络通信层，因此每一个控制周期都严格执行
`ePWM SOC -> ADC 锁存/中断 -> ctl_input_callback -> ctl_dispatch ->
ctl_output_callback -> ePWM 更新`。

## 可维护的运行边界

工程选择的 `csp|cctl` 模块拥有唯一的可执行程序 `main()`，并从
`gmp_base_entry()` 进入。标准 GMP 顺序会先执行 `setup_peripheral()`、
`ctl_init()` 和 `src/user_main.c` 中用户实现的 `init()`。随后 CSP 从
`gmp_csp_post_process()` 调用工程固定实现的 `csp_cctl_project_configure()`，由
该钩子注册构建信息、持久拓扑和回调。CSP 再启动服务，在每次
`gmp_csp_loop()` 中推进
一个被控对象步，并在 `gmp_csp_exit()` 统一注销。工程不得再定义第二个进程入口，
不得覆盖用户 `init()`/`mainloop()`，也不应在被控对象回调中重复初始化控制器。

工程私有的 `xplt/mcu_simulation.hpp/.cpp` 聚合七路 ADC 输入、三路互补 ePWM、
ADC SOC/中断分发和 eQEP。C 兼容的 `xplt.peripheral.*` 只保存控制器可见的
寄存器、标度通道以及模拟 ADC ISR。该 ISR 是工程中唯一调用
`gmp_base_ctl_step()` 的位置：ePWM SOC 触发 ADC 并锁存 ADC、编码器寄存器后才
进入 ISR，不会从被控对象 mainloop 调用控制步。`xplt.ctl_interface.h` 负责控制器
回调映射，并把状态机的输出使能/禁用连接到 CSP。通用 TI 风格外设原语仍位于
`cctl/component/control_peripheral`。

运行期的 MCU 聚合接口只分成 `control_outputs()` 和 `control_inputs()`：前者
采样 PWM/SOC 输出并送给主电路，后者一次性接收 ADC 调理电压和转子位置，并在
SOC 到来时完成 ADC 锁存、结果寄存器搬运、eQEP 搬运、ISR、PWM 寄存器更新和
中断确认。工程不逐个操作 ADC/PWM/QEP 成员。通用外设通过静态 `make()` 接收
SDPE 形参；配置对象是只读值对象。构造函数只创建一次配置，`initialize()` 仅
复位已有外设状态。ADC 内部保存 ISR 函数指针，并保证结果搬运发生在 ISR 之前。

芯片后台代码与控制 ISR 使用两套独立调度。SDPE 的
`CCTL_SIM_USER_CODE_FREQUENCY_HZ` 当前为 33 kHz；芯片算力调度器按该频率调用
用户 `mainloop()` 和控制器后台 `ctl_mainloop()`，不再采用“每次控制中断执行
若干次用户代码”的旧模型。ADC 仍以 20 kHz 产生控制中断并独占
`gmp_base_ctl_step()`。因此 4 s 回归应得到 132000 次用户/后台调用和 80000 次
控制计算，两者互不绑定。

控制器计算保持 CSP 默认的 `ctrl_gt=float`，外设和电机模型使用
`sim_real_gt=double`。完整的数值类型与生命周期约定见
`csp/cctl/doc/README_CN.md`。

运行 `build_test.bat` 即可完成 SDPE、GMP 源文件、CMake 接口、主电路代码、
编译和闭环回归这七个阶段。脚本只从环境变量 `GMP_PRO_LOCATION` 定位 GMP；
生成物位于 GMP 仓库局部目录
`%GMP_PRO_LOCATION%\tmp\cctl\suite\mcs_pmsm_nt\cctl_build`。

工程配置遵循两套 GMP 管理机制：

- `sdpe_mgr/sdpe_requirement.json` 是硬件与仿真参数的唯一配置源，生成
  `sdpe_mgr/ctrl_settings.h`。ADC 分辨率/参考电压、eQEP 线数、ePWM 时钟/周期/
  死区、ADC 触发比较值、仿真时长、负载、输出缓冲区和暂停策略均由这里管理；
  不要手工修改生成头。
- `gmp_src_mgr/gmp_framework_config.json` 选择控制器依赖、`csp|cctl`、
  `cctl|component|circuit_model` 和 `cctl|component|control_peripheral`；
  依赖闭包还会带入 CCTL 数值求解器和 DSA。源管理器生成本地扁平源以及
  `gmp_config.cmake`，项目 CMake 只
  `include` 该文件，不再手工枚举 GMP 库源文件，也不再借用 `simulate` 工程。
  CMake 生成器直接从已选模块及其依赖闭包汇总 `inc_dirs`，`src_only` 模式不
  依赖可能过期的 `gmp_compiler_includes.txt`。
- `hw/PMSM.CIR` 是项目主电路。`hw/generate_code.bat` 只解析一次网表，将 JSON
  写入 `hw/generated`，默认生成 `hw/generated/eigen/pmsmcircuit.hpp` 和
  `pmsmcircuit.archive`。头文件保存固定维结构与加载逻辑，归档保存去重后的
  Eigen 矩阵池；CMake 会把归档复制到可执行文件目录。
  将环境变量 `MATRIX_BACKEND` 显式设为 `fp` 或 `all` 时，还会生成定点模块
  `hw/generated/fp/pmsmcircuit_fp.hpp`，类名为 `PmsmCircuitFp`。Eigen 由 GMP 安装程序维护的 vcpkg
  环境提供，不引用已弃用的 `third_party` 副本。工程 `vcpkg.json` 同时声明
  `eigen3` 和 `nlohmann-json`；Visual Studio 文件夹模式选择
  `windows-msvc-release` 或 `windows-msvc-debug` Preset 后，会经过 GMP 的
  vcpkg 包装 toolchain。私有安装只读取安装阶段准备好的共享包，经典安装才由
  系统 vcpkg 按 manifest 自动恢复。

仿真使用 100 ns 电路/电机步长。主 ePWM 在每个 50 us（20 kHz）中心对齐载波
的 CMPB 上数事件产生一次 ADC SOC；当前 SDPE 值为 250 TBCLK，testbench 会
断言该事件落在三个下桥同时导通的 low-side 采样窗口。ADC 在 SOC 到来前只
更新模拟输入，触发时同时锁存七路结果、置中断 pending，并立即执行控制主中断。
TI 风格外设模型位于 `cctl/component/control_peripheral`：带 SOC/中断握手的 12/16 位右对齐
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
2136SINV 的私有仿真标定：电压分压为 `1/48`，当前网表的电流灵敏度为
`5 mOhm * 11 = 0.055 V/A`，零电流偏置为 1.65 V。网表每相使用 22 kΩ
反馈和两段串联的 1 kΩ 输入电阻，因此闭环有效增益为 11；
`hw/validate_generated_model.py` 会在编译前从生成矩阵反算三相电流和母线电压
DC 增益，并与私有 SDPE 参数比较，同时检查三相采样交叉耦合和
A→PWM1/2、B→PWM3/4、C→PWM5/6 的桥臂路由。物理 A/B/C 下桥采样网络
现在分别直接对应 `VADC_IA/IB/IC`，testbench 不再交换 A/C 通道。

默认 Eigen 结果写入 `mcs_pmsm_nt_cctl.csv`，包含 PWM 比较值、三相电压电流、dq 电流、
转矩、负载、编码器计数和七路原始 ADC code。测试同时检查 ADC/ePWM/eQEP、
每载波一次 SOC、ADC 中断确认、三下桥采样窗口、桥臂无直通、数值有限性、
限流和 300 rpm 速度闭环。
大文件可直接用 `tools/cctl_studio/result_viewer/run_result_viewer.bat` 打开；该工具
按需后台加载列、保留极值降采样，并支持多张图分别选曲线及联动缩放。

## 运行时分层

`cctl/dsa/spsc_record_ring.hpp` 是与平台无关的预分配 SPSC 无锁定长记录环。
`csp/cctl` 在其上实现 `initialize`、`start`、`step`、`interface_transfer`、
`run` 和 `finalize`。标准 GMP 主线程执行仿真，文件输出和控制台进度由两个
服务线程完成。仿真热路径只进行一次
非阻塞记录拷贝；32 MB 环满时丢弃新记录而不阻塞求解器。文件线程按 1 MB
批量格式化和写入，控制台线程每 1 s 更新进度、ETA、已完成仿真时间、瞬时
求解吞吐率（Mstep/s）、队列占用和丢弃数。最终摘要还会给出环形队列峰值及
文件线程实际用于格式化/写入的 `writer_busy` 时间；该时间发生在独立线程，
不会直接累加到求解器热路径。交互终端中的状态行和进度条会在
固定光标锚点原位刷新，不会不断追加新行；进度条会读取当前控制台可视宽度，
在扣除百分比后尽量撑满整行，并在窗口缩放后自动调整。输出重定向到文件或
CTest 时只打印最终状态，避免日志中出现控制序列和逐秒输出。

本工程输出两个独立采样率的文件：`*_circuit.csv` 按 SDPE 参数
`CCTL_SIM_CIRCUIT_RECORD_FREQUENCY_HZ`（默认 100 kHz）记录电路与电机量；
`*_control.csv` 只在 ADC 中断完成后记录 PWM、ADC、编码器和 16 个
`scope_00..scope_15` 控制软件示波器通道。两个流使用独立 SPSC 环，但仍由同一个
文件线程写盘。运行可执行文件时增加 `--viewer` 可自动加载两个文件并进入动态刷新。

连续观察可运行 `mcs_pmsm_nt_cctl.exe --continuous --viewer`。CSP 会忽略 4 s
有限回归时长，持续仿真直到控制台输入 `q`；Viewer 同时以 20 Hz 刷新并默认显示
最新 0.1 s 时间窗。退出后仍会完整排空两个 CSV 队列并打印 PASS/FAIL 摘要。

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

无参数启动 `mcs_pmsm_nt_cctl.exe` 是新的交互入口：CCTL CSP 校验
`GMP_PRO_LOCATION`，使用 GMP 私有 Python 打开 Simulation Viewer Manager，
启动器实例随后退出，再由 Viewer 通过 JSON 受管协议重启仿真器。需要原有有限
时长命令行运行时传入 `--headless`。工程 CMake 从与 Eigen 相同的 GMP vcpkg
安装中链接 CSP 所需的 `nlohmann_json::nlohmann_json`。

Viewer 的 `Data Link` 页通过同一受管进程连接本工程的标准 Data Link 服务。
工程 SDPE 定义 `ENABLE_GMP_DL_PIL_SERVER` 和 `GMP_PIL_DL_BASE_COMMAND=0x10`，
`gmp_src_mgr` 选择 `dev|datalink|pil`；`xplt.peripheral.cpp` 把 Viewer 原始字节送入
`user_main.c` 已有的 Data Link 状态机，并返回其标准线协议帧。INFO v3 当前报告
Echo、PIL、Tunable、Memory 四个设施。该 Server 开关与硬件用
`ENABLE_GMP_DL_PIL_SIM` 分离，因此有限/连续 CCTL 闭环仍由仿真 ADC ISR 驱动，
而 Stop/Pause 状态仍能执行发现、在线参数访问和 PIL 单步。

## 矩阵后端选择

无后缀目标 `mcs_pmsm_nt_cctl` 和 `build_test.bat` 均默认使用 Eigen。启动信息
会明确显示 `build=Release optimized=yes`；若从 IDE 误运行 Debug/`/Od` 版本，
程序会立即打印性能警告。Eigen 的小型固定维表达式在未优化构建下可能慢数十倍，
性能回归必须通过 `build_test.bat` 或显式的 CMake Release 配置运行。

Visual Studio 打开本目录时会读取 `CMakePresets.json`。选择
`Windows MSVC Release (recommended)` 使用原有浮点模块；选择
`Windows MSVC Release - Fixed Point (_fp)` 使用 `pmsmcircuit_fp`。对应启动程序
分别是 `mcs_pmsm_nt_cctl.exe` 和 `mcs_pmsm_nt_cctl_fp.exe`，目标选择器中也会
同时显示两者。`Windows MSVC Debug` 保留真正的 `/Od` 调试语义。Visual Studio
会记住上次选择，因此已经打开过本工程的
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

命令行定点回归仍为显式选择：运行 `build_test.bat --with-fp` 会同时生成两种模块，
以 `CCTL_BUILD_FP_BACKEND=ON` 构建 `mcs_pmsm_nt_cctl_fp`，并执行独立闭环测试。
`_fp` 模块使用 Q8.24 状态/输入/信号、各矩阵族独立系数 Q 格式和有符号 64 位
混合 Q 累加器；量程约定为直流母线及电机电流端口 8 V/A、调理源端口 4 V、
物理电路信号 16。旧参数 `--with-fixed` 保留为 `--with-fp` 的兼容别名。

已验证的 4 s 回归中，浮点与 `_fp` 在末 50 ms 的平均转速分别为
269.4495 rpm 和 269.4412 rpm，差约 0.0083 rpm。`_fp` 在主机上更慢是预期行为，
因为它是可移植的 FPGA/HLS 算术参考，而不是面向 AVX 的 Eigen 优化路径。

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
GMP 数值主线程，同时让文件线程和控制台线程与它真正并行。

主电路生成脚本支持环境变量 `DISCRETIZATION_METHOD=forward_euler`、
`backward_euler` 或 `rk4`。RK4 对线性仿射模型预计算为单次矩阵推进，不增加
运行时阶段数；但本电路的 pF 级调理/寄生电容使系统高度刚性，100 ns 步长的
RK4 回归会在首步产生非有限值。因此工程默认保持后向欧拉，RK4 选项用于时间
常数较温和的拓扑或进一步减小步长后的精度对照。
