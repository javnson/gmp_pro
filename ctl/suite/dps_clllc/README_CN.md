# GMP CLLLC / DAB 数字电源工程

[English](README.md) | **简体中文**

本工程面向 Dioscuri 双向隔离变换器，在 TMS320F280025C 硬件和 MATLAB/Simulink SIL 仿真中复用同一套控制源代码。调制指令同时改变开关频率与原副边移相角：指令幅值越大，频率由 150 kHz 向 75 kHz 移动且移相增大；符号表示期望的能量流向。

## 调试层级

- `BUILD_LEVEL=1`：开环调制和底层 PWM 验证。
- `BUILD_LEVEL=2`：原边/输入电流闭环。
- `BUILD_LEVEL=3`：电压闭环。
- `BUILD_LEVEL=4`：电压环与电流环并行竞争的 CC/CV 控制。

工程的标称谐振频率为 100 kHz，运行频率范围为 75–150 kHz。控制中断通常每两个 PWM 周期执行一次，GMP 系统节拍由独立的 CPU Timer0 以 1 kHz 提供。

## 目录

- `src/`：硬件与 SIL 共用的控制算法。
- `project/`：F280025C 和 Simulink 工程。
- `sdpe_general/`：谐振腔、TMCS1133B5A、AMC1311、标幺值、控制目标和环路参数等公用配置。
- `project/f280025c_dioscuri/src/sdpe_mgr/`：BUILD_LEVEL、PWM 排序、ADC、CPU Timer、UART 与 GPIO 等平台配置。
- `doc/commissioning.md`：英文调试、映射与验证说明。

Dioscuri 共引出 6 对 PWM：原边 `PRI_P/PRI_N` 分别使用 EPWM4/EPWM7，副边 `SEC_P/SEC_N` 分别使用 EPWM1/EPWM2，EPWM3/EPWM5 预留给 FSBB。SDPE 以 `DIOSCURI_PWM_PAIR1_BASE` 至 `DIOSCURI_PWM_PAIR6_BASE` 输出资源，并允许四个 CLLLC 逻辑桥臂重新选排，以便不改控制算法即可校正物理电流方向。四个桥臂必须使用不同的 PWM 对，同步主机必须位于这四对中。

在 `BUILD_LEVEL=1` 下，可通过 Datalink 在线写入 `g_modulation_target_user`，直接改变有符号的变频/移相目标；`g_modulation_command` 保留为只读的调制器实际指令。固定的 `CLLLC_OPEN_LOOP_COMMAND_PU` 宏已经移除。

`GPIO_ENABLE` 为 GPIO41，连接 SN74LVC8T245PWR 的低有效 OE#。SysConfig 默认输出高电平；软件先屏蔽缓冲器并配置 PWM，清除四路 Trip 后才拉低使能，停机时则先拉高 OE# 再强制 Trip。

生成时必须先运行 `sdpe_general/sdpe_generate.bat`，再运行硬件工程 `src/sdpe_mgr/sdpe_generate.bat`，最后执行 `src/gmp_src_mgr/gmp_generate_src.bat` 并在 CCS 中构建 Debug 配置。详细电气参数、资源映射和构建方法请参阅[英文完整说明](README.md)。
