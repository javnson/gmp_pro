# GMP CLLLC / DAB 数字电源工程

[English](README.md) | **简体中文**

本工程面向 Dioscuri 双向隔离变换器，在 TMS320F280025C 硬件和 MATLAB/Simulink SIL 仿真中复用同一套控制源代码。正向功率传输采用变频 CLLLC 调制，反向功率传输采用谐振频率下的次级桥移相 DAB 调制。

## 调试层级

- `BUILD_LEVEL=1`：开环调制和底层 PWM 验证。
- `BUILD_LEVEL=2`：电流闭环。
- `BUILD_LEVEL=3`：电压环与电流环并行竞争控制。

工程的标称谐振频率为 100 kHz，运行频率范围为 75–150 kHz。控制中断通常每两个 PWM 周期执行一次，GMP 系统节拍由独立的 CPU Timer0 以 1 kHz 提供。

## 目录

- `src/`：硬件与 SIL 共用的控制算法。
- `project/`：F280025C 和 Simulink 工程。
- `doc/commissioning.md`：英文调试、映射与验证说明。

PCB 信号与 ePWM 编号的交叉映射是硬件设计的一部分，移植或整理代码时不要按编号直觉重新排列。详细电气参数、资源映射和构建方法请参阅[英文完整说明](README.md)。
