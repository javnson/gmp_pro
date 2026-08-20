# PMSM 矢量控制模板

[English](README.md) | **简体中文**

这是 GMP 当前推荐的永磁同步电机矢量控制模板。C2000、STM32 与 PC 仿真目标共用同一套控制器源码，并通过逐级提高 `BUILD_LEVEL`，从外设联调逐步验证到闭环运动控制。

## 主要能力

- 磁场定向电流环与运动控制；
- 调制、编码器与速度处理、斜坡和 ADC 标定；
- CiA 402 状态机与 GMP Data Link；
- 公共 SDPE 控制配置和目标专用外设映射。

仓库当前包含以下目标目录：`cctl`、`f280039c_Iris_node`、`f280049c`、`f29h85x_lp_3phgan`、`simulate`、`stm32f405`、`stm32g431` 和 `stm32g474_hrtim`。其中 `cctl` 是不经过网络的进程内联合仿真目标，将 MNA 三相主电路、电流源 PMSM、TI 风格 ADC/ePWM/eQEP 和公共控制器以 500:1 多速率调度连接起来；运行方法见 [`project/cctl/README_CN.md`](project/cctl/README_CN.md)。F29H85x 目标要求 CCS 21 或更高版本，使用 `GMP-Core-C29x` Product，并通过两层 SDPE 分别绑定 LaunchPad 与 BOOSTXL-3PHGANINV 资源。新 PMSM 应用通常应从本套件开始，并保留公共 `src/` 与目标 `project/` 分层。

控制器公共参数位于 `sdpe_general/sdpe_requirement.json`，目标板参数位于对应 `project/<target>/sdpe_mgr/sdpe_requirement.json`。修改 JSON 后运行相应目录的 `sdpe_generate.bat`；不要手工修改生成的设置头文件。

上电调试必须从最低构建级开始。启用电流、速度或位置闭环前，先确认电流偏置、相序、编码器方向、PWM 极性和保护动作。参数辨识或闭环调试都可能使电机带电或旋转。

## DQ-PI 与 DQ-LADRC1 同带宽仿真记录（2026-08-08）

FOC 电流环默认使用 DQ-PI。在 `foc_core.h` 中取消注释 `#define ENABLE_FOC_LADRC_CTRL`，或定义同名编译宏，可切换为 DQ-LADRC1。两种配置都调用 `ctl_auto_tuning_foc_core()` 和 `ctl_init_foc_core()`。

可重复的比较程序位于 `ctl/component/intrinsic/complex/tests/host_sim/foc_current_loop_compare.c`。条件为：20 kHz 采样、`Ld=Lq=50 uH`、`Rs=0.13 ohm`、电压基值 `24/sqrt(3) V`、电流基值 `10 A`、q 轴参考从 0 跳变到 0.3 pu、圆限幅 0.9 pu。PI 交越带宽和 LADRC 的 `fc` 都是 707.355 Hz，LADRC 使用 `fo=2*fc`；前馈与交叉耦合关闭。

| 控制器 | 带宽 (Hz) | 10%-90% 上升时间 (ms) | 2% 调节时间 (ms) | 超调 (%) | IAE | 最终 iq (pu) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| DQ-PI | 707.355 | 0.450 | 0.950 | -0.000010 | 0.000052500 | 0.300000 |
| DQ-LADRC1 | 707.355 | 1.000 | 1.700 | -0.000020 | 0.000116859 | 0.300000 |

在这个标称一阶对象上，PI 响应更快；两种控制器都无明显超调并收敛。该结果是回归比较，不代表参数失配或扰动条件下的鲁棒性结论。

`mcs_pmsm_nt` 与 `mcs_pmsm_id` 的 Windows x64 Debug SIL 工程也分别用默认 PI 和 `ENABLE_FOC_LADRC_CTRL` 配置完成过编译链接；两套 `MCS_STD_PMSM_MODEL.slx` 均通过 UDP 建立连接并无错误运行到 0.1 s。

## F280049C 处理器在环

F280049C 目标提供本地 SDPE 功能 `ENABLE_GMP_DL_PIL_SIM`，它与公共套件配置相互独立。关闭时保留普通物理 ADC/PWM 控制路径；开启时仅由通过校验的 Data Link PIL 请求触发控制器计算，并保持物理门极/PWM 输出处于安全状态。UART/UDP 端点、命令基址、掩码和对象通道映射也由目标 SDPE 管理。

可重复的分阶段流程、MATLAB 运行器、安全约束和硬件记录见 [`project/f280049c/src/pil`](project/f280049c/src/pil/README_CN.md)。
