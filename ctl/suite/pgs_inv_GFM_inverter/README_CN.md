# 三相构网型变流器（GFM）

[English](README.md) | **简体中文**

该工程沿用 `gfl_inv_ctrl_t` 作为内层电流环，在其外部增加独立电压环、
可替换的构网外环，以及通用的 PLL 到 GFM 平滑切换模块。公共控制代码供
PC 仿真、F280039C Iris、LaunchXL-F280049C 和 STM32G431 目标复用。

## BUILD_LEVEL

| 等级 | 功能 |
|---|---|
| 1 | 开环电压、PWM 与采样极性检查 |
| 2 | 使用电流内核内部 RG 的电流闭环 |
| 3 | 独立 LC 电容电压闭环 |
| 4 | PLL 定向的并网电流闭环 |
| 5 | PLL 同步及连续锁定判断，平滑迁移到 GFM 角度和电压环，解列后进行负载阶跃 |

电压环使用普通 PI。圆限幅和方限幅可分别启用并分别设置阈值，两者均作用于
完整的 d-q 电流指令；最终限幅结果通过
`ctl_pid_clamping_correction_using_real_output()` 返回积分器。电容
\(\omega C\) 耦合前馈可单独关闭。

`inv_gfm_droop_ctrl` 是当前的 P-f/Q-V 构网外环。它不拥有电压 PI 和角度源，
因此后续可替换为虚拟同步机、虚拟阻抗、惯性同步或其他构网算法。
`inv_gfm_transition` 负责跟踪 PLL、以 PLL 角度初始化构网角度、归一化相量混合，
并同步渐变电流指令。

启用 `USING_3D_SVPWM` 后使用 A/B/C/N 四桥臂调制，并启用基于可调 QPR 的
零序电流控制。硬件目标必须先完成第四桥臂映射和保护验证。

MATLAB R2024b 自动化 SIL 流程见
[`project/simulate`](project/simulate/README.md)。当前结论来自软件仿真与主机
单元测试，不代表新增 GFM 功能已经完成硬件验证。

