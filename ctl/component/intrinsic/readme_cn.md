# CTL 通用控制组件

[English](readme.md) | **简体中文**

本目录保存通用控制积木，当前源码树是模块清单的依据：

| 分组 | 已存在的实现 |
| --- | --- |
| `basic` | 分频、滞环、饱和、斜率限制、状态序列器 |
| `continuous` | PI、PID、抗饱和 PID、LADRC1/2、SOGI、跟踪 PID |
| `discrete` | biquad、direct form、通用/FIR 滤波、PID、SOGI、lead-lag、pole-zero、PR 与整定器、信号发生器、跟踪 PID |
| `complex` | d/q PI 与 d/q LADRC1 |
| `advance` | backstepping、FDRC、模糊控制/PID、ILC、IMC、LMS、MRAC、成对 LUT、重复控制、sinc 插值、SMC、曲面搜索 |
| `protection` | 通用保护/保护槽、ITOC、PT100x、暂降暂升检测 |

请包含具体模块头，并由源管理器选择其注册源码。当前
`ctl/component/intrinsic.h` 仍引用已删除的 `continuous/s_function.h` 和
`discrete/z_function.h`，不能作为可编译的总入口。

不同模块的函数名、参数和单位并不统一，必须阅读所选头文件；不要根据 PID 示例推断其他模块 API。编译、仿真和实物状态以注册表的 `sys_compile`、`sys_sim`、`sys_hw` 为准。当前 `complex` 目录有主机测试，但这不代表所有 intrinsic 模块均已验证。
