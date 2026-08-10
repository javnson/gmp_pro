# 异步电机矢量控制工程（MCS_ACM_NT）

[English](README.md) | **简体中文**

## 工程定位

本工程是 GMP 的异步电机（ACIM）分层调试例程，结构参考 `mcs_pmsm_nt`，但电流内核、角度链路和前馈模型按异步电机物理模型独立实现。控制器通过 `BUILD_LEVEL` 从 V/f 开环逐级推进到电流环、真实磁场定向和速度环。

最重要的接口约束是：

- 编码器给出转子轴位置和机械速度，用于机械环。
- 电流环必须使用转子磁链角和同步电气速度，不能直接使用编码器角。
- 全库角度采用周标幺：`1 pu = 360° = 2π rad`。

## 模块边界

| 模块 | 路径 | 职责 |
|---|---|---|
| ACIM 电流环 | `ctl/component/motor_control/current_loop/imfoc_core.h` | 采样、Clarke/Park、d/q PI、ACIM 解耦、限幅和 iPark |
| 有感磁场角计算 | `ctl/component/motor_control/observer/acim_pos_calc.h` | 根据机械速度、`id/iq` 和滑差计算同步磁链角 |
| 无感磁链观测 | `ctl/component/motor_control/observer/acim_fo.h` | 电流/电压复合磁链、PLL 磁场角、同步速度、滑差和机械速度 |
| 通用角度 PLL | `ctl/component/motor_control/observer/ato_pll.h` | 由周标幺相位误差输出周标幺角度和速度 |

这些模块处于同一层级。`imfoc_core` 不包含位置计算器、磁链观测器或 PMSM `foc_core`，更换观测器不会改变电流内核结构。

## BUILD_LEVEL

| 等级 | 角度来源 | 电流环 | 目标 |
|---|---|---|---|
| 1 | I/F 斜坡 | 关闭 | V/f、相序、PWM 极性、母线和采样检查 |
| 2 | I/F 斜坡 | 闭环 | 验证 Clarke/Park、电流极性和 d/q PI |
| 3 | 有感位置计算器或无感 FO | 闭环 | 验证真实磁场定向；无感版本包含 I/F 捕获和交接保护 |
| 4 | 同 Level 3 | 电流环 + 速度环 | 验证机械速度闭环和限流 |

默认 `BUILD_LEVEL=1`。公共参数位于 `sdpe_general/sdpe_requirement.json`，仿真平台覆盖参数位于 `project/simulate/sdpe_mgr/sdpe_requirement.json`。

## 生成与构建

先运行公共和平台 SDPE 生成器：

```powershell
cd E:\lib\gmp_pro\ctl\suite\mcs_acm_nt\sdpe_general
.\sdpe_generate.bat
cd ..\project\simulate\sdpe_mgr
.\sdpe_generate.bat
```

Visual Studio 仿真工程支持构建时覆盖：

```powershell
MSBuild GMP_Motor_Control_simulink.sln /t:Build /p:Configuration=Release /p:Platform=x64 /p:BuildLevel=3 /p:AcimFeedbackMode=1
```

`AcimFeedbackMode=1` 为有感，`2` 为无感。SIL 模型为 `project/simulate/MCS_STD_ACM_MODEL.slx`，自动记录脚本为 `project/simulate/commissioning/run_build_level_sil.m`。

## 无感启动约束

电压模型在静止和极低速不可观。示例采用：

1. I/F 强制磁化并初始化 PLL；
2. 到达捕获频率后，电流环仍使用 I/F 磁场角，而 PLL 独立捕获；
3. 磁链健康、角度和速度连续满足条件后才交接；
4. 交接后若同步速度失配，锁存失败并退回 I/F，直到控制器重新清零。

当前 SIL 已通过 Level 1、Level 2、Level 3 有感和 Level 4 有感验证。无感交接保护已验证，但所提供模型上的交接后闭环尚未通过，因此不能直接用于硬件闭环。详见 [调试记录](doc/commissioning_record_cn.md)。

## TI controlSUITE 对照

算法和标幺推导对照 TI `aci_fe.h`、`aci_fe_const.h`、`aci_se.h` 和 `aci_se_const.h`：

- 电流模型、定子磁链参考、反电势梯形积分和转子磁链反算与 ACIFE 对齐；
- 滑差基式与 ACISE 对齐；
- GMP 使用 PLL 提取磁链角，不直接使用 `atan2`，以避免硬件噪声和偏置导致的角度跳变及差分速度脉冲；
- GMP 磁链基值为 `Vbase/Wbase`，因此系数形式与 TI 的 `Lm*Ibase` 磁链基值不同，但物理量等价。

## 硬件上电前检查

- 从 `BUILD_LEVEL=1` 开始，不跨级调试。
- 核对相电压峰值、电流峰值和电气角速度基值。
- 核对 PWM 命令到相电压的比例 `MCS_FO_COMMAND_VOLTAGE_SCALE`。
- 有感模式先确认编码器机械方向，再确认位置计算器磁链角方向。
- 无感模式必须记录交接前后磁链幅值、磁链角、同步速度、滑差和回退锁存状态。
