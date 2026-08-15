# 带重复控制的单相逆变/整流套件

[English](readme.md) | **简体中文**

本套件是双向单相逆变器和有源前端整流器的 GMP 参考工程。F280039C Iris 硬件目标与 PC 仿真目标共用 `src/` 控制器代码，并通过接口对象绑定采样和调制信号。

## 主要能力

- 准比例谐振（QPR）电流控制与频域重复控制（FDRC）；
- 基于 SOGI 的单相 PLL；
- 带软启动的有功/无功指令；
- 单极性 SPWM、死区处理和分阶段保护；
- CiA 402 状态管理与 GMP Data Link。

公共控制参数位于 `sdpe_general/sdpe_requirement.json`；目标的 `BUILD_LEVEL`、采样、PWM、传感器和通信绑定位于 `project/<target>/sdpe_mgr/sdpe_requirement.json`。修改需求后先生成公共层，再生成目标层。`sdpe_mgr/ctrl_settings.h` 是生成文件，不应手工修改；`xplt` 不再保存第二份设置头。

## BUILD_LEVEL

| 等级 | 验证内容 |
| --- | --- |
| 1 | 隔离电阻负载上的正弦电压开环，检查采样极性、PWM 映射和功率级 |
| 2 | 隔离电阻负载上的交流电流闭环，检查 QPR、电网电压前馈和可选 FDRC |
| 3 | 并网有符号 P/Q 指令；正 P 向电网送电，负 P 整流吸收 |
| 4 | 实测有功功率外环生成电流指令 |
| 5 | 有源整流直流母线电压外环，从不控整流的实测功率工作点接管 |

公共 SDPE 的 `SINV_ENABLE_REPETITIVE_CONTROL` 是 FDRC 总开关。关闭时所有等级都保持 FDRC 禁用；开启时，控制器在进入运行态并延时后投入重复控制，避免学习启动暂态。

必须依次验证各级。并网或提高母线电压前，应在隔离低压条件下确认测量标度、极性、PWM、死区、保护和安全停机。CiA 402 状态机只有在校准和相应准入条件满足后才允许使能输出。

## 仿真与验证

`project/simulate` 包含三个功率级模型和 BUILD_LEVEL 1–5 自动回归。详细生成、构建、端口、信号映射、验收指标及当前结果见[仿真工程说明](project/simulate/README.md)和 [UDP/SIL 实验报告](doc/README.md)。归档记录表明五个等级均完成 SIL 回归；这不等价于所有等级已经完成硬件验证。
