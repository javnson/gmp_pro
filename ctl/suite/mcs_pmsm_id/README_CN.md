# PMSM 矢量控制与离线参数辨识

[English](README.md) | **简体中文**

本套件在 GMP PMSM 矢量控制模板上增加了离线参数辨识引擎。它保留多平台和递增 `BUILD_LEVEL` 工作流，可在完成基础驱动验证后辨识电机参数并为电流环整定提供输入。

## 辨识范围

- 定子电阻和电感；
- 永磁体磁链；
- 逆变器死区补偿；
- 机械惯量和摩擦；
- 带故障诊断的编码器偏置与极对数标定；
- 正反向静态/负载转矩；
- 初始电流环 PI 整定。

公共实现集中在 `src/`，其中包括 `pmsm_offline_id_if` 接口和普通的 `ctl_main` 控制入口。仓库当前包含 `f280039c_Iris_node`、`f280049c`、`simulate`、`stm32f405`、`stm32g431` 和 `stm32g474_hrtim` 目标目录。

参数辨识会有意给电机上电或使其旋转。启动辨识前，必须在低构建级确认电流标度、相序、保护限值、转子可自由转动以及急停行为。所有辨识开关、激励量、时间、速度比例和编码器故障阈值由 `sdpe_general/sdpe_requirement.json` 管理；修改后应重新运行公共与目标的 `sdpe_generate.bat`，不要手工修改生成头文件。

## PC SIL 工作流

仿真目标提供两条共用同一个原生控制器可执行文件与打包 UDP ABI 的路径。快速平均值对象回归在 PowerShell 中运行：

```powershell
cd ctl\suite\mcs_pmsm_id\project\simulate
python .\run_pmsm_id_sil_fast.py --build
```

该运行器完成编码器标定、电气辨识、恒定 Iq 加速和 PWM 关闭后的滑行，并写出 JSON/CSV 结果。当前回归在仿真 14.2535 s 到达 `COMPLETE (8)`；检测到四对极，编码器偏置精确到仿真编码器 LSB。惯量、黏性摩擦与静态负载转矩误差分别为 -2.45%、-1.95% 和 -9.21%，电气参数误差保持在 2.4% 内。

编码器安全路径可用 `--encoder-fault random`、`stuck` 和 `nonuniform` 重复验证，分别检测随机跳变、轴不运动和电气周期锚点不均匀，并在故障时物理禁止 PWM。

高保真 Simulink 开关对象使用 `run_pmsm_id_sil.m`。短烟雾测试会先执行编码器准备阶段；最终关联建议使用 1 us 对象步长，以匹配模型中的 1 us 逆变器死区，并为机械滑行留出足够时间。这一路径计算量较大是预期行为。

## DQ-PI 与 DQ-LADRC1 同带宽仿真记录（2026-08-08）

FOC 电流环默认使用 DQ-PI。在 `foc_core.h` 中取消注释 `#define ENABLE_FOC_LADRC_CTRL`，或定义同名编译宏，可切换为 DQ-LADRC1。两种配置都调用 `ctl_auto_tuning_foc_core()` 和 `ctl_init_foc_core()`。

可重复的比较程序位于 `ctl/component/intrinsic/complex/tests/host_sim/foc_current_loop_compare.c`。条件为：20 kHz 采样、`Ld=Lq=50 uH`、`Rs=0.13 ohm`、电压基值 `24/sqrt(3) V`、电流基值 `10 A`、q 轴参考从 0 跳变到 0.3 pu、圆限幅 0.9 pu。PI 交越带宽和 LADRC 的 `fc` 都是 707.355 Hz，LADRC 使用 `fo=2*fc`；前馈与交叉耦合关闭。

| 控制器 | 带宽 (Hz) | 10%-90% 上升时间 (ms) | 2% 调节时间 (ms) | 超调 (%) | IAE | 最终 iq (pu) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| DQ-PI | 707.355 | 0.450 | 0.950 | -0.000010 | 0.000052500 | 0.300000 |
| DQ-LADRC1 | 707.355 | 1.000 | 1.700 | -0.000020 | 0.000116859 | 0.300000 |

在这个标称一阶对象上，PI 响应更快；两种控制器都无明显超调并收敛。该结果是回归比较，不代表参数失配或扰动条件下的鲁棒性结论。

`mcs_pmsm_nt` 与 `mcs_pmsm_id` 的 Windows x64 Debug SIL 工程也分别用默认 PI 和 `ENABLE_FOC_LADRC_CTRL` 配置完成过编译链接；两套 `MCS_STD_PMSM_MODEL.slx` 均通过 UDP 建立连接并无错误运行到 0.1 s。
