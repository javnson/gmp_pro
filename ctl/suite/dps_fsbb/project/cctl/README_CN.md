# FSBB 本机 CCTL 联合仿真

[English](README.md) | **简体中文**

工程复用 [FSBB 控制器](../../src/ctl_main.c)和 [用户调度器](../../src/user_main.c)。GMP CCTL 在同一进程内运行 MCU 外设、控制器和开关电路。电路步长为 100 ns，ePWM 每 50 µs 触发一次 ADC，ADC 锁存后进入 `gmp_base_ctl_step()`，控制器更新两路 PWM 以及使能状态。CiA402 状态机仍由 FSBB 自带的 1 ms 后台任务驱动。

[生成器](hw/generate_closed_loop.py)读取已有的 [FSBB.CIR](hw/FSBB.CIR)，保留四开关拓扑，用共用 SDPE 中的 1.5 mH 电感、两组 440 µF 电容和 20 Ω 负载生成闭环 Eigen 模型。开关使用 Simulink 的 9.3 mΩ 导通电阻。电路采用 GMP 的 `half_bridge_approx`，死区内按 TINA 模型的体二极管参数计算续流，开关接通后优先采用沟道导通状态。ADC 按共用传感器增益、偏置与 12 位分辨率量化 `[Vin, Vout, IL, Iout]`。TINA 的 PWM1/2 对应 Buck 上/下管，PWM4/3 对应 Boost 上/下管。GMP 的 Boost 比较值以**下管**为基准，传给模拟 ePWM 时转换为上管比较值。

## 构建与实验

先安装 GMP 和 Visual Studio C++ 工具。在本目录依次执行所需等级。

```powershell
.\build_test.bat 1 --no-pause
.\build_test.bat 2 --no-pause
.\build_test.bat 3 --no-pause
```

| 等级 | 实验配置 |
| --- | --- |
| 1 | 24 V 输入，12 V 开环电压给定 |
| 2 | 24 V 输入，5 A 电感电流给定 |
| 3 | 24 V 输出给定，输入在 1.2 s 由 32 V 阶跃至 18 V，跨越 Buck 和 Boost |

等级 1 和 2 各跑 1.2 s，等级 3 跑 1.8 s。脚本检查控制中断与 ADC SOC 一一对应、桥臂交叠、保护故障和稳态跟踪。原始 CSV 位于 `%GMP_PRO_LOCATION%\tmp\cctl\suite\dps_fsbb\level_<等级>`，可用 GMP Simulation Viewer 打开。

- `fsbb_level_<等级>_circuit.csv` 按 100 kHz 保存电路电压、电流和四路门极。
- `fsbb_level_<等级>_control.csv` 每次控制中断保存 ADC 码、比较值、给定、CiA402 状态和故障字。

## 仿真结果

Windows Release 构建的三组 CTest 均通过。每组保护故障和 CSV 丢样均为零，ADC 触发数和控制中断数一致。

| 等级 | 稳态测量 | ADC / ISR 次数 |
| --- | --- | ---: |
| 1 | 12 V 开环给定，输出 11.467 V | 24,000 / 24,000 |
| 2 | 5 A 电流给定，电感电流 5.002 A，输出 45.761 V | 24,000 / 24,000 |
| 3 | 输出在 Buck 时为 24.009 V，在 Boost 时为 24.008 V | 36,000 / 36,000 |

稳态值取各段末尾 50 ms 的 ADC 触发时刻平均值。Level 3 阶跃后 4.018 ms，100 ns 步长记录到 22.653 V 的最低输出，约 55.73 ms 后输出保持在 24 V ±2% 内。Boost 下管指令占空比由 4.97% 升至 32.76%。这些结果适用于本机开关模型，硬件波形有待实测。[Level 3 波形](../../doc/simulation_result/cctl/level_3_waveforms.png)
