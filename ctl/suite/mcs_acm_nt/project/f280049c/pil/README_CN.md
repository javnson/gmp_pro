# F280049C ACIM 处理器在环仿真

本目录提供 `MCS_STD_ACM_MODEL.slx` 的可复现 PIL 流程。目标工程的 SDPE requirement 是唯一配置源，统一管理 PIL 开关、BUILD_LEVEL、Data Link 命令号、串口波特率、UDP 端点、掩码和通道映射。

## 安全边界

`ENABLE_GMP_DL_PIL_SIM` 是 F280049C 目标工程的独立 SDPE 功能开关。关闭时，物理 ADC 中断、控制器调度和 PWM 路径保持原有行为；开启时，只有通过严格校验的 Data Link PIL STEP 请求才能执行 `ctl_dispatch()`。物理 ADC 中断只继续维护 GMP 系统时基，不再运行控制器；任何物理 PWM 使能请求都会被强制转换为 Trip 且关闭门极。

PIL 固件不能用于带功率运行。恢复物理控制前，必须在 SDPE 中关闭 PIL 开关，重新生成并编译固件。

## 分级运行

1. 在 `sdpe_mgr/sdpe_requirement.json` 中选择 BUILD_LEVEL 1～4，并开启 `ENABLE_GMP_DL_PIL_SIM`。
2. 先生成公共 SDPE 输出，再生成 F280049C 目标输出，并同步 `gmp_src_mgr`。
3. 编译并下载 F280049C 固件。
4. 从 `tools/gmp_pil_server/gmp_debugger` 启动无界面桥接器：

   ```powershell
   python -m apis.examples.pil_bridge --sdpe ../../../ctl/suite/mcs_acm_nt/project/f280049c/sdpe_mgr/sdpe_requirement.json --port COM5 --trace ../../../ctl/suite/mcs_acm_nt/project/f280049c/pil/results/manual/build_level_1/bridge_trace.csv
   ```

5. 在 MATLAB 中执行：

   ```matlab
   cd('ctl/suite/mcs_acm_nt/project/f280049c/pil');
   run_pil_stage(1, 0.05, "results/manual/build_level_1");
   ```

桥接器会把所选 ADC、PWM、Monitor 通道及串口往返时间保存为 CSV；MATLAB 会保存完整 `SimulationOutput` 和 JSON 清单。STEP 超时将终止仿真且不会自动重试，因为无法判断响应丢失前下位机是否已经执行该步，贸然重试可能造成一次输入执行两次控制。

## 标准数据契约

- Simulink 到桥接器：`<d24I16d8i>`，264 bytes。
- 桥接器到 Simulink：`<d8I8I16d>`，200 bytes。
- 下位机 RX：固定 tick、digital 字段，以及由 `GMP_PIL_RX_MASK` 选择的 ADC/panel 字段。
- 下位机 TX：固定 digital 字段，以及由 `GMP_PIL_TX_MASK` 选择的 PWM/DAC/monitor 字段。

`run_pil_stage` 会在仿真前校验 Simulink 向量尺寸，并用 SDPE 中的 UDP 配置覆盖模型运行参数，不会修改原始模型文件。

## 实板验证记录

2026-08-10 已在 LaunchXL-F280049C 上完成隔离 PIL `BUILD_LEVEL=1`：目标固件完成
编译和下载，同一 ACIM 模型完成 0.005 s 联调，桥接器严格记录 100 个 20 kHz
控制步，末帧模型时间为 0.004975 s，目标平均往返约 7.25 ms，且物理输出使能字
全程为 0。验证同时确认 UDP S-Function 仅在主求解步推进目标，次步复用缓存结果，
不会让一个模型采样执行多次目标控制。MATLAB 结果与清单位于
`results/timing_fix_build_level_1`。`BUILD_LEVEL=2～4` 尚未实板验收，必须按顺序推进。

## XDS110 延迟

PIL 是存在因果依赖的请求/响应闭环，因此其墙钟运行速度受单步往返延迟限制，而不是
受控制算法本身 20 kHz 的执行能力限制。TI 的测试表明，XDS110 虚拟串口在约
230400 baud 以上会将小数据包聚合 14～16 ms。一次 PIL 单步包含两个 USB 方向，因而
可以解释 256000 baud 下测得的约 30 ms 目标往返时间。

F280049C 的 SDPE 配置现使用标准的 115200 baud，远低于该 XDS110 阈值。PIL 构建还会让
Data Link 服务直接在后台循环中运行，不再等待普通的 2 ms 周期任务；关闭 PIL 后的
物理控制调度路径完全不变。如需亚毫秒级往返，应使用可调低延迟计时器的 USB-UART
适配器，或使用原生 USB/bulk 传输替代 XDS110 VCOM。

本次 ACIM Level 1 验证使用 115200 baud，目标平均往返约 7.25 ms。PIL 的墙钟速度
不代表目标电流环算力；它主要受 XDS110 串口请求/响应延迟限制。
