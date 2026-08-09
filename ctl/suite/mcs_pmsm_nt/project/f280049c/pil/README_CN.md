# F280049C PMSM 处理器在环仿真

本目录提供 `MCS_STD_PMSM_MODEL.slx` 的可复现 PIL 流程。目标工程的 SDPE requirement 是唯一配置源，统一管理 PIL 开关、BUILD_LEVEL、Data Link 命令号、串口波特率、UDP 端点、掩码和通道映射。

## 安全边界

`ENABLE_GMP_DL_PIL_SIM` 是 F280049C 目标工程的独立 SDPE 功能开关。关闭时，物理 ADC 中断、控制器调度和 PWM 路径保持原有行为；开启时，只有通过严格校验的 Data Link PIL STEP 请求才能执行 `ctl_dispatch()`。物理 ADC 中断只继续维护 GMP 系统时基，不再运行控制器；任何物理 PWM 使能请求都会被强制转换为 Trip 且关闭门极。

PIL 固件不能用于带功率运行。恢复物理控制前，必须在 SDPE 中关闭 PIL 开关，重新生成并编译固件。

## 分级运行

1. 在 `sdpe_mgr/sdpe_requirement.json` 中选择 BUILD_LEVEL 1～4，并开启 `ENABLE_GMP_DL_PIL_SIM`。
2. 先生成公共 SDPE 输出，再生成 F280049C 目标输出，并同步 `gmp_src_mgr`。
3. 编译并下载 F280049C 固件。
4. 从 `tools/gmp_pil_server/gmp_debugger` 启动无界面桥接器：

   ```powershell
   python -m apis.examples.pil_bridge --sdpe ../../../ctl/suite/mcs_pmsm_nt/project/f280049c/sdpe_mgr/sdpe_requirement.json --port COM5 --trace ../../../ctl/suite/mcs_pmsm_nt/project/f280049c/pil/results/manual/build_level_1/bridge_trace.csv
   ```

5. 在 MATLAB 中执行：

   ```matlab
   cd('ctl/suite/mcs_pmsm_nt/project/f280049c/pil');
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

2026-08-09 已在 LaunchXL-F280049C 上完成四个调试级别。保存的串口轨迹、MATLAB
结果、清单、日志和指标摘要位于
[`results/20260809_hardware_pil`](results/20260809_hardware_pil/README.md)。PIL 开启
和关闭的目标工程均已完成编译验证；当前连接板卡最终烧录的是隔离输出的 PIL
`BUILD_LEVEL=1` 配置。
