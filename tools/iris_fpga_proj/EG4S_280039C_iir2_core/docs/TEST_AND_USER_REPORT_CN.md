# 测试与使用报告

[English](TEST_AND_USER_REPORT.md)

## 测试结论

2026-08-04 使用 Icarus Verilog 12.0（development build）和 Python 3.13.0 运行
`tests/run_tests.ps1`，全部通过：

| 测试 | 覆盖内容 | 结果 |
| --- | --- | --- |
| `tb_iir2_operator` | 递归冲激响应、TLAST、数值饱和 | PASS |
| `tb_iir2_pipeline` | 两节 SOS 级联、计算值、输出背压稳定性 | PASS |
| `tb_control_core_axi` | AXI4-Lite 读写、默认直通、shadow/commit、结果计数 | PASS |
| `tb_epwm_modulator` | PWM 周期、互补死区、故障锁存和清除 | PASS |
| `tb_data_adapters` | ADC 偏移二进制/Q3.28 转换与 DAC 反向映射 | PASS |
| `tb_spi_axi_bridge` | TD SPI 读取 AXI 高低半字并写控制寄存器 | PASS |
| `tb_external_memory_system` | 双 burst 控制；注入/捕获；非对齐请求拒绝 | PASS |
| TD 顶层展开 | 使用时钟管理器仿真替身展开完整板级模块 | PASS |
| `test_host_tools.py` | 定点/传递函数转换、寄存器镜像、扫频和频响分析 | 5 项 PASS |

完整测试集退出码为 0，最终 Icarus 运行没有 RTL 警告。当前环境未安装 AMD Vivado；已安装的
TD 为 5.0，而原工程元数据要求 TD 6.2，因此本报告不宣称 TD 布局布线和时序结果。ADC、DAC、PWM、
外存和闭环控制的实物验证仍需在目标板上完成。

## 实时控制使用

1. 集成 `iir2_control_core`，连接 AXI4-Lite 和 AXI4-Stream。
2. 将每节 B0、B1、B2、A1、A2 写入 shadow 窗口。
3. 写 `ACTIVE_SECTIONS`；确认 `STATUS.busy=0` 后向 `COMMIT` 写 1。提交会清除历史状态。
4. 将 `CONTROL.run` 置 1，以 Q3.28 形式发送和接收 AXI4-Stream 样本。
5. 通过 `LATEST_RESULT`、`SAMPLE_COUNT` 和 `SAT_COUNT` 监控。

复位后的系数是恒等传递；有效节数为 0 时全部旁路，但 AXI 握手仍保持完整。

## 外存批处理使用

1. 集成 `iir2_external_memory_system`，把完整 AXI4 主口连接平台的 DDR/SDRAM 控制器。
2. 源、目标缓冲区必须互不重叠、4 字节对齐，长度不少于 `样本数 × 4`。带 CPU cache 的平台应在 DMA 前
   flush 源缓冲区、完成后 invalidate 目标缓冲区，或使用一致性内存。
3. 写 `DMA_READ_BASE`、`DMA_WRITE_BASE` 和 `DMA_SAMPLE_COUNT`。
4. 写 `DMA_CONTROL=0x3` 选择控制器批处理并启动；轮询 `DMA_STATUS`，等待两个 busy 位清零、
   done-seen 置位，并检查 error 为 0。

该模式把外存输入逐字送入 SOS 容器并将结果写回目标缓冲区，负载数据不会进入 FPGA BRAM。

## 注入辨识使用

外存封装额外提供注入输出和测量输入 AXI4-Stream。写 `DMA_CONTROL=0x7` 后，DDR 读数据被送到
`m_axis_injection_*`（通常接 DAC 缩放和输出），`s_axis_measurement_*`（通常来自同步 ADC）写入
DDR。两个方向均严格处理 `DMA_SAMPLE_COUNT` 个字，最后一个字必须携带 TLAST。

生成对数扫频 CSV 和可直接装入 DDR 的小端 Q3.28 文件：

```powershell
python host\gmp_fpga_control.py make-injection --sample-rate 20000 --duration 10 `
  --f-start 1 --f-stop 5000 --amplitude 0.05 --csv injection.csv --binary injection.bin
```

采集后构造含 `input`、`output` 两列的 CSV，并计算频响：

```powershell
python host\gmp_fpga_control.py analyze --input measurement.csv `
  --sample-rate 20000 --output frequency_response.csv
```

在功率硬件上使用前，必须根据实际对象确认注入幅值、功率边界、故障保护、同步采样和抗混叠滤波。

## 传递函数下装

从 scipy 格式 SOS 文件生成寄存器和 TD SPI 写序列：

```powershell
python host\gmp_fpga_control.py compile --sos-json host\example_sos.json `
  --output controller_registers.json
```

安装 SciPy 后也可直接输入传递函数：

```powershell
python host\gmp_fpga_control.py compile --numerator "0.1,0.1" `
  --denominator "1,-0.8" --output controller_registers.json
```

输出 JSON 同时包含浮点系数、Q3.28 字、AXI 字节地址和 TD SPI 事务；超出可表示范围的系数会被拒绝。
