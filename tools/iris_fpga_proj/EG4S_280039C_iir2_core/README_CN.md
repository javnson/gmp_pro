# GMP FPGA IIR2 高级控制器

[English](readme.md)

本工程实现了面向 GMP 的可移植 FPGA 控制链：

- 复用 ADS8688 采样与 DAC8563 输出模块；
- Q3.28 定点、带饱和保护的流水化 IIR2 算子，以及参数化 SOS 级联容器；
- AXI4-Stream 样本通道和 AXI4-Lite 配置/结果寄存器；
- 仿照 DSP ePWM 的中心/边沿对齐调制、影子比较、死区、相位同步和故障关断；
- 面向外部 DDR/SDRAM 的 AXI4 burst DMA，大容量注入、采样和结果数据不占用 BRAM；
- TD EG4S20 板级顶层，以及连接 C2000 的 SPI 到 AXI4-Lite 桥；
- 传递函数/SOS 编译、扫频注入生成和频率响应分析上位机工具。

三个主要入口分别为：实时流控制 `iir2_control_core`、外存批处理
`iir2_external_memory_system`、TD 板级 `td_iir2_control_top`。

运行测试：

```powershell
cd E:\lib\gmp_pro\tools\iris_fpga_proj\EG4S_280039C_iir2_core
.\tests\run_tests.ps1
```

详细内容见[架构说明](docs/ARCHITECTURE_CN.md)、[寄存器映射](docs/REGISTER_MAP_CN.md)和
[测试与使用报告](docs/TEST_AND_USER_REPORT_CN.md)。
