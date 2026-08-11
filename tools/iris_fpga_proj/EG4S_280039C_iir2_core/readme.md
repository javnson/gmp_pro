# GMP FPGA IIR2 controller

[中文说明](README_CN.md)

This project implements a portable FPGA control path for the GMP repository:

- ADS8688 acquisition and DAC8563 output using the existing board drivers;
- a fixed-point, saturating, registered IIR2 operator and a parameterized SOS cascade;
- AXI4-Stream sample interfaces and AXI4-Lite configuration/result registers;
- an ePWM-style center/edge-aligned modulator with shadow compare, dead time, phase sync, and trip shutdown;
- an AXI4 external-memory DMA that streams bulk injection/sample/result data without payload BRAM;
- a TD EG4S20 top-level with a C2000-compatible SPI-to-AXI-Lite bridge;
- host tools for transfer-function/SOS compilation, logarithmic injection generation, and measured frequency-response analysis.

The portable live-stream top is `iir2_control_core`. The external-memory top is
`iir2_external_memory_system`. The TD board top is `td_iir2_control_top`.

Run all available tests from PowerShell:

```powershell
cd E:\lib\gmp_pro\tools\iris_fpga_proj\EG4S_280039C_iir2_core
.\tests\run_tests.ps1
```

See [architecture](docs/ARCHITECTURE.md), [register map](docs/REGISTER_MAP.md),
and the [test and user report](docs/TEST_AND_USER_REPORT.md).
