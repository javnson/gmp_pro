# GMP VCore

**English** | [简体中文](readme_cn.md)

`vcore` contains HDL/Verilog support and experimental hardware-oriented control
implementations.

| Directory | Purpose |
| --- | --- |
| `bus_driver` | Experimental CAN, I2C, SPI, and UART RTL plus selected testbenches |
| `components` | CORDIC, DDS, ePWM, FIR, and PID RTL experiments |
| `math_blocks` | Saturating arithmetic and vendor-generated DSP macro wrappers |
| `riscv` | RISC-V-related experiments and integration notes |
| `utilities` | Small reusable RTL helpers |
| `vctl` | Verilog control-runtime experiments |

This area is exploratory. Interfaces may change more quickly than `core` and
`ctl`; production projects should pin the exact implementation they validate.
Keep synthesizable logic separate from host-only generators and document the
toolchain, device, timing constraints, and verification method with each design.
