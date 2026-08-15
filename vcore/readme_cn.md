# GMP VCore

[English](readme.md) | **简体中文**

`vcore` 保存 HDL/Verilog 支持以及面向硬件控制实现的实验内容。

| 目录 | 作用 |
| --- | --- |
| `bus_driver` | CAN、I2C、SPI、UART 实验 RTL 及部分 testbench |
| `components` | CORDIC、DDS、ePWM、FIR、PID 实验 RTL |
| `math_blocks` | 饱和运算和厂商生成的 DSP 宏包装 |
| `riscv` | RISC-V 相关实验和集成说明 |
| `utilities` | 小型可复用 RTL 辅助模块 |
| `vctl` | Verilog 控制运行时实验 |

该目录仍属于探索性模块，接口变化速度可能高于 `core` 和 `ctl`。用于正式工程时应固定已验证的具体实现，并在设计旁记录综合工具链、目标器件、时序约束和验证方式。可综合逻辑应与仅在主机运行的生成工具明确分离。
