# GMP PIL 与 Data Link 工具

[English](README.md) | **简体中文**

上位机统一位于 `gmp_debugger`，请根据目标系统的最小寻址单元选择入口：

- `gmp_debugger/run_u8.bat`：字节寻址的 STM32、x86 和现代 CPU。
- `gmp_debugger/run_u16.bat`：16 位寻址的传统 DSP，主要服务于 TI C28x。

两个入口使用同一套上位机代码和 DL 线协议。下位机缓冲区存储和原生地址仍由
`GMP_PORT_DATA_SIZE_PER_BYTES` 选择对应的 C 后端。

`stm32_dl_dbger` 保存用于验证 u8 后端的 NUCLEO-C092RC 固件工程。
