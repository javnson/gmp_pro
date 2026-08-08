# GMP PIL 与 Data Link 工具

[English](README.md) | **简体中文**

请根据目标系统的最小寻址单元选择调试器：

- `gmp_debugger_u8`：用于字节寻址的 STM32、x86 和现代 CPU，要求
  `GMP_PORT_DATA_SIZE_PER_BYTES == 1`，也是新工程的默认选择。
- `gmp_debugger_u16`：用于 16 位寻址的传统 DSP，主要服务于 TI C28x，要求
  `GMP_PORT_DATA_SIZE_PER_BYTES == 2`。

两套工具使用相同的 DL 线协议，区别在于下位机缓冲区存储和原生地址契约。
新的字节寻址工程应运行 `gmp_debugger_u8/run.bat`。

`stm32_dl_dbger` 保存用于验证 u8 后端的 NUCLEO-C092RC 固件工程。
