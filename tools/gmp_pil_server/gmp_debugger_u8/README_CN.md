# GMP Debugger u8

[English](README.md) | **简体中文**

GMP Debugger u8 是面向字节寻址 CPU 的默认上位机工具，适用于
`GMP_PORT_DATA_SIZE_PER_BYTES == 1` 的 STM32、x86 及绝大多数现代 MCU/CPU。

完成 GMP 安装后运行 `run.bat`。该入口选择 u8 目标契约，并复用
`../gmp_debugger_u16` 中统一维护的调试器界面。DL 串口线协议保持一致；
两套目标契约的差别在于下位机缓冲区存储方式和原生内存地址解释。

Memory Perspective 中直接填写普通字节地址。连接随附的 NUCLEO-C092RC
验证固件时，请选择 ST-Link 虚拟串口并使用 `921600 8-N-1`。

`DSA Trigger/Scope` 页面可以触发 C092 目标，通过 Memory Perspective 读取 400 点
正弦/余弦快照，并绘制两个通道。
