# NUCLEO-C092RC GMP Data Link u8 验证工程

[English](README.md) | **简体中文**

这是一个可独立使用的 STM32CubeMX 工程，用于在 STM32C092RCT6 上验证字节寻址的
GMP Data Link 后端。USART2 通过 PA2/PA3 连接 ST-Link 虚拟串口，参数为
`921600 8-N-1`。RX 使用循环 DMA，并同时处理半传输、传输完成和 UART Idle 事件；
TX 的帧头和载荷均使用 DMA。

工程遵循 GMP 标准目录和入口：

- `user/`：应用、协作式调度器和 CTL 生命周期接口；
- `xplt/`：STM32 CSP 适配、UART DMA、定时器和 LED；
- `gmp_src_mgr/gmp_inc` 与 `gmp_src_mgr/gmp_src`：工程内生成的 GMP 头文件和源文件；
- `Core/Src/main.c` 通过 `gmp_base_entry()` 进入 GMP；
- 调度器只注册 Data Link 服务和 LED 心跳两个任务。

Keil 与 CMake 仅引用该工程目录内的文件。完成一次 `gmp_src_mgr` 生成后，可以把整个
`stm32_dl_dbger` 目录复制到 GMP 仓库之外构建，不再需要 `GMP_PRO_LOCATION`，也没有
任何 GMP 仓库绝对路径。

## Data Link 与 DSA 服务

固件提供目标信息（`0x02`）、ECHO（`0x00`）、四个可读写 Tunable 变量
（`0x30`/`0x31`）和 Memory Perspective（`0x50`/`0x51`）。

TIM3 以 1 kHz 采样并产生 50 Hz 正弦/余弦信号。GMP DSA Trigger 负责触发一次一致性
采集，DSA Scope 每通道记录 400 点，即连续 20 个信号周期。两通道采用 SoA 布局，
共 800 个 float、3200 字节，并作为只读 Memory Perspective 区域暴露。上位机的
`DSA Trigger/Scope` 页可以完成触发、读取和绘图。

## 重新生成工程内 GMP 文件

只有重新生成阶段需要 GMP 开发环境：

```bat
gmp_src_mgr\gmp_generate_all.bat
```

当前已经保留可直接构建的生成结果；`gmp_compiler_includes.txt` 也只包含工程相对路径。

## Keil MDK-ARM

使用 Keil uVision 打开 `MDK-ARM/stm32_dl_dbger.uvprojx`，并使用 Arm Compiler 6
构建。`generate_keil.bat` 可重新执行 CubeMX 生成，并自动恢复工程内的 `user`、
`xplt` 和 GMP 生成源文件分组；它不会重新生成 GMP 源文件。

## CMake 与硬件验证

```powershell
cmake --preset Debug
cmake --build --preset Debug
python smoke_test.py
```

烧写 CMake ELF 或 Keil HEX 并复位后运行测试。测试会自动识别 NUCLEO 的 ST-Link
虚拟串口，并验证转义、CRC、满 MTU DMA ECHO、Tunable、Memory Perspective、DSA
元数据、400 点正弦/余弦快照以及只读保护。
