# GMP Data Link 协议

English documentation: [readme_dl_protocol.md](readme_dl_protocol.md)

## 设计目标

GMP Data Link（DL）是 PIL、Tunable Parameters 和 Memory Perspective 共用的分帧传输层。
它同时支持常规字节寻址 CPU，以及最小 C 寻址单元为 16 位的 TI C28x 类目标。

用户始终包含以下公共头文件：

```c
#include <core/dev/datalink.h>
#include <core/dev/tunable.h>
#include <core/dev/mem_presp.h>
#include <core/dev/scope.h>
```

使用这些服务的工程始终编译同一组源文件：

```text
core/dev/src/gmp_datalink.c
core/dev/src/gmp_tunable.c
core/dev/src/gmp_mem_presp.c
core/dev/src/gmp_scope.c
```

## 后端自动选择

公共头文件和固定的 C 源文件根据 `GMP_PORT_DATA_SIZE_PER_BYTES` 选择实现：

| 宏值 | 后端 | 目标模型 |
| --- | --- | --- |
| `1` | `u8` | STM32、x86、Arm 等字节寻址 CPU |
| `2` | `u16` | TI C28x 等 16 位单元寻址 CPU |

其他值会触发编译错误。实际后端位于 `core/dev/datalink/`，应用层不应直接包含其中的文件。

u8 后端直接使用 `uint8_t` 表示协议字节，不再引入额外的字节类型别名。平台无关的用户
代码和外设适配代码使用 GMP 基本单元 `data_gt`：它在 STM32 上自然为 8 位，在 C28x 上
自然为 16 位。后端选择器会阻止 u8 实现在非字节寻址平台上编译。

## 线协议

两个后端的线上协议完全一致：

```text
'{' escaped-header '}' [payload payload-crc16]
```

解码后的头部固定为 6 字节：序列号 1 字节、命令 1 字节、Little-Endian 载荷长度 2 字节、
头部 CRC16 2 字节。头部中的 `{`、`}`、`%` 使用 `%` 加原字节 XOR `0x20` 转义。
Payload 不转义；非空 Payload 后附 Little-Endian CRC16，空 Payload 不附 Payload CRC。
CRC 使用初值 `0xFFFF` 和多项式 `0x1021`。

## 运行时接入

初始化 `gmp_datalink_t` 后，在 UART ISR 或 DMA 回调中调用 `gmp_dev_dl_push_byte()` 或
`gmp_dev_dl_push_str()`。主循环持续调用 `gmp_dev_dl_loop_cb()`：

- 收到 `GMP_DL_EVENT_RX_OK` 时，按业务优先级交给 PIL、Tunable、Memory Perspective 或
  用户命令；无人处理时调用 `gmp_dev_dl_default_rx_handler()`。
- 收到 `GMP_DL_EVENT_TX_RDY` 时，发送帧头和载荷；硬件不再使用缓冲区之后调用
  `gmp_dev_dl_tx_state_done()`。

DMA 可以将帧头与载荷串联为两次发送。循环 RX DMA 应在半传输、全传输和/或 UART Idle
事件中提交新增的数据区间，从而使连续数据流不依赖空闲间隔。

## Tunable 与 Memory Perspective

Tunable 通过静态字典开放离散变量，并为每项指定原生类型和 RO/RW 权限。读命令为
`base_cmd`，写命令为 `base_cmd + 1`。在 `base_cmd + 1` 上发送一个基本单元的载荷表示
按索引查询目标上报的参数描述符，有效写请求长度更长，因此两者不会产生歧义。版本 2
描述符只上报 ID、原生类型、权限和名称。需要显示单位时可直接写入名称，例如
`Signal Frequency (Hz)`，从而避免每个嵌入式字典项额外保存一个单位指针。上位机仍兼容
旧版版本 1 描述符。

Memory Perspective 只允许访问显式注册的沙箱区域，请求格式为：

```text
[address:u32][item-size:u8][item-count:u16][write-data...]
```

协议地址和长度始终以字节表示。u8 目标直接使用原生字节地址；C28x/u16 目标继续使用历史
字节地址约定，并由后端换算为本机字地址。仅允许 1、2、4 字节元素，完整访问必须落在同一
沙箱区域内并满足权限。

Memory Perspective 同样在 `base_cmd + 1` 上使用一个基本单元的载荷查询区域描述符，
版本 2 描述符只包含名称、地址、长度和权限。内存数据解释方式由 Memory 页面选择；
波形布局、通道数、深度和采样率只属于独立的 Data Link Scope 服务。上位机仍可解析
旧版版本 1 描述符，但会忽略其中已经废弃的显示元数据。

## Data Link Scope

Data Link Scope 与 Memory Perspective 相互独立。目标注册具名波形资源及配置、启动、状态
回调，上位机无需获知物理地址。一个 Scope 命令通过载荷首字节区分 Discover、Configure、
Arm、Status 和 Read 操作，从而以一个命令号完成一整页示波器工具的功能。

## 配套工具

- `tools/gmp_pil_server/gmp_debugger/run_u8.bat`：字节寻址 CPU 入口。
- `tools/gmp_pil_server/gmp_debugger/run_u16.bat`：DSP/C28x 入口。
- `tools/gmp_pil_server/stm32_dl_dbger`：NUCLEO-C092RC u8 固件与硬件冒烟测试。

两个入口共享同一个调试器和线协议编解码器，其区别仅用于标识目标内存地址模型。
