# GMP 设备与外设服务

`core/dev` 提供平台无关的外设契约、通信服务和具体器件驱动。芯片寄存器与厂商 SDK 实现属于 `csp` 或工程 `xplt`，不应进入本目录的公共驱动。

| 内容 | 说明 |
| --- | --- |
| `peripheral_types.h` | CSP 可覆盖的外设句柄类型 |
| `peripheral_port.h` | GPIO、SPI、UART、I2C 及历史 CAN 的 `gmp_hal_*` 契约 |
| `driver` | [基于公共 HAL 的具体器件驱动](driver/README.md) |
| `datalink` | [数据链路与调试服务](datalink/README.md) |
| `can` | [静态异步 CAN/CAN FD 服务](can/README.md) |
| `print` | [可选 UART 打印适配](print/README.md) |
| `rtshell` | 尚未注册为 Facility 的实时 Shell 占位设计 |

Data Link 协议：[English](datalink/readme_dl_protocol.md) | [简体中文](datalink/readme_dl_protocol_cn.md)

## 句柄与实现边界

`peripheral_types.h` 把 `GMP_PORT_UART_T`、`GMP_PORT_SPI_T`、`GMP_PORT_I2C_T`、`GMP_PORT_CAN_T` 和 `GMP_PORT_GPIO_T` 映射成相应的 `*_halt`。CSP 可在包含该头文件前定义这些宏；未定义时多数句柄退回 `void *`，GPIO 同时以 `GMP_GPIO_NOT_IMPLEMENTED` 标记。

所有函数签名和返回语义以 `peripheral_port.h` 为准。写入、配置和总线传输通常返回 `ec_gt`；读取电平、查询忙状态或可用长度等查询函数直接返回值，因此不能笼统假设所有 HAL 函数都返回错误码。

SPI 的 CSP 层实现 `gmp_hal_spi_bus_write/read/transfer`，不负责片选。公共逻辑设备函数负责拉低/释放 CS 和按字节序列化。UART 阻塞读写带超时，其中 `gmp_hal_uart_read` 还通过 `bytes_read` 报告超时前接收的长度。

## CAN 两套接口

定义 `SPECIFY_ENABLE_GMP_CAN_SERVICE` 时，使用 `core/dev/can/can.h` 中的静态异步服务：标准帧为 `gmp_can_frame_t`，支持 Classic CAN 与 CAN FD，队列存储由调用者提供，CSP/扩展钩子契约见 `can_hook.h`。

未定义该宏时，`peripheral_port.h` 保留历史 Classic CAN 草案接口。此时帧类型是：

```c
typedef struct
{
    uint32_t id;
    fast_gt is_extended;
    fast_gt is_remote;
    fast_gt dlc;
    uint32_t data_32[2];
} gmp_can_msg_t;
```

在 C28x 这类 `CHAR_BIT` 不等于 8 的目标上，应通过 `gmp_can_payload_get_u8()`、`gmp_can_payload_set_u8()` 及对应的 16/32 位辅助函数访问逻辑字节，不要把有效载荷改成 `uint8_t[8]`。历史草案 API 与新的 `gmp_can_frame_t` 服务不可混用。

## 器件驱动规则

- 驱动只能依赖 `gmp_type.h`、`peripheral_types.h`、`peripheral_port.h` 及自身类别公共头；不得依赖完整 `<gmp_core.h>` 或 CTL。
- 运行状态放入调用者持有的设备对象，不使用隐藏的可变全局状态。
- 静态存储与明确超时优先；ISR 路径不得死等。
- 新驱动放在 `driver/<category>/<device>.h` 与相应 `src` 中，并在 `tools/facilities_generator/src_mgr/gmp_framework_dic.json` 注册独立 Facility 模块。
- 验证至少包括所注册模块的编译；仿真和硬件状态只有在完成对应测试后才能标记。

移植新芯片时，还应结合 [CSP 使用与扩展指南](../../csp/CSP_GUIDE.md) 核对运行时钩子、外设契约和工程 `xplt` 绑定。
