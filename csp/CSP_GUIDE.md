# GMP CSP 使用与扩展指南

[CSP English overview](readme.md) | [CSP 中文概述](readme_cn.md)

## 1. 定位

CSP（Chip Support Package）把 `core` 和 `ctl` 的平台无关代码连接到具体芯片、操作系统或仿真环境。寄存器、厂商 SDK、中断入口和板级引脚映射应留在 CSP 或工程自己的 `xplt` 目录；公共控制算法不应依赖厂商头文件。

完整 GMP 应用包含仓库根目录的 `<gmp_core.h>`。仅使用可移植 CTL 算法时，可以定义 `GMP_CTL_PORTABLE` 后包含同一入口；该模式不会装配 CSP、运行时或外设服务。

## 2. 当前平台目录

仓库当前包含：

- MCU/DSP：`at32`、`c28x_pinmux`、`c28x_syscfg`、`c29x_syscfg`、`hc32`、`nation32_h47x`、`nxp_imxrt`、`rpi_pico`、`silan_spc6l64`、`stm32`；
- SoC：`zynq7`、`zynq_ultrascale`；
- 主机/仿真：`windows_simulink`、`linux_simulink`、`matlab_simulink_level2`；
- 最小模板：`null`。

目录存在只表示仓库保留了相应实现或模板，不等价于所有外设、所有器件和所有工具链都经过同等级验证。应以目标目录的源码、工程文件和局部 README 为准。

## 3. 平台契约

一个完整 CSP 通常提供下列入口：

| 文件 | 作用 |
| --- | --- |
| `csp.config.h` | 平台默认配置与功能选择 |
| `csp.general.h` | C 平台声明和厂商头文件 |
| `csp.general.hpp` | 可选的 C++ 平台入口 |
| `csp.typedef.h` | GMP 基础类型或外设句柄类型覆盖 |

运行时钩子以 `core/rt/csp_port.h` 为准，主要包括启动、后台循环、后处理和退出。外设函数以 `core/dev/peripheral_port.h` 为准。`core/dev/peripheral_types.h` 会为 `GMP_PORT_UART_T`、`GMP_PORT_SPI_T`、`GMP_PORT_I2C_T`、`GMP_PORT_CAN_T` 和 `GMP_PORT_GPIO_T` 提供默认不透明句柄；CSP 可在包含它之前覆盖这些宏。

工程专用的 `xplt.config.h` 先于 `csp.config.h` 和 GMP 默认配置生效。启用 CTL 时，工程还应提供 `ctl_main.h` 与 `xplt.ctl_interface.h`，由 `ctl/framework/ctl_dispatch.h` 装配快速控制路径。通常由硬件 ISR 或 SIL 收包循环调用 `gmp_base_ctl_step()`，依次执行输入回调、控制器调度和输出回调。

## 4. 新平台最小流程

1. 从 `csp/null` 或最接近的现有平台开始，建立上述 CSP 头文件和 `src/` 实现。
2. 在 `csp.typedef.h` 中按需定义 `GMP_PORT_*_T`；未覆盖的外设句柄默认为 `void *`，GPIO 还会标记为未实现。
3. 实现 `core/rt/csp_port.h` 中实际启用的运行时钩子。
4. 依据 `core/dev/peripheral_port.h` 实现应用真正选择的 `gmp_hal_*` 契约。不要假设每个平台已经实现全部外设。
5. 在工程 `xplt` 层完成引脚、ADC/PWM 标度、ISR 和安全输出绑定。
6. 把新模块加入 `tools/facilities_generator/src_mgr/gmp_framework_dic.json`，再用项目 `gmp_src_mgr` 生成头文件与源文件。
7. 先做编译验证，再按实际能力做仿真或硬件验证，并据实记录验证层级。

## 5. 外设注意事项

- SPI 的 CSP 层只实现 `gmp_hal_spi_bus_write/read/transfer`；`peripheral_port.h` 中的平台无关设备层负责片选与序列化。
- UART 读写契约包含超时；`gmp_hal_uart_read` 还通过 `bytes_read` 返回超时前已接收的长度。
- I2C 接口当前以 `gmp_hal_iic_*` 命名，地址和寄存器长度的精确定义以函数声明及目标实现为准。
- 定义 `SPECIFY_ENABLE_GMP_CAN_SERVICE` 时使用 `core/dev/can` 的静态异步 CAN/CAN FD 服务；未定义时，`peripheral_port.h` 仍暴露历史 Classic CAN 草案 API。两套帧类型和 CSP 钩子不可混用。

## 6. 验证清单

- 确认 `xplt.config.h`、`csp.config.h` 和架构类型组合后没有冲突；
- 确认 PWM 安全极性、ADC 标度、相序、保护和中断优先级；
- 对所有阻塞外设路径验证超时与错误返回；
- 对快速控制路径测量最坏执行时间，避免在 ISR 中加入后台任务；
- 对 C28x 等非 8 位字节模型，使用 GMP 的 `byte_gt` 和协议封装函数，不直接假设 `char` 为 8 位；
- 从最低 `BUILD_LEVEL` 开始硬件调试。`BUILD_LEVEL` 的含义由具体套件定义，并非全仓库统一枚举。
