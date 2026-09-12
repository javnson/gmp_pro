# GMP STM32 Nucleo-144 平台规约

本目录用于统一管理 STM32 Nucleo-144 板卡。它完整继承 Nucleo-64 的控制资源
基线，并将板载以太网 MAC、RMII PHY 连接和确定性的网络验收端点设为必选项。

## 必选资源

- TIM1 或 TIM8：三对互补、中心对齐 PWM，并由 OC4 产生 ADC 采样触发。应用
  明确使能以前，物理 PWM 输出必须保持关闭。
- TIM3 或 TIM4：编码器 A/B 正交输入，以及原生或 GPIO 中断形式的 Z 索引。
- 至少六路固定 ADC 反馈输入，由 PWM 时基触发采样。
- 连接 ST-Link VCP 的串口，RX 使用循环 DMA、TX 使用 DMA，承载 GMP DL。
- 三个板载用户 LED、一路 I2C、1 kHz 系统 Tick。板级元件分别导出三颗 LED，
  双核目标可为两个内核分配独立心跳，并保留一颗故障/链路指示灯。
- IOC 必须保持串行线调试启用（PA13 SWDIO、PA14 SWCLK）。
- 以太网 MAC、板载 PHY 和介质接口；板级元件必须给出本地管理 MAC 和静态
  验收地址。
- DAC、CAN/FDCAN 仍为可选能力。

每块板对应一个 IOC 子目录和一个本地 SDPE 工程；所有板公用
`src/user`、`src/xplt` 和 `src/gmp_src_mgr`。差异只记录在
`ctl/hardware_preset/sdpe_src/mcu_board` 的板级元件中，并生成
`ctrl_settings.h` 稳定别名。

## H753ZI 首个参考实现

- Cortex-M7 400 MHz；TIM1/ADC 控制周期 20 kHz
- ST-Link VCP：USART3，921600 baud
- LAN8742 RMII；MAC：`02:47:4D:50:14:01`
- 静态地址：`192.168.137.2/24`；网关：`192.168.137.1`
- UDP 回显验收端口：`50000`
- GMP DL：TCP `50001`、UDP `50002`

板端通过编译宏 `GMP_NUCLEO_ETH_DL_TRANSPORT` 只选择一种 Ethernet DL 服务，值为
`GMP_NUCLEO_ETH_DL_TCP` 或 `GMP_NUCLEO_ETH_DL_UDP`。H753ZI 构建脚本提供等价的
`-DatalinkTransport TCP|UDP` 参数；默认使用 TCP。端口属于 SDPE 板级元件参数，
不会散落在 user/xplt 业务代码中。USART3 DL 始终保留；示例为 UART 和 Ethernet
分别创建独立的 DL 状态机及设施对象，两端可同时在线。Tunable 和 Memory 指向同一份
应用状态，Scope 则各自拥有采集状态和缓冲区，避免并发组帧和采集互相覆盖。
启用控制中断时，两套 Scope 都由 `ctl_dispatch()` 在每个 20 kHz 控制周期同步采样；
后台调度器只处理协议，不负责实时采集。需要降低显示采样率时使用 Scope 自身的
`sample_divider`。

首次使用或 IOC 更新后，先在目标目录用 STM32CubeMX 的命令行模式执行
`generate_cubemx.txt`。随后使用 `stm32h753zi_nucleo/build.ps1` 编译、
`flash.ps1` 烧录，再运行 `smoke_test.py --transport tcp|udp` 完成 GMP DL 和以太网联合验收。
`dual_link_test.py --network-transport tcp|udp` 会同时打开 ST-Link VCP 和 Ethernet，
以同步的大帧事务验证两条链路，并跨链路写入、读回和恢复 Tunable 参数。
构建脚本会按 IOC 指定的固件版本补齐 CubeMX 板级模板未生成的 ADC HAL
文件；固件包默认从用户的 STM32Cube 仓库查找，也可通过环境变量
`STM32_CUBE_REPOSITORY` 指定仓库根目录。以太网测试前应按板卡手册确认
PHY 相关焊桥/跳线（JP6、SB72）处于接通状态。

`start_sdpe.bat` 是 Nucleo-144 目录统一的 SDPE 启动入口。

## H755ZI-Q 双核参考目标

`stm32h755zi_nucleo` 同时运行两个独立的 GMP 调度器。Cortex-M7 负责
TIM1/ADC 控制链路、TIM3 QEP、I2C1、Ethernet/LwIP 和 LD1；Cortex-M4 通过
HSEM0 唤醒，独占 USART3 及其 RX/TX DMA，并驱动 LD2；LD3 预留为共享故障/
链路状态指示。构建参数 `-DatalinkTransport TCP|UDP` 只切换 CM7 的以太网
DL 协议，两种构建中 CM4 串口 DL 始终启用。
