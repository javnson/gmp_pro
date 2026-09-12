# GMP F28388D ControlCARD 三核参考工程

[English](README.md)

这是 TMDSCNCD28388D ControlCARD 的 GMP 标准参考工程。CPU1、CPU2 和
Cortex-M4 Communication Manager（CM）统一使用 C28x 的 system-u16 数据模型，
从而保证三核之间的 Data Link 数据结构和命令载荷只有一套 ABI。

## 三核职责

| 内核 | 运行职责 | GMP 调度器 | 所属外设 |
| --- | --- | --- | --- |
| CPU1 | 控制 I/O、多核启动、串口 Data Link | 有 | ePWM、ADC、eQEP、SCI-A、I2C-A、SPI-C/SD、DAC、D1/D2 |
| CPU2 | 确定性的正弦/余弦计算 | 有 | CPU 定时器及 CPU1/CM 消息 RAM |
| CM | 专用通信处理器 | 有 | Ethernet、USB、EtherCAT 和网口 Data Link |

CPU1 是唯一的板级初始化管理者：它在启动阶段统一完成通信时钟、复位、引脚复用、
USB device 模式、EtherCAT ESC RAM 初始化和共享外设分配，然后才启动其他内核。
运行期通信所有权交给 CM；两个 C28x CPU 都不运行 USB、Ethernet 或 EtherCAT 服务。

## ControlCARD 资源规约

- EPWM1/2/3 输出三对互补 PWM，EPWM1 SOCA 同步启动全部六路反馈采样。
- 固定反馈通道为 ADCA3/4/5 和 ADCC2/3/4。
- EQEP1 用作 ABZ/index 捕获接口。
- SCI-A 连接 XDS100 虚拟串口，以 115200 baud 承载 system-u16 GMP DL。
- I2C-A 是通用板级控制总线。
- SPI-C 和 GPIO103 完成 SD 卡槽的基础初始化。
- DACA/B/C 作为可选模拟调试输出。
- D1、D2 均初始化。默认版本不启用 CAN-A，因为 ControlCARD 标准 CAN TX
  和 D1 共用 GPIO31；需要 CAN 时应建立显式的无冲突变体。

可编辑的外设配置源是 `cpu1.syscfg` 和 `cpu2.syscfg`，其中 CPU2 按设计不占用引脚。
工程规约固定使用 C2000Ware 5.04 对应的 SysConfig 打开和保存这些文件。

## Data Link 双链路

示例同时保持两条调试链路可用：

| 链路 | 所属内核 | 地址 | 数据模型 |
| --- | --- | --- | --- |
| SCI-A | CPU1 | XDS 虚拟串口，115200 8-N-1 | system-u16 |
| Ethernet TCP | CM | `192.168.137.2:50001` | system-u16 |
| Ethernet UDP | CM | `192.168.137.2:50002` | system-u16 |

编译 CM 的 `CM_FLASH_TCP_U16` 或 `CM_FLASH_UDP_U16` 配置即可选择 TCP/UDP；
上位机 USB 网卡通常配置为 `192.168.137.1/24`，不设置网关。CM 会将每个 u16 DL
单元的低八位打包到以太网字节流，接收时再扩展为 u16，既适配 CM 的八位访存能力，
又不破坏 C28x 系统的数据结构一致性。

两条链路使用相同命令基址：Tunable `0x30`、Memory `0x50`、Scope `0x60`。
它们都能调节 CPU2 正弦波的频率、增益和偏置，并采集两通道计算波形。

## 配置和生成边界

- `src/sdpe_mgr/sdpe_requirement.json` 是工程 SDPE 配置源。
- 可复用板卡 schema/entity 位于上级 `sdpe_component`。
- `src/common/ctrl_settings.h` 及其硬件 preset 是 SDPE 生成结果。
- 根目录唯一的 `gmp_src_mgr` 为三个工程提供同一份 GMP 公共源码。
- `src/common/tricore_shared.h` 定义三核消息 RAM ABI；静态检查按 bit 数比较，避免
  C28x 的 16-bit `CHAR_BIT` 和 CM 的 8-bit `CHAR_BIT` 造成误判。

在本目录运行统一构建入口：

```powershell
.\tools\build.ps1
```

脚本会依次校验和生成 SDPE、生成公共 GMP 源码、在全新 CCS workspace 中导入三个
工程，编译 CPU1、CPU2 以及 TCP/UDP 两个 CM 配置，并把四个镜像复制到已忽略的
`artifacts` 目录。

## 烧录和验证

烧录顺序固定为 CM → CPU2 → CPU1，且最后启动 CPU1。这样 CPU1 先建立共享外设
归属，再从 Flash 启动 CPU2 和 CM。

```powershell
.\tools\flash.ps1 -EthernetProtocol Tcp
.\tools\test_dl.ps1 -Link Both -EthernetProtocol Tcp -SerialPort COM74
```

验证 UDP 时把两个命令中的协议改为 `Udp`。在 DL 发现和读回通过后，可给测试命令
增加 `-CaptureScope`，进一步采集一帧波形。

## 实板验证结果

以下项目已于 2026-09-12 在 TMDSCNCD28388D ControlCARD 实板完成验证。XDS 虚拟
串口为 `COM74`，电脑端 USB 网卡地址为 `192.168.137.1/24`。完成调试器辅助诊断后，
三个镜像又按 CM -> CPU2 -> CPU1 的顺序完成烧录和校验；在没有 DSS 调试会话的
Flash 启动状态下，串口和 TCP 两行测试再次完整通过。

| 路径 | 已完成检查 | 结果 |
| --- | --- | --- |
| CPU1 SCI-A system-u16 | Tunable 发现/读回、Memory 发现/读回、双通道 400 点 Scope 采集 | 通过 |
| CM Ethernet TCP system-u16 | Tunable 发现/读回、Memory 发现/读回、双通道 400 点 Scope 采集 | 通过 |
| CM Ethernet UDP system-u16 | Tunable 发现/读回、Memory 发现/读回、双通道 400 点 Scope 采集 | 通过 |

TCP 测试期间 CM 完成 37 组请求/响应，DL、FIFO、CRC、网络和 lwIP `ERR_MEM` 错误
均为 0，客户端断开后 TCP 发送队列回到 0。UDP 与串口联合测试期间，CPU2 完成
63,242 次正弦/余弦更新，三个内核的调度器心跳均持续推进。当前留在 CM Flash 中的
是已校验通过的 TCP system-u16 镜像。

TI `NO_SYS` Ethernet 移植会在 EMAC 中断路径中提交接收报文，而 GMP 在 CM
调度器中生成应答。为避免两个上下文并发修改 lwIP raw TCP PCB 的发送队列，调度器
对 raw API 的访问使用该移植提供的 `SYS_ARCH_PROTECT` 原语保护。

## 当前协议完成范围

CM 已完成 Ethernet/lwIP 初始化以及 TCP/UDP GMP Data Link 服务；CPU1 会在 CM
启动前完成 USB device 模式、EtherCAT 控制器/ESC RAM 的初始化和所有权移交。
CM 上的 USB class/应用协议、完整 EtherCAT SSC 应用仍是后续扩展点，不能把
“控制器初始化成功”表述为 USB 或 EtherCAT 协议互通已经完成。

当前参考构建环境为 CCS 12.8.1、C2000Ware 5.04.00.00、C28 编译器 22.6.1.LTS、
ARM 编译器 20.2.7.LTS。编译通过不等于真实板卡的烧录、物理链路及 DL 功能验证通过。
