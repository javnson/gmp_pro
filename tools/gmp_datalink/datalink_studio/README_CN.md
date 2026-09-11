# GMP Data Link Studio

[English](README.md) | **简体中文**

本目录保存唯一维护的 GMP Data Link 与处理器在环 Python/PyQt 上位机。u8 与
u16 目标使用相同的线协议，因此串口、TCP 和 UDP 共享全部协议编解码、资源发现、
示波器和界面代码。

完成 GMP 安装后，根据目标运行一个带环境守卫的入口：

- `run_u8.bat`：字节寻址目标，包括 STM32、x86 和大多数现代 CPU。
- `run_u16.bat`：16 位寻址 DSP 目标，主要包括 TI C28x。
- `run_eth_tcp_u8.bat` / `run_eth_tcp_u16.bat`：TCP 目标。
- `run_eth_udp_u8.bat` / `run_eth_udp_u16.bat`：UDP 目标。

Ethernet 入口默认连接 `192.168.137.2`，TCP 使用 `50001`，UDP 使用 `50002`。
可在启动前通过 `GMP_DATALINK_ETH_HOST` 和 `GMP_DATALINK_ETH_PORT` 覆盖。TCP
按连续字节流解析；UDP 每个 Data Link 请求和响应各占一个数据报。

入口选择只用于标识窗口中的目标契约，不会分叉上位机协议实现。下位机 C 后端仍由
`GMP_PORT_DATA_SIZE_PER_BYTES` 自动选择。

Memory Perspective 和 Tunable 页面可以导入目标上报的具名资源。Data Link Scope
是独立服务，可设置触发模式、源、Level 和预触发位置。Continuous Display 打开后会
立即配置并启动采集，并在每次完整快照后自动重新布防；关闭时只停止后续重触发，保留
当前画面。Waveform Persistence 在独立分组中显示衰减的历史波形。Waveform Export
可以将当前帧，或仍被保留的所有余晖帧连同当前帧保存为 CSV。

Tunable 页的 **Import C** 支持当前四字段 `gmp_param_item_t` 初始化器：
`{address, type, permission, name}`。`name` 可以是 C 字符串、`NULL` 或空字符串；
没有有效名称时，界面使用地址表达式作为参数名称。为兼容现有工程，旧三字段初始化器
以及早期工具使用的五字段“名称、单位”格式仍可导入。

System Log 为每个页面分配固定配色，并在 Log Sources 下拉列表中提供可勾选过滤项。

`HermesDatalinkQt.connect_transport()` 允许宿主工具提供字节写回调，并用
`feed_transport()` 注入接收字节；物理串口和受管进程因此共享同一套帧编解码、优先级
队列和功能页面。CCTL Result Viewer 的 Data Link 页使用的就是该入口。

`HermesDatalinkQt.connect_network(protocol, host, port)` 是图形界面的公共网络入口，
`protocol` 为 `tcp` 或 `udp`。所有页面仍共享一个连接和一个发送队列。

Memory Perspective 在线协议中始终使用字节地址。从 C28x map 文件取得原生字地址后，
手动输入前需要乘以 2。

## 无界面 API

自动化脚本、测试程序和 AI 调试 Agent 可以在不打开 PyQt 界面的情况下使用相同服务。
公共同步 Python API 支持资源发现、带类型的 Tunable 访问、白名单分块内存访问、触发
或连续 Scope 采集以及 CSV 波形导出。

请参阅[英文 API 手册](apis/README.md)或[中文 API 手册](apis/README_CN.md)。
