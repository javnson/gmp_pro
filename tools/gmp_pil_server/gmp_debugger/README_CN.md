# GMP Data Link Debugger

[English](README.md) | **简体中文**

本目录保存唯一维护的 GMP Data Link 与处理器在环 Python/PyQt 上位机。u8 与
u16 目标使用相同的串行线协议，因此共享全部协议编解码、资源发现、示波器和界面代码。

完成 GMP 安装后，根据目标运行一个带环境守卫的入口：

- `run_u8.bat`：字节寻址目标，包括 STM32、x86 和大多数现代 CPU。
- `run_u16.bat`：16 位寻址 DSP 目标，主要包括 TI C28x。

入口选择只用于标识窗口中的目标契约，不会分叉上位机协议实现。下位机 C 后端仍由
`GMP_PORT_DATA_SIZE_PER_BYTES` 自动选择。

Memory Perspective 和 Tunable 页面可以导入目标上报的具名资源。Data Link Scope
是独立服务，可设置触发模式、源、Level 和预触发位置。Continuous Display 会在每次
完整快照后自动重新配置和布防；Waveform Persistence 在独立分组中显示衰减的历史波形。

System Log 为每个页面分配固定配色，并在 Log Sources 下拉列表中提供可勾选过滤项。

Memory Perspective 在线协议中始终使用字节地址。从 C28x map 文件取得原生字地址后，
手动输入前需要乘以 2。
