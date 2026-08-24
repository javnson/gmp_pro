# AXI4-Lite 寄存器映射

[English](REGISTER_MAP.md)

地址均为字节偏移，数据均为 32 位小端。

| 偏移 | 名称 | 属性 | 说明 |
| --- | --- | --- | --- |
| `0x000/004` | ID/VERSION | 只读 | `0x49325232`，版本 1.0 |
| `0x008` | CONTROL | 读写/W1P | bit0 运行，bit1 清状态，bit2 自动 ADC，bit3 自动 DAC，bit8 清计数/错误 |
| `0x00C` | STATUS | 只读 | 流水线忙、手动样本待处理、配置错误、已有结果 |
| `0x010/014` | MANUAL/LATEST | 写/只读 | 手动 Q3.28 样本、最近结果 |
| `0x018/01C` | SAMPLE/SAT_COUNT | 只读 | 完成样本数、至少一节发生饱和的时钟数 |
| `0x020` | ACTIVE_SECTIONS | shadow | 有效节数 `0..NUM_SECTIONS` |
| `0x024` | COMMIT | W1P | bit0 在空闲时原子提交系数并清状态 |
| `0x028/02C` | FORMAT/NUM | 只读 | 位宽、小数位、综合节数 |
| `0x030/034` | ADC_CONTROL/SELECTED | 读写/只读 | 掩码、通道、自动控制和所选原始样本 |
| `0x040+0x14*n` | SOS[n] | shadow | 依次为 B0、B1、B2、A1、A2，均为 Q3.28 |
| `0x0A0..0x0B0` | PWM | 读写 | 控制、周期、比较、死区、相位 |
| `0x0B4/0x0B8` | DAC_MANUAL | 读写 | 两两打包的 4 通道手动码值 |
| `0x0C0..0x0DC` | ADC_CH0..7 | 只读 | 8 通道原始样本 |
| `0x0E0` | DMA_CONTROL | 读写/W1P | bit0 外存模式，bit1 启动，bit2 注入辨识路由 |
| `0x0E4/0x0E8` | DMA_BASE | 读写 | 4 字节对齐的读/写基址 |
| `0x0EC` | DMA_COUNT | 读写 | 32 位样本数 |
| `0x0F0` | DMA_STATUS | 只读 | 读忙、写忙、完成锁存、错误 |

TD SPI 每次事务包含 16 位命令和 16 位数据。命令 bit15 为读标志，bits14:8 为 16 位寄存器序号；
偶数序号访问 AXI 字低半字，奇数序号访问高半字。为保证读数据在第二帧前返回，FPGA 系统时钟应不低于
SPI 时钟的 4 倍。
