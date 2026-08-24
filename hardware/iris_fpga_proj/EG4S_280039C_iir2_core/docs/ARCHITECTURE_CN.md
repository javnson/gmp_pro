# 架构说明

[English](ARCHITECTURE.md)

## 数据路径

实时控制路径为 `ADC → adc_axis_adapter → iir2_control_core → DAC/ePWM`；批量辨识路径为
`外部存储器 → AXI4 MM2S → IIR2 AXI4-Stream → AXI4 S2MM → 外部存储器`。

配置接口选择 AXI4-Lite，因为系数和外设寄存器不需要 burst；样本计算选择 AXI4-Stream，利用
`valid/ready/last` 完成无损背压；大容量外存搬运使用支持 burst 的完整 AXI4。

DMA 只保存地址、计数和握手状态，不实例化负载 RAM，因此不会为样本数据推断 BRAM。它使用最大
`MAX_BURST_LEN` 的 INCR burst，保证不跨越 4 KiB 边界，读写通道可并行，每个方向最多一个在途 burst。

## IIR2 数值与流水线

默认数据和系数均为 32 位有符号 Q3.28，计算式为：

`y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]`

每节 5 个乘法并行执行，乘积、展宽累加、缩放饱和结果分别寄存。单节从接收到完成经过三个寄存边沿，
每 3 个时钟可接收一个新样本；级联中的多个物理算子同时处理不同样本。100 MHz 时，首节启动能力约
33.3 Msps，远高于常见电力电子控制频率。

系数采用 shadow/commit，只有流水线空闲时才原子更新。系数和每节的 `x1/x2/y1/y2` 反馈状态规模很小，
保留在触发器中；若把实时反馈状态放入外存，每个控制周期的延迟会受仲裁影响。注入波形、连续采样、
测量记录和批量结果全部放入外部存储器。

## 平台边界

控制、DMA、AXI 和调制 RTL 均为厂商无关 Verilog。AMD Xilinx 可将 AXI4 主接口连接 MIG、SmartConnect
或 Zynq PS 的 HP/HPC 端口，将 AXI4-Lite 接处理器互联。TD 平台应把同一 AXI4 主接口接到所选外存控制器。
当前 EG4S20 约束文件未提供外存引脚，因此已提交的 TD 顶层运行实时 ADC/DAC 路径；实际带外存的 TD 板
需要补充其存储控制器 IP 和引脚约束。
