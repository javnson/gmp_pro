# GMP 通信与在环调试系统 (Datalink & Debugging Toolset) 参考手册

## 概述

GMP 调试工具集是一套专为嵌入式控制系统（如电机控制、电力电子变换器）设计的高性能、非阻塞式通信架构。它以 **Datalink (DL) 通信协议** 为核心枢纽，向上支撑了三大核心应用场景：

1. **PIL 在环仿真引擎**：实现 Simulink 算法与 DSP 硬件的时序级闭环。
2. **Tunable 参数整定台**：提供安全的、基于白名单的高频参数读写。
3. **Argos 内存透视仪**：支持跨平台、带安全沙箱的绝对物理地址直接内存访问 (DMA)。

------

## 第一章：核心驱动 - Datalink (DL) 通信协议

DL 层是整个系统的高速公路，它负责字节流的粘包处理、转义、双重 CRC 校验以及数据的收发调度，确保上层组件只需关注纯净的业务载荷 (Payload)。

### 1. 协议帧结构

DL 协议采用强鲁棒性的帧结构，保证在极高波特率下的断线重连与错包拦截：

- **帧头**：目标 ID (Seq) + CMD 指令码 + 载荷长度 + 头部 CRC16。
- **载荷**：变长业务数据 (Payload)。
- **帧尾**：载荷 CRC16。

### 2. 初始化与物理层绑定

在系统启动时，必须首先初始化数据链路层对象：

C

```
#include <core/dev/pil_core.h>

gmp_datalink_t dl;

void system_init(void) 
{
    gmp_dev_dl_init(&dl); // 初始化环形队列与状态机
}
```

### 3. 数据接收 (RX) 与无锁压栈

底层的 UART 接收必须尽可能轻量。在硬件中断 (ISR) 中，只需将物理字节读出，并压入 DL 的 O(1) 无锁环形队列中：

C

```
void flush_dl_rx_buffer(void)
{
    uint16_t fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE);
    data_gt rxBuf[ISR_LOCAL_BUF_SIZE];

    if (fifoLevel > 0) {
        SCI_readCharArray(IRIS_UART_USB_BASE, rxBuf, fifoLevel);
        gmp_dev_dl_push_str(&dl, rxBuf, fifoLevel); // 极速压栈
    }
}

interrupt void INT_IRIS_UART_USB_RX_ISR(void)
{
    flush_dl_rx_buffer();
    // 处理 Overrun 溢出保护与中断清除
    if (SCI_getRxStatus(IRIS_UART_USB_BASE) & SCI_RXSTATUS_OVERRUN) {
        SCI_clearOverflowStatus(IRIS_UART_USB_BASE);
    }
    SCI_clearInterruptStatus(IRIS_UART_USB_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INT_IRIS_UART_USB_RX_INTERRUPT_ACK_GROUP);
}
```

### 4. 消息路由与业务分发 (Task Loop)

DL 的状态机必须在主循环（或低优先级 Task）中持续轮询。通过 `switch(e)` 响应发送就绪和接收成功事件，构建起**责任链模式**的指令拦截网络：

C

```
gmp_task_status_t tsk_dl_protocol(gmp_task_t* tsk)
{
    flush_dl_rx_buffer(); // 保证缓冲区流动
    gmp_dl_event_t e = gmp_dev_dl_loop_cb(&dl);

    switch (e)
    {
    case GMP_DL_EVENT_TX_RDY:
        // 物理发送逻辑：先发头部，再发载荷
        gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl), 
                           gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10);
        if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0) {
            gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_pld_ptr(&dl), 
                               gmp_dev_dl_get_tx_hw_pld_size(&dl), 10);
        }
        gmp_dev_dl_tx_state_done(&dl); // 释放总线锁
        break;

    case GMP_DL_EVENT_RX_OK:
        // 1. 交由上层建筑逐级拦截 (PIL -> Tunable -> Argos)
        // 此处代码见后续章节...

        // 2. 开发者自定义指令拓展 (如 0x99 ECHO 测试)
        if (dl.rx_head.cmd == 0x99) {
            gmp_dev_dl_tx_request(&dl, dl.rx_head.seq_id, GMP_DL_CMD_ECHO, 
                                  dl.expected_payload_len, dl.payload_buf);
            gmp_dev_dl_msg_handled(&dl); // 声明指令已被消费
        } else {
            gmp_dev_dl_default_rx_handler(&dl); // 兜底处理：返回 NACK
        }
        break;
    }
    return GMP_TASK_DONE;
}
```

------

## 第二章：PIL 在环仿真引擎 (Processor-in-the-Loop)

PIL 引擎通过接管硬件平台的 IO 和控制环路，使得 Simulink 能够注入虚拟的 ADC 采样，并回收 DSP 计算出的 PWM 占空比，实现时序级仿真的闭环。

### 1. 引擎挂载

初始化时，为其分配专用的 Base CMD (如 `0x10`) 并绑定 DL 对象：

C

```
gmp_pil_sim_t pil;

void system_init(void) 
{
    gmp_pil_sim_init(&pil, &dl, 0x10);
}
```

### 2. 消息拦截

在 `tsk_dl_protocol` 的 `GMP_DL_EVENT_RX_OK` 事件中，将其置于责任链的顶层（因为 PIL 对实时性要求极高）：

C

```
    case GMP_DL_EVENT_RX_OK:
        // 如果指令属于 PIL 引擎，内部会自动消费并组包回应，break 跳出责任链
        if(gmp_pil_sim_rx_cb(&pil)) break; 
        // ...
```

------

## 第三章：Tunable 变量在线整定台

Tunable 模块采用**静态字典映射机制**，允许上位机通过轻量级的 ID 索引，安全地访问和修改系统中的离散变量（如 PI 控制器的增益 `kp`）。

### 1. 构建数据字典

通过白名单方式声明允许上位机访问的变量，并分配只读 (RO) 或可读写 (RW) 权限：

C

```
float kp1, spd1;

// 静态定义节点 M1 的开放字典
const gmp_param_item_t dict_m1[] = {
    { &kp1,  GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW }, // 可被上位机修改
    { &spd1, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO }, // 仅供上位机监视
};
```

### 2. 引擎挂载与拦截

支持实例化多个 Tunable 对象，分别管理不同的逻辑节点（例如双电机控制）：

C

```
gmp_param_tunable_t srv_m1;
gmp_param_tunable_t srv_m2;

void system_init(void) 
{
    // M1 占用 CMD 0x30/0x31, M2 占用 0x40/0x41
    gmp_param_tunable_init(&srv_m1, &dl, 0x30, dict_m1, 2);
    gmp_param_tunable_init(&srv_m2, &dl, 0x40, dict_m2, 2);
}
```

在 RX 事件流中进行拦截：

C

```
    case GMP_DL_EVENT_RX_OK:
        if (gmp_pil_sim_rx_cb(&pil)) break;
        if (gmp_param_tunable_rx_cb(&srv_m1)) break; // 拦截 M1 指令
        if (gmp_param_tunable_rx_cb(&srv_m2)) break; // 拦截 M2 指令
        // ...
```

------

## 第四章：Argos 内存透视仪 (Memory Perspective)

Argos 机制提供了超越变量字典的绝对物理地址访问能力。为了防止指针飞线导致硬件 HardFault，它引入了**内存安全沙箱 (Sandbox)**，支持连续内存段的高速分片拉取。

### 1. 定义安全沙箱 (Sandbox Regions)

声明允许被透视访问的物理内存区间。为抹平 DSP (字寻址) 与 ARM (字节寻址) 的跨平台差异，必须在编译期使用 `GMP_PORT_DATA_SIZE_PER_BYTES` 宏对物理容量进行修正：

C

```
float memory_pool[1024];

const gmp_mem_region_t mem_regions[] = {
    // 区域 1: 开放一段连续的波形缓存区供上位机提取 (读写权限均可设置)
    {
        .base_addr   = memory_pool, // 直接传入原生指针，规避链接期算术报错
        .byte_length = sizeof(memory_pool) * GMP_PORT_DATA_SIZE_PER_BYTES, // 统一换算为 Byte 容量
        .perm        = GMP_MEM_PERM_RW
    }
};

const uint16_t mem_regions_count = sizeof(mem_regions) / sizeof(mem_regions[0]);
```

### 2. 引擎挂载与拦截

为 Argos 服务分配独立的 Base CMD（例如 `0x50`）：



```C
gmp_mem_persp_t mem_persp_server;

void system_init(void) 
{
    gmp_mem_persp_init(&mem_persp_server, &dl, 0x50, mem_regions, mem_regions_count);
}
```

将其置于指令责任链中：



```C
    case GMP_DL_EVENT_RX_OK:
        if (gmp_pil_sim_rx_cb(&pil)) break;
        if (gmp_param_tunable_rx_cb(&srv_m1)) break;
        
        // 拦截属于 Argos 的绝对地址访问指令
        if (gmp_mem_persp_rx_cb(&mem_persp_server)) break; 
        
        // ...
```

## 第五章：全局主控台与总线监控 (Global Console)

软件右侧的常驻面板是整个系统的物理层大管家，所有子页面的数据收发均受其调度与监控。

### 1. 串口连接与状态指示

- **配置参数**：支持标准串口参数选择，波特率最高可支持至 `2000000`。下拉框支持超宽设备名显示，方便识别虚拟串口或特定的仿真器。
- **状态互锁**：点击“打开串口”后，按钮会自动变为带有左侧绿色高亮条的“关闭串口”状态，同时锁定配置下拉框，防止运行中误触。

### 2. 总线利用率监控 (Bus Load Monitor)

得益于底层的双轨统计引擎，您可以直观地看到总线的真实健康度：

- **TX/RX 分轨测速**：实时显示上位机下发 (TX) 与下位机上报 (RX) 的真实物理吞吐量 (kB/s)。
- **动态负载进度条**：根据当前波特率计算出的理论极限带宽，实时渲染总线占用率。
  - **< 50% (安全)**：进度条显示为 **绿色 / 深紫色**。
  - **50% ~ 80% (警戒)**：进度条变为 **橙色**。
  - **> 80% (拥堵)**：进度条变为 **红色**，此时系统可能出现微小的调度延迟，建议降低自动刷新率。

------

## 第六章：基础协议调试工具 (Tab 1 & Tab 2)

### 1. 标准串口调试助手 (RAW)

用于抓取和盲发底层的原生物理字节（不带 DL 协议头尾）。

- **智能流合并**：系统会自动将连续同向的 RAW 字节流拼接到同一行，提供终端级 (Terminal) 的阅读体验。
- **低权盲发**：在此页面通过 Hex 或 ASCII 盲发的数据被赋予**最低优先级 (Priority=2)**，随意狂点发送也绝对不会阻塞后台高速运行的闭环仿真。

### 2. GMP DL 协议测试 (ECHO)

专门用于探嗅和发送标准 DL 协议帧。

- **NACK 智能高亮**：如果收到下位机的拒绝响应 (CMD = 0x01)，系统会用醒目的 **粉红色** 渲染整行日志，并尝试解析错误码。
- **自定义封装**：输入目标 Seq/ID 与 CMD，在文本框填入 Payload，软件会自动计算 CRC 并完成字节转义发送。

------

## 第七章：PIL 在环仿真工作站 (Tab 3 & Tab 4)

这两个页面配合使用，构成了 Simulink 与 DSP 硬件之间的时序级同步网桥。

### 1. PIL 仿真引擎底座 (Tab 3)

- **Mask 掩码同步**：仿真模型往往极其庞大，通过勾选 `RX Mask` 和 `TX Mask` 下方的复选框，可以精准控制哪些通道参与通信。**支持按住鼠标左键滑动批量勾选**。
- **状态对齐**：修改 Mask 后，必须点击“同步 Mask 状态至下位机”。成功后，按钮变为绿色 `✅ Mask 已同步`，方可启动桥接。

### 2. Simulink-PIL 网桥 (Tab 4)

- **UDP 双向透传**：配置好 MATLAB 节点的 IP 与端口，并设置与 Simulink S-Function 一致的“仿真步长 (Step Size)”。
- **自动接管与容错**：
  - 点击“启动 PIL 桥接服务”后，软件将**自动锁定**第 3 页面的手动控制权，完全由 UDP 流量驱动。
  - **双重看门狗**：当串口返回超时，系统会自动重传 (`Priority=1`)；当 MATLAB 侧计算卡顿（拖拽 UI 导致），系统会自动挂起并打出警告日志，恢复后无缝续传。
- **在环指标实时图表**：内置浅色系折线图，实时绘制闭环推进频率 (Hz)，并统计丢包率与重传次数，仿真质量一目了然。

------

## 第八章：Tunable 参数在线整定台 (Tab 5)

工业级的动态变量监视与修改中枢，具备**绝对特权 (Priority=0)**，确保调参指令瞬间送达。

- **多实例管理**：支持点击 `➕ 添加可调对象` 无限开启新标签页，每个标签页对应一个独立的逻辑节点 (Base CMD)。
- **配置导入/导出**：
  - **从 C 代码提取**：直接复制下位机的 `const gmp_param_item_t dict[]` 数组定义，一键解析出包含名称、类型、读写权限的字典。
  - **JSON 持久化**：解析好的字典与刷新频率可保存为 JSON 文件，下次打开秒加载。
- **安全交互状态机**：
  - **全局定时刷新**：启用后，变量框会自动更新。数值不变底色为浅绿，发生突变则会**闪烁黄灯** 800ms 后褪色。
  - **编辑互锁 (Edit Lock)**：当光标点入输入框开始敲击时，输入框变为橙色警示。此时全局刷新会自动为该变量“让路”，防止你输入到一半被下位机数据覆盖；按下回车发送成功后，恢复自动刷新。

------

## 第九章：Argos 内存透视分析仪 (Tab 6)

高级调试后门，支持跨越变量字典，通过绝对物理地址直接访问硬件内存 (DMA) 并导出波形。

### 1. 寻址与颗粒度配置

- **物理地址换算警告**：在 DSP (C2000) 平台抓取数据时，从 map 文件查到的字地址必须 **乘以 2** 转换为字节地址填入上位机。
- **颗粒度选项**：严格按照硬件位宽选择 `8-bit`、`16-bit` 或 `32-bit`。
- **智能分片提取**：输入大容量长度（如 `4000 Bytes`），点击读取后，上位机会自动将其切分为 `128 Bytes` 的碎片分批索要，彻底避免挤爆通信总线。

### 2. 网格交互与局部写 (Local Overwrite)

- **动态类型强转 (CAST)**：
  - **局部强转**：选中几个 Hex 格子，右键选择 `🔄 强转类型 -> F32`，该区域会高亮并融合显示为浮点数。
  - **全局批量强转**：右键选择 `🌐 全局批量强转`，瞬间将整个内存段格式化为同构的数组，极为适合波形缓存分析。
- **双击局部覆写**：直接双击某个已强转的浮点数（或原始 Hex 单元格），输入新数值后回车。系统将自动生成对应长度的原子修改指令下发。由于沙箱保护机制，若该区域为 RO (只读)，将会被下位机拒绝并弹红字警告。
- **CSV 数据导出**：调试完成后，点击 `📊 导出为 CSV`，当前网格中所有已应用类型强转的真实物理数据（含地址偏移量表头），将被完美导出，可直接投入 Excel 或 MATLAB 进行画图分析。