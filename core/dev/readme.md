# GMP 跨平台硬件抽象层 (HAL) 移植开发指南

## 1. 架构设计哲学 (Design Philosophy)

GMP (Generic Motor Platform) HAL 层的核心目标是：**为上层应用（如 FOC 电机控制算法、CANopen 协议栈）提供绝对统一、内存安全、无死锁隐患的物理外设接口，同时彻底抹平不同芯片架构（如 ARM 的 8-bit Byte 与 DSP 的 16-bit Byte）之间的硬件鸿沟。**

当您拿到一款全新的芯片（如 NXP、Infineon 或新的国产 MCU）需要将其接入 GMP 框架时，请务必遵循以下三大铁律：

1. **零动态内存 (Zero Malloc)**：工业控制严禁内存碎片。所有的缓冲、队列必须在初始化时由上层静态分配并传入。
2. **严禁死等 (No Infinite Blocking)**：FOC 中断（如 10kHz）对时间极其敏感。底层 CSP（芯片支持包）驱动中，凡是涉及到 `while` 检查硬件状态的地方，**必须**强制加入 `timeout` 超时退出机制。
3. **分层解耦 (Two-Layer Abstraction)**：
   - **CSP 层 (Layer 1)**：即 `gmp_hal_xxx_bus_xxx`，只负责操作底层的寄存器或调用原厂库，不涉及任何软件逻辑。
   - **逻辑设备层 (Layer 2)**：即 `gmp_hal_xxx_dev_xxx`，负责管理 CS（片选）引脚、大小端转换、报文序列化以及高低优先级队列调度。

------

## 2. 核心数据类型与语义规范

### 2.1 句柄 (Handles)

框架大量使用不透明指针（Opaque Pointers）来传递硬件实例：

- `gpio_halt` / `uart_halt` / `spi_halt` / `iic_halt` / `can_halt`。
- **移植建议**：在 CSP 实现中，直接将其强转为对应芯片的原厂句柄（如 STM32 的 `UART_HandleTypeDef*`）或寄存器基址（如 C2000 的 `uint32_t base_address`）。

### 2.2 错误码 (`ec_gt`)

所有硬件操作必须返回统一的错误码，禁止返回 `void`：

- `GMP_EC_OK`：操作成功。
- `GMP_EC_BUSY`：硬件外设正忙（发送邮箱满），非阻塞调用时应立即返回此状态。
- `GMP_EC_TIMEOUT`：硬件无响应或线路断开，超时止损。
- `GMP_EC_NACK`：I2C 特有的无应答错误。

------

## 3. 外设接口实现指南 (API Semantics)

### 3.1 GPIO 接口 (`gmp_hal_gpio_xxx`)

最基础的数字输入输出。

- **API 语义**：`set_dir` (方向)、`write` (输出)、`read` (输入)。
- **移植要点**：必须处理传入句柄为 `NULL` 的安全情况。在具有引脚复用功能的芯片上，此处的初始化不包含复用配置，仅处理方向和电平。

### 3.2 UART 接口 (`gmp_hal_uart_xxx`)

被设计为带超时保护的安全阻塞式流水管，支持变长数据和断帧恢复。

- **`gmp_hal_uart_write(uart, data, len, timeout)`**：
  - 将 `len` 字节的数据发完，或超时后强行退出。
- **`gmp_hal_uart_read(uart, data, len, timeout, bytes_read)`**：
  - **语义精髓**：尝试读取 `len` 长度的数据。如果线路被拔掉导致超时，CSP 必须通过 `bytes_read` 返回超时前**已经成功读到**的字节数，以支持协议栈的断帧重组。
- **`gmp_hal_uart_get_rx_available(uart)`**：
  - 返回底层硬件 FIFO 中现存的字节数。如果是 STM32 等没有深层 FIFO 的芯片，有数据则返回 1，无数据返回 0。用于上层非阻塞嗅探。

### 3.3 SPI 总线与逻辑设备 (`gmp_hal_spi_xxx`)

SPI 采用了强烈的**两层架构**，这也是移植的核心所在。

#### Layer 1: CSP 物理层 (`gmp_hal_spi_bus_xxx`)

- **职责**：CSP 开发者**只**需要实现这三个函数：`bus_write`, `bus_read`, `bus_transfer`。
- **规则**：**绝对不要在这些函数里操作 CS（片选）引脚！** 硬件 SPI 必须配置为 8-bit 数据宽度（在 C2000 DSP 上，如果设为 8-bit，要注意数据需要左移对齐的问题，参考 `peripheral_driver.c`）。

#### Layer 2: 逻辑设备层 (`gmp_hal_spi_dev_xxx`)

- 这一层是平台无关的，已经由 GMP Core 实现。
- **机制**：它会将 `spi_halt bus` 和 `gpio_halt cs_pin` 绑定为一个 `spi_device_halt`。每次调用 `read_16b` 或 `write_reg` 时，Core 会**自动拉低 CS，发送序列化数据，再拉高 CS**。
- **序列化安全**：所有数据被强制按照 **MSB-First（大端）** 串行化为字节流，彻底免疫芯片平台的 Endianness 差异。

### 3.4 I2C 总线 (`gmp_hal_iic_xxx`)

对 I2C 的通信序列进行了高度工业化抽象。

- **寻址约定**：传入的 `dev_addr` 为 **7 位右对齐**地址。如果在 STM32 上实现，CSP 需要将其左移 1 位 (`addr << 1`)；在 C2000 上则直接写入寄存器。
- **核心 API 分类**：
  - `write_cmd`：无子地址的盲发（通常用于唤醒或复位）。
  - `write/read_reg`：操作特定寄存器（如读写传感器配置），自动处理 Address -> Restart -> Data 序列。
  - `write/read_mem`：连续块读写（如 EEPROM）。

### 3.5 CAN 总线 (Boss 级核心架构)

CAN 的设计是整个 HAL 中最精密的部分。为了同时兼容 **独立邮箱架构 (如 DSP 32 Mailboxes)** 和 **FIFO流水线架构 (如 STM32)**，采用事件驱动与中断泵机制。

#### 3.5.1 报文内存安全 (`gmp_can_msg_t`)

C

```
typedef struct {
    uint32_t id;
    bool is_extended; bool is_remote; uint8_t dlc;
    uint32_t data_32[2]; // 核心：强制 64 位对齐！
} gmp_can_msg_t;
```

- **严禁修改**：由于 DSP 的一个 byte 是 16 位的，传统的 `uint8_t data[8]` 会导致内存崩溃。使用 `data_32[2]` 结合 `gmp_can_payload_get/set_u8` 等原语，实现了全平台绝对安全的跨边界取值。

#### 3.5.2 CSP 物理层移植要求

新芯片移植时，CSP 层只需做两件事：

1. **实现硬件发送 `gmp_hal_can_bus_write`**：
   - 检查硬件邮箱/FIFO 是否有空位。
   - **有空位**：填入数据触发发送，返回 `GMP_EC_OK`。
   - **没空位**：**严禁死等！** 立刻返回 `GMP_EC_BUSY`。
2. **挂载中断发动机 (ISR Hooks)**：
   - **发送完成中断 (TX Empty)**：硬件每发完一帧，CSP 必须在中断内调用 `gmp_can_node_tx_isr_pump(node)`，将软件队列里的数据“泵”入硬件。
   - **接收中断 (RX Pending)**：硬件每收到一帧，CSP 必须在中断内将其转为标准报文，并调用 `gmp_can_node_rx_isr_router(node, &msg)`，由路由器决定是触发实时的 PDO 回调，还是压入低优先级的 SDO 队列。

#### 3.5.3 逻辑调度层 (`gmp_can_node_t`)

GMP Core 维护了双端队列（Deque）。

- **发送（双端入队）**：高优报文（如 PDO）调用 `push_front` 插队，普通报文（如 SDO）调用 `push_back` 排队。
- **接收（分流过滤）**：通过 `fast_rx_mask` 将具有强实时要求的报文直接分发至 FOC 控制环，其余报文进入 `rx_slow_queue` 供主循环慢慢轮询。

------

## 4. 给移植者的 Checklist

当您开始将此框架移植到新的 MCU 时，请对照以下步骤：

- [ ] 在 `peripheral_mapping.h` 中，将抽象句柄类型映射为新芯片的底层类型（或直接保持 `void*` 并在 `.c` 中强转）。
- [ ] 实现基础的 GPIO 引脚电平控制。
- [ ] 实现 UART，**务必在内部实现超时逻辑**（可借用系统 SysTick 或硬件 Timer）。
- [ ] 实现 SPI Layer 1 的三个 `bus_xxx` 函数，确保其为纯粹的字节流进出，不控制 CS，不改变端序。
- [ ] 实现 I2C，注意 7-bit 地址的左右对齐要求，并在必要时清除标志位防止总线挂死。
- [ ] 实现 CAN 的 `bus_write`（必须非阻塞）并配置好 TX/RX 硬件中断的回调对接，验证 Deque 的进出是否正常。

------

*注：本指南配合 `peripheral_port.h` 食用体验最佳。所有上层传感器外设（如 AD9834, DAC8563, AS5048A 等）均以此 HAL 层为基座，底层 CSP 的可靠移植是整个系统稳定的根基。*