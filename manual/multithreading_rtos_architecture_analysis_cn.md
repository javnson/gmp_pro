# GMP 多线程与 RTOS 兼容性分析及演进建议

[返回手册索引](README_CN.md)

> 分析基线：`developping` 分支，提交 `9687fcaf`；分析日期：2026-09-12。
> 本文分析仓库自有代码、典型 CSP 和 `ctl/suite` 工程。第三方库与生成代码仅用于确认集成方式，不据此推断 GMP 已具备相应能力。

## 1. 结论摘要

当前 GMP **支持并发场景，并已有一个可运行的 FreeRTOS 参考工程，但尚不能
称为“仓库级完整支持多线程或 RTOS”**。

已有能力主要包括：

1. `core/pm/function_scheduler` 提供单执行流、非抢占、协作式的后台任务调度；
2. 大多数控制套件采用“高频控制 ISR + 后台主循环”的两域并发模型；
3. CCTL 在宿主机侧使用 `std::thread`、互斥量、条件变量和原子 SPSC 队列隔离文件及控制台 I/O；
4. STM32H755 示例采用两个独立镜像和两个独立 GMP 调度器，属于 AMP；
5. 当前工作区存在 F28388D CPU1/CPU2/CM 三核草案，但它不是已提交基线，且共享内存协议仍需补齐内存序和一致性保证。

阻止 GMP 直接、安全地运行在多线程或 RTOS 上的主要问题是：

- 临界区接口仅表示“关/开本核中断”，没有保存旧状态，不能可靠嵌套，也不能充当 RTOS 互斥量或跨核锁；
- `core/pm` 调度器本身不是线程安全对象，`YIELD` 语义与实现不一致，`BUSY` 可无限占用调度器；
- 控制对象、调参变量、内存分配器、打印缓冲区和部分全局服务缺少明确的所有权及同步契约；
- 多个 FIFO 只使用 `volatile` 索引，适合受约束的单核 ISR/主循环 SPSC 场景，不具备 C/C++ 内存模型下的跨线程、跨核可移植性；
- 运行时初始化会在 `gmp_base_init()` 末尾进入可能启用中断的 `gmp_csp_post_process()`，难以保证 RTOS 对象和任务已先准备完毕；
- 已有 STM32 FreeRTOS 最小后端和 H753 参考工程，但通用 OSAL、线程竞态测试、
  更多 RTOS/平台端口和套件级数据所有权改造仍未完成。

推荐采用渐进式架构，而不是把每个现有 `gmp_task_t` 直接映射为一个 RTOS 线程：

- 控制 ISR 或最高优先级控制任务保持对 CTL 状态的**单写者所有权**；
- `core/pm` 首先作为 RTOS 中的一个兼容工作任务运行，最大限度保留现有套件；
- 通信、监控、日志等服务通过有界队列、邮箱或双缓冲与控制域交换命令和快照；
- 新增轻量 OS 抽象层，明确区分中断屏蔽、RTOS 互斥、ISR 安全事件、原子操作和跨核同步；
- 在兼容模式稳定后，再按收益逐个迁移服务为原生 RTOS 任务。

### 1.1 已落地的 H753 FreeRTOS 参考实现

本次在 [`csp/stm32/Nucleo_144_RTOS`](../csp/stm32/Nucleo_144_RTOS)
新增了独立工程，原 `Nucleo_144/stm32h753zi_nucleo` 裸机工程文件保持不变。
实现采用本文建议的第一阶段架构：FreeRTOS 拥有系统调度权，一个高优先级
RTOS 任务调用 GMP 的 `prepare/activate/loop`，`core/pm` 只在该任务内部调度
非阻塞函数；20 kHz ADC DMA 控制中断保持独立，另有普通用户任务并行运行。

2026-09-12 在实物 NUCLEO-H753ZI 上完成了下载与 SWD 双样本验证：GMP 服务
循环、独立 RTOS 用户任务、`core/pm` 心跳以及 ADC DMA 控制中断的计数均持续
增长。具体构建配置、内存占用和读数见
[`validation.md`](../csp/stm32/Nucleo_144_RTOS/stm32h753zi_nucleo/validation.md)。
这证明目标执行模型可行，但并不改变本文对通用 OSAL、数据所有权、队列内存序、
调参事务和 `core/pm` 公平性问题的判断。

## 2. 范围与判定标准

本次重点检查：

- `core/pm` 的调度、状态机、定时和工作流设施；
- `core/rt` 的启动、循环和控制步进生命周期；
- `core/base`、`core/dev`、`core/mm`、`core/protocol` 中可能跨执行域访问的基础设施；
- `csp` 中临界区、时钟、宿主机线程和多核实现；
- `ctl/suite/*/src` 与典型目标工程中控制 ISR、后台任务和通信任务的关系；
- 原生测试对调度语义和并发行为的覆盖。

本文使用以下术语：

- **并发**：ISR、主循环、DMA、线程或其他核心在时间上可能交错执行；
- **多线程安全**：在规定的线程调用关系下，数据竞争、生命周期和内存可见性均有明确保证；
- **RTOS 兼容**：初始化顺序、阻塞规则、ISR API、优先级、静态内存和时钟可由 RTOS 端口可靠实现；
- **SMP**：多个核心共享一个 OS 和地址空间；
- **AMP**：每个核心运行独立镜像或独立调度器，通过 IPC/共享内存协作。

## 3. 当前执行模型

### 3.1 GMP 运行时

默认入口位于 [`core/rt/gmp_runtime.h`](../core/rt/gmp_runtime.h)：

```text
gmp_base_entry
  ├─ gmp_base_init
  │    ├─ CSP 启动与外设初始化
  │    ├─ ctl_init / init
  │    └─ gmp_csp_post_process（部分平台可能在此启用中断）
  └─ while (true)
       └─ gmp_base_loop
            ├─ CSP loop / 状态机派发
            ├─ mainloop
            └─ ctl_mainloop

硬件定时或 ADC/PWM ISR
  └─ gmp_base_ctl_step
       ├─ ctl_input_callback
       ├─ ctl_dispatch
       └─ ctl_output_callback
```

这个拆分使 RTOS 适配具有可行基础：RTOS 入口可以绕过 `gmp_base_entry()` 的永久循环，单独调用初始化和循环函数。但当前“准备资源”和“允许中断/开始运行”仍混在同一阶段，尚不足以形成安全的 RTOS 启动协议。

### 3.2 `core/pm` 调度器

[`core/pm/function_scheduler/function_scheduler.h`](../core/pm/function_scheduler/function_scheduler.h) 和 [`function_scheduler.c`](../core/pm/function_scheduler/src/function_scheduler.c) 定义了固定容量为 16 的任务表。每次 `gmp_scheduler_dispatch()` 最多执行一个任务：

- 若存在 `blocking_task`，只运行该任务；
- 否则从任务表下标 0 开始，运行第一个到期任务；
- `BUSY` 会使任务持续占有调度器；
- 非 `BUSY` 返回值会更新 `last_run`。

因此它是**单线程协作式步骤调度器**，不是线程调度器，也不是 RTOS 内核。它没有任务栈、抢占、优先级继承、事件等待、核亲和性、取消、加入、就绪队列或截止期管理。

### 3.3 `ctl/suite` 的实际使用方式

9 个套件目录中，8 个共享 `src/user_main.c` 使用 `gmp_scheduler_t`。典型任务包括：

- 1～10 ms：通信、监控、慢速保护和部分 `ctl_mainloop`；
- 500～1000 ms：LED、状态显示；
- 启动或状态推进任务。

高频闭环通常不在该调度器中执行，而由目标工程的 ADC/PWM ISR 调用 `gmp_base_ctl_step()`。这实际上已形成两个执行域：

| 执行域 | 当前职责 | 实时属性 |
|---|---|---|
| 控制 ISR | 采样、控制律、PWM 输出 | 高频、硬实时、不可阻塞 |
| 主循环/PM | 通信、监控、状态推进、LED | 低频、协作式、软实时 |

这是合理的起点，但两个域之间共享的控制对象、状态标志、保护量和调参变量缺少统一的所有权协议。

## 4. 已存在的并发与多核模式

| 模式 | 仓库证据 | 当前评价 |
|---|---|---|
| 单核 ISR + 主循环 | 大多数 `ctl/suite` 目标 | 已广泛使用，但同步契约分散 |
| 宿主机工作线程 | [`csp/cctl`](../csp/cctl) | 实现较成熟，控制线程与 I/O 线程通过原子 SPSC 隔离 |
| STM32H755 AMP | [`csp/stm32/Nucleo_144/README.md`](../csp/stm32/Nucleo_144/README.md) | 两个独立 GMP 调度器；HSEM 主要用于启动，不等价于共享 RTOS |
| F28388D 三核 AMP | `csp/c28x_syscfg/controlcard_28388` 当前工作区 | 尚未提交的草案；不能作为发布能力，消息一致性仍需加固 |
| RTOS | STM32 配置多为 `USE_RTOS=0`，C29x 链接 NoRTOS | 没有统一端口和参考工程 |
| SMP | 无统一跨核原子、锁、缓存维护和测试 | 当前不支持 |

CCTL 的做法值得复用：主仿真/控制状态仍由一个线程拥有，文件和控制台服务运行在工作线程；记录通过预分配、无锁 SPSC 队列传递，满时丢弃而不是阻塞控制路径。参见 [`cctl/dsa/spsc_record_ring.hpp`](../cctl/dsa/spsc_record_ring.hpp) 和 [`csp/cctl/src/csp_cctl.cpp`](../csp/cctl/src/csp_cctl.cpp)。

## 5. 关键问题与风险

### 5.1 临界区接口不能承担通用同步

[`core/base/gmp_base.h`](../core/base/gmp_base.h) 只暴露：

```c
void gmp_base_enter_critical(void);
void gmp_base_leave_critical(void);
```

STM32、Nation32、C28x、C29x 和 Zynq7 的典型实现是无条件关中断，再无条件开中断。存在以下问题：

- 不保存进入前的中断状态，嵌套调用可能提前开中断；
- 从 ISR 调用时，退出可能改变原有中断屏蔽状态；
- 只能保护本核心，不能阻止另一核心或 DMA 访问；
- 没有定义编译器屏障、CPU 内存屏障或缓存维护；
- 若未来把它替换成 RTOS mutex，ISR 调用将非法，且可能发生优先级反转。

RP2040 端口保存每核中断状态并维护嵌套深度，是更好的本核临界区实现，但仍不是跨核锁。Windows/CCTL 使用递归互斥量，只适用于线程上下文。不同 CSP 对同一接口的语义实际上并不一致。

**风险等级：P0。** 在引入 RTOS 或多核前必须先拆分同步语义。

### 5.2 `core/pm` 语义与公平性问题

调度器存在以下可观察行为：

1. 文档把 `GMP_TASK_STATUS_YIELD` 描述为需要继续调用，但实现只对 `BUSY` 保留 `blocking_task`，所以 `YIELD` 实际等同于本轮完成；
2. `BUSY` 任务可以无限垄断调度器，禁用该任务也不会立即解除阻塞；
3. 每轮从下标 0 扫描，周期为 0 或总是到期的前部任务可能使后部任务饥饿；
4. 完成时把 `last_run` 设置为当前时间，执行延迟会累积为周期漂移；
5. 没有明确的超期补偿、跳帧或追赶策略；
6. 任务表和状态字段没有同步，运行期增加任务或并发派发不安全；
7. 现有单元测试把 “BUSY 保持优先直到完成” 固化为期望，但没有覆盖 `YIELD`、公平性、回绕、并发访问和截止期。

**风险等级：P1。** 不妨碍当前单循环使用，但会放大 RTOS 兼容层中的优先级和响应问题。

### 5.3 `volatile` 不是线程或多核同步

[`core/base/ds/ring_buf.h`](../core/base/ds/ring_buf.h)、DataLink FIFO 和多个套件状态使用 `volatile`。`volatile` 只能约束部分编译器访问，不能建立线程之间的 happens-before，也不能保证跨核缓存一致性。

CANopen 文档已明确限定其环形缓冲区为单核 ISR/主循环 SPSC，并指出多核适配器必须提供序列化或内存屏障。DataLink 也隐含“一个硬件 FIFO 所有者”的模型。这些限制应上升为所有队列 API 的正式契约，而不是依赖调用者推断。

**风险等级：P0/P1。** 单核受控 SPSC 可继续使用；跨线程或跨核必须换用原子索引与 acquire/release，或在端口层显式加锁。

### 5.4 控制状态和在线调参缺少一致快照

多个套件的后台通信任务可以通过 Tunable/DataLink 直接读写变量，而控制 ISR 同时读取或更新控制器状态。当前实现中，Tunable 通过 `memcpy` 或直接解引用访问目标地址，没有：

- 所有权检查；
- 原子宽度约束；
- 版本号或事务边界；
- 在控制采样边界生效的提交机制；
- 多字段参数的一致快照。

即使单个 16/32 位访问在某个平台上天然原子，多字段参数组也可能出现“半新半旧”。在 RTOS、多核或有缓存的平台上，这会成为真实的数据竞争和控制稳定性风险。

**风险等级：P0。** 控制对象应由控制域单写，通信域只提交命令消息；参数在采样边界统一验证并生效。

### 5.5 运行时生命周期不适合直接启动 RTOS

`gmp_base_init()` 末尾调用 `gmp_csp_post_process()`，其注释允许平台在此启用中断。若控制 ISR 此时开始运行，而队列、事件组、工作任务或 RTOS 调度器尚未启动，ISR 可能访问未初始化对象。

反过来，如果先启动 RTOS 再调用完整初始化，平台初始化中的阻塞或全局状态变化又可能影响其他任务。

**风险等级：P0。** 必须把“准备硬件和软件对象”与“激活中断和业务流量”拆开。

### 5.6 其他非重入服务

- [`core/rt/src/gmp_runtime.c`](../core/rt/src/gmp_runtime.c) 的内部打印使用静态缓冲区和 `vsprintf`，不支持并发调用，且缺少长度保护；
- [`core/mm/src/gmp_mm_block_memory.c`](../core/mm/src/gmp_mm_block_memory.c) 的空闲结构和全局错误状态无锁；
- CAN 服务对部分队列操作使用当前临界区，但注册、生命周期和 ISR/任务语义没有统一约束；
- tick 回绕契约不一致，`gmp_base_time_sub()` 与直接无符号相减的接口行为不同；部分平台 tick 的可见性定义也不同。

**风险等级：P1/P2。** 应通过上下文对象、静态资源、端口锁和统一时间 API 分阶段修复。

## 6. 推荐目标架构

### 6.1 四个执行域

建议把系统划分为职责明确的执行域，而不是按模块任意建线程：

| 域 | 典型实现 | 允许行为 | 禁止行为 |
|---|---|---|---|
| Control | ADC/PWM ISR，或最高优先级周期任务 | 读采样、更新 CTL、提交输出、消费已验证命令 | 阻塞、动态分配、日志格式化、直接等待锁 |
| Supervisor | `core/pm` 兼容任务或中优先级任务 | 状态机、慢速保护、模式切换、健康监控 | 直接并发修改 CTL 内部状态 |
| I/O & Comms | CAN/UART/Ethernet 任务及 ISR | 收包、协议解析、形成命令、发送遥测 | 持有控制对象指针并任意写入 |
| Logging & Storage | 最低优先级任务或宿主机工作线程 | 文件、控制台、诊断持久化 | 反向阻塞控制路径 |

核心原则是：**控制状态单写者、命令消息化、遥测快照化、实时路径有界化。**

### 6.2 新增 OS/并发端口层

建议新增 `core/osal`（名称可在实现 RFC 中最终确定），核心代码只依赖抽象，不直接包含 FreeRTOS 头文件。最小能力集如下：

```c
typedef uintptr_t gmp_irq_state_t;

gmp_irq_state_t gmp_irq_save(void);
void gmp_irq_restore(gmp_irq_state_t state);

bool gmp_in_isr(void);
void gmp_memory_fence_acquire(void);
void gmp_memory_fence_release(void);

gmp_status_t gmp_mutex_lock(gmp_mutex_t *m, gmp_tick_t timeout);
void gmp_mutex_unlock(gmp_mutex_t *m);

gmp_status_t gmp_event_wait(gmp_event_t *e, gmp_tick_t timeout);
void gmp_event_signal(gmp_event_t *e);
bool gmp_event_signal_from_isr(gmp_event_t *e, bool *need_yield);
```

接口必须明确分为：

1. **本核 IRQ 临界区**：保存/恢复旧状态，可嵌套，时间极短；
2. **线程互斥**：支持优先级继承，只能在线程上下文使用；
3. **ISR 到任务通知**：使用 `from_isr` 变体，不做阻塞；
4. **SMP 自旋锁/原子**：仅用于极短跨核元数据更新；
5. **内存屏障和缓存维护**：用于共享内存、DMA 和非一致缓存平台；
6. **时间与等待**：提供单调时钟、绝对截止期和可测试的虚拟时钟。

建议同时定义编译期能力：

```c
GMP_CAP_RTOS
GMP_CAP_SMP
GMP_CAP_ATOMICS
GMP_CAP_CACHE_COHERENT
GMP_CAP_ISR_NOTIFY
GMP_CAP_STATIC_OBJECTS
```

模块在缺少所需能力时应编译失败或显式退化，不能静默假设。

### 6.3 队列和共享数据规范

至少提供三类经过验证的数据通道：

- `gmp_spsc_queue`：固定容量、单生产者单消费者、原子索引和 acquire/release；
- `gmp_mailbox`：最新值覆盖，适合控制命令和设定值；
- `gmp_snapshot`：版本化双缓冲或 seqlock，适合多字段遥测快照。

每个接口必须声明生产者/消费者数量、ISR 可用性、满队列策略和缓存要求。控制域不应因为队列满而等待；可以丢弃旧遥测，但命令丢弃必须计数和上报。

### 6.4 调参事务

将当前“收到报文后直接写目标地址”改为：

```text
通信任务解析 → 写入 staging 参数块 → 完整性/范围校验
            → 发布版本号或命令 → 控制采样边界读取并一次性提交
```

对于单个天然原子标量，可提供受限快速路径；参数组、浮点结构体和控制器系数必须使用快照提交。参数元数据应增加访问域、原子宽度、校验器和提交回调。

## 7. `core/pm` 与 RTOS 的兼容方案

### 7.1 三种后端模式

建议保留一个统一的任务描述，但支持三种执行后端：

| 后端 | 用途 | 建议 |
|---|---|---|
| Bare-metal cooperative | 维持现有 MCU 主循环 | 必须保留，作为最小配置 |
| RTOS cooperative worker | 一个 RTOS 任务驱动整个 PM 调度器 | 第一阶段默认方案，兼容性最好 |
| Native RTOS task | 明确标记的通信、日志等服务映射为独立任务 | 后续按需启用，不自动一任务一线程 |

兼容工作任务的伪流程：

```text
while running:
    dispatch all currently-ready steps within configured budget
    compute nearest absolute deadline
    wait until deadline or event notification
```

与当前高频空转调用相比，它可以节省 CPU，同时保留现有回调和上下文。必须配置每次唤醒的最大执行步数或时间预算，避免一个 PM 任务长时间占用 RTOS 优先级。

### 7.2 修正任务状态语义

建议为下一版 API 明确定义：

- `COMPLETE`：本次作业完成，按下一个绝对周期释放；
- `READY`：主动让出，但仍保持就绪，放到同优先级队尾；
- `WAIT`：等待事件或超时，不占用执行器；
- `FAULT`：停止任务并记录故障；
- 兼容层将旧 `DONE/BUSY/YIELD` 映射到新语义，并对无限 `BUSY` 设置预算或看门狗。

周期任务建议使用 `next_release += period`，并由策略决定超期后“跳过”“追赶一次”或“全部追赶”，避免用 `last_run = now` 造成长期漂移。

### 7.3 任务描述扩展

可以在不要求动态内存的前提下扩展任务元数据：

```c
typedef struct {
    const char *name;
    gmp_task_step_fn step;
    void *context;
    uint32_t period_us;
    uint32_t deadline_us;
    uint16_t budget_us;
    uint8_t priority_class;
    uint8_t flags;          /* ISR-safe, native-RTOS, skip-missed... */
    uint32_t affinity_mask; /* 0 表示端口默认 */
} gmp_task_desc_t;
```

裸机后端可以忽略不支持的调度提示，但必须给出可诊断的能力结果。RTOS 后端只把标记为 `native-RTOS` 且经过审查的服务转换为独立任务。

### 7.4 FreeRTOS/CMSIS-RTOS2 映射建议

| GMP 抽象 | FreeRTOS | CMSIS-RTOS2 |
|---|---|---|
| PM 兼容工作任务 | 静态 task + `vTaskDelayUntil`/notification | thread + thread flags/delay-until |
| ISR 通知 | task notification/queue `FromISR` | 支持 ISR 的 flags/message API |
| mutex | priority-inheritance mutex | `osMutexPrioInherit` |
| SPSC | GMP 自有原子固定队列，或受限 stream buffer | GMP 自有队列或 message queue |
| 单调时钟 | tick 扩展或硬件计时器 | kernel tick + 硬件计时器 |
| 静态对象 | `StaticTask_t`、静态队列 | 端口按实现能力声明 |

不要在 `ctl` 中直接调用上述 RTOS API。所有映射应位于 CSP/OSAL 端口，以便继续支持 NoRTOS、宿主机和其他内核。

## 8. 推荐 RTOS 启动顺序

把现有生命周期拆为至少两个阶段：

```text
早期启动（中断保持禁用）
  → gmp_base_prepare
      CSP 时钟/引脚/外设基础配置
      CTL 与应用对象初始化
      创建静态队列、邮箱、PM 调度器
  → 创建 RTOS 任务
  → 启动 RTOS 内核
  → GMP bootstrap task
      gmp_base_activate
      清 pending、启动外设触发、最后启用控制中断
  → 正常运行
```

裸机入口可以按 `prepare → activate → while(gmp_base_loop)` 组合，保持现有使用体验。停机也应提供对称的 `deactivate/stop`，先禁止新控制触发，再排空或丢弃后台工作。

## 9. 控制套件迁移方式

### 9.1 保持控制环单写者

首选布局：

- ADC/PWM ISR 独占控制器对象、观测器积分状态、PWM 输出和保护快速状态；
- PM/RTOS supervisor 通过命令邮箱请求启停、模式或设定值变化；
- ISR 在采样边界读取新版本命令，验证后生效；
- ISR 发布固定大小遥测快照，后台任务读取副本；
- 通信任务不直接调用会修改控制内部状态的接口。

若特定平台必须把控制环改为 RTOS 任务，应由硬件 ISR 仅采样和通知，控制任务固定核、最高优先级、静态栈，并用实测证明唤醒抖动满足预算。在证明前，默认继续使用硬件 ISR。

### 9.2 首个参考套件

建议选择一个具备控制 ISR、DataLink/监控任务和较完整保护逻辑的套件作为参考，例如 `mcs_pmsm_nt` 或 `dps_fsbb`。迁移步骤：

1. 标注每个共享变量的所有者和读者；
2. 把在线调参改为 staging + sample-boundary commit；
3. 把遥测改为 snapshot；
4. 现有 PM 任务整体放入一个 RTOS worker；
5. 仅在测量后将通信或日志独立为原生任务；
6. 记录控制 ISR 的最坏执行时间、抖动、队列水位和丢包计数。

## 10. 多核与 RTOS 的组合

### 10.1 AMP 优先于共享可变控制状态

H755 和 F28388D 更适合先采用 AMP：每核独立构建、独立运行时和独立 PM/RTOS 实例，核心之间只交换版本化消息。不要把控制器对象或调度器结构放入共享 RAM 后由多核共同修改。

共享消息建议包含：

```c
struct gmp_ipc_header {
    uint32_t magic;
    uint16_t abi_version;
    uint16_t payload_size;
    uint32_t sequence;
    uint32_t flags;
};
```

发布协议必须规定：单写者、写 payload、release fence、发布 sequence、触发 IPC；消费者收到通知后 acquire，复制 payload，再次验证 sequence。若缓存不一致，还必须执行平台指定的 clean/invalidate。仅把结构体声明为 `volatile` 不足以完成该协议。

### 10.2 当前 F28388D 草案的注意事项

分析时工作区中存在 CPU1/CPU2/CM 应用及共享内存代码，但大部分仍为未跟踪或未提交内容，因此本文只把它视为设计输入，不把它列为 GMP 发布能力。

当前 sequence begin/end 标记是正确方向，但消费者应在两次序号读取之间复制完整 payload，并使用端口提供的 acquire/release 屏障。若先检查序号、随后再从共享结构逐字段消费，写者可能在检查后开始下一次更新，仍会得到撕裂快照。

### 10.3 SMP 的额外门槛

只有在以下能力完成后才应宣称支持 SMP：

- 所有共享队列使用原子内存序或经验证的锁；
- 调度器实例有单一所有者，或其注册/派发完全同步；
- 端口提供核 ID、核亲和、跨核中断、自旋锁和缓存一致性契约；
- 全局服务消除静态非重入缓冲和未保护链表；
- 通过多核压力测试及目标硬件长稳测试。

## 11. 分阶段实施计划

以下工作量为粗略工程估算，不含特定板卡 BSP 缺陷修复。

### 阶段 0：冻结契约和补测试（约 1～2 人周）

- 记录 ISR/主循环/线程 API 调用上下文；
- 修正或版本化 `YIELD`、`BUSY`、周期和回绕语义；
- 新增保存/恢复式 IRQ 临界区 API，保留旧接口兼容包装；
- 为队列、DataLink、Tunable 写明生产者/消费者与所有权；
- 增加虚拟时钟调度测试、嵌套临界区测试和公平性测试。

### 阶段 1：OSAL 与 RTOS 兼容执行器（约 2～4 人周）

- 实现 NoRTOS、宿主机和 FreeRTOS/CMSIS-RTOS2 端口；
- 将运行时拆为 prepare/activate/deactivate；
- 实现一个 RTOS PM worker，支持绝对截止期和事件唤醒；
- 默认使用静态任务、静态队列和预分配内存；
- 增加能力宏和构建期检查。

### 阶段 2：单核 RTOS 参考平台（约 3～6 人周）

- 选择一个 STM32 或 C28x/C29x 实际目标；
- 保持控制 ISR，迁移通信和监控到 PM worker；
- 建立命令邮箱和遥测快照；
- 测量 ISR WCET、唤醒抖动、栈水位和队列水位；
- 提供无 RTOS/RTOS 双配置回归。

### 阶段 3：套件迁移与原生服务任务（约 4～8 人周）

- 迁移首个代表性 suite；
- 将经过审查的通信、日志服务设为原生 RTOS 任务；
- 改造 Tunable 为事务提交；
- 为 CAN、DataLink 和诊断统一 ISR/任务 API；
- 将经验固化为 suite 模板和代码审查清单。

### 阶段 4：AMP 加固，再评估 SMP

- 把 H755/F28388D IPC 统一为版本化消息层；
- 实现内存屏障、缓存维护和 ABI 校验；
- 做跨核故障、复位、超时和版本不匹配测试；
- 只有业务确有共享地址空间需求时再投入 SMP，不把 SMP 作为 RTOS 支持的前置条件。

## 12. 建议的代码变更地图

| 位置 | 建议变化 |
|---|---|
| `core/base/gmp_base.h` | 增加 IRQ save/restore、上下文检测和时间能力；旧临界接口标记兼容语义 |
| `core/osal/`（新增） | mutex、event、atomic/fence、time、thread/backend 能力接口 |
| `core/pm/function_scheduler` | 明确状态机、公平轮转、绝对周期、预算、事件等待和单所有者约束 |
| `core/rt` | prepare/activate/deactivate；提供 bare-metal 与 RTOS 入口组合 |
| `core/base/ds` | 增加有内存序的 SPSC/mailbox/snapshot；保留受限单核实现 |
| `core/dev/datalink` | 标注 ISR/任务 API；调参 staging/commit；统计溢出和拒绝 |
| `core/dev/can` | 分离 ISR-safe 与 task-only 操作，明确注册期和运行期生命周期 |
| `core/mm` | 支持外部锁或单所有者 allocator；错误状态实例化或线程局部化 |
| `csp/<platform>` | 实现能力表、IRQ 状态、RTOS 适配、屏障及缓存操作 |
| `ctl/suite/*` | 控制域单写者；命令邮箱；遥测快照；任务上下文声明 |
| `core/unit_test` | 调度、回绕、并发、内存序和失败策略测试 |

## 13. 验证矩阵与验收标准

### 13.1 自动化测试

- 调度器：`YIELD/READY` 公平性、`BUSY` 预算、禁用/取消、周期漂移、tick 回绕、超期策略；
- 同步：嵌套 IRQ 保存恢复、ISR 通知、mutex 误用诊断、SPSC 满/空回绕；
- 并发：Linux/宿主机用真实线程压力测试，并在可用构建上运行 ThreadSanitizer；
- 生命周期：中断在 activate 前不得观察未初始化对象，deactivate 后不得产生新控制输出；
- IPC：序号回绕、撕裂注入、丢通知、对端复位、ABI 不匹配和缓存维护钩子；
- 构建矩阵：NoRTOS、FreeRTOS、CMSIS-RTOS2 包装层，以及至少一个宿主机端口。

### 13.2 目标硬件验收

- 控制周期抖动和 WCET 满足项目预算，报告最大值而不仅是平均值；
- 控制 ISR 中无动态分配、无不可界定循环、无阻塞锁；
- 所有队列有容量依据、水位和溢出计数；
- 任务栈使用有高水位监测，静态配置留有余量；
- 通信风暴、日志阻塞和后台任务故障不得扩大为控制环超期；
- 运行 24～72 小时压力测试无数据竞争、死锁、看门狗复位或序号错误；
- 多核目标需额外验证单核复位、对端掉线和版本不匹配时进入安全状态。

### 13.3 “支持 RTOS”发布门槛

只有同时满足以下条件，平台文档才应标记为 RTOS supported：

1. 有受维护的 OSAL 端口和参考配置；
2. 有明确的 ISR/task API 上下文和优先级模型；
3. 有至少一个套件的硬件运行记录；
4. NoRTOS 回归未破坏；
5. 并发测试和生命周期测试进入 CI；
6. 文档列出静态内存、tick、缓存和多核限制。

## 14. 不建议的方案

- 不要把每个现有 `gmp_task_t` 自动创建成一个 RTOS 线程；这会增加栈内存、调度抖动和锁依赖，却没有修复共享状态所有权；
- 不要用全局关中断代替 mutex，也不要让 mutex 出现在控制 ISR；
- 不要只添加 `volatile` 或平台编译屏障就宣称跨核安全；
- 不要让协议层直接写控制器内部对象；
- 不要在实时路径使用通用 `malloc/free`、格式化日志或无界队列；
- 不要同时首次引入 RTOS、重写控制调度和共享多核控制状态，应分阶段保留可回退路径。

## 15. 建议优先决策

开始实现前，需要由维护者明确以下项目级决策：

1. 首个参考内核：建议 FreeRTOS，并通过 OSAL 保留 CMSIS-RTOS2 映射；
2. 首个参考板卡和 suite：建议选择现有硬件、调试链和回归最完整的组合；
3. 控制 ISR 的周期、最大允许抖动和 WCET 预算；
4. 旧 `BUSY/YIELD` 行为是否必须保持二进制/源代码兼容；
5. 是否允许动态任务创建；建议首版禁止，在初始化期静态创建；
6. F28388D 当前多核草案是否计划纳入正式基线，以及各核的固定职责与复位策略。

## 16. 最终判断

GMP 当前已经具备向 RTOS 演进所需的两个重要基础：控制步进与后台循环已经分离，CCTL 也证明了“实时状态单线程拥有、异步服务通过 SPSC 通信”的设计可行。真正的缺口不在于创建线程的 API，而在于**同步语义、数据所有权、生命周期、内存模型和验证体系**。

因此，最稳妥的路线是先把 `core/pm` 定位为可插拔的协作执行器：裸机继续由主循环驱动，RTOS 下由一个工作任务驱动。控制环仍保持单写者，通信和监控通过有界消息交换。待这一层完成并经过参考平台验证后，再选择性地把服务任务原生化，并把 AMP 消息协议加固。这样既能保留现有 `ctl/suite` 的可复用性，也能避免一次性把控制代码暴露给抢占、多核和复杂锁语义。
