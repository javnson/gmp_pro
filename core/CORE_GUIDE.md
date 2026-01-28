# GMP Core 核心模块完整指南

## 📚 目录

1. [概述](#概述)
2. [模块架构](#模块架构)
3. [标准化模块（std）](#标准化模块std)
4. [设备接口模块（dev）](#设备接口模块dev)
5. [内存管理模块（mm）](#内存管理模块mm)
6. [进程管理模块（pm）](#进程管理模块pm)
7. [使用指南](#使用指南)
8. [扩展开发指南](#扩展开发指南)

---

## 概述

GMP Core 是整个 GMP 平台的核心基础库，提供了跨平台的标准化支持、内存管理、进程调度、设备接口等基础设施。无论使用 C 还是 C++ 开发，Core 都提供了统一的接口和抽象。

### 核心特性

- ✅ **跨平台标准化**: 统一不同编译器、芯片架构的类型和宏定义
- ✅ **模块化设计**: 各模块独立、低耦合
- ✅ **轻量高效**: 适用于资源受限的嵌入式系统
- ✅ **C/C++ 双支持**: 提供 `gmp_core.h` 和 `gmp_core.hpp` 两套头文件

### 主要头文件

```c
// C 风格项目
#include <gmp_core.h>

// C++ 风格项目
#include <gmp_core.hpp>
```

---

## 模块架构

```
core/
├── std/              # 标准化模块：跨平台支持
│   ├── gmp.std.h    # 总入口头文件
│   ├── arch/        # 不同架构支持（ARM、C28x、x86等）
│   ├── cc/          # 编译器支持（GCC、MSVC、TI等）
│   ├── cfg/         # 配置文件
│   └── ds/          # 数据结构（链表、环形缓冲等）
│
├── dev/              # 设备接口模块
│   ├── at_device.h  # AT命令解析器
│   ├── interface.h  # 通用设备接口
│   ├── ring_buf.h   # 环形缓冲区
│   ├── peripheral_port.h  # 外设端口抽象
│   └── rtshell/     # 实时Shell（命令行交互）
│
├── mm/               # 内存管理模块
│   └── block_mem.h  # 块内存管理器
│
├── pm/               # 进程管理模块
│   ├── scheduler.hpp      # 调度器
│   ├── state_machine.h    # 状态机
│   ├── workflow.hpp       # 工作流引擎
│   ├── timing_manager.h   # 时序管理器
│   └── function_scheduler.h  # 函数调度器
│
└── src/              # 实现源文件
    ├── gmp_at_device.c
    ├── gmp_mm_block_memory.c
    └── ...
```

---

## 标准化模块（std）

**位置**: `core/std/`

**功能**: 屏蔽不同平台、编译器、架构的差异，提供统一的类型定义和宏

### 关键文件

#### 1. 架构支持（arch/）

支持的架构：
- ARM Cortex-M0/M1/M4 (`arm_m*.h`)
- TI C28x DSP (`c28x.h`)
- RISC-V 32位 (`risc_v_32.h`)
- x86/x86_64 (`x86.h`, `x86_64.h`)

每个架构文件定义：
- 字长、对齐方式
- 中断控制宏
- 架构特定优化（如 `__clz`、`__rbit` 等内建函数）

#### 2. 编译器支持（cc/）

支持的编译器：
- GCC/G++ (`cc.gnuc.inl`)
- MSVC (`cc.msvc.inl`)
- TI C2000 Compiler (`cc.c2000.inl`)
- ARM Compiler (`cc.armcc.inl`)

编译器文件定义：
- 内联宏（`GMP_INLINE`、`GMP_STATIC_INLINE`、`GMP_NOINLINE`）
- 对齐宏（`GMP_ALIGN(n)`）
- 弱符号、打包等编译器特性

#### 3. 类型定义（cfg/types.cfg.h）

```c
// 基本类型别名
typedef int32_t     fast_gt;      // 快速整数（通常用于标志）
typedef int32_t     time_gt;      // 时间戳类型
typedef int32_t     ctrl_gt;      // 控制类型（标幺值）
typedef uint32_t    size_gt;      // 大小类型
typedef uint8_t     data_gt;      // 数据字节类型
typedef int32_t     ec_gt;        // 错误码类型

// 根据平台调整字长
#ifdef GMP_USE_16BIT_FAST
    typedef int16_t fast_gt;
#endif
```

#### 4. 错误码（cfg/errorcode.cfg.h）

```c
// 标准错误码定义
#define GMP_EC_OK               (0)       // 成功
#define GMP_EC_ERROR            (-1)      // 通用错误
#define GMP_EC_INVALID_PARAM    (-2)      // 无效参数
#define GMP_EC_OUT_OF_MEMORY    (-3)      // 内存不足
#define GMP_EC_TIMEOUT          (-4)      // 超时
#define GMP_EC_BUSY             (-5)      // 设备忙
// ... 更多错误码
```

#### 5. 通用宏（gmp.std.h）

```c
// 断言
#define GMP_ASSERT(expr)            // 运行时断言
#define GMP_ASSERT_PTR(ptr)         // 指针有效性检查
#define GMP_STATIC_ASSERT(expr)     // 编译期断言

// 未使用变量（消除警告）
#define GMP_UNUSED_VAR(x)  ((void)(x))

// 数组大小
#define GMP_ARRAY_SIZE(arr)  (sizeof(arr) / sizeof((arr)[0]))

// 最小/最大值
#define GMP_MIN(a, b)  ((a) < (b) ? (a) : (b))
#define GMP_MAX(a, b)  ((a) > (b) ? (a) : (b))

// 绝对值
#define GMP_ABS(x)  ((x) < 0 ? -(x) : (x))
```

### 使用示例

```c
#include <gmp.std.h>

void my_function(void* ptr) {
    GMP_ASSERT_PTR(ptr);  // 检查指针
    
    fast_gt flag = 0;     // 使用标准类型
    time_gt timestamp = get_time();
    
    // 平台无关的代码
}
```

---

## 设备接口模块（dev）

**位置**: `core/dev/`

**功能**: 提供通信外设的统一抽象接口

### 1. AT 命令解析器（at_device.h）

AT 命令解析器用于通过串口与设备交互，支持类似调制解调器的 AT 命令格式。

#### 核心结构

```c
// AT 命令类型
typedef enum {
    AT_CMD_TYPE_EXEC,   // AT+CMD         执行命令
    AT_CMD_TYPE_QUERY,  // AT+CMD?        查询
    AT_CMD_TYPE_TEST,   // AT+CMD=?       测试/帮助
    AT_CMD_TYPE_SETUP   // AT+CMD=<args>  设置
} at_cmd_type_t;

// AT 命令状态
typedef enum {
    AT_STATUS_OK = 0,    // 命令执行成功
    AT_STATUS_ERROR,     // 命令执行失败
    AT_STATUS_PENDING    // 命令等待异步完成
} at_status_t;

// AT 命令对象
typedef struct {
    const char* name;           // 命令名称（如 "PWM"）
    uint16_t name_length;       // 命令名长度
    uint16_t attr;              // 命令属性
    
    // 命令处理函数
    at_status_t (*handler)(
        struct _tag_at_device_entity* dev, 
        at_cmd_type_t type, 
        char* args, 
        uint16_t length
    );
    
    const char* help_info;      // 帮助信息
} at_device_cmd_t;

// AT 设备实体
typedef struct _tag_at_device_entity {
    ringbuf_t buffer;                   // 接收缓冲区
    char cmd_buffer[AT_LINE_MAX_LEN];   // 命令行缓冲
    
    at_device_cmd_t* cmd_table;         // 命令表
    uint16_t cmd_table_size;            // 命令数量
    
    // 异步命令支持
    const at_device_cmd_t* pending_cmd;
    char* pending_args;
    at_cmd_type_t pending_type;
    
    // 错误处理回调
    void (*error_handler)(struct _tag_at_device_entity* dev, at_error_code_t err);
} at_device_entity_t;
```

#### 使用示例

```c
#include <core/dev/at_device.h>

// 1. 定义命令处理函数
at_status_t enable_handler(at_device_entity_t* dev, 
                          at_cmd_type_t type, 
                          char* args, 
                          uint16_t len)
{
    if (type == AT_CMD_TYPE_EXEC) {
        // 执行使能操作
        motor_enable();
        gmp_base_print("Motor enabled\r\n");
        return AT_STATUS_OK;
    }
    return AT_STATUS_ERROR;
}

// 2. 定义命令表
at_device_cmd_t at_cmds[] = {
    {
        .name = "ENABLE",
        .name_length = 6,
        .attr = 0,
        .handler = enable_handler,
        .help_info = "Enable motor operation"
    },
    // ... 更多命令
};

// 3. 初始化 AT 设备
at_device_entity_t at_dev;

void at_init() {
    gmp_init_at_device(&at_dev, at_cmds, GMP_ARRAY_SIZE(at_cmds));
}

// 4. 在串口接收中断中处理
void UART_IRQHandler() {
    char ch = read_uart();
    gmp_at_device_push_byte(&at_dev, ch);
}

// 5. 在主循环中解析
void main_loop() {
    gmp_at_device_dispatch(&at_dev);
}
```

#### 常用命令示例

```bash
AT+ENABLE           # 使能电机
AT+POWEROFF         # 关闭输出
AT+RST              # 复位
AT+SPD=1000         # 设置速度为 1000 rpm
AT+SPD?             # 查询当前速度
AT+HELP=?           # 显示帮助信息
```

### 2. 环形缓冲区（ring_buf.h）

```c
typedef struct {
    data_gt* buffer;        // 缓冲区指针
    size_gt capacity;       // 容量
    size_gt head;           // 头指针
    size_gt tail;           // 尾指针
    fast_gt full_flag;      // 满标志
} ringbuf_t;

// API
void gmp_init_ringbuf(ringbuf_t* rb, data_gt* buf, size_gt size);
fast_gt gmp_ringbuf_push(ringbuf_t* rb, data_gt data);
fast_gt gmp_ringbuf_pop(ringbuf_t* rb, data_gt* data);
fast_gt gmp_ringbuf_is_empty(const ringbuf_t* rb);
fast_gt gmp_ringbuf_is_full(const ringbuf_t* rb);
```

### 3. 实时Shell（rtshell/）

实时Shell 提供更高级的命令行交互功能：
- 命令历史记录
- Tab 自动补全
- 表达式解析
- 变量查看/修改

---

## 内存管理模块（mm）

**位置**: `core/mm/`

**功能**: 提供高效的块内存管理器

### 块内存管理器（block_mem.h）

#### 核心思想

将内存分为固定大小的块，适用于：
- 频繁分配/释放相同大小对象的场景
- 避免内存碎片
- 确定性的分配/释放时间

#### 核心结构

```c
// 内存块头
typedef struct _tag_gmp_mem_block_head {
    uint_least16_t magic_number;  // 魔数（验证）
    size_gt block_index;           // 块索引
    size_gt block_size;            // 块大小（单位：block_size_unit）
    struct _tag_gmp_mem_block_head* next;  // 下一个块
} gmp_mem_block_head;

// 内存区域头
typedef struct _tag_gmp_mem_area_head {
    void* entry;                   // 内存入口
    size_gt block_size_unit;       // 块大小单位
    size_gt capacity;              // 总容量
    size_gt used;                  // 已使用
    uint32_t memory_state;         // 内存状态
    struct _tag_gmp_mem_area_head* next;   // 下一个区域
    struct _tag_gmp_mem_block_head* head;  // 第一个块头
    data_gt assigned_flag;         // 分配标志
} gmp_mem_area_head;
```

#### API 函数

```c
// 初始化内存管理器
gmp_mem_area_head* gmp_mm_setup_block_memory(
    void* memory_entry,        // 内存起始地址
    uint32_t memory_size,      // 内存大小（字节）
    size_gt block_size_unit    // 块大小单位
);

// 分配内存
void* gmp_mm_block_alloc(
    gmp_mem_area_head* handle, 
    size_gt length             // 需要的块数量
);

// 释放内存
void gmp_mm_block_free(
    gmp_mem_area_head* handle, 
    void* ptr
);
```

#### 使用示例

```c
#include <core/mm/block_mem.h>

// 1. 准备内存池
#define MEM_POOL_SIZE  (4096)
static uint8_t mem_pool[MEM_POOL_SIZE];

// 2. 初始化
gmp_mem_area_head* mem_mgr;

void mem_init() {
    mem_mgr = gmp_mm_setup_block_memory(
        mem_pool, 
        MEM_POOL_SIZE, 
        16  // 块大小为 16 字节
    );
}

// 3. 分配内存
void* my_alloc(size_t size) {
    // 计算需要的块数
    size_gt blocks = (size + 15) / 16;
    return gmp_mm_block_alloc(mem_mgr, blocks);
}

// 4. 释放内存
void my_free(void* ptr) {
    gmp_mm_block_free(mem_mgr, ptr);
}

// 5. 使用
void test() {
    int* data = (int*)my_alloc(sizeof(int) * 10);
    // ... 使用 data
    my_free(data);
}
```

#### 优势

- ✅ 无碎片：块大小固定
- ✅ 快速：O(1) 分配和释放
- ✅ 可预测：内存使用量可控
- ✅ 安全：魔数校验，防止野指针

---

## 进程管理模块（pm）

**位置**: `core/pm/`

**功能**: 提供任务调度、状态机、工作流等高层抽象

### 1. 函数调度器（function_scheduler.h）

轻量级的周期任务调度器，适用于后台任务。

#### 使用示例

```c
#include <core/pm/function_scheduler.h>

// 定义任务函数
void task_led_blink(void* param) {
    toggle_led();
}

void task_sensor_read(void* param) {
    read_sensors();
}

// 初始化调度器
gmp_scheduler_t sched;

void scheduler_init() {
    gmp_init_scheduler(&sched, 1000);  // 1ms 时基
    
    // 添加任务
    gmp_add_task(&sched, task_led_blink, NULL, 500);    // 500ms 周期
    gmp_add_task(&sched, task_sensor_read, NULL, 100);  // 100ms 周期
}

// 在主循环或定时器中调用
void main_loop() {
    while (1) {
        gmp_scheduler_dispatch(&sched);
        delay_ms(1);
    }
}
```

### 2. 状态机（state_machine.h）

提供状态机框架，支持：
- 状态进入/退出回调
- 状态转换条件
- 子状态机
- 事件驱动

#### 核心概念

```c
// 状态定义
typedef enum {
    STATE_IDLE,
    STATE_RUNNING,
    STATE_ERROR,
    STATE_SHUTDOWN
} motor_state_t;

// 状态机对象（示意）
typedef struct {
    motor_state_t current_state;
    motor_state_t prev_state;
    
    // 回调函数
    void (*on_enter)(void);
    void (*on_exit)(void);
} state_machine_t;
```

#### 使用示例

```c
// 状态进入回调
void on_enter_running() {
    enable_pwm();
    gmp_base_print("Motor running\r\n");
}

void on_exit_running() {
    disable_pwm();
}

// 状态转换
void state_machine_update(state_machine_t* sm) {
    switch (sm->current_state) {
        case STATE_IDLE:
            if (start_command_received()) {
                sm->prev_state = STATE_IDLE;
                sm->current_state = STATE_RUNNING;
                on_enter_running();
            }
            break;
            
        case STATE_RUNNING:
            if (stop_command_received()) {
                on_exit_running();
                sm->current_state = STATE_IDLE;
            }
            if (error_detected()) {
                on_exit_running();
                sm->current_state = STATE_ERROR;
            }
            break;
            
        case STATE_ERROR:
            if (reset_command_received()) {
                sm->current_state = STATE_IDLE;
            }
            break;
    }
}
```

### 3. 工作流引擎（workflow.hpp）

用于管理复杂的异步任务流程，支持：
- 长延时任务（非阻塞）
- 任务依赖关系
- 并行任务
- 错误处理和重试

#### 应用场景

- 复杂的设备初始化流程
- 多步骤通信协议
- 设备标定流程
- 系统自检

---

## 使用指南

### 项目集成步骤

#### 1. 包含头文件

**C 项目：**
```c
#include <gmp_core.h>

int main(void) {
    // 初始化 GMP
    gmp_entry();
    
    // 你的主循环
    while (1) {
        // ...
    }
}
```

**C++ 项目：**
```cpp
#include <gmp_core.hpp>

int main(void) {
    gmp_entry();
    
    while (1) {
        // ...
    }
}
```

#### 2. 配置编译选项

在工程的 include 路径中添加：
```
gmp_pro/
gmp_pro/core/
gmp_pro/ctl/
```

#### 3. 添加源文件

从 `core/src/` 添加需要的源文件：
- `gmp_at_device.c` （如果使用 AT 命令）
- `gmp_mm_block_memory.c` （如果使用内存管理）
- 其他根据需要添加

### 常用配置宏

在项目的配置文件或编译选项中定义：

```c
// 使能 FPU
#define GMP_USE_FPU  1

// 使用 16 位快速类型（资源受限平台）
#define GMP_USE_16BIT_FAST  1

// AT 命令缓冲区大小
#define AT_DEVICE_RX_BUFFER  128
#define AT_LINE_MAX_LEN      64

// 内存管理配置
#define GMP_MEM_BLOCK_SIZE   16

// 调试输出
#define GMP_ENABLE_DEBUG_PRINT  1
```

---

## 扩展开发指南

### 添加新的设备接口

#### 场景：添加 SPI 设备接口

```c
// 1. 在 core/dev/ 创建 spi_device.h

#ifndef _FILE_SPI_DEVICE_H_
#define _FILE_SPI_DEVICE_H_

#include <gmp.std.h>

// SPI 设备结构
typedef struct {
    uint32_t clock_freq;       // 时钟频率
    uint8_t mode;              // SPI 模式 (0-3)
    uint8_t bits_per_word;     // 每字长度
    
    // 硬件相关回调
    void (*cs_assert)(void);
    void (*cs_deassert)(void);
    uint8_t (*transfer_byte)(uint8_t data);
} spi_device_t;

// API
void gmp_init_spi_device(spi_device_t* dev, uint32_t freq, uint8_t mode);
ec_gt gmp_spi_write(spi_device_t* dev, const uint8_t* data, size_gt len);
ec_gt gmp_spi_read(spi_device_t* dev, uint8_t* data, size_gt len);
ec_gt gmp_spi_transfer(spi_device_t* dev, const uint8_t* tx, uint8_t* rx, size_gt len);

#endif
```

```c
// 2. 在 core/src/ 创建 gmp_spi_device.c

#include <core/dev/spi_device.h>

void gmp_init_spi_device(spi_device_t* dev, uint32_t freq, uint8_t mode) {
    GMP_ASSERT_PTR(dev);
    
    dev->clock_freq = freq;
    dev->mode = mode;
    dev->bits_per_word = 8;
}

ec_gt gmp_spi_write(spi_device_t* dev, const uint8_t* data, size_gt len) {
    GMP_ASSERT_PTR(dev);
    GMP_ASSERT_PTR(data);
    
    dev->cs_assert();
    
    for (size_gt i = 0; i < len; i++) {
        dev->transfer_byte(data[i]);
    }
    
    dev->cs_deassert();
    
    return GMP_EC_OK;
}

// ... 其他函数实现
```

### 添加新的数据结构

#### 场景：添加队列

```c
// 1. 在 core/std/ds/ 创建 queue.h

#ifndef _FILE_QUEUE_H_
#define _FILE_QUEUE_H_

#include <gmp.std.h>

typedef struct {
    data_gt* buffer;
    size_gt capacity;
    size_gt head;
    size_gt tail;
    size_gt count;
} queue_t;

// API
void gmp_init_queue(queue_t* q, data_gt* buf, size_gt size);
ec_gt gmp_queue_enqueue(queue_t* q, data_gt data);
ec_gt gmp_queue_dequeue(queue_t* q, data_gt* data);
fast_gt gmp_queue_is_empty(const queue_t* q);
fast_gt gmp_queue_is_full(const queue_t* q);
size_gt gmp_queue_size(const queue_t* q);

#endif
```

### 添加新的架构支持

#### 场景：添加 ARM Cortex-M7 支持

```c
// 在 core/std/arch/ 创建 arm_m7.h

#ifndef _FILE_ARM_M7_H_
#define _FILE_ARM_M7_H_

// 架构定义
#define GMP_ARCH_ARM_M7  1
#define GMP_ARCH_32BIT   1
#define GMP_HAS_FPU      1
#define GMP_HAS_DSP      1

// 字长和对齐
#define GMP_WORD_SIZE    4
#define GMP_ALIGN_SIZE   4

// 中断控制
#define GMP_DISABLE_IRQ()  __disable_irq()
#define GMP_ENABLE_IRQ()   __enable_irq()

// 内建函数
#define GMP_CLZ(x)   __builtin_clz(x)
#define GMP_RBIT(x)  __builtin_arm_rbit(x)

// 缓存操作
#define GMP_DCACHE_CLEAN()    SCB_CleanDCache()
#define GMP_DCACHE_INVALIDATE()  SCB_InvalidateDCache()

#endif
```

### 添加自定义调度策略

```c
// 优先级调度器示例

typedef struct {
    void (*task)(void* param);
    void* param;
    uint8_t priority;        // 0 = 最高优先级
    uint32_t period_ms;
    uint32_t last_run;
} priority_task_t;

typedef struct {
    priority_task_t* tasks;
    size_gt task_count;
    uint32_t tick;
} priority_scheduler_t;

void gmp_priority_scheduler_dispatch(priority_scheduler_t* sched) {
    // 按优先级排序
    // 执行到期的最高优先级任务
    priority_task_t* highest = NULL;
    
    for (size_gt i = 0; i < sched->task_count; i++) {
        if (sched->tick - sched->tasks[i].last_run >= sched->tasks[i].period_ms) {
            if (highest == NULL || sched->tasks[i].priority < highest->priority) {
                highest = &sched->tasks[i];
            }
        }
    }
    
    if (highest != NULL) {
        highest->task(highest->param);
        highest->last_run = sched->tick;
    }
}
```

---

## 调试技巧

### 1. 使用断言

```c
void my_function(void* ptr, size_gt len) {
    GMP_ASSERT_PTR(ptr);
    GMP_ASSERT(len > 0 && len < MAX_SIZE);
    
    // ... 安全的代码
}
```

### 2. 错误码追踪

```c
ec_gt result = gmp_mm_block_alloc(...);
if (result != GMP_EC_OK) {
    gmp_base_print("Alloc failed: %d\r\n", result);
    // 错误处理
}
```

### 3. 内存泄漏检测

```c
// 定期检查内存使用
void check_memory() {
    size_gt used = mem_mgr->used;
    gmp_base_print("Memory used: %u/%u\r\n", used, mem_mgr->capacity);
    
    if (used > mem_mgr->capacity * 0.9) {
        gmp_base_print("WARNING: Low memory!\r\n");
    }
}
```

---

## 常见问题

### Q1: 如何选择合适的内存管理策略？

**A:**
- **小对象频繁分配**：使用块内存管理器（`block_mem.h`）
- **大对象/变长对象**：使用平台的 `malloc/free`
- **实时系统**：避免动态分配，使用静态内存池

### Q2: AT 命令解析器卡住了怎么办？

**A:**
- 检查串口中断是否正常触发
- 检查 `gmp_at_device_dispatch()` 是否在主循环中定期调用
- 检查环形缓冲区是否溢出（`flag_overwrite` 标志）

### Q3: 调度器任务执行不准时？

**A:**
- 检查任务执行时间是否超过周期
- 检查是否有高优先级中断抢占
- 考虑使用硬件定时器触发调度

### Q4: 跨平台移植时遇到编译错误？

**A:**
- 检查架构和编译器配置是否正确
- 在 `csp.config.h` 中定义正确的宏
- 确保包含了正确的 `arch/` 和 `cc/` 文件

---

## 总结

GMP Core 提供了构建嵌入式控制系统所需的全部基础设施：

| 模块 | 功能 | 适用场景 |
|------|------|----------|
| **std** | 跨平台标准化 | 所有项目 |
| **dev** | 设备接口 | 串口通信、命令行交互 |
| **mm** | 内存管理 | 需要动态内存分配的场景 |
| **pm** | 进程管理 | 多任务、状态机、工作流 |

**开发建议：**
1. 从简单开始，逐步添加需要的模块
2. 优先使用标准接口，避免平台特定代码
3. 充分利用 AT 命令调试和在线控制
4. 合理使用内存管理，避免碎片和泄漏
5. 状态机和调度器可大幅简化复杂逻辑

---

**版本历史：**
- v1.0 (2026-01-27): 初始版本

**参考资料：**
- `core/readme.md` - Core 模块简介
- `core/dev/readme.md` - 设备接口说明
- `core/pm/readme.md` - 进程管理说明
- 各模块头文件中的详细注释
