# GMP Scheduler 用户使用手册

## 1. 简介

GMP Scheduler 是一个专为嵌入式环境设计的轻量级任务调度器。它不依赖复杂的操作系统（RTOS），而是通过在主循环中高效地轮询和分发任务，实现多任务的伪并行处理。

**核心特点：**

- **轻量级**：极低的 RAM 和 Flash 占用。
- **非阻塞**：基于协作式多任务思想，任务需主动释放 CPU。
- **状态机支持**：独特的 `BUSY` 状态机制，允许长耗时任务分片执行，避免看门狗超时。
- **简单易用**：无需上下文切换（Context Switch）的开销，纯 C 语言实现。

## 2. 快速集成

### 2.1 依赖移植

在使用调度器之前，您需要提供一个基础的时间获取函数，用于驱动调度器的心跳。

请确保在项目中实现了以下函数，这个函数通常由CSP（芯片支持包）提供：

```C
// 返回系统当前的 Tick 数 (单位通常为 ms)
time_gt gmp_base_get_system_tick(void);
```

### 2.2 定义任务函数

任务函数必须符合 `gmp_task_handler_t` 原型。函数接收自身的指针作为参数，并返回执行状态。

```C
// 示例：LED 闪烁任务
gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    // 执行具体的硬件操作
    toggle_led();
    
    // 返回 DONE 表示本次调度完成，进入休眠等待下一个周期
    return GMP_TASK_DONE;
}
```

### 2.3 注册与启动

在 `main` 函数或系统初始化阶段进行配置：

```C
// 1. 定义调度器实例
gmp_scheduler_t g_sched;

// 2. 定义任务列表
gmp_task_t g_tasks[] = {
   // 名称,        回调函数,      周期(ms), 上次运行, 使能, 用户参数
   { "blink",     tsk_blink,     1000,     0,       1,    NULL },
   { "monitor",   tsk_monitor,   500,      0,       1,    NULL }
};

void main(void) {
    // 3. 初始化调度器
    gmp_scheduler_init(&g_sched);

    // 4. 添加任务
    for(int i = 0; i < sizeof(g_tasks)/sizeof(gmp_task_t); ++i) {
        gmp_scheduler_add_task(&g_sched, &g_tasks[i]);
    }

    // 5. 主循环调度
    while (1) {
        gmp_scheduler_dispatch(&g_sched);
    }
}
```

------

## 3. 核心机制详解

### 3.1 任务状态返回值

这是本调度器最强大的功能，允许灵活控制任务流。

- **`GMP_TASK_DONE` (任务完成)**
  - 含义：任务本次逻辑执行完毕。
  - 行为：调度器会更新该任务的 `last_run` 时间戳。任务将休眠，直到下一个周期时间到达（`current - last_run >= period`）。
- **`GMP_TASK_BUSY` (任务忙/未完成)**
  - 含义：任务逻辑太长，或者需要等待某个条件，但不想阻塞死等。
  - 行为：调度器**不会**更新时间戳。在下一次 `dispatch` 调用时，调度器会**优先**再次调用该任务（忽略其他任务的周期检查），直到该任务返回 `DONE`。
  - **用途**：用于将一个耗时 100ms 的操作，拆分成 10 次 10ms 的操作，避免卡死主循环。

#### 示例：使用 BUSY 拆分长任务

```C
gmp_task_status_t tsk_long_process(gmp_task_t* tsk)
{
    // 利用 tsk->run_state 记录内部状态
    switch(tsk->run_state) {
        case 0:
            start_adc_conversion();
            tsk->run_state = 1;
            return GMP_TASK_BUSY; // 立即返回，不阻塞等待ADC
            
        case 1:
            if (check_adc_ready()) {
                read_adc_value();
                tsk->run_state = 0; // 重置状态
                return GMP_TASK_DONE; // 彻底完成，进入休眠
            }
            return GMP_TASK_BUSY; // ADC还没好，下一轮再来看看
    }
    return GMP_TASK_DONE;
}
```

### 3.2 调度策略

`gmp_scheduler_dispatch` 函数每次执行只会运行**一个**任务：

1. **优先处理阻塞任务**：检查是否有任务上次返回了 `BUSY`。如果有，立即运行它。
2. **轮询周期任务**：如果没有阻塞任务，则遍历任务列表，找到第一个“时间已到”且“已使能”的任务并执行。

因此，必须在 `while(1)` 中高频调用 `dispatch` 函数。

------

## 4. API 参考

| **API 函数**             | **描述**                                                     |
| ------------------------ | ------------------------------------------------------------ |
| `gmp_scheduler_init`     | 初始化调度器结构体，清空任务列表和计数器。                   |
| `gmp_scheduler_add_task` | 向调度器注册一个新任务。如果超过 `GMP_SCHEDULER_MAX_TASKS` 会返回非零错误码。 |
| `gmp_scheduler_dispatch` | 调度器的核心引擎。需要在主循环中无限调用。                   |

## 5. 结构体成员说明 (`gmp_task_t`)

您可以直接在任务函数中访问或修改 `tsk` 指针指向的成员：

- `period`: 运行周期 (ms)。可以在运行时动态修改，例如实现 LED 变频闪烁。
- `is_enabled`: 任务开关。设置为 0 可暂停任务，设置为 1 恢复任务。
- `user_data`: `void*` 指针，用于传递私有参数，避免使用全局变量。
- `run_state`: 专门留给用户使用的状态变量，配合 `GMP_TASK_BUSY` 实现状态机。

------

## 6. 注意事项

1. **最大任务数**：默认支持 16 个任务。如需更多，请在头文件引入前定义宏 `GMP_SCHEDULER_MAX_TASKS`。
2. **非抢占式**：由于是协作式调度，如果某个任务的函数执行时间过长（例如使用了 `delay_ms(100)`），会阻塞所有其他任务的执行。请务必保持任务函数短小精悍。
3. **系统 Tick**：`gmp_base_get_system_tick` 的精度直接影响调度的准确性，建议使用硬件定时器产生的毫秒计数。