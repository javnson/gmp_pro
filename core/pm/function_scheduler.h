/**
 * @file timing_manager.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2026-01-10
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef GMP_SCHEDULER_H
#define GMP_SCHEDULER_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// TASK Status
typedef enum
{
    GMP_TASK_DONE, // 任务本次执行完毕，进入休眠等待下一周期
    GMP_TASK_BUSY, // 任务未完成（阻塞/长任务），下一次 dispatch 立即再次调用
    GMP_TASK_YIELD // 任务处于前台，下一次dispatch保持调用
} gmp_task_status_t;

struct _tag_gmp_scheduler_t;
struct _tag_gmp_task_t;

// task routine callback
// 参数 self: 允许任务在函数内部修改自己的属性（如修改周期实现变频闪烁）
typedef gmp_task_status_t (*gmp_task_handler_t)(struct _tag_gmp_task_t* tsk);

// Task Control Block
typedef struct _tag_gmp_task_t
{
    const char* name;           // 任务名称（调试用）
    gmp_task_handler_t handler; // 任务函数指针
    time_gt period;             // 执行周期 (ms)，0 表示单次运行或由逻辑控制
    time_gt last_run;           // 上次运行的时间戳
    fast_gt is_enabled;         // 任务使能开关
    void* user_data;            // 私有数据指针

    // 运行时状态
    fast_gt run_state; // 内部状态机步进 (用于长任务分步执行)
} gmp_task_t;

#ifndef GMP_SCHEDULER_MAX_TASKS
#define GMP_SCHEDULER_MAX_TASKS (16)
#endif // GMP_SCHEDULER_MAX_TASKS

// Scheduler Entity
typedef struct _tag_gmp_scheduler_t
{
    // --- 配置与容器 ---
    gmp_task_t* task_list[GMP_SCHEDULER_MAX_TASKS]; // 任务指针数组
    uint16_t task_count;                            // 当前注册任务数

    // --- 运行时上下文 ---
    gmp_task_t* blocking_task; // 当前处于 BUSY 状态的任务 (断点)
    gmp_task_t* current_task;  // 当前正在执行的任务 (调试/监控用)

    // --- 统计信息 ---
    uint32_t dispatch_cnt; // 调度器被调用的总次数 (心跳计数)
    uint32_t busy_cnt;     // 发生阻塞重入的次数
} gmp_scheduler_t;

// API
void gmp_scheduler_init(gmp_scheduler_t* sched);
fast_gt gmp_scheduler_add_task(gmp_scheduler_t* sched, gmp_task_t* task);
void gmp_scheduler_dispatch(gmp_scheduler_t* sched);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // GMP_SCHEDULER_H


