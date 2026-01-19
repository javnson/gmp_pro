/**
 * @file gmp_process_mgr.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2026-01-10
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

#include <core/pm/function_scheduler.h>

void gmp_scheduler_init(gmp_scheduler_t* sched)
{
    int i;

    gmp_base_assert(sched);

    // 清空所有状态
    sched->task_count = 0;
    sched->blocking_task = NULL;
    sched->current_task = NULL;
    sched->dispatch_cnt = 0;
    sched->busy_cnt = 0;

    for (i = 0; i < GMP_SCHEDULER_MAX_TASKS; i++)
    {
        sched->task_list[i] = NULL;
    }
}

fast_gt gmp_scheduler_add_task(gmp_scheduler_t* sched, gmp_task_t* task)
{
    gmp_base_assert(sched);
    gmp_base_assert(task);

    // border condition
    if (sched->task_count >= GMP_SCHEDULER_MAX_TASKS)
    {
        gmp_base_print("task queue is full.\r\n");
        return 1;
    }

    // 初始化任务默认参数
    //task->last_run = gmp_base_get_system_tick();
    task->run_state = 0;

    sched->task_list[sched->task_count++] = task;
    return 0;
}

void gmp_scheduler_dispatch(gmp_scheduler_t* sched)
{
    int i;

    gmp_base_assert(sched);

    // 1. 统计调度次数 (可用于评估主循环负载)
    sched->dispatch_cnt++;

    time_gt current_tick = gmp_base_get_system_tick();
    gmp_task_status_t status;
    gmp_task_t* target_task = NULL;

    // ---------------------------------------------------------
    // Phase 1: Check Pending task
    // ---------------------------------------------------------
    if (sched->blocking_task != NULL)
    {
        target_task = sched->blocking_task;

        // udpate context
        sched->current_task = target_task;
        sched->busy_cnt++; // counting busy waiting ticks

        // execute the task
        status = target_task->handler(target_task);

        // update status
        if (status == GMP_TASK_DONE)
        {
            target_task->last_run = current_tick; // update time stape
            sched->blocking_task = NULL;          // clear pending
        }
        // if retval is BUSY，blocking_task will keep

        sched->current_task = NULL; // clear context
        return;                     // one task per loop
    }

    // ---------------------------------------------------------
    // Phase 2: 轮询寻找下一个到期的任务
    // ---------------------------------------------------------
    for (i = 0; i < sched->task_count; i++)
    {
        gmp_task_t* task = sched->task_list[i];

        if (task && task->is_enabled)
        {
            // trigger condition judge
            if ((current_tick - task->last_run) >= task->period)
            {
                target_task = task;

                // update context
                sched->current_task = target_task;

                // execute task
                status = target_task->handler(target_task);

                // update status
                if (status == GMP_TASK_BUSY)
                {
                    sched->blocking_task = target_task; // flag of pending
                }
                else
                {
                    target_task->last_run = current_tick; // flag of complete
                }

                sched->current_task = NULL; // clear context
                return;                     // one task per loop
            }
        }
    }
}
