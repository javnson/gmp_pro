/**
 * @file user_main.c
 * @brief Application scheduler and board-level task composition.
 */

#include <gmp_core.h>

#include "user_main.h"
#include "user_dl.h"
#include <xplt.peripheral.h>

gmp_scheduler_t scheduler;
volatile uint32_t startup_task_runs;
volatile uint32_t heartbeat_task_runs;
volatile uint32_t can_task_runs;

/** @brief Execute startup work exactly once and disable this task. */
gmp_task_status_t user_task_startup_once(gmp_task_t* task)
{
    startup_task_runs++;
    task->is_enabled = 0;
    return GMP_TASK_DONE;
}

/** @brief Toggle the selected board LED. */
gmp_task_status_t user_task_heartbeat(gmp_task_t* task)
{
    GMP_UNUSED_VAR(task);
    heartbeat_task_runs++;
    xplt_toggle_user_led();
    return GMP_TASK_DONE;
}

/** @brief Service CAN independently from the serial Data Link protocol. */
gmp_task_status_t user_task_can(gmp_task_t* task)
{
    GMP_UNUSED_VAR(task);
    can_task_runs++;
    xplt_can_service();
    return GMP_TASK_DONE;
}

/** Scheduler table remains global so task state is easy to inspect in CCS. */
gmp_task_t tasks[] = {
    {"startup_once", user_task_startup_once, 1U, 0U, 1, NULL},
    {"heartbeat", user_task_heartbeat, GMP_HEARTBEAT_TASK_PERIOD_MS, 0U, 1, NULL},
    {"datalink", user_dl_task, GMP_DATALINK_TASK_PERIOD_MS, 0U, 1, NULL},
    {"can", user_task_can, GMP_CAN_TASK_PERIOD_MS, 0U, 1, NULL},
};

void init(void)
{
    size_gt task_index;

    user_dl_init();
    gmp_scheduler_init(&scheduler);
    for (task_index = 0; task_index < sizeof(tasks) / sizeof(tasks[0]); ++task_index)
        (void)gmp_scheduler_add_task(&scheduler, &tasks[task_index]);
    xplt_start_sample_timer();
}

void mainloop(void)
{
    user_dl_background();
    gmp_scheduler_dispatch(&scheduler);
}
