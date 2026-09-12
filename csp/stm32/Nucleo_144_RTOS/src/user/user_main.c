/**
 * @file user_main.c
 * @brief GMP cooperative task set hosted by the FreeRTOS service task.
 */

#include <gmp_core.h>

#include <core/pm/function_scheduler/function_scheduler.h>
#include <xplt.peripheral.h>

#include "user_dl.h"

static gmp_scheduler_t scheduler;

static gmp_task_status_t user_task_heartbeat(gmp_task_t* task)
{
    GMP_UNUSED_VAR(task);
#if GMP_NUCLEO_ENABLE_STATUS_LED
    xplt_toggle_status_led();
#endif
    return GMP_TASK_DONE;
}

/* Keep the periodic low-rate task before the 1 ms Data Link task. The current
   core/pm dispatcher always scans from index zero, so a continuously due task
   in slot zero would otherwise starve later tasks. */
static gmp_task_t tasks[] = {
    {"heartbeat", user_task_heartbeat, 500U, 0U, 1, NULL, 0},
    {"datalink", user_dl_task, 1U, 0U, 1, NULL, 0},
};

void init(void)
{
    size_gt index;

    gmp_scheduler_init(&scheduler);
    for (index = 0U; index < sizeof(tasks) / sizeof(tasks[0]); ++index)
        (void)gmp_scheduler_add_task(&scheduler, &tasks[index]);
    user_dl_init();
}

void mainloop(void)
{
    gmp_scheduler_dispatch(&scheduler);
}
