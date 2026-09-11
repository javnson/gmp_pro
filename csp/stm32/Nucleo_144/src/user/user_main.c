/** @file user_main.c Shared scheduler, Data Link, and status heartbeat. */

#include <gmp_core.h>

#include <core/pm/function_scheduler/function_scheduler.h>
#include <xplt.ethernet.h>
#include <xplt.peripheral.h>

#include "user_dl.h"
#include "user_main.h"

static gmp_scheduler_t scheduler;

static gmp_task_status_t user_task_heartbeat(gmp_task_t* task)
{
    GMP_UNUSED_VAR(task);
#if GMP_NUCLEO_ENABLE_STATUS_LED
    xplt_toggle_status_led();
#endif
#if GMP_NUCLEO_DUAL_CORE
#if defined(CORE_CM4)
    gmp_nucleo_dual_core_status.cm4_scheduler_heartbeats++;
#else
    gmp_nucleo_dual_core_status.cm7_scheduler_heartbeats++;
#endif
#endif
    return GMP_TASK_DONE;
}

#if !GMP_NUCLEO_ENABLE_CONTROL
static gmp_task_status_t user_task_scope_sample(gmp_task_t* task)
{
    GMP_UNUSED_VAR(task);
    user_dl_scheduler_step();
    return GMP_TASK_DONE;
}
#endif

static gmp_task_t tasks[] = {
    {"datalink", user_dl_task, 1U, 0U, 1, NULL, 0},
#if !GMP_NUCLEO_ENABLE_CONTROL
    {"scope", user_task_scope_sample, 1U, 0U, 1, NULL, 0},
#endif
    {"heartbeat", user_task_heartbeat, 500U, 0U, 1, NULL, 0},
};

void init(void)
{
    size_gt index;

#if GMP_NUCLEO_DUAL_CORE && defined(CORE_CM7)
    user_dl_dual_core_bootstrap();
#endif
    gmp_scheduler_init(&scheduler);
    for (index = 0U; index < sizeof(tasks) / sizeof(tasks[0]); ++index)
        (void)gmp_scheduler_add_task(&scheduler, &tasks[index]);
    user_dl_init();
}

void mainloop(void)
{
#if GMP_NUCLEO_ENABLE_ETHERNET_DL
    xplt_eth_poll();
#endif
    gmp_scheduler_dispatch(&scheduler);
}
