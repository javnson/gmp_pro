/**
 * @file sysconfig_adapter_cm.c
 * @brief GMP runtime adapter for the Cortex-M core in C2000 multicore parts.
 */

#include <gmp_core.h>

static volatile time_gt c28x_syscfg_cm_system_tick;

time_gt gmp_base_get_system_tick(void)
{
    return c28x_syscfg_cm_system_tick;
}

void gmp_c28x_syscfg_cm_step_tick(void)
{
    c28x_syscfg_cm_system_tick++;
}

void gmp_csp_startup(void)
{
    c28x_syscfg_cm_system_tick = 0U;
    gmp_c28x_syscfg_cm_device_init();
}

void gmp_csp_loop(void)
{
    gmp_c28x_syscfg_cm_device_loop();
}

void gmp_csp_post_process(void)
{
    gmp_c28x_syscfg_cm_post_start();
    (void)Interrupt_enableInProcessor();
}

void gmp_csp_exit(void)
{
}

void gmp_csp_stuck_routine(void)
{
    for (;;)
    {
    }
}

void gmp_csp_not_implement(void)
{
}

void main(void)
{
    gmp_base_entry();
}
