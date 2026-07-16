/**
 * @file rpi_pico_common.c
 * @brief System, critical-section and watchdog hooks for the Pico SDK.
 */

#include <gmp_core.h>

#include <pico/platform.h>
#include <hardware/watchdog.h>

#ifndef GMP_RPI_PICO_WATCHDOG_TIMEOUT_MS
#define GMP_RPI_PICO_WATCHDOG_TIMEOUT_MS 0U
#endif

/* RP2040 and RP2350 both have two cores. State must be private per core. */
static uint32_t g_critical_irq_state[2];
static uint32_t g_critical_depth[2];

uart_halt debug_uart = NULL;

/**
 * @brief Enter a core-local, nestable interrupt critical section.
 */
void gmp_base_enter_critical(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    uint32_t core = get_core_num() & 1U;

    if (g_critical_depth[core] == 0U)
    {
        g_critical_irq_state[core] = irq_state;
    }
    ++g_critical_depth[core];
}

/**
 * @brief Leave a core-local critical section and restore the previous IRQ state.
 */
void gmp_base_leave_critical(void)
{
    uint32_t core = get_core_num() & 1U;

    if (g_critical_depth[core] == 0U)
    {
        return;
    }

    --g_critical_depth[core];
    if (g_critical_depth[core] == 0U)
    {
        restore_interrupts(g_critical_irq_state[core]);
    }
}

/**
 * @brief Return the number of milliseconds elapsed since boot.
 * @return Monotonic system time in milliseconds.
 */
time_gt gmp_base_get_system_tick(void)
{
    return (time_gt)to_ms_since_boot(get_absolute_time());
}

/**
 * @brief Enable the watchdog when a nonzero timeout is configured.
 */
void gmp_hal_wd_enable(void)
{
#if GMP_RPI_PICO_WATCHDOG_TIMEOUT_MS > 0U
    watchdog_enable(GMP_RPI_PICO_WATCHDOG_TIMEOUT_MS, true);
#endif
}

/**
 * @brief Disable the hardware watchdog.
 */
void gmp_hal_wd_disable(void)
{
    watchdog_disable();
}

/**
 * @brief Reload the watchdog counter when watchdog support is enabled.
 */
void gmp_hal_wd_feed(void)
{
#if GMP_RPI_PICO_WATCHDOG_TIMEOUT_MS > 0U
    watchdog_update();
#endif
}

/**
 * @brief Run early CSP initialization.
 */
void gmp_csp_startup(void)
{
    gmp_hal_wd_enable();
}

/**
 * @brief Run the CSP hook immediately before entering the main loop.
 */
void gmp_csp_post_process(void)
{
}

/**
 * @brief Run periodic CSP maintenance.
 */
void gmp_csp_loop(void)
{
    gmp_hal_wd_feed();
}

/**
 * @brief Run the CSP exit hook.
 */
void gmp_csp_exit(void)
{
}

/**
 * @brief Halt the current core with interrupts disabled.
 */
void gmp_csp_stuck_routine(void)
{
    (void)save_and_disable_interrupts();
    for (;;)
    {
        tight_loop_contents();
    }
}

/**
 * @brief Handle an invocation of an unsupported CSP feature.
 */
void gmp_csp_not_implement(void)
{
    gmp_csp_stuck_routine();
}

/**
 * @brief Compatibility wrapper for older GMP system-stuck hooks.
 */
void gmp_port_system_stuck(void)
{
    gmp_csp_stuck_routine();
}

/**
 * @brief Compatibility wrapper for older GMP exit hooks.
 */
void gmp_exit_routine(void)
{
    gmp_csp_exit();
}
