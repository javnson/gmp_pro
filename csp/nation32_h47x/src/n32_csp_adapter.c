/**
 * @file n32_csp_adapter.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2026-08-06
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// This file provide a set of function that CSP must defined.

#include <gmp_core.h>
#include <stdarg.h>
#include <stdio.h>

// Millisecond tick advanced by the target project's Cortex-M SysTick handler.
static volatile time_gt gmp_n32_system_tick = 0;

// User should invoke this function to get time (system tick).
time_gt gmp_base_get_system_tick(void)
{
    return gmp_n32_system_tick;
}

// This function should be called in order to get system tick.
void gmp_step_system_tick(void)
{
    ++gmp_n32_system_tick;
}

/**
 * @brief This function may fresh IWDG counter.
 * This function should be implemented by CSP,
 * Every Loop routine, this function would be called.
 * CSP implementation should ensure that the function has only one thing is to feed the watchdog
 */
void gmp_hal_wd_feed(void)
{
}

void gmp_hal_wd_enable(void)
{
}

void gmp_hal_wd_disable(void)
{
}

// This function may be called and used to initilize all the peripheral.
void gmp_csp_startup(void)
{
}

// This function would be called when fatal error occorred.
void gmp_csp_stuck_routine(void)
{
    __disable_irq();
    GMP_DBG_SWBP;
    for (;;)
    {
    }
}

// This function would be called when all the initilization process happened.
void gmp_csp_post_process(void)
{
}

// This function is unreachable.
void gmp_csp_exit(void)
{
}

void gmp_csp_not_implement(void)
{
    gmp_csp_stuck_routine();
}

// This function may invoke when main loop occurred.
void gmp_csp_loop(void)
{
}

uart_halt debug_uart = NULL;

// implement the gmp_debug_print routine.
size_gt gmp_base_print_n32(const char* p_fmt, ...)
{
    static data_gt str[GMP_BASE_PRINT_CHAR_EXT];
    va_list args;
    int formatted_length;
    size_gt output_length;

    if ((debug_uart == NULL) || (p_fmt == NULL))
        return 0;

    va_start(args, p_fmt);
    formatted_length = vsnprintf((char *)str, sizeof(str), p_fmt, args);
    va_end(args);

    if (formatted_length <= 0)
        return 0;

    output_length = (size_gt)formatted_length;
    if (output_length >= (size_gt)sizeof(str))
        output_length = (size_gt)sizeof(str) - 1U;

    if (gmp_hal_uart_write(debug_uart, str, output_length, 100U) != GMP_EC_OK)
        return 0;

    return output_length;
}
