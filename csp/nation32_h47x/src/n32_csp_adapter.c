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
#include <stdio.h>
#include <string.h>

// System Tick
time_gt DSPC2000_SystemTick = 0;

// User should invoke this function to get time (system tick).
time_gt gmp_base_get_system_tick(void)
{
    return DSPC2000_SystemTick;
}

// This function should be called in order to get system tick.
void gmp_step_system_tick(void)
{
    DSPC2000_SystemTick += 1;
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

// This function may be called and used to initilize all the peripheral.
void gmp_csp_startup(void)
{
}

// This function would be called when fatal error occorred.
void gmp_port_system_stuck(void)
{
}

// This function would be called when all the initilization process happened.
void gmp_csp_post_process(void)
{
}

// This function is unreachable.
void gmp_exit_routine(void)
{
}

// This function may invoke when main loop occurred.
void gmp_csp_loop(void)
{
}

uart_halt debug_uart = 0;

// implement the gmp_debug_print routine.
size_gt gmp_base_print_n32(const char* p_fmt, ...)
{
    //// if no one was specified to output, just ignore the request.
    //if (debug_uart == NULL)
    //{
    //    return 0;
    //}

    //// size_gt size = (size_gt)strlen(p_fmt);

    //static data_gt str[GMP_BASE_PRINT_CHAR_EXT];
    //memset(str, 0, GMP_BASE_PRINT_CHAR_EXT);

    //va_list vArgs;
    //va_start(vArgs, p_fmt);
    //vsprintf((char*)str, (char const*)p_fmt, vArgs);
    //va_end(vArgs);

    //size_gt length = (size_gt)strlen((char*)str);

    //HAL_UART_Transmit_DMA(debug_uart, str, length);

    //return length;

    return 0;
}
