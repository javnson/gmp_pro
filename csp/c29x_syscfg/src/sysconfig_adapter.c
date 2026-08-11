/**
 * @file sysconfig_adapter.c
 * @brief GMP lifecycle adapter for byte-addressed TI C29x devices.
 */

#include <gmp_core.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static volatile time_gt c29x_system_tick;
static uint32_t c29x_tick_divider = 1U;
static uint32_t c29x_tick_phase;

uart_halt debug_uart = 0U;

time_gt gmp_base_get_system_tick(void)
{
    return c29x_system_tick;
}

void gmp_c29x_set_tick_divider(uint32_t calls_per_tick)
{
    c29x_tick_divider = (calls_per_tick == 0U) ? 1U : calls_per_tick;
    c29x_tick_phase = 0U;
}

void gmp_step_system_tick(void)
{
    ++c29x_tick_phase;
    if (c29x_tick_phase >= c29x_tick_divider)
    {
        c29x_tick_phase = 0U;
        ++c29x_system_tick;
    }
}

void gmp_csp_startup(void)
{
    Device_init();
    Board_init();
    c29x_system_tick = 0U;
    c29x_tick_phase = 0U;
}

void gmp_csp_post_process(void)
{
    EINT;
    Interrupt_enableGlobal();
}

void gmp_csp_loop(void)
{
}

size_gt gmp_base_print_c29xsyscfg(const char *format, ...)
{
    static data_gt buffer[GMP_BASE_PRINT_CHAR_EXT];
    int formatted_length;
    size_gt write_length;
    va_list arguments;

    if ((debug_uart == 0U) || (format == NULL))
    {
        return 0U;
    }

    va_start(arguments, format);
    formatted_length = vsnprintf((char *)buffer, sizeof(buffer), format, arguments);
    va_end(arguments);

    if (formatted_length <= 0)
    {
        return 0U;
    }

    write_length = (size_gt)formatted_length;
    if (write_length >= (size_gt)sizeof(buffer))
    {
        write_length = (size_gt)sizeof(buffer) - 1U;
    }

    if (gmp_hal_uart_write(debug_uart, buffer, write_length, 10U) != GMP_EC_OK)
    {
        return 0U;
    }
    return write_length;
}

int main(void)
{
    gmp_base_entry();
    return 0;
}
