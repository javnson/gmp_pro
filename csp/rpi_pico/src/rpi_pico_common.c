
#include <gmp_core.h>



time_gt gmp_base_get_system_tick(void)
{
    return (time_gt)to_ms_since_boot(get_absolute_time());
}

/**
 * @brief 喂狗程序：刷新硬件看门狗计数器
 */
void gmp_hal_wd_feed(void)
{
    // 检查看门狗是否已经启动，如果已启动则刷新
    if (watchdog_caused_reboot() || watchdog_enable_caused_reboot())
    {
        // 这里的逻辑可以根据实际项目需求调整，通常直接调用更新即可
    }

    watchdog_update();
}

// This function may be called and used to initialize all the peripheral.
void gmp_csp_startup(void)
{
}

// This function would be called when fatal error occurred.
void gmp_port_system_stuck(void)
{
}

// This function would be called when all the initialization process happened.
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

uart_halt debug_uart;
