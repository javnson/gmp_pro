/**
 * @file user_main.c
 * @brief LED blink application implemented with the GMP task scheduler.
 */

#include <gmp_core.h>
#include <core/pm/function_scheduler.h>

#include <user_main.h>
#include <xplt.peripheral.h>

static gmp_scheduler_t g_scheduler;

/** GMP task that toggles the board LED every 500 milliseconds. */
static gmp_task_t g_blink_task = {
    "blink_led",
    NULL,
    500U,
    0U,
    1,
    NULL,
    0
};

/**
 * @brief Toggle the board LED when scheduled.
 * @param task Pointer to the blink task control block.
 * @return GMP_TASK_DONE after completing the GPIO update.
 */
static gmp_task_status_t blink_led_task(gmp_task_t* task)
{
    static fast_gt led_level = GMP_HAL_GPIO_LOW;

    (void)task;
    led_level = led_level == GMP_HAL_GPIO_LOW ? GMP_HAL_GPIO_HIGH : GMP_HAL_GPIO_LOW;
    (void)gmp_hal_gpio_write(user_led, led_level);
    return GMP_TASK_DONE;
}

/**
 * @brief Initialize the blink application after controller initialization.
 */
void init(void)
{
    gmp_scheduler_init(&g_scheduler);
    g_blink_task.handler = blink_led_task;
    if (gmp_scheduler_add_task(&g_scheduler, &g_blink_task) != 0)
    {
        gmp_csp_stuck_routine();
    }

    if (xplt_start_main_interrupt() != GMP_EC_OK)
    {
        gmp_csp_stuck_routine();
    }
}

/**
 * @brief Dispatch one ready GMP task.
 */
void mainloop(void)
{
    gmp_scheduler_dispatch(&g_scheduler);
}
