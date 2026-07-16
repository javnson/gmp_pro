/**
 * @file xplt.peripheral.c
 * @brief Pico GPIO and periodic interrupt setup for the GMP blink example.
 */

#include <gmp_core.h>

#include <pico/time.h>

#include <xplt.peripheral.h>

#define MAIN_INTERRUPT_PERIOD_US 1000

#ifndef PICO_DEFAULT_LED_PIN
#error "The selected Pico board does not define PICO_DEFAULT_LED_PIN."
#endif

gpio_halt user_led = GMP_RPI_PICO_GPIO(PICO_DEFAULT_LED_PIN);

static struct repeating_timer g_main_timer;
static bool g_main_timer_started;

/**
 * @brief Adapt the Pico repeating timer callback to the GMP main ISR.
 * @param timer Pico SDK repeating timer descriptor.
 * @return Always true to keep the timer active.
 */
static bool main_timer_callback(struct repeating_timer* timer)
{
    (void)timer;
    MainISR();
    return true;
}

/**
 * @brief Initialize peripherals before GMP application initialization.
 */
void setup_peripheral(void)
{
    (void)gmp_hal_gpio_set_dir(user_led, GMP_HAL_GPIO_DIR_OUT);
    (void)gmp_hal_gpio_write(user_led, GMP_HAL_GPIO_LOW);
}

ec_gt xplt_start_main_interrupt(void)
{
    if (g_main_timer_started)
    {
        return GMP_EC_OK;
    }

    g_main_timer_started = add_repeating_timer_us(
        -MAIN_INTERRUPT_PERIOD_US, main_timer_callback, NULL, &g_main_timer);
    return g_main_timer_started ? GMP_EC_OK : GMP_EC_GENERAL_ERROR;
}

void MainISR(void)
{
    gmp_base_ctl_step();
}
