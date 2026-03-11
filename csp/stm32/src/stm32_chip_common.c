/**
 * @file stm32_chip_common.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Computing Model
#include <csp/stm32/common/computing_model.stm32.h>

#ifdef HAL_CORDIC_MODULE_ENABLED

#endif // HAL_CORDIC_MODULE_ENABLED

//////////////////////////////////////////////////////////////////////////
// GPIO Model
#include <csp/stm32/common/gpio_model.stm32.h>

#ifdef HAL_GPIO_MODULE_ENABLED

ec_gt gmp_hal_gpio_set_dir(gpio_halt hgpio, gpio_dir_et dir)
{
    /* Treat NULL handle safely if it implies an unassigned pin */
    if (hgpio == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    gmp_gpio_stm32_t* gpio = (gmp_gpio_stm32_t*)hgpio;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = gpio->pin;

    if (dir == GMP_HAL_GPIO_DIR_OUT)
    {
        /* Configure pin as Push-Pull Output, High Speed, No Pull-up/down */
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    }
    else
    {
        /* Configure pin as Input, Floating (No Pull) */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        /* Speed is not applicable for Input mode */
    }

    /* Apply the configuration using STM32 HAL */
    HAL_GPIO_Init(gpio->port, &GPIO_InitStruct);

    return GMP_EC_OK;
}

ec_gt gmp_hal_gpio_write(gpio_halt hgpio, fast_gt level)
{
    if (hgpio == NULL)
        return GMP_EC_GENERAL_ERROR;

    gmp_gpio_stm32_t* gpio = (gmp_gpio_stm32_t*)hgpio;

    /* Map GMP logic levels to STM32 HAL PinStates */
    GPIO_PinState state = (level == GMP_HAL_GPIO_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(gpio->port, gpio->pin, state);

    return GMP_EC_OK;
}

fast_gt gmp_hal_gpio_read(gpio_halt hgpio)
{
    /* Safely return 0 (LOW) for a NULL handle */
    if (hgpio == NULL)
        return 0;

    gmp_gpio_stm32_t* gpio = (gmp_gpio_stm32_t*)hgpio;

    /* Read the pin state using STM32 HAL */
    GPIO_PinState state = HAL_GPIO_ReadPin(gpio->port, gpio->pin);

    /* Return mapped fast_gt integer */
    return (state == GPIO_PIN_SET) ? 1 : 0;
}
#endif // HAL_GPIO_MODULE_ENABLED



//////////////////////////////////////////////////////////////////////////
// System Model
#include <csp/stm32/common/sys_model.stm32.h>

#ifdef HAL_RCC_MODULE_ENABLED

time_gt gmp_base_get_system_tick(void)
{
    return HAL_GetTick();
}

#endif

/**
 * @brief This function may fresh IWDG counter.
 * This function should be implemented by CSP,
 * Every Loop routine, this function would be called.
 * CSP implementation should ensure that the function has only one thing is to feed the watchdog
 */
void gmp_hal_wd_feed(void)
{
#if defined SPECIFY_ENABLE_FEED_WATCHDOG
    HAL_IWDG_Refresh(&hiwdg);
#endif // SPECIFY_ENABLE_FEED_WATCHDOG
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

uart_halt debug_uart;

// implement the gmp_debug_print routine.
size_gt gmp_base_print_c28xsyscfg(const char* p_fmt, ...)
{
    // if no one was specified to output, just ignore the request.
    if (debug_uart == NULL)
    {
        return 0;
    }

    // size_gt size = (size_gt)strlen(p_fmt);

    static data_gt str[GMP_BASE_PRINT_CHAR_EXT];
    memset(str, 0, GMP_BASE_PRINT_CHAR_EXT);

    va_list vArgs;
    va_start(vArgs, p_fmt);
    vsprintf((char*)str, (char const*)p_fmt, vArgs);
    va_end(vArgs);

    size_gt length = (size_gt)strlen((char*)str);

    HAL_UART_Transmit_DMA(debug_uart, str, length);

    return length;
}
