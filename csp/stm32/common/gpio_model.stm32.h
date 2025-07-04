/**
 * @file gpio_model.stm32.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2024-09-30
 * 
 * @copyright Copyright GMP(c) 2024
 * 
 */

#ifndef _FILE_GPIO_MODEL_STM32_H_
#define _FILE_GPIO_MODEL_STM32_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifdef HAL_GPIO_MODULE_ENABLED



    /**
     * @brief Setup GPIO port and pin.
     * @param hgpio handle of GPIO
     * @param gpio_port GPIO port of STM32
     * @param gpio_pin GPIO pin of STM32    
     */
    void gmp_hal_gpio_init(gpio_model_stm32_t *hgpio, GPIO_TypeDef *gpio_port, uint32_t gpio_pin);

    /**
     * @brief Set mode of gpio port, mode 0: input, mode 1: output.
     * @param hgpio handle of gpio. Type of GPIO handle is given by CSP.
     * @param mode target mode of GPIO. mode 0 is input mode, 1 is output mode.
     */
    ec_gt gmp_hal_gpio_set_mode(gpio_model_stm32_t *hgpio, fast_gt mode);

    /**
     * @brief Write GPIO port. This port must be an output port.
     * Or, undefined things may happen.
     * @param hgpio handle of GPIO
     * @param level target electrical level of GPIO port.
     */
    ec_gt gmp_hal_gpio_write(gpio_model_stm32_t *hgpio, fast_gt level);

    /**
     * @brief Read GPIO port, This port should be an input port.
     * Or the return value is undefined.
     * @param hgpio handle of GPIO
     * @return fast_gt return GPIO electrical level
     */
    fast_gt gmp_hal_gpio_read(gpio_model_stm32_t *hgpio);

    /**
     * @brief Set GPIO electrical level to high.
     * if GPIO mode is not output mode, the result is undefined.
     * @param hgpio handle of GPIO
     */
    ec_gt gmp_hal_gpio_set(gpio_model_stm32_t *hgpio);

    /**
     * @brief Set GPIO electrical level to low.
     * if GPIO mode is not output mode, the result is undefined.
     * @param hgpio handle of GPIO
     */
    ec_gt gmp_hal_gpio_clear(gpio_model_stm32_t *hgpio);

#endif // HAL_GPIO_MODULE_ENABLED

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GPIO_MODEL_STM32_H_
