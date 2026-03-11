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
 * @brief STM32 specific GPIO handle structure.
 * @note  Users should instantiate this structure in their board support package 
 * (BSP) and pass its pointer as the gpio_halt handle to the GMP layer.
 */
typedef struct
{
    GPIO_TypeDef* port; /**< STM32 GPIO Port (e.g., GPIOA, GPIOB) */
    uint16_t pin;       /**< STM32 GPIO Pin (e.g., GPIO_PIN_4) */
} gmp_gpio_stm32_t;

#endif // HAL_GPIO_MODULE_ENABLED

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GPIO_MODEL_STM32_H_
