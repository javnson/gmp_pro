/**
 * @file gmp_ctl_portable_config.h
 * @brief Copyable CTL portable configuration template for STM32 projects.
 */

#ifndef _FILE_GMP_CTL_PORTABLE_STM32_CONFIG_H_
#define _FILE_GMP_CTL_PORTABLE_STM32_CONFIG_H_

#include "sdpe_gmp_ctl_portable_settings.h"

#ifndef SPECIFY_CTRL_GT_TYPE
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#endif

#ifndef SPECIFY_PARAMETER_GT_TYPE
#define SPECIFY_PARAMETER_GT_TYPE USING_FLOAT_FPU
#endif

#ifndef GMP_PORT_ADC_T
#define GMP_PORT_ADC_T uint16_t
#endif

#ifndef GMP_PORT_DAC_T
#define GMP_PORT_DAC_T uint16_t
#endif

#ifndef GMP_PORT_PWM_T
#define GMP_PORT_PWM_T uint32_t
#endif

/* HAL_GetTick has this signature in STM32 HAL.  Defining the macro here avoids
 * a GMP runtime dependency while keeping time-based CTL modules operational. */
uint32_t HAL_GetTick(void);
#define GMP_CTL_PORTABLE_GET_TICK() ((uint32_t)HAL_GetTick())

/* Optional example:
 * #define GMP_CTL_PORTABLE_ASSERT(expr) ((expr) ? (void)0 : Error_Handler())
 */

#endif // _FILE_GMP_CTL_PORTABLE_STM32_CONFIG_H_
