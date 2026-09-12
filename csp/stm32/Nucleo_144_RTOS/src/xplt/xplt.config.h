/**
 * @file xplt.config.h
 * @brief GMP configuration overrides for the NUCLEO-H753ZI RTOS reference.
 */

#ifndef GMP_STM32_NUCLEO_144_RTOS_XPLT_CONFIG_H
#define GMP_STM32_NUCLEO_144_RTOS_XPLT_CONFIG_H

#include <ctrl_settings.h>

/* The IOC regular-conversion order differs from the legacy entity aliases.
   Keep the correction local so the existing bare-metal project is unchanged. */
#undef GMP_NUCLEO_ADC_FB0_RANK
#undef GMP_NUCLEO_ADC_FB1_RANK
#undef GMP_NUCLEO_ADC_FB2_RANK
#undef GMP_NUCLEO_ADC_FB3_RANK
#undef GMP_NUCLEO_ADC_FB4_RANK
#undef GMP_NUCLEO_ADC_FB5_RANK
#define GMP_NUCLEO_ADC_FB0_RANK 5
#define GMP_NUCLEO_ADC_FB1_RANK 1
#define GMP_NUCLEO_ADC_FB2_RANK 0
#define GMP_NUCLEO_ADC_FB3_RANK 3
#define GMP_NUCLEO_ADC_FB4_RANK 4
#define GMP_NUCLEO_ADC_FB5_RANK 2

#define SPECIFY_ENABLE_GMP_CTL
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#define SPECIFY_DISABLE_GMP_LOGO
#define SPECIFY_BASE_PRINT_NOT_IMPL

#endif // GMP_STM32_NUCLEO_144_RTOS_XPLT_CONFIG_H
