/**
 * @file xplt.config.h
 * @brief Shared GMP configuration for STM32 Nucleo-64 control targets.
 */

#ifndef GMP_STM32_NUCLEO_64_XPLT_CONFIG_H
#define GMP_STM32_NUCLEO_64_XPLT_CONFIG_H

#include <ctrl_settings.h>

#define SPECIFY_ENABLE_GMP_CTL
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#define SPECIFY_DISABLE_GMP_LOGO
#define SPECIFY_BASE_PRINT_NOT_IMPL

#endif // GMP_STM32_NUCLEO_64_XPLT_CONFIG_H
