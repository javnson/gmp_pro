/**
 * @file xplt.config.h
 * @brief Shared GMP configuration for STM32 Nucleo-144 control targets.
 */

#ifndef GMP_STM32_NUCLEO_144_XPLT_CONFIG_H
#define GMP_STM32_NUCLEO_144_XPLT_CONFIG_H

#include <ctrl_settings.h>

/* Backward-compatible defaults for board entities generated before v0.6. */
#ifndef GMP_NUCLEO_QEP_SOFTWARE_INDEX
#define GMP_NUCLEO_QEP_SOFTWARE_INDEX (0U)
#endif
#ifndef GMP_NUCLEO_QEP_LEGACY_EXTI_CALLBACK
#define GMP_NUCLEO_QEP_LEGACY_EXTI_CALLBACK (0U)
#endif
#ifndef GMP_NUCLEO_ADC_REGULAR_DMA
#define GMP_NUCLEO_ADC_REGULAR_DMA (0U)
#endif
#ifndef GMP_NUCLEO_ADC_HAS_CALIBRATION
#define GMP_NUCLEO_ADC_HAS_CALIBRATION (1U)
#endif
#ifndef GMP_NUCLEO_ADC_TRIGGER_BRIDGE
#define GMP_NUCLEO_ADC_TRIGGER_BRIDGE (0U)
#endif
#ifndef GMP_NUCLEO_CAN_HAS_STBY
#define GMP_NUCLEO_CAN_HAS_STBY (0U)
#endif

#define SPECIFY_ENABLE_GMP_CTL
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#define SPECIFY_DISABLE_GMP_LOGO
#define SPECIFY_BASE_PRINT_NOT_IMPL

#endif // GMP_STM32_NUCLEO_144_XPLT_CONFIG_H


