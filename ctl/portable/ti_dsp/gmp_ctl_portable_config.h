/**
 * @file gmp_ctl_portable_config.h
 * @brief Copyable CTL portable configuration template for TI C2000 DSPs.
 */

#ifndef _FILE_GMP_CTL_PORTABLE_TI_DSP_CONFIG_H_
#define _FILE_GMP_CTL_PORTABLE_TI_DSP_CONFIG_H_

#include "sdpe_gmp_ctl_portable_settings.h"

/* C28x/C29x devices with an FPU should normally keep this selection.  A C28x
 * project using IQmath may change it to USING_FIXED_TI_IQ_LIBRARY and provide
 * the IQmath headers/library in the project. */
#ifndef SPECIFY_CTRL_GT_TYPE
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#endif

#ifndef SPECIFY_PARAMETER_GT_TYPE
#define SPECIFY_PARAMETER_GT_TYPE USING_FLOAT_FPU
#endif

/* Override these only when the target ABI or peripheral interfaces require a
 * different raw representation. */
#ifndef GMP_PORT_DATA_T
/* C28x uses a 16-bit byte and therefore does not provide uint8_t. */
#define GMP_PORT_DATA_T unsigned char
#endif

#ifndef GMP_PORT_ADC_T
#define GMP_PORT_ADC_T uint16_t
#endif

#ifndef GMP_PORT_DAC_T
#define GMP_PORT_DAC_T int16_t
#endif

#ifndef GMP_PORT_PWM_T
#define GMP_PORT_PWM_T int32_t
#endif

/* Optional examples:
 * #define GMP_CTL_PORTABLE_ASSERT(expr) ((expr) ? (void)0 : user_fault())
 * #define GMP_CTL_PORTABLE_GET_TICK()   ((uint32_t)CpuTimer0.InterruptCount)
 */

#endif // _FILE_GMP_CTL_PORTABLE_TI_DSP_CONFIG_H_
