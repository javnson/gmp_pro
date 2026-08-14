/**
 * @file gmp_ctl_portable_config.h
 * @brief Numeric-domain configuration for hosted CTL unit tests.
 */

#ifndef GMP_CTL_UNIT_TEST_PORTABLE_CONFIG_H
#define GMP_CTL_UNIT_TEST_PORTABLE_CONFIG_H

#define SPECIFY_DISABLE_CSP
#define SPECIFY_ENABLE_GMP_CTL
#define SPECIFY_PARAMETER_GT_TYPE USING_DOUBLE_FPU
#define SPECIFY_REAL_GT_TYPE      USING_DOUBLE_FPU

#if defined(GMP_CTL_TEST_IQMATH)
#define SPECIFY_CTRL_GT_TYPE USING_FIXED_TI_IQ_LIBRARY
#elif defined(GMP_CTL_TEST_DOUBLE)
#define SPECIFY_CTRL_GT_TYPE USING_DOUBLE_FPU
#else
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#endif

#define GMP_PORT_DATA_T unsigned char

#endif /* GMP_CTL_UNIT_TEST_PORTABLE_CONFIG_H */
