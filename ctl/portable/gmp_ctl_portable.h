/**
 * @file gmp_ctl_portable.h
 * @brief Minimal no-CSP contract used by CTL algorithms.
 *
 * Define GMP_CTL_PORTABLE globally and put one platform template directory
 * (ti_dsp or stm32) on the compiler include path.  That directory must provide
 * gmp_ctl_portable_config.h.
 */

#ifndef _FILE_GMP_CTL_PORTABLE_H_
#define _FILE_GMP_CTL_PORTABLE_H_

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <core/std/cfg/options.cfg.h>

#ifndef GMP_CTL_PORTABLE_CONFIG_HEADER
#define GMP_CTL_PORTABLE_CONFIG_HEADER <gmp_ctl_portable_config.h>
#endif
#include GMP_CTL_PORTABLE_CONFIG_HEADER

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef SPECIFY_DISABLE_CSP
#define SPECIFY_DISABLE_CSP
#endif

#ifndef SPECIFY_ENABLE_GMP_CTL
#define SPECIFY_ENABLE_GMP_CTL
#endif

#ifndef SPECIFY_CTRL_GT_TYPE
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#endif

#ifndef SPECIFY_PARAMETER_GT_TYPE
#define SPECIFY_PARAMETER_GT_TYPE USING_FLOAT_FPU
#endif

#ifndef GMP_STATIC_INLINE
#define GMP_STATIC_INLINE static inline
#endif

#ifndef GMP_INLINE
#define GMP_INLINE inline
#endif

#ifndef FAST_FUNCTION
#define FAST_FUNCTION GMP_STATIC_INLINE
#endif

#ifndef GMP_UNUSED_VAR
#define GMP_UNUSED_VAR(value) ((void)(value))
#endif

#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(value) ((void)(value))
#endif

#ifndef TEXT_STRING
#define TEXT_STRING(value) (value)
#endif

/* Basic CTL-visible scalar types.  Every type can be replaced in the selected
 * platform configuration before this header defines its default. */
#ifndef GMP_PORT_DATA_T
#define GMP_PORT_DATA_T int8_t
#endif
typedef GMP_PORT_DATA_T data_gt;

#ifndef GMP_PORT_FAST_T
#define GMP_PORT_FAST_T int_fast32_t
#endif
typedef GMP_PORT_FAST_T fast_gt;

#ifndef GMP_PORT_FAST16_T
#define GMP_PORT_FAST16_T int_fast16_t
#endif
typedef GMP_PORT_FAST16_T fast16_gt;

#ifndef GMP_PORT_FAST32_T
#define GMP_PORT_FAST32_T int_fast32_t
#endif
typedef GMP_PORT_FAST32_T fast32_gt;

#ifndef GMP_PORT_TIME_T
#define GMP_PORT_TIME_T uint32_t
#endif
typedef GMP_PORT_TIME_T time_gt;

#ifndef GMP_PORT_TIME_MAXIMUM
#define GMP_PORT_TIME_MAXIMUM UINT32_MAX
#endif

#ifndef GMP_PORT_SIZE_T
#define GMP_PORT_SIZE_T size_t
#endif
typedef GMP_PORT_SIZE_T size_gt;

#ifndef GMP_PORT_ADDR_T
#define GMP_PORT_ADDR_T uintptr_t
#endif
typedef GMP_PORT_ADDR_T addr_gt;

#ifndef GMP_PORT_ADDR32_T
#define GMP_PORT_ADDR32_T uint32_t
#endif
typedef GMP_PORT_ADDR32_T addr32_gt;

#ifndef GMP_PORT_ADDR16_T
#define GMP_PORT_ADDR16_T uint16_t
#endif
typedef GMP_PORT_ADDR16_T addr16_gt;

#ifndef GMP_PORT_DIFF_T
#define GMP_PORT_DIFF_T int32_t
#endif
typedef GMP_PORT_DIFF_T diff_gt;

#ifndef GMP_PORT_PARAM_T
#define GMP_PORT_PARAM_T int32_t
#endif
typedef GMP_PORT_PARAM_T param_gt;

#ifndef GMP_PORT_ADC_T
#define GMP_PORT_ADC_T uint32_t
#endif
typedef GMP_PORT_ADC_T adc_gt;

#ifndef GMP_PORT_DAC_T
#define GMP_PORT_DAC_T int32_t
#endif
typedef GMP_PORT_DAC_T dac_gt;

#ifndef GMP_PORT_PWM_T
#define GMP_PORT_PWM_T int32_t
#endif
typedef GMP_PORT_PWM_T pwm_gt;

/* Assertion is the only diagnostic contract used directly by ordinary CTL
 * components.  A project may redirect it or compile it out in its config. */
#ifndef GMP_CTL_PORTABLE_ASSERT
#if defined(DISABLE_CTL_LIB_ASSERT)
#define GMP_CTL_PORTABLE_ASSERT(condition) ((void)(condition))
#else
#define GMP_CTL_PORTABLE_ASSERT(condition) assert(condition)
#endif
#endif

#ifndef gmp_base_assert
#define gmp_base_assert(condition) GMP_CTL_PORTABLE_ASSERT(condition)
#endif

#ifndef gmp_ctl_assert
#define gmp_ctl_assert(condition) GMP_CTL_PORTABLE_ASSERT(condition)
#endif

/* Time-based CTL helpers use one optional hook.  Projects that do not use such
 * a component do not need to provide gmp_ctl_portable_get_tick(). */
#ifndef GMP_CTL_PORTABLE_GET_TICK
time_gt gmp_ctl_portable_get_tick(void);
#define GMP_CTL_PORTABLE_GET_TICK() gmp_ctl_portable_get_tick()
#endif

GMP_STATIC_INLINE time_gt gmp_base_get_system_tick(void)
{
    return (time_gt)GMP_CTL_PORTABLE_GET_TICK();
}

GMP_STATIC_INLINE time_gt gmp_base_get_ctrl_tick(void)
{
    return (time_gt)GMP_CTL_PORTABLE_GET_TICK();
}

GMP_STATIC_INLINE time_gt gmp_base_time_sub(time_gt t1, time_gt t0)
{
    return (time_gt)(t1 - t0);
}

GMP_STATIC_INLINE time_gt gmp_base_get_diff_system_tick(time_gt t0)
{
    return gmp_base_time_sub(gmp_base_get_system_tick(), t0);
}

GMP_STATIC_INLINE time_gt gmp_base_get_diff_ctrl_tick(time_gt t0)
{
    return gmp_base_time_sub(gmp_base_get_ctrl_tick(), t0);
}

GMP_STATIC_INLINE fast_gt gmp_base_is_delay_elapsed(time_gt t0, uint32_t delay_t)
{
    return (fast_gt)(gmp_base_get_diff_system_tick(t0) >= (time_gt)delay_t);
}

GMP_STATIC_INLINE fast_gt gmp_base_is_ctrl_delay_elapsed(time_gt t0, uint32_t delay_t)
{
    return (fast_gt)(gmp_base_get_diff_ctrl_tick(t0) >= (time_gt)delay_t);
}

#include <ctl/ctl.config.h>
#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
}
#endif

#endif // _FILE_GMP_CTL_PORTABLE_H_
