/**
 * @file arm_cmsis_macros.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a set of macros for floating-point arithmetic using the ARM CMSIS-DSP library.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides the concrete implementation of the control library's math
 * functions using the ARM CMSIS-DSP library for Cortex-M processors. It maps
 * the generic control math macros to their corresponding hardware-accelerated
 * CMSIS-DSP functions. This implementation is based on `float32_t`.
 */

#ifndef _FILE_ARM_CMSIS_MACROS_H_
#define _FILE_ARM_CMSIS_MACROS_H_

#include "arm_math.h"

/**
 * @defgroup MC_CMSIS_MACROS CMSIS-DSP Math Macros
 * @ingroup MC_DEFINES
 * @brief A collection of macros for hardware-accelerated arithmetic using CMSIS-DSP.
 *
 * This module defines the low-level math operations when the library is
 * configured to use the ARM CMSIS-DSP library as its primary math engine.
 * `ctrl_gt` becomes `float32_t`.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Type Conversion Macros                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_TYPE_CONVERSION_CMSIS Type Conversion Macros (CMSIS-DSP)
 * @ingroup MC_CMSIS_MACROS
 * @brief Macros for converting between `float32_t` and other data types.
 * @{
 */

#define float2ctrl(x) ((float32_t)(x)) /**< @brief Converts a literal to `ctrl_gt` (float32_t). */
#define ctrl2float(x) ((float32_t)(x)) /**< @brief Converts a `ctrl_gt` value to a standard float32_t. */
#define int2ctrl(x)   ((float32_t)(x)) /**< @brief Converts an integer to `ctrl_gt`. */
#define ctrl2int(x)   ((int32_t)(x))   /**< @brief Converts a `ctrl_gt` value to an integer (truncates). */

/**
 * @brief Computes the fractional part of a `ctrl_gt` value (x mod 1).
 * @param x The input value.
 * @return The fractional part of x.
 */
#define ctrl_mod_1(x) ((float32_t)(((float32_t)(x)) - ((int32_t)(x))))

/** @} */ // end of MC_TYPE_CONVERSION_CMSIS group

/*---------------------------------------------------------------------------*/
/* Arithmetic Macros                                                         */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ARITHMETIC_CMSIS Arithmetic Macros (CMSIS-DSP)
 * @ingroup MC_CMSIS_MACROS
 * @brief Macros for basic arithmetic operations using CMSIS-DSP functions where available.
 * @{
 */

#define ctl_mul(A, B) ((float32_t)((float32_t)(A) * (B))) /**< @brief Multiplies two `ctrl_gt` values. */
#define ctl_div(A, B) ((float32_t)((float32_t)(A) / (B))) /**< @brief Divides two `ctrl_gt` values. */
#define ctl_add(A, B) ((float32_t)((float32_t)(A) + (B))) /**< @brief Adds two `ctrl_gt` values. */
#define ctl_sub(A, B) ((float32_t)((float32_t)(A) - (B))) /**< @brief Subtracts two `ctrl_gt` values. */

#define ctl_div2(A) ((float32_t)((float32_t)(A) * 0.5f))  /**< @brief Divides a `ctrl_gt` value by 2. */
#define ctl_div4(A) ((float32_t)((float32_t)(A) * 0.25f)) /**< @brief Divides a `ctrl_gt` value by 4. */
#define ctl_mul2(A) ((float32_t)((float32_t)(A) * 2.0f))  /**< @brief Multiplies a `ctrl_gt` value by 2. */
#define ctl_mul4(A) ((float32_t)((float32_t)(A) * 4.0f))  /**< @brief Multiplies a `ctrl_gt` value by 4. */

#define ctl_abs(A) (fabsf(A)) /**< @brief Computes the absolute value of a `ctrl_gt` value. */

/**
 * @brief Saturates a `ctrl_gt` value between a positive and negative limit using CMSIS-DSP.
 * @param A The input value.
 * @param Pos The upper limit.
 * @param Neg The lower limit.
 * @return The saturated value.
 */
GMP_STATIC_INLINE float32_t ctl_sat_inline(float32_t A, float32_t Pos, float32_t Neg)
{
    float32_t result;
    arm_clip_f32(&A, &result, Neg, Pos, 1);
    return result;
}
#define ctl_sat(A, Pos, Neg) ctl_sat_inline((A), (Pos), (Neg))

/** @} */ // end of MC_ARITHMETIC_CMSIS group

/*---------------------------------------------------------------------------*/
/* Non-linear Function Macros                                                */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_NONLINEAR_CMSIS Non-linear Function Macros (CMSIS-DSP)
 * @ingroup MC_NONLINEAR_CMSIS
 * @brief Macros for non-linear mathematical functions using CMSIS-DSP.
 * @{
 */

/**
 * @brief Computes the sine of an angle given in per-unit.
 * @note Assumes a `PI` constant is defined. The input is scaled by 2*PI to convert to radians.
 * @param A The input angle in per-unit (0.0 to 1.0 represents 0 to 2¦Ð).
 * @return The sine of the angle. Maps to `arm_sin_f32()`.
 */
#define ctl_sin(A) (arm_sin_f32(2.0f * PI * (A)))

/**
 * @brief Computes the cosine of an angle given in per-unit.
 * @note Assumes a `PI` constant is defined. The input is scaled by 2*PI to convert to radians.
 * @param A The input angle in per-unit (0.0 to 1.0 represents 0 to 2¦Ð).
 * @return The cosine of the angle. Maps to `arm_cos_f32()`.
 */
#define ctl_cos(A) (arm_cos_f32(2.0f * PI * (A)))

/**
 * @brief Computes the square root of a value using CMSIS-DSP.
 * @param A The input value.
 * @return The square root of A. Maps to `arm_sqrt_f32()`.
 */
GMP_STATIC_INLINE float32_t ctl_sqrt_inline(float32_t A)
{
    float32_t result;
    arm_sqrt_f32(A, &result);
    return result;
}
#define ctl_sqrt(A) ctl_sqrt_inline(A)

/**
 * @brief Computes the fast inverse square root of a value using CMSIS-DSP.
 * @param A The input value.
 * @return The inverse square root of A. Maps to `arm_rsqrt_f32()`.
 */
GMP_STATIC_INLINE float32_t ctl_isqrt_inline(float32_t A)
{
    float32_t result;
    arm_rsqrt_f32(A, &result);
    return result;
}
#define ctl_isqrt(A) ctl_isqrt_inline(A)

// For the following functions, CMSIS-DSP does not provide a direct equivalent.
// We fall back to the standard C math library.
#define ctl_tan(A)                                                                                                     \
    (tanf(2.0f * PI * (A))) /**< @brief Computes tangent from a per-unit angle. Uses standard math lib. */
#define ctl_atan2(Y, X) (atan2f((Y), (X))) /**< @brief Computes the arc-tangent. Uses standard math lib. */
#define ctl_exp(A)      (expf((A)))        /**< @brief Computes the base-e exponential. Uses standard math lib. */
#define ctl_ln(A)       (logf((A)))        /**< @brief Computes the natural logarithm. Uses standard math lib. */

/** @} */ // end of MC_NONLINEAR_CMSIS group

/**
 * @brief A macro to indicate that `ctrl_gt` is defined as a CMSIS-DSP compatible float type.
 * This can be used for conditional compilation in other parts of the library.
 */
#define CTRL_GT_IS_CMSIS_FLOAT

/** @} */ // end of MC_CMSIS_MACROS group

#endif // _FILE_ARM_CMSIS_MACROS_H_
