/**
 * @file cla_macros.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a set of macros for floating-point arithmetic using the TI CLA Math library.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides the concrete implementation of the control library's math
 * functions using the Texas Instruments C2000 Control Law Accelerator (CLA)
 * math library. It maps the generic control math macros to their corresponding
 * hardware-accelerated CLA functions and intrinsics.
 */

#ifndef _FILE_CLA_MACROS_H_
#define _FILE_CLA_MACROS_H_

#include <libraries/math/CLAmath/c28/include/CLAmath.h>

/**
 * @defgroup MC_CLA_MACROS CLA Floating-Point Math Macros
 * @ingroup MC_DEFINES
 * @brief A collection of macros for hardware-accelerated floating-point arithmetic.
 *
 * This module defines the low-level math operations when the library is
 * configured to use the CLA math library as its primary math engine.
 * `ctrl_gt` becomes `float32_t`.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Helper Inline Functions & Macros                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief Clamps a float32_t value within a specified range using CLA intrinsics.
 * @param[in] in The input value.
 * @param[in] min_val The minimum allowed value.
 * @param[in] max_val The maximum allowed value.
 * @return The saturated value.
 */
GMP_STATIC_INLINE float32_t saturation_static_inline(float32_t in, float32_t min_val, float32_t max_val)
{
    float32_t out = in;
    out = __fmax(out, min_val);
    out = __fmin(out, max_val);
    return (out);
}

/*---------------------------------------------------------------------------*/
/* Type Conversion Macros                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_TYPE_CONVERSION_CLA Type Conversion Macros (CLA)
 * @ingroup MC_CLA_MACROS
 * @brief Macros for converting between `float32_t` and other data types.
 * @{
 */

#define float2ctrl(x) ((float32_t)(x)) /**< @brief Converts a literal to `ctrl_gt` (float32_t). */
#define ctrl2float(x) ((float32_t)(x)) /**< @brief Converts a `ctrl_gt` value to a standard float32_t. */
#define int2ctrl(x)   ((float32_t)(x)) /**< @brief Converts an integer to `ctrl_gt`. */
#define ctrl2int(x)   ((int)(x))       /**< @brief Converts a `ctrl_gt` value to an integer (truncates). */

/**
 * @brief Computes the fractional part of a `ctrl_gt` value (x mod 1).
 * @param x The input value.
 * @return The fractional part of x.
 */
#define ctrl_mod_1(x) ((float32_t)(((float32_t)(x)) - ((int32_t)(x))))

/** @} */ // end of MC_TYPE_CONVERSION_CLA group

/*---------------------------------------------------------------------------*/
/* Arithmetic Macros                                                         */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ARITHMETIC_CLA Arithmetic Macros (CLA)
 * @ingroup MC_CLA_MACROS
 * @brief Macros for basic arithmetic operations using CLA intrinsics where available.
 * @{
 */

#define ctl_mul(A, B) ((float32_t)((float32_t)(A) * (B))) /**< @brief Multiplies two `ctrl_gt` values. */
#define ctl_div(A, B)                                                                                                  \
    ((float32_t)(__divf32((A), (B)))) /**< @brief Divides two `ctrl_gt` values using the CLA division intrinsic. */
#define ctl_add(A, B) ((float32_t)((float32_t)(A) + (B))) /**< @brief Adds two `ctrl_gt` values. */
#define ctl_sub(A, B) ((float32_t)((float32_t)(A) - (B))) /**< @brief Subtracts two `ctrl_gt` values. */

#define ctl_div2(A) ((float32_t)(__divf32((float32_t)(A), 2.0f))) /**< @brief Divides a `ctrl_gt` value by 2. */
#define ctl_div4(A) ((float32_t)(__divf32((float32_t)(A), 4.0f))) /**< @brief Divides a `ctrl_gt` value by 4. */
#define ctl_mul2(A) ((float32_t)((float32_t)(A) * 2.0f))          /**< @brief Multiplies a `ctrl_gt` value by 2. */
#define ctl_mul4(A) ((float32_t)((float32_t)(A) * 4.0f))          /**< @brief Multiplies a `ctrl_gt` value by 4. */

#define ctl_abs(A) (fabsf(A)) /**< @brief Computes the absolute value of a `ctrl_gt` value. */
#define ctl_sat(A, Pos, Neg)                                                                                           \
    saturation_static_inline((A), (Neg),                                                                               \
                             (Pos)) /**< @brief Saturates a `ctrl_gt` value between a positive and negative limit. */

/** @} */ // end of MC_ARITHMETIC_CLA group

/*---------------------------------------------------------------------------*/
/* Non-linear Function Macros                                                */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_NONLINEAR_CLA Non-linear Function Macros (CLA)
 * @ingroup MC_CLA_MACROS
 * @brief Macros for non-linear mathematical functions using CLA intrinsics.
 * @{
 */

#define ctl_sin(A)                                                                                                     \
    (__sinpuf32((A))) /**< @brief Computes sine from a per-unit angle. Maps to CLA intrinsic `__sinpuf32()`. */
#define ctl_cos(A)                                                                                                     \
    (__cospuf32((A))) /**< @brief Computes cosine from a per-unit angle. Maps to CLA intrinsic `__cospuf32()`. */
#define ctl_tan(A)                                                                                                     \
    (tanf(2.0f * PI * (A))) /**< @brief Computes tangent from a per-unit angle. Uses standard math lib. */
#define ctl_atan2(Y, X) (__atan2((Y), (X))) /**< @brief Computes the arc-tangent. Maps to CLA intrinsic `__atan2()`. */
#define ctl_exp(A)      (expf((A)))         /**< @brief Computes the base-e exponential. Uses standard math lib. */
#define ctl_ln(A)       (logf((A)))         /**< @brief Computes the natural logarithm. Uses standard math lib. */
#define ctl_sqrt(A)     (CLAsqrt((A)))      /**< @brief Computes the square root. Maps to `CLAsqrt()`. */
#define ctl_isqrt(A)                                                                                                   \
    (__eisqrtf32((A))) /**< @brief Computes the inverse square root. Maps to CLA intrinsic `__eisqrtf32()`. */

/** @} */ // end of MC_NONLINEAR_CLA group

/**
 * @brief A macro to indicate that `ctrl_gt` is defined as a CLA-compatible float type.
 * This can be used for conditional compilation in other parts of the library.
 */
#define CTRL_GT_IS_FLOAT

/** @} */ // end of MC_CLA_MACROS group

#endif // _FILE_CLA_MACROS_H_
