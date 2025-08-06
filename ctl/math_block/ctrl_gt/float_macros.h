/**
 * @file float_macros.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a set of macros for floating-point arithmetic when ctrl_gt is float.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides the concrete implementation of the control library's math
 * functions for the single-precision floating-point (`float`) data type. It
 * defines the `ctrl_gt` type and a suite of macros for basic and advanced
 * mathematical operations.
 */

#ifndef _FILE_FLOAT_MACROS_H_
#define _FILE_FLOAT_MACROS_H_

#include <math.h>

/**
 * @defgroup MC_FLOAT_MACROS Floating-Point Math Macros
 * @ingroup MC_CTRL_GT_IMPL
 * @brief A collection of macros and inline functions for floating-point arithmetic.
 *
 * This module defines the low-level math operations when the library is
 * configured to use `float` as its primary control data type (`ctrl_gt`).
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Helper Inline Functions                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Clamps a floating-point value within a specified range.
 * @param[in] A The input value.
 * @param[in] min_val The minimum allowed value.
 * @param[in] max_val The maximum allowed value.
 * @return The saturated value.
 */
GMP_STATIC_INLINE float saturation_static_inline(float A, float min_val, float max_val)
{
    if (A < min_val)
        return min_val;
    else if (A > max_val)
        return max_val;
    else
        return A;
}

/**
 * @brief Computes the absolute value of a float.
 * @param[in] A The input value.
 * @return The absolute value of A.
 */
GMP_STATIC_INLINE float abs_static_inline(float A)
{
    if (A < 0)
        return (-A);
    else
        return A;
}

/*---------------------------------------------------------------------------*/
/* Type Conversion Macros                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_TYPE_CONVERSION_FLOAT Type Conversion Macros (Float)
 * @ingroup MC_FLOAT_MACROS
 * @brief Macros for converting between `float` and other data types.
 * @{
 */

#define float2ctrl(x) ((float)(x)) /**< @brief Converts a float literal to `ctrl_gt`. */
#define ctrl2float(x) ((float)(x)) /**< @brief Converts a `ctrl_gt` value to a standard float. */
#define int2ctrl(x)   ((float)(x)) /**< @brief Converts an integer to `ctrl_gt`. */
#define ctrl2int(x)   ((int)(x))   /**< @brief Converts a `ctrl_gt` value to an integer (truncates). */

/**
 * @brief Computes the fractional part of a `ctrl_gt` value (x mod 1).
 * @param x The input value.
 * @return The fractional part of x.
 */
#define ctrl_mod_1(x) ((float)(((float)(x)) - ((int32_t)(x))))

/** @} */ // end of MC_TYPE_CONVERSION_FLOAT group

/*---------------------------------------------------------------------------*/
/* Arithmetic Macros                                                         */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ARITHMETIC_FLOAT Arithmetic Macros (Float)
 * @ingroup MC_FLOAT_MACROS
 * @brief Macros for basic arithmetic operations.
 * @{
 */

#define ctl_mul(A, B) ((float)((float)(A) * (B))) /**< @brief Multiplies two `ctrl_gt` values. */
#define ctl_div(A, B) ((float)((float)(A) / (B))) /**< @brief Divides two `ctrl_gt` values. */
#define ctl_add(A, B) ((float)((float)(A) + (B))) /**< @brief Adds two `ctrl_gt` values. */
#define ctl_sub(A, B) ((float)((float)(A) - (B))) /**< @brief Subtracts two `ctrl_gt` values. */

#define ctl_div2(A) ((float)((float)(A) / 2.0f)) /**< @brief Divides a `ctrl_gt` value by 2. */
#define ctl_div4(A) ((float)((float)(A) / 4.0f)) /**< @brief Divides a `ctrl_gt` value by 4. */
#define ctl_mul2(A) ((float)((float)(A) * 2.0f)) /**< @brief Multiplies a `ctrl_gt` value by 2. */
#define ctl_mul4(A) ((float)((float)(A) * 4.0f)) /**< @brief Multiplies a `ctrl_gt` value by 4. */

#define ctl_abs(A) abs_static_inline(A) /**< @brief Computes the absolute value of a `ctrl_gt` value. */
#define ctl_sat(A, Pos, Neg)                                                                                           \
    saturation_static_inline((A), (Neg),                                                                               \
                             (Pos)) /**< @brief Saturates a `ctrl_gt` value between a positive and negative limit. */

/** @} */ // end of MC_ARITHMETIC_FLOAT group

/*---------------------------------------------------------------------------*/
/* Non-linear Function Macros                                                */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_NONLINEAR_FLOAT Non-linear Function Macros (Float)
 * @ingroup MC_FLOAT_MACROS
 * @brief Macros for non-linear mathematical functions like trigonometry and roots.
 * @{
 */

/**
 * @brief Computes the sine of an angle given in per-unit.
 * @note Assumes a `PI` constant is defined. The input is scaled by 2*PI.
 * @param A The input angle in per-unit (0.0 to 1.0 represents 0 to 2¦Ð).
 * @return The sine of the angle.
 */
#define ctl_sin(A) sinf(2.0f * PI * (A))

/**
 * @brief Computes the cosine of an angle given in per-unit.
 * @note Assumes a `PI` constant is defined. The input is scaled by 2*PI.
 * @param A The input angle in per-unit (0.0 to 1.0 represents 0 to 2¦Ð).
 * @return The cosine of the angle.
 */
#define ctl_cos(A) cosf(2.0f * PI * (A))

/**
 * @brief Computes the tangent of an angle given in per-unit.
 * @note Assumes a `PI` constant is defined. The input is scaled by 2*PI.
 * @param A The input angle in per-unit (0.0 to 1.0 represents 0 to 2¦Ð).
 * @return The tangent of the angle.
 */
#define ctl_tan(A) tanf(2.0f * PI * (A))

#define ctl_atan2(Y, X) atan2f((Y), (X))    /**< @brief Computes the arc-tangent of Y/X. */
#define ctl_exp(A)      expf((A))           /**< @brief Computes the base-e exponential of A. */
#define ctl_ln(A)       logf((A))           /**< @brief Computes the natural logarithm of A. */
#define ctl_sqrt(A)     sqrtf((A))          /**< @brief Computes the square root of A. */
#define ctl_isqrt(A)    (1.0f / sqrtf((A))) /**< @brief Computes the inverse square root of A. */

/** @} */ // end of MC_NONLINEAR_FLOAT group

/**
 * @brief A macro to indicate that `ctrl_gt` is defined as a floating-point type.
 * This can be used for conditional compilation in other parts of the library.
 */
#define CTRL_GT_IS_FLOAT

/** @} */ // end of MC_FLOAT_MACROS group

#endif // _FILE_FLOAT_MACROS_H_
