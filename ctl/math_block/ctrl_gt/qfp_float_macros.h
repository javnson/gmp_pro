/**
 * @file qfp_float_macros.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a set of macros for floating-point arithmetic using a QFP library.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides the concrete implementation of the control library's math
 * functions using a QFP (Quick Floating Point) or similar library. It maps
 * the generic control math macros to their corresponding QFP functions.
 */

#ifndef _FILE_QPF_FLOAT_MACROS_H_
#define _FILE_QPF_FLOAT_MACROS_H_

// #include <qfp_math.h> // User should include the actual QFP library header here.

/**
 * @defgroup MC_QFP_MACROS QFP Floating-Point Math Macros
 * @ingroup MC_CTRL_GT_IMPL
 * @brief A collection of macros for floating-point arithmetic based on a QFP library.
 *
 * This module defines the low-level math operations when the library is
 * configured to use a QFP library as its primary math engine. `ctrl_gt` is `float`.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Helper Inline Functions & Macros                                          */
/*---------------------------------------------------------------------------*/

#ifndef saturation_macro
#define saturation_macro(_a, _max, _min) ((_a) <= (_min)) ? (_min) : (((_a) >= (_max)) ? (_max) : (_a))
#endif

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
 * @defgroup MC_TYPE_CONVERSION_QFP Type Conversion Macros (QFP)
 * @ingroup MC_QFP_MACROS
 * @brief Macros for converting between `float` and other data types using QFP functions.
 * @{
 */

#define float2ctrl(x) ((float)(x))       /**< @brief Converts a literal to `ctrl_gt` (float). */
#define ctrl2float(x) ((float)(x))       /**< @brief Converts a `ctrl_gt` value to a standard float. */
#define int2ctrl(x)   (qfp_int2float(x)) /**< @brief Converts an integer to `ctrl_gt`. Maps to `qfp_int2float()`. */
#define ctrl2int(x)                                                                                                    \
    (qfp_float2int(x)) /**< @brief Converts a `ctrl_gt` value to an integer. Maps to `qfp_float2int()`. */

/**
 * @brief Computes the fractional part of a `ctrl_gt` value (x mod 1).
 * @param x The input value.
 * @return The fractional part of x.
 */
#define ctrl_mod_1(x) ((float)(((float)(x)) - ((int32_t)(x))))

/** @} */ // end of MC_TYPE_CONVERSION_QFP group

/*---------------------------------------------------------------------------*/
/* Arithmetic Macros                                                         */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ARITHMETIC_QFP Arithmetic Macros (QFP)
 * @ingroup MC_QFP_MACROS
 * @brief Macros for basic arithmetic operations using QFP functions.
 * @{
 */

#define ctl_mul(A, B) (qfp_fmul(A, B)) /**< @brief Multiplies two `ctrl_gt` values. Maps to `qfp_fmul()`. */
#define ctl_div(A, B) (qfp_fdiv(A, B)) /**< @brief Divides two `ctrl_gt` values. Maps to `qfp_fdiv()`. */
#define ctl_add(A, B) (qfp_fadd(A, B)) /**< @brief Adds two `ctrl_gt` values. Maps to `qfp_fadd()`. */
#define ctl_sub(A, B) (qfp_fsub(A, B)) /**< @brief Subtracts two `ctrl_gt` values. Maps to `qfp_fsub()`. */

#define ctl_div2(A) (qfp_fdiv(A, 2.0f)) /**< @brief Divides a `ctrl_gt` value by 2. */
#define ctl_div4(A) (qfp_fdiv(A, 4.0f)) /**< @brief Divides a `ctrl_gt` value by 4. */
#define ctl_mul2(A) (qfp_fmul(A, 2.0f)) /**< @brief Multiplies a `ctrl_gt` value by 2. */
#define ctl_mul4(A) (qfp_fmul(A, 4.0f)) /**< @brief Multiplies a `ctrl_gt` value by 4. */

#define ctl_abs(A) abs_static_inline(A) /**< @brief Computes the absolute value of a `ctrl_gt` value. */
#define ctl_sat(A, Pos, Neg)                                                                                           \
    saturation_macro((A), (Pos),                                                                                       \
                     (Neg)) /**< @brief Saturates a `ctrl_gt` value between a positive and negative limit. */

#define pwm_mul(A, B)        ((pwm_gt)((((float)(A)) * ((float)(B)))))
#define pwm_sat(A, Pos, Neg) ((pwm_gt)saturation_macro(((float)(A)), ((float)(Pos)), ((float)(Neg))))

/** @} */ // end of MC_ARITHMETIC_QFP group

/*---------------------------------------------------------------------------*/
/* Non-linear Function Macros                                                */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_NONLINEAR_QFP Non-linear Function Macros (QFP)
 * @ingroup MC_QFP_MACROS
 * @brief Macros for non-linear mathematical functions using QFP.
 * @{
 */

#define ctl_sin(A)        (qfp_fsin(A))      /**< @brief Computes sine. Maps to `qfp_fsin()`. */
#define ctl_cos(A)        (qfp_fcos(A))      /**< @brief Computes cosine. Maps to `qfp_fcos()`. */
#define ctl_tan(A)        (qfp_ftan(A))      /**< @brief Computes tangent. Maps to `qfp_ftan()`. */
#define ctl_atan2(Y, X)   (qfp_fatan2(Y, X)) /**< @brief Computes the arc-tangent. Maps to `qfp_fatan2()`. */
#define ctl_exp(A)        (qfp_fexp(A))      /**< @brief Computes the base-e exponential. Maps to `qfp_fexp()`. */
#define ctl_ln(A)         (qfp_fln(A))       /**< @brief Computes the natural logarithm. Maps to `qfp_fln()`. */
#define ctl_pow(B, Index) (qfp_fexp(ctl_mul(qfp_fln(Index), B))) /**< @brief Compute the B^Index power of B */
#define ctl_sqrt(A)       (qfp_fsqrt(A))                 /**< @brief Computes the square root. Maps to `qfp_fsqrt()`. */
#define ctl_isqrt(A)      (qfp_fdiv(1.0f, qfp_fsqrt(A))) /**< @brief Computes the inverse square root. */

/** 
 * @} 
 */ // end of MC_NONLINEAR_QFP group

/**
 * @brief A macro to indicate that `ctrl_gt` is defined as a QFP-compatible float type.
 * This can be used for conditional compilation in other parts of the library.
 */
#define CTRL_GT_IS_QFP_FLOAT

/**  
 * @} 
 */ // end of MC_QFP_MACROS group

#endif // _FILE_QPF_FLOAT_MACROS_H_
