/**
 * @file iqmath_macros.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a set of macros for fixed-point arithmetic using the IQmath library.
 * @version 0.1
 * @date 2042-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides the concrete implementation of the control library's math
 * functions using the Texas Instruments IQmath fixed-point library. It maps
 * the generic control math macros to their corresponding IQmath functions.
 */

#ifndef _FILE_IQMATH_MACROS_H_
#define _FILE_IQMATH_MACROS_H_

#include <third_party/iqmath/IQmathLib.h>

/**
 * @defgroup MC_IQMATH_MACROS IQmath Fixed-Point Math Macros
 * @ingroup MC_CTRL_GT_IMPL
 * @brief A collection of macros for fixed-point arithmetic based on IQmath.
 *
 * This module defines the low-level math operations when the library is
 * configured to use IQmath as its primary math engine (`ctrl_gt` becomes `_iq`).
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Helper Inline Functions & Macros                                          */
/*---------------------------------------------------------------------------*/

#ifndef saturation_macro
#define saturation_macro(_a, _max, _min) ((_a) <= (_min)) ? (_min) : (((_a) >= (_max)) ? (_max) : (_a))
#endif

/**
 * @brief Computes the absolute value of an _iq number.
 * @param[in] A The input value.
 * @return The absolute value of A.
 */
GMP_STATIC_INLINE _iq abs_static_inline(_iq A)
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
 * @defgroup MC_TYPE_CONVERSION_IQ Type Conversion Macros (IQmath)
 * @ingroup MC_IQMATH_MACROS
 * @brief Macros for converting between `_iq` and other data types.
 * @{
 */

#define float2ctrl(x) (_IQ(x))     /**< @brief Converts a float literal to `_iq`. Maps to `_IQ()`. */
#define ctrl2float(x) (_IQtoF(x))  /**< @brief Converts an `_iq` value to a float. Maps to `_IQtoF()`. */
#define int2ctrl(x)   (_IQ(x))     /**< @brief Converts an integer to `_iq`. Maps to `_IQ()`. */
#define ctrl2int(x)   (_IQint(x))  /**< @brief Converts an `_iq` value to its integer part. Maps to `_IQint()`. */
#define ctrl_mod_1(x) (_IQfrac(x)) /**< @brief Computes the fractional part of an `_iq` value. Maps to `_IQfrac()`. */

/** @} */ // end of MC_TYPE_CONVERSION_IQ group

/*---------------------------------------------------------------------------*/
/* Arithmetic Macros                                                         */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ARITHMETIC_IQ Arithmetic Macros (IQmath)
 * @ingroup MC_IQMATH_MACROS
 * @brief Macros for basic arithmetic operations.
 * @{
 */

#define ctl_mul(A, B) (_IQmpy(A, B)) /**< @brief Multiplies two `_iq` values. Maps to `_IQmpy()`. */
#define ctl_div(A, B) (_IQdiv(A, B)) /**< @brief Divides two `_iq` values. Maps to `_IQdiv()`. */
#define ctl_add(A, B) ((A) + (B))    /**< @brief Adds two `_iq` values. */
#define ctl_sub(A, B) ((A) - (B))    /**< @brief Subtracts two `_iq` values. */

#define ctl_div2(A) (_IQdiv2(A)) /**< @brief Divides an `_iq` value by 2. Maps to `_IQdiv2()`. */
#define ctl_div4(A) (_IQdiv4(A)) /**< @brief Divides an `_iq` value by 4. Maps to `_IQdiv4()`. */
#define ctl_mul2(A) (_IQmpy2(A)) /**< @brief Multiplies an `_iq` value by 2. Maps to `_IQmpy2()`. */
#define ctl_mul4(A) (_IQmpy4(A)) /**< @brief Multiplies an `_iq` value by 4. Maps to `_IQmpy4()`. */

#define ctl_abs(A) abs_static_inline(A) /**< @brief Computes the absolute value of an `_iq` value. */
#define ctl_sat(A, Pos, Neg)                                                                                           \
    saturation_macro((A), (Pos), (Neg)) /**< @brief Saturates an `_iq` value between a positive and negative limit. */

#define pwm_mul(A, B)        (_IQmpy(A, B))
#define pwm_sat(A, Pos, Neg) saturation_macro((A), (Pos), (Neg))

/** @} */ // end of MC_ARITHMETIC_IQ group

/*---------------------------------------------------------------------------*/
/* Non-linear Function Macros                                                */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_NONLINEAR_IQ Non-linear Function Macros (IQmath)
 * @ingroup MC_IQMATH_MACROS
 * @brief Macros for non-linear mathematical functions.
 * @{
 */

#define ctl_sin(A)        (_IQsinPU(A)) /**< @brief Computes sine from a per-unit angle. Maps to `_IQsinPU()`. */
#define ctl_cos(A)        (_IQcosPU(A)) /**< @brief Computes cosine from a per-unit angle. Maps to `_IQcosPU()`. */
#define ctl_tan(A)        (_IQdiv(_IQsinPU(A), _IQcosPU(A))) /**< @brief Computes tangent from a per-unit angle. */
#define ctl_atan2(Y, X)   (_IQatan2PU((Y), (X))) /**< @brief Computes the per-unit arc-tangent. Maps to `_IQatan2PU()`. */
#define ctl_exp(A)        (_IQexp(A))            /**< @brief Computes the base-e exponential. Maps to `_IQexp()`. */
#define ctl_ln(A)         (_IQlog(A))            /**< @brief Computes the natural logarithm. Maps to `_IQlog()`. */
#define ctl_pow(B, Index) (_IQexp(ctl_mul(_IQlog(Index), B))) /**< @brief Compute the B^Index power of B */
#define ctl_sqrt(A)       (_IQsqrt(A))  /**< @brief Computes the square root. Maps to `_IQsqrt()`. */
#define ctl_isqrt(A)      (_IQisqrt(A)) /**< @brief Computes the inverse square root. Maps to `_IQisqrt()`. */

/** @} */ // end of MC_NONLINEAR_IQ group

/**
 * @brief A macro to indicate that `ctrl_gt` is defined as a fixed-point type.
 * This can be used for conditional compilation in other parts of the library.
 */
#define CTRL_GT_IS_FIXED

/** @} */ // end of MC_IQMATH_MACROS group

#endif // _FILE_IQMATH_MACROS_H_
