/**
 * @file complex.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a complex number type and related mathematical operations.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a standard implementation for complex number arithmetic,
 * which is fundamental for various signal processing and control algorithms,
 * particularly in transformations like the Park and Clarke transforms.
 */

#ifndef _FILE_GMP_CTL_MATH_COMPLEX_H_
#define _FILE_GMP_CTL_MATH_COMPLEX_H_

#include <math.h> // Required for sqrt in ctl_cabs

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Complex Number Math                                                       */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_COMPLEX Complex Number Math
 * @ingroup MC_EXT_TYPES
 * @brief A collection of types and functions for complex number arithmetic.
 * @{
 */

/**
 * @brief Data structure for representing a complex number.
 */
typedef struct _tag_complex_t
{
    ctrl_gt real; /**< @brief The real part of the complex number. */
    ctrl_gt imag; /**< @brief The imaginary part of the complex number. */
} ctl_complex_t;

/**
 * @brief Adds two complex numbers.
 * @param a The first complex number.
 * @param b The second complex number.
 * @return The result of the addition (a + b).
 */
GMP_STATIC_INLINE ctl_complex_t ctl_cadd(ctl_complex_t a, ctl_complex_t b)
{
    ctl_complex_t result;
    result.real = a.real + b.real;
    result.imag = a.imag + b.imag;
    return result;
}

/**
 * @brief Subtracts one complex number from another.
 * @param a The minuend.
 * @param b The subtrahend.
 * @return The result of the subtraction (a - b).
 */
GMP_STATIC_INLINE ctl_complex_t ctl_csub(ctl_complex_t a, ctl_complex_t b)
{
    ctl_complex_t result;
    result.real = a.real - b.real;
    result.imag = a.imag - b.imag;
    return result;
}

/**
 * @brief Multiplies two complex numbers.
 *
 * The multiplication is performed as:
 * @f[
 * (a_r + j a_i)(b_r + j b_i) = (a_r b_r - a_i b_i) + j(a_r b_i + a_i b_r)
 * @f]
 *
 * @param a The first complex number.
 * @param b The second complex number.
 * @return The result of the multiplication (a * b).
 */
GMP_STATIC_INLINE ctl_complex_t ctl_cmul(ctl_complex_t a, ctl_complex_t b)
{
    ctl_complex_t result;
    result.real = a.real * b.real - a.imag * b.imag;
    result.imag = a.real * b.imag + a.imag * b.real;
    return result;
}

/**
 * @brief Divides one complex number by another.
 *
 * The division is performed as:
 * @f[
 * \frac{a}{b} = \frac{a \cdot b^*}{|b|^2}
 * @f]
 *
 * @param a The numerator.
 * @param b The denominator.
 * @return The result of the division (a / b). Returns {0, 0} if the denominator is zero.
 */
GMP_STATIC_INLINE ctl_complex_t ctl_cdiv(ctl_complex_t a, ctl_complex_t b)
{
    ctl_complex_t result;
    ctrl_gt den = b.real * b.real + b.imag * b.imag;

    if (den == 0)
    {
        result.real = 0;
        result.imag = 0;
        return result;
    }

    result.real = (a.real * b.real + a.imag * b.imag) / den;
    result.imag = (a.imag * b.real - a.real * b.imag) / den;
    return result;
}

/**
 * @brief Computes the complex conjugate of a number.
 * @param a The input complex number.
 * @return The complex conjugate of a.
 */
GMP_STATIC_INLINE ctl_complex_t ctl_cconj(ctl_complex_t a)
{
    ctl_complex_t result;
    result.real = a.real;
    result.imag = -a.imag;
    return result;
}

/**
 * @brief Computes the squared magnitude (modulus) of a complex number.
 * This is computationally cheaper than `ctl_cabs` as it avoids a square root.
 * @param a The input complex number.
 * @return The squared magnitude (|a|^2).
 */
GMP_STATIC_INLINE ctrl_gt ctl_cabs_sq(ctl_complex_t a)
{
    return a.real * a.real + a.imag * a.imag;
}

/**
 * @brief Computes the magnitude (modulus or absolute value) of a complex number.
 * @param a The input complex number.
 * @return The magnitude (|a|).
 */
GMP_STATIC_INLINE ctrl_gt ctl_cabs(ctl_complex_t a)
{
    // Assuming a standard sqrt function is available.
    // Replace with a library-specific fast sqrt if available.
    return sqrt(ctl_cabs_sq(a));
}

/** @} */ // end of MC_COMPLEX group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_CTL_MATH_COMPLEX_H_
