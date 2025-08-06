/**
 * @file vector2.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a 2D vector type and related mathematical operations.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a standard implementation for 2D vector arithmetic,
 * which is a fundamental building block for many control and physics calculations.
 */

#ifndef _FILE_CTL_VECTOR2_H_
#define _FILE_CTL_VECTOR2_H_

#include <math.h> // Required for sqrt in ctl_vector2_mag

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* 2D Vector Math                                                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_VECTOR2 2D Vector Math
 * @ingroup MC_DEFINES
 * @brief A collection of types and functions for 2D vector arithmetic.
 * @{
 */

/**
 * @brief Data structure for representing a 2D vector.
 */
typedef struct _tag_ctl_vector2_t
{
    ctrl_gt dat[2]; /**< @brief Array storing the x (dat[0]) and y (dat[1]) components of the vector. */
} ctl_vector2_t, vector2_gt;

/**
 * @brief Clears a 2D vector, setting its components to zero.
 * @param[out] vec Pointer to the vector to be cleared.
 */
GMP_STATIC_INLINE void ctl_vector2_clear(ctl_vector2_t* vec)
{
    vec->dat[0] = 0;
    vec->dat[1] = 0;
}

/**
 * @brief Copies the contents of one 2D vector to another.
 * @param[out] dup Pointer to the destination vector.
 * @param[in]  vec Pointer to the source vector.
 */
GMP_STATIC_INLINE void ctl_vector2_copy(ctl_vector2_t* dup, ctl_vector2_t* vec)
{
    dup->dat[0] = vec->dat[0];
    dup->dat[1] = vec->dat[1];
}

/**
 * @brief Adds two 2D vectors.
 * @param a The first vector.
 * @param b The second vector.
 * @return The resulting vector (a + b).
 */
GMP_STATIC_INLINE ctl_vector2_t ctl_vector2_add(ctl_vector2_t a, ctl_vector2_t b)
{
    ctl_vector2_t result;
    result.dat[0] = a.dat[0] + b.dat[0];
    result.dat[1] = a.dat[1] + b.dat[1];
    return result;
}

/**
 * @brief Subtracts one 2D vector from another.
 * @param a The minuend vector.
 * @param b The subtrahend vector.
 * @return The resulting vector (a - b).
 */
GMP_STATIC_INLINE ctl_vector2_t ctl_vector2_sub(ctl_vector2_t a, ctl_vector2_t b)
{
    ctl_vector2_t result;
    result.dat[0] = a.dat[0] - b.dat[0];
    result.dat[1] = a.dat[1] - b.dat[1];
    return result;
}

/**
 * @brief Multiplies a 2D vector by a scalar value.
 * @param vec The vector to be scaled.
 * @param scalar The scalar value.
 * @return The resulting scaled vector.
 */
GMP_STATIC_INLINE ctl_vector2_t ctl_vector2_scale(ctl_vector2_t vec, ctrl_gt scalar)
{
    ctl_vector2_t result;
    result.dat[0] = vec.dat[0] * scalar;
    result.dat[1] = vec.dat[1] * scalar;
    return result;
}

/**
 * @brief Calculates the dot product of two 2D vectors.
 * @f[
 * a \cdot b = a_x b_x + a_y b_y
 * @f]
 * @param a The first vector.
 * @param b The second vector.
 * @return The dot product.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector2_dot(ctl_vector2_t a, ctl_vector2_t b)
{
    return a.dat[0] * b.dat[0] + a.dat[1] * b.dat[1];
}

/**
 * @brief Calculates the squared magnitude (length) of a 2D vector.
 * This is computationally cheaper than `ctl_vector2_mag` as it avoids a square root.
 * @param vec The input vector.
 * @return The squared magnitude of the vector.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector2_mag_sq(ctl_vector2_t vec)
{
    return vec.dat[0] * vec.dat[0] + vec.dat[1] * vec.dat[1];
}

/**
 * @brief Calculates the magnitude (length) of a 2D vector.
 * @param vec The input vector.
 * @return The magnitude of the vector.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector2_mag(ctl_vector2_t vec)
{
    return sqrt(ctl_vector2_mag_sq(vec));
}

/**
 * @brief Normalizes a 2D vector to produce a unit vector (a vector with length 1).
 * @param vec The vector to be normalized.
 * @return The normalized (unit) vector. Returns a zero vector if the magnitude is zero.
 */
GMP_STATIC_INLINE ctl_vector2_t ctl_vector2_normalize(ctl_vector2_t vec)
{
    ctl_vector2_t result = {{0, 0}};
    ctrl_gt mag = ctl_vector2_mag(vec);
    if (mag > 1e-9) // Use a small epsilon to avoid division by zero
    {
        result.dat[0] = vec.dat[0] / mag;
        result.dat[1] = vec.dat[1] / mag;
    }
    return result;
}

/** @} */ // end of MC_VECTOR2 group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_VECTOR2_H_
