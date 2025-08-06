/**
 * @file vector4.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a 4D vector type and related mathematical operations.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a standard implementation for 4D vector arithmetic.
 * While less common than 2D or 3D vectors in motor control, they are useful
 * in advanced applications, such as representing quaternions or in homogeneous
 * coordinates for 3D graphics.
 */

#ifndef _FILE_GMP_CTL_VECTOR4_H_
#define _FILE_GMP_CTL_VECTOR4_H_

#include <math.h> // Required for sqrt in ctl_vector4_mag

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* 4D Vector Math                                                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_VECTOR4 4D Vector Math
 * @ingroup MC_DEFINES
 * @brief A collection of types and functions for 4D vector arithmetic.
 * @{
 */

/**
 * @brief Data structure for representing a 4D vector.
 */
typedef struct _tag_ctl_vector4_t
{
    ctrl_gt dat[4]; /**< @brief Array storing the x, y, z, and w components of the vector. */
} ctl_vector4_t, vector4_gt;

/**
 * @brief Clears a 4D vector, setting its components to zero.
 * @param[out] vec Pointer to the vector to be cleared.
 */
GMP_STATIC_INLINE void ctl_vector4_clear(ctl_vector4_t* vec)
{
    vec->dat[0] = 0;
    vec->dat[1] = 0;
    vec->dat[2] = 0;
    vec->dat[3] = 0;
}

/**
 * @brief Sets all components of a 4D vector to a single scalar value.
 * @param[out] vec Pointer to the vector to be set.
 * @param[in]  v   The scalar value to set all components to.
 */
GMP_STATIC_INLINE void ctl_vector4_set(ctl_vector4_t* vec, ctrl_gt v)
{
    vec->dat[0] = v;
    vec->dat[1] = v;
    vec->dat[2] = v;
    vec->dat[3] = v;
}

/**
 * @brief Copies the contents of one 4D vector to another.
 * @param[out] dup Pointer to the destination vector.
 * @param[in]  vec Pointer to the source vector.
 */
GMP_STATIC_INLINE void ctl_vector4_copy(ctl_vector4_t* dup, ctl_vector4_t* vec)
{
    dup->dat[0] = vec->dat[0];
    dup->dat[1] = vec->dat[1];
    dup->dat[2] = vec->dat[2];
    dup->dat[3] = vec->dat[3];
}

/**
 * @brief Adds two 4D vectors.
 * @param a The first vector.
 * @param b The second vector.
 * @return The resulting vector (a + b).
 */
GMP_STATIC_INLINE ctl_vector4_t ctl_vector4_add(ctl_vector4_t a, ctl_vector4_t b)
{
    ctl_vector4_t result;
    result.dat[0] = a.dat[0] + b.dat[0];
    result.dat[1] = a.dat[1] + b.dat[1];
    result.dat[2] = a.dat[2] + b.dat[2];
    result.dat[3] = a.dat[3] + b.dat[3];
    return result;
}

/**
 * @brief Subtracts one 4D vector from another.
 * @param a The minuend vector.
 * @param b The subtrahend vector.
 * @return The resulting vector (a - b).
 */
GMP_STATIC_INLINE ctl_vector4_t ctl_vector4_sub(ctl_vector4_t a, ctl_vector4_t b)
{
    ctl_vector4_t result;
    result.dat[0] = a.dat[0] - b.dat[0];
    result.dat[1] = a.dat[1] - b.dat[1];
    result.dat[2] = a.dat[2] - b.dat[2];
    result.dat[3] = a.dat[3] - b.dat[3];
    return result;
}

/**
 * @brief Multiplies a 4D vector by a scalar value.
 * @param vec The vector to be scaled.
 * @param scalar The scalar value.
 * @return The resulting scaled vector.
 */
GMP_STATIC_INLINE ctl_vector4_t ctl_vector4_scale(ctl_vector4_t vec, ctrl_gt scalar)
{
    ctl_vector4_t result;
    result.dat[0] = vec.dat[0] * scalar;
    result.dat[1] = vec.dat[1] * scalar;
    result.dat[2] = vec.dat[2] * scalar;
    result.dat[3] = vec.dat[3] * scalar;
    return result;
}

/**
 * @brief Calculates the dot product of two 4D vectors.
 * @f[
 * a \cdot b = a_x b_x + a_y b_y + a_z b_z + a_w b_w
 * @f]
 * @param a The first vector.
 * @param b The second vector.
 * @return The dot product.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector4_dot(ctl_vector4_t a, ctl_vector4_t b)
{
    return a.dat[0] * b.dat[0] + a.dat[1] * b.dat[1] + a.dat[2] * b.dat[2] + a.dat[3] * b.dat[3];
}

/**
 * @brief Calculates the squared magnitude (length) of a 4D vector.
 * This is computationally cheaper than `ctl_vector4_mag` as it avoids a square root.
 * @param vec The input vector.
 * @return The squared magnitude of the vector.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector4_mag_sq(ctl_vector4_t vec)
{
    return vec.dat[0] * vec.dat[0] + vec.dat[1] * vec.dat[1] + vec.dat[2] * vec.dat[2] + vec.dat[3] * vec.dat[3];
}

/**
 * @brief Calculates the magnitude (length) of a 4D vector.
 * @param vec The input vector.
 * @return The magnitude of the vector.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector4_mag(ctl_vector4_t vec)
{
    return sqrt(ctl_vector4_mag_sq(vec));
}

/**
 * @brief Normalizes a 4D vector to produce a unit vector (a vector with length 1).
 * @param vec The vector to be normalized.
 * @return The normalized (unit) vector. Returns a zero vector if the magnitude is zero.
 */
GMP_STATIC_INLINE ctl_vector4_t ctl_vector4_normalize(ctl_vector4_t vec)
{
    ctl_vector4_t result = {{0, 0, 0, 0}};
    ctrl_gt mag = ctl_vector4_mag(vec);
    if (mag > 1e-9) // Use a small epsilon to avoid division by zero
    {
        result.dat[0] = vec.dat[0] / mag;
        result.dat[1] = vec.dat[1] / mag;
        result.dat[2] = vec.dat[2] / mag;
        result.dat[3] = vec.dat[3] / mag;
    }
    return result;
}

/** @} */ // end of MC_VECTOR4 group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_CTL_VECTOR4_H_
