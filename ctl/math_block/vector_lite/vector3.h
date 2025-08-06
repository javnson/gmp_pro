/**
 * @file vector3.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a 3D vector type and related mathematical operations.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a standard implementation for 3D vector arithmetic,
 * which is fundamental for many control, physics, and graphics calculations.
 */

#ifndef _FILE_CTL_VECTOR3_H_
#define _FILE_CTL_VECTOR3_H_

#include <math.h> // Required for sqrt in ctl_vector3_mag

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* 3D Vector Math                                                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_VECTOR3 3D Vector Math
 * @ingroup MC_LINEAR_ALGEBRA
 * @brief A collection of types and functions for 3D vector arithmetic.
 * @{
 */

/**
 * @brief Data structure for representing a 3D vector.
 */
typedef struct _tag_ctl_vector3_t
{
    ctrl_gt dat[3]; /**< @brief Array storing the x (dat[0]), y (dat[1]), and z (dat[2]) components. */
} ctl_vector3_t, vector3_gt;

/**
 * @brief Clears a 3D vector, setting its components to zero.
 * @param[out] vec Pointer to the vector to be cleared.
 */
GMP_STATIC_INLINE void ctl_vector3_clear(ctl_vector3_t* vec)
{
    vec->dat[0] = 0;
    vec->dat[1] = 0;
    vec->dat[2] = 0;
}

/**
 * @brief Sets all components of a 3D vector to a single scalar value.
 * @param[out] vec Pointer to the vector to be set.
 * @param[in]  v   The scalar value to set all components to.
 */
GMP_STATIC_INLINE void ctl_vector3_set(ctl_vector3_t* vec, ctrl_gt v)
{
    vec->dat[0] = v;
    vec->dat[1] = v;
    vec->dat[2] = v;
}

/**
 * @brief Copies the contents of one 3D vector to another.
 * @param[out] dup Pointer to the destination vector.
 * @param[in]  vec Pointer to the source vector.
 */
GMP_STATIC_INLINE void ctl_vector3_copy(ctl_vector3_t* dup, ctl_vector3_t* vec)
{
    dup->dat[0] = vec->dat[0];
    dup->dat[1] = vec->dat[1];
    dup->dat[2] = vec->dat[2];
}

/**
 * @brief Adds two 3D vectors.
 * @param a The first vector.
 * @param b The second vector.
 * @return The resulting vector (a + b).
 */
GMP_STATIC_INLINE ctl_vector3_t ctl_vector3_add(ctl_vector3_t a, ctl_vector3_t b)
{
    ctl_vector3_t result;
    result.dat[0] = a.dat[0] + b.dat[0];
    result.dat[1] = a.dat[1] + b.dat[1];
    result.dat[2] = a.dat[2] + b.dat[2];
    return result;
}

/**
 * @brief Subtracts one 3D vector from another.
 * @param a The minuend vector.
 * @param b The subtrahend vector.
 * @return The resulting vector (a - b).
 */
GMP_STATIC_INLINE ctl_vector3_t ctl_vector3_sub(ctl_vector3_t a, ctl_vector3_t b)
{
    ctl_vector3_t result;
    result.dat[0] = a.dat[0] - b.dat[0];
    result.dat[1] = a.dat[1] - b.dat[1];
    result.dat[2] = a.dat[2] - b.dat[2];
    return result;
}

/**
 * @brief Multiplies a 3D vector by a scalar value.
 * @param vec The vector to be scaled.
 * @param scalar The scalar value.
 * @return The resulting scaled vector.
 */
GMP_STATIC_INLINE ctl_vector3_t ctl_vector3_scale(ctl_vector3_t vec, ctrl_gt scalar)
{
    ctl_vector3_t result;
    result.dat[0] = vec.dat[0] * scalar;
    result.dat[1] = vec.dat[1] * scalar;
    result.dat[2] = vec.dat[2] * scalar;
    return result;
}

/**
 * @brief Calculates the dot product of two 3D vectors.
 * @f[
 * a \cdot b = a_x b_x + a_y b_y + a_z b_z
 * @f]
 * @param a The first vector.
 * @param b The second vector.
 * @return The dot product.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector3_dot(ctl_vector3_t a, ctl_vector3_t b)
{
    return a.dat[0] * b.dat[0] + a.dat[1] * b.dat[1] + a.dat[2] * b.dat[2];
}

/**
 * @brief Calculates the cross product of two 3D vectors.
 * @f[
 * a \times b = \begin{pmatrix} a_y b_z - a_z b_y \\ a_z b_x - a_x b_z \\ a_x b_y - a_y b_x \end{pmatrix}
 * @f]
 * @param a The first vector.
 * @param b The second vector.
 * @return The resulting vector from the cross product.
 */
GMP_STATIC_INLINE ctl_vector3_t ctl_vector3_cross(ctl_vector3_t a, ctl_vector3_t b)
{
    ctl_vector3_t result;
    result.dat[0] = a.dat[1] * b.dat[2] - a.dat[2] * b.dat[1];
    result.dat[1] = a.dat[2] * b.dat[0] - a.dat[0] * b.dat[2];
    result.dat[2] = a.dat[0] * b.dat[1] - a.dat[1] * b.dat[0];
    return result;
}

/**
 * @brief Calculates the squared magnitude (length) of a 3D vector.
 * This is computationally cheaper than `ctl_vector3_mag` as it avoids a square root.
 * @param vec The input vector.
 * @return The squared magnitude of the vector.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector3_mag_sq(ctl_vector3_t vec)
{
    return vec.dat[0] * vec.dat[0] + vec.dat[1] * vec.dat[1] + vec.dat[2] * vec.dat[2];
}

/**
 * @brief Calculates the magnitude (length) of a 3D vector.
 * @param vec The input vector.
 * @return The magnitude of the vector.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector3_mag(ctl_vector3_t vec)
{
    return sqrt(ctl_vector3_mag_sq(vec));
}

/**
 * @brief Normalizes a 3D vector to produce a unit vector (a vector with length 1).
 * @param vec The vector to be normalized.
 * @return The normalized (unit) vector. Returns a zero vector if the magnitude is zero.
 */
GMP_STATIC_INLINE ctl_vector3_t ctl_vector3_normalize(ctl_vector3_t vec)
{
    ctl_vector3_t result = {{0, 0, 0}};
    ctrl_gt mag = ctl_vector3_mag(vec);
    if (mag > 1e-9) // Use a small epsilon to avoid division by zero
    {
        result.dat[0] = vec.dat[0] / mag;
        result.dat[1] = vec.dat[1] / mag;
        result.dat[2] = vec.dat[2] / mag;
    }
    return result;
}

/** @} */ // end of MC_VECTOR3 group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_VECTOR3_H_
