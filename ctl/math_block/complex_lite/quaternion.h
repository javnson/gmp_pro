/**
 * @file quaternion.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a quaternion type and related mathematical operations.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a standard implementation for quaternion arithmetic,
 * which is essential for representing and performing 3D rotations without
 * issues like gimbal lock.
 */

#ifndef _FILE_GMP_CTL_MATH_QUATERNION_H_
#define _FILE_GMP_CTL_MATH_QUATERNION_H_

#include <math.h> // Required for sqrt in ctl_qnorm

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Quaternion Number Math                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_QUATERNION Quaternion Number Math
 * @ingroup MC_EXT_TYPES
 * @brief A collection of types and functions for quaternion arithmetic.
 * @{
 */

/**
 * @brief Data structure for representing a quaternion (w, x, y, z).
 */
typedef struct _tag_quaternion_t
{
    ctrl_gt w; /**< @brief The scalar (real) part of the quaternion. */
    ctrl_gt x; /**< @brief The i (vector) part of the quaternion. */
    ctrl_gt y; /**< @brief The j (vector) part of the quaternion. */
    ctrl_gt z; /**< @brief The k (vector) part of the quaternion. */
} ctl_quaternion_t;

/**
 * @brief Adds two quaternions.
 * @param a The first quaternion.
 * @param b The second quaternion.
 * @return The result of the addition (a + b).
 */
GMP_STATIC_INLINE ctl_quaternion_t ctl_qadd(ctl_quaternion_t a, ctl_quaternion_t b)
{
    ctl_quaternion_t result;
    result.w = a.w + b.w;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

/**
 * @brief Subtracts one quaternion from another.
 * @param a The minuend.
 * @param b The subtrahend.
 * @return The result of the subtraction (a - b).
 */
GMP_STATIC_INLINE ctl_quaternion_t ctl_qsub(ctl_quaternion_t a, ctl_quaternion_t b)
{
    ctl_quaternion_t result;
    result.w = a.w - b.w;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

/**
 * @brief Multiplies two quaternions (Hamilton product).
 *
 * The multiplication is performed as:
 * @f[
 * q_a q_b = (w_a w_b - x_a x_b - y_a y_b - z_a z_b) + \\
 * (w_a x_b + x_a w_b + y_a z_b - z_a y_b)i + \\
 * (w_a y_b - x_a z_b + y_a w_b + z_a x_b)j + \\
 * (w_a z_b + x_a y_b - y_a x_b + z_a w_b)k
 * @f]
 *
 * @param a The first quaternion.
 * @param b The second quaternion.
 * @return The result of the multiplication (a * b).
 */
GMP_STATIC_INLINE ctl_quaternion_t ctl_qmul(ctl_quaternion_t a, ctl_quaternion_t b)
{
    ctl_quaternion_t result;
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return result;
}

/**
 * @brief Computes the conjugate of a quaternion.
 * @param q The input quaternion.
 * @return The conjugate of q.
 */
GMP_STATIC_INLINE ctl_quaternion_t ctl_qconj(ctl_quaternion_t q)
{
    ctl_quaternion_t result;
    result.w = q.w;
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;
    return result;
}

/**
 * @brief Computes the squared norm (magnitude) of a quaternion.
 * This is computationally cheaper than `ctl_qnorm` as it avoids a square root.
 * @param q The input quaternion.
 * @return The squared norm (|q|^2).
 */
GMP_STATIC_INLINE ctrl_gt ctl_qnorm_sq(ctl_quaternion_t q)
{
    return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
}

/**
 * @brief Computes the norm (magnitude) of a quaternion.
 * @param q The input quaternion.
 * @return The norm (|q|).
 */
GMP_STATIC_INLINE ctrl_gt ctl_qnorm(ctl_quaternion_t q)
{
    return ctl_sqrt(ctl_qnorm_sq(q));
}

/**
 * @brief Normalizes a quaternion to produce a unit quaternion.
 * A unit quaternion has a norm of 1 and is used to represent rotations.
 * @param q The quaternion to normalize.
 * @return The normalized (unit) quaternion. Returns {0,0,0,0} if the norm is zero.
 */
GMP_STATIC_INLINE ctl_quaternion_t ctl_qnormalize(ctl_quaternion_t q)
{
    ctl_quaternion_t result = {0, 0, 0, 0};
    ctrl_gt norm = ctl_qnorm(q);

    if (norm > 1e-9) // Use a small epsilon to avoid division by zero
    {
        result.w = q.w / norm;
        result.x = q.x / norm;
        result.y = q.y / norm;
        result.z = q.z / norm;
    }
    return result;
}

/** @} */ // end of MC_QUATERNION group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_CTL_MATH_QUATERNION_H_
