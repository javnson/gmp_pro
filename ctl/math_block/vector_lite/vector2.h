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

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* 2D Vector Math                                                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_VECTOR2 2D Vector Math
 * @ingroup MC_LINEAR_ALGEBRA
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
    vec->dat[0] = float2ctrl(0.0f);
    vec->dat[1] = float2ctrl(0.0f);
}

/**
 * @brief Copies the contents of one 2D vector to another.
 * @param[out] dup Pointer to the destination vector.
 * @param[in]  vec Pointer to the source vector.
 */
GMP_STATIC_INLINE void ctl_vector2_copy(ctl_vector2_t* dup, const ctl_vector2_t* vec)
{
    dup->dat[0] = vec->dat[0];
    dup->dat[1] = vec->dat[1];
}

/**
 * @brief Adds two 2D vectors.
 * @param[out] result return the result of vector (a + b).
 * @param[in]  a The first vector.
 * @param[in]  b The second vector.
 */
GMP_STATIC_INLINE void ctl_vector2_add(ctl_vector2_t* result, const ctl_vector2_t* a, const ctl_vector2_t* b)
{
    result->dat[0] = a->dat[0] + b->dat[0];
    result->dat[1] = a->dat[1] + b->dat[1];
}

/**
 * @brief Subtracts one 2D vector from another.
 * @param[out] result return the result of vector (a - b).
 * @param[in]  a The minuend vector.
 * @param[in]  b The subtrahend vector.
 */
GMP_STATIC_INLINE void ctl_vector2_sub(ctl_vector2_t* result, const ctl_vector2_t* a, const ctl_vector2_t* b)
{
    result->dat[0] = a->dat[0] - b->dat[0];
    result->dat[1] = a->dat[1] - b->dat[1];
}

/**
 * @brief Multiplies a 2D vector by a scalar value.
 * @param[out] result return the result of vector (scalar * vec).
 * @param[in]  vec The vector to be scaled.
 * @param[in]  scalar The scalar value.
 */
GMP_STATIC_INLINE void ctl_vector2_scale(ctl_vector2_t* result, const ctl_vector2_t* vec, ctrl_gt scalar)
{
    result->dat[0] = ctl_mul(vec->dat[0], scalar);
    result->dat[1] = ctl_mul(vec->dat[1], scalar);
}

/**
 * @brief Calculates the dot product of two 2D vectors.
 * @f[
 * a \cdot b = a_x b_x + a_y b_y
 * @f]
 * @param[in] a The first vector.
 * @param[in] b The second vector.
 * @return The dot product.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector2_dot(const ctl_vector2_t* a, const ctl_vector2_t* b)
{
    return ctl_mul(a->dat[0], b->dat[0]) + ctl_mul(a->dat[1], b->dat[1]);
}

/**
 * @brief Calculates the squared magnitude (length) of a 2D vector.
 * This is computationally cheaper than `ctl_vector2_mag` as it avoids a square root.
 * @param[in] vec The input vector.
 * @return The squared magnitude of the vector.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector2_mag_sq(const ctl_vector2_t* vec)
{
    return ctl_mul(vec->dat[0], vec->dat[0]) + ctl_mul(vec->dat[1], vec->dat[1]);
}

/**
 * @brief Calculates the magnitude (length) of a 2D vector.
 * @param[in] vec The input vector.
 * @return The magnitude of the vector.
 */
GMP_STATIC_INLINE ctrl_gt ctl_vector2_mag(const ctl_vector2_t* vec)
{
    return ctl_sqrt(ctl_vector2_mag_sq(vec));
}

/**
 * @brief Normalizes a 2D vector to produce a unit vector (a vector with length 1).
 * @param[out] result The normalized (unit) vector. Returns a zero vector if the magnitude is zero.
 * @param[in]  vec The vector to be normalized.
 */
GMP_STATIC_INLINE void ctl_vector2_normalize(ctl_vector2_t* result, const ctl_vector2_t* vec)
{
    ctrl_gt mag = ctl_vector2_mag(vec);
    if (mag > CTL_EPSILON)
    {
        result->dat[0] = ctl_div(vec->dat[0], mag);
        result->dat[1] = ctl_div(vec->dat[1], mag);
    }
    else
    {
        ctl_vector2_clear(result);
    }
}

/**
 * @brief Saturates a 2D vector using a squared circle radius.
 * @details Vectors outside the circle are scaled without changing direction.
 * The scale is calculated as
 * `radius_sq * isqrt(radius_sq * magnitude_sq)`, combining square root and
 * reciprocal operations. `result` may alias `vec`.
 * @param[out] result The saturated vector.
 * @param[in]  vec The input vector.
 * @param[in]  radius_sq Non-negative squared circle radius.
 */
GMP_STATIC_INLINE void ctl_vector2_sat_circle_sq(ctl_vector2_t* result, const ctl_vector2_t* vec,
                                                 ctrl_gt radius_sq)
{
    ctrl_gt x = vec->dat[0];
    ctrl_gt y = vec->dat[1];
    ctrl_gt mag_sq;

    gmp_base_assert(radius_sq >= float2ctrl(0.0f));

    mag_sq = ctl_mul(x, x) + ctl_mul(y, y);

    if (mag_sq > radius_sq)
    {
        if (radius_sq > CTL_EPSILON)
        {
            ctrl_gt scale = ctl_mul(radius_sq, ctl_isqrt(ctl_mul(radius_sq, mag_sq)));
            result->dat[0] = ctl_mul(x, scale);
            result->dat[1] = ctl_mul(y, scale);
            return;
        }
        ctl_vector2_clear(result);
        return;
    }

    result->dat[0] = x;
    result->dat[1] = y;
}

/**
 * @brief Conservatively saturates a 2D vector using a first-order approximation.
 * @details For `ratio = magnitude_sq / radius_sq`, this function approximates
 * `isqrt(ratio)` by its first-order Taylor expansion at the circle boundary:
 * `1.5 - 0.5 * ratio`. Because inverse square root is convex, this tangent is
 * no greater than the exact scale for `ratio >= 1`, so the result remains inside
 * the requested circle. Ratios greater than or equal to 3 produce a zero vector
 * instead of a negative scale. This version uses no square-root operation.
 * `result` may alias `vec`.
 * @param[out] result The conservatively saturated vector.
 * @param[in]  vec The input vector.
 * @param[in]  radius_sq Non-negative squared circle radius.
 */
GMP_STATIC_INLINE void ctl_vector2_sat_circle_sq_taylor(ctl_vector2_t* result, const ctl_vector2_t* vec,
                                                        ctrl_gt radius_sq)
{
    ctrl_gt x = vec->dat[0];
    ctrl_gt y = vec->dat[1];
    ctrl_gt mag_sq;

    gmp_base_assert(radius_sq >= float2ctrl(0.0f));

    mag_sq = ctl_mul(x, x) + ctl_mul(y, y);

    if (mag_sq > radius_sq)
    {
        ctrl_gt ratio;
        ctrl_gt scale;

        if (radius_sq <= CTL_EPSILON)
        {
            ctl_vector2_clear(result);
            return;
        }

        ratio = ctl_div(mag_sq, radius_sq);
        if (ratio >= float2ctrl(3.0f))
        {
            ctl_vector2_clear(result);
            return;
        }

        scale = CTL_CTRL_CONST_3_OVER_2 - ctl_div2(ratio);
        result->dat[0] = ctl_mul(x, scale);
        result->dat[1] = ctl_mul(y, scale);
        return;
    }

    result->dat[0] = x;
    result->dat[1] = y;
}

/**
 * @brief Saturates a 2D vector to a circle centered at the origin.
 * @details This compatibility interface calculates the squared radius once and
 * delegates to `ctl_vector2_sat_circle_sq`. Repeated real-time calls should cache
 * the squared radius and call the `_sq` interface directly.
 * @param[out] result The saturated vector.
 * @param[in]  vec The input vector.
 * @param[in]  radius Non-negative circle radius.
 */
GMP_STATIC_INLINE void ctl_vector2_sat_circle(ctl_vector2_t* result, const ctl_vector2_t* vec, ctrl_gt radius)
{
    gmp_base_assert(radius >= float2ctrl(0.0f));
    ctl_vector2_sat_circle_sq(result, vec, ctl_mul(radius, radius));
}

/**
 * @brief Saturates each component of a 2D vector to an axis-aligned rectangle.
 * @details `result` may alias any input vector.
 * @param[out] result The saturated vector.
 * @param[in]  vec The input vector.
 * @param[in]  limit_max Per-axis upper limits.
 * @param[in]  limit_min Per-axis lower limits.
 */
GMP_STATIC_INLINE void ctl_vector2_sat_rect(ctl_vector2_t* result, const ctl_vector2_t* vec,
                                            const ctl_vector2_t* limit_max, const ctl_vector2_t* limit_min)
{
    ctrl_gt x = vec->dat[0];
    ctrl_gt y = vec->dat[1];

    gmp_base_assert(limit_max->dat[0] >= limit_min->dat[0]);
    gmp_base_assert(limit_max->dat[1] >= limit_min->dat[1]);

    result->dat[0] = ctl_sat(x, limit_max->dat[0], limit_min->dat[0]);
    result->dat[1] = ctl_sat(y, limit_max->dat[1], limit_min->dat[1]);
}

/**
 * @brief Saturates each component of a 2D vector to a symmetric square.
 * @details This is a convenience form of rectangular saturation with identical
 * limits on both axes. `result` may alias `vec`.
 * @param[out] result The saturated vector.
 * @param[in]  vec The input vector.
 * @param[in]  limit Non-negative component magnitude limit.
 */
GMP_STATIC_INLINE void ctl_vector2_sat_square(ctl_vector2_t* result, const ctl_vector2_t* vec, ctrl_gt limit)
{
    ctrl_gt x = vec->dat[0];
    ctrl_gt y = vec->dat[1];

    gmp_base_assert(limit >= float2ctrl(0.0f));
    result->dat[0] = ctl_sat(x, limit, -limit);
    result->dat[1] = ctl_sat(y, limit, -limit);
}

/** 
 * @} 
 */ // end of MC_VECTOR2 group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_VECTOR2_H_
