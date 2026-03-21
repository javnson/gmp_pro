/**
 * @file matrix3.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a 3x3 matrix type and related mathematical operations.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a standard implementation for 3x3 matrix arithmetic,
 * which is fundamental for linear transformations in 3D space, such as rotations.
 * The matrix elements are stored in row-major order:
 * [m00, m01, m02, m10, m11, m12, m20, m21, m22].
 */

#ifndef _FILE_GMP_CTL_MATRIX3_H_
#define _FILE_GMP_CTL_MATRIX3_H_

#include <ctl/math_block/vector_lite/vector3.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* 3x3 Matrix Math                                                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_MATRIX3 3x3 Matrix Math
 * @ingroup MC_LINEAR_ALGEBRA
 * @brief A collection of types and functions for 3x3 matrix arithmetic.
 * @{
 */

/**
 * @brief Data structure for representing a 3x3 matrix.
 * The elements are stored in row-major order:
 * | dat[0]  dat[1]  dat[2] |
 * | dat[3]  dat[4]  dat[5] |
 * | dat[6]  dat[7]  dat[8] |
 */
typedef struct _tag_ctl_matrix3_t
{
    ctrl_gt dat[9];
} ctl_matrix3_t;

/**
 * @brief Gets an element from the matrix at the specified row and column.
 * @param[in] mat Pointer to the matrix.
 * @param[in] row The row index (0, 1, or 2).
 * @param[in] col The column index (0, 1, or 2).
 * @return The value of the element at (row, col).
 */
GMP_STATIC_INLINE ctrl_gt ctl_matrix3_get(const ctl_matrix3_t* mat, int row, int col)
{
    return mat->dat[row * 3 + col];
}

/**
 * @brief Sets an element in the matrix at the specified row and column.
 * @param[out] mat Pointer to the matrix.
 * @param[in] row The row index (0, 1, or 2).
 * @param[in] col The column index (0, 1, or 2).
 * @param[in] value The new value for the element.
 */
GMP_STATIC_INLINE void ctl_matrix3_set(ctl_matrix3_t* mat, int row, int col, ctrl_gt value)
{
    mat->dat[row * 3 + col] = value;
}

/**
 * @brief Clears a 3x3 matrix, setting all its elements to zero.
 * @param[out] matrix Pointer to the matrix to be cleared.
 */
GMP_STATIC_INLINE void ctl_matrix3_clear(ctl_matrix3_t* matrix)
{
    int i;
    for (i = 0; i < 9; ++i)
    {
        matrix->dat[i] = float2ctrl(0.0f);
    }
}

/**
 * @brief Sets a 3x3 matrix to the identity matrix.
 * @param[out] matrix Pointer to the matrix to be set.
 */
GMP_STATIC_INLINE void ctl_matrix3_set_identity(ctl_matrix3_t* matrix)
{
    ctl_matrix3_clear(matrix);
    matrix->dat[0] = float2ctrl(1.0f);
    matrix->dat[4] = float2ctrl(1.0f);
    matrix->dat[8] = float2ctrl(1.0f);
}

/**
 * @brief Copies the contents of one 3x3 matrix to another.
 * @param[out] dup Pointer to the destination matrix.
 * @param[in]  src Pointer to the source matrix.
 */
GMP_STATIC_INLINE void ctl_matrix3_copy(ctl_matrix3_t* dup, const ctl_matrix3_t* src)
{
    int i;
    for (i = 0; i < 9; ++i)
    {
        dup->dat[i] = src->dat[i];
    }
}

/**
 * @brief Adds two 3x3 matrices.
 * @param[out] result The resulting matrix (a + b).
 * @param[in]  a The first matrix.
 * @param[in]  b The second matrix.
 */
GMP_STATIC_INLINE void ctl_matrix3_add(ctl_matrix3_t* result, const ctl_matrix3_t* a, const ctl_matrix3_t* b)
{
    int i;
    for (i = 0; i < 9; ++i)
    {
        result->dat[i] = a->dat[i] + b->dat[i];
    }
}

/**
 * @brief Subtracts one 3x3 matrix from another.
 * @param[out] result The resulting matrix (a - b).
 * @param[in]  a The minuend matrix.
 * @param[in]  b The subtrahend matrix.
 */
GMP_STATIC_INLINE void ctl_matrix3_sub(ctl_matrix3_t* result, const ctl_matrix3_t* a, const ctl_matrix3_t* b)
{
    int i;
    for (i = 0; i < 9; ++i)
    {
        result->dat[i] = a->dat[i] - b->dat[i];
    }
}

/**
 * @brief Multiplies a 3x3 matrix by a scalar value.
 * @param[out] result The resulting scaled matrix.
 * @param[in]  mat The matrix to be scaled.
 * @param[in]  scalar The scalar value.
 */
GMP_STATIC_INLINE void ctl_matrix3_scale(ctl_matrix3_t* result, const ctl_matrix3_t* mat, ctrl_gt scalar)
{
    int i;
    for (i = 0; i < 9; ++i)
    {
        result->dat[i] = ctl_mul(mat->dat[i], scalar);
    }
}

/**
 * @brief Multiplies two 3x3 matrices.
 * @param[out] result The resulting matrix (a * b).
 * @param[in]  a The first matrix.
 * @param[in]  b The second matrix.
 */
GMP_STATIC_INLINE void ctl_matrix3_mul(ctl_matrix3_t* result, const ctl_matrix3_t* a, const ctl_matrix3_t* b)
{
    result->dat[0] = ctl_mul(a->dat[0], b->dat[0]) + ctl_mul(a->dat[1], b->dat[3]) + ctl_mul(a->dat[2], b->dat[6]);
    result->dat[1] = ctl_mul(a->dat[0], b->dat[1]) + ctl_mul(a->dat[1], b->dat[4]) + ctl_mul(a->dat[2], b->dat[7]);
    result->dat[2] = ctl_mul(a->dat[0], b->dat[2]) + ctl_mul(a->dat[1], b->dat[5]) + ctl_mul(a->dat[2], b->dat[8]);

    result->dat[3] = ctl_mul(a->dat[3], b->dat[0]) + ctl_mul(a->dat[4], b->dat[3]) + ctl_mul(a->dat[5], b->dat[6]);
    result->dat[4] = ctl_mul(a->dat[3], b->dat[1]) + ctl_mul(a->dat[4], b->dat[4]) + ctl_mul(a->dat[5], b->dat[7]);
    result->dat[5] = ctl_mul(a->dat[3], b->dat[2]) + ctl_mul(a->dat[4], b->dat[5]) + ctl_mul(a->dat[5], b->dat[8]);

    result->dat[6] = ctl_mul(a->dat[6], b->dat[0]) + ctl_mul(a->dat[7], b->dat[3]) + ctl_mul(a->dat[8], b->dat[6]);
    result->dat[7] = ctl_mul(a->dat[6], b->dat[1]) + ctl_mul(a->dat[7], b->dat[4]) + ctl_mul(a->dat[8], b->dat[7]);
    result->dat[8] = ctl_mul(a->dat[6], b->dat[2]) + ctl_mul(a->dat[7], b->dat[5]) + ctl_mul(a->dat[8], b->dat[8]);
}

/**
 * @brief Multiplies a 3x3 matrix by a 3D column vector.
 * @param[out] result The resulting transformed vector (mat * vec).
 * @param[in]  mat The matrix.
 * @param[in]  vec The vector.
 */
GMP_STATIC_INLINE void ctl_matrix3_mul_vector(ctl_vector3_t* result, const ctl_matrix3_t* mat, const ctl_vector3_t* vec)
{
    result->dat[0] =
        ctl_mul(mat->dat[0], vec->dat[0]) + ctl_mul(mat->dat[1], vec->dat[1]) + ctl_mul(mat->dat[2], vec->dat[2]);
    result->dat[1] =
        ctl_mul(mat->dat[3], vec->dat[0]) + ctl_mul(mat->dat[4], vec->dat[1]) + ctl_mul(mat->dat[5], vec->dat[2]);
    result->dat[2] =
        ctl_mul(mat->dat[6], vec->dat[0]) + ctl_mul(mat->dat[7], vec->dat[1]) + ctl_mul(mat->dat[8], vec->dat[2]);
}

/**
 * @brief Calculates the determinant of a 3x3 matrix.
 * @param[in] mat The input matrix.
 * @return The determinant of the matrix.
 */
GMP_STATIC_INLINE ctrl_gt ctl_matrix3_det(const ctl_matrix3_t* mat)
{
    return ctl_mul(mat->dat[0], ctl_mul(mat->dat[4], mat->dat[8]) - ctl_mul(mat->dat[5], mat->dat[7])) -
           ctl_mul(mat->dat[1], ctl_mul(mat->dat[3], mat->dat[8]) - ctl_mul(mat->dat[5], mat->dat[6])) +
           ctl_mul(mat->dat[2], ctl_mul(mat->dat[3], mat->dat[7]) - ctl_mul(mat->dat[4], mat->dat[6]));
}

/**
 * @brief Calculates the inverse of a 3x3 matrix.
 * @param[out] result The inverse of the matrix. Returns a zero matrix if the determinant is zero.
 * @param[in]  mat The matrix to be inverted.
 */
GMP_STATIC_INLINE void ctl_matrix3_inv(ctl_matrix3_t* result, const ctl_matrix3_t* mat)
{
    ctrl_gt det = ctl_matrix3_det(mat);

    if (ctl_abs(det) < CTL_EPSILON)
    {
        ctl_matrix3_clear(result);
        return;
    }

    /* Optimization: Calculate 1/det once, then multiply */
    ctrl_gt inv_det = ctl_div(float2ctrl(1.0f), det);

    result->dat[0] = ctl_mul(ctl_mul(mat->dat[4], mat->dat[8]) - ctl_mul(mat->dat[5], mat->dat[7]), inv_det);
    result->dat[1] = ctl_mul(ctl_mul(mat->dat[2], mat->dat[7]) - ctl_mul(mat->dat[1], mat->dat[8]), inv_det);
    result->dat[2] = ctl_mul(ctl_mul(mat->dat[1], mat->dat[5]) - ctl_mul(mat->dat[2], mat->dat[4]), inv_det);

    result->dat[3] = ctl_mul(ctl_mul(mat->dat[5], mat->dat[6]) - ctl_mul(mat->dat[3], mat->dat[8]), inv_det);
    result->dat[4] = ctl_mul(ctl_mul(mat->dat[0], mat->dat[8]) - ctl_mul(mat->dat[2], mat->dat[6]), inv_det);
    result->dat[5] = ctl_mul(ctl_mul(mat->dat[2], mat->dat[3]) - ctl_mul(mat->dat[0], mat->dat[5]), inv_det);

    result->dat[6] = ctl_mul(ctl_mul(mat->dat[3], mat->dat[7]) - ctl_mul(mat->dat[4], mat->dat[6]), inv_det);
    result->dat[7] = ctl_mul(ctl_mul(mat->dat[1], mat->dat[6]) - ctl_mul(mat->dat[0], mat->dat[7]), inv_det);
    result->dat[8] = ctl_mul(ctl_mul(mat->dat[0], mat->dat[4]) - ctl_mul(mat->dat[1], mat->dat[3]), inv_det);
}

/**
 * @brief Transposes a 3x3 matrix.
 * @param[out] result The transposed matrix.
 * @param[in]  mat The matrix to be transposed.
 */
GMP_STATIC_INLINE void ctl_matrix3_trans(ctl_matrix3_t* result, const ctl_matrix3_t* mat)
{
    result->dat[0] = mat->dat[0];
    result->dat[1] = mat->dat[3];
    result->dat[2] = mat->dat[6];

    result->dat[3] = mat->dat[1];
    result->dat[4] = mat->dat[4];
    result->dat[5] = mat->dat[7];

    result->dat[6] = mat->dat[2];
    result->dat[7] = mat->dat[5];
    result->dat[8] = mat->dat[8];
}

/**
 * @brief Calculates the trace of a 3x3 matrix (sum of diagonal elements).
 * @param[in] mat The input matrix.
 * @return The trace of the matrix.
 */
GMP_STATIC_INLINE ctrl_gt ctl_matrix3_trace(const ctl_matrix3_t* mat)
{
    return mat->dat[0] + mat->dat[4] + mat->dat[8];
}
/** @} */ // end of MC_MATRIX3 group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_CTL_MATRIX3_H_
