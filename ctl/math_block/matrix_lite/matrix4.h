/**
 * @file matrix4.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a 4x4 matrix type and related mathematical operations.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a standard implementation for 4x4 matrix arithmetic,
 * which is fundamental for transformations in homogeneous coordinates (3D graphics)
 * and other advanced applications. The matrix elements are stored in row-major order.
 */

#ifndef _FILE_GMP_CTL_MATRIX4_H_
#define _FILE_GMP_CTL_MATRIX4_H_

#include <ctl/math_block/vector_lite/vector4.h> // Depends on the 4D vector module
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* 4x4 Matrix Math                                                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_MATRIX4 4x4 Matrix Math
 * @ingroup MC_LINEAR_ALGEBRA
 * @brief A collection of types and functions for 4x4 matrix arithmetic.
 * @{
 */

/**
 * @brief Data structure for representing a 4x4 matrix.
 * The elements are stored in row-major order.
 */
typedef struct _tag_ctl_matrix4_t
{
    ctrl_gt dat[16];
} ctl_matrix4_t;

/**
 * @brief Gets an element from the matrix at the specified row and column.
 * @param[in] mat Pointer to the matrix.
 * @param[in] row The row index (0-3).
 * @param[in] col The column index (0-3).
 * @return The value of the element at (row, col).
 */
GMP_STATIC_INLINE ctrl_gt ctl_matrix4_get(const ctl_matrix4_t* mat, int row, int col)
{
    return mat->dat[row * 4 + col];
}

/**
 * @brief Sets an element in the matrix at the specified row and column.
 * @param[out] mat Pointer to the matrix.
 * @param[in] row The row index (0-3).
 * @param[in] col The column index (0-3).
 * @param[in] value The new value for the element.
 */
GMP_STATIC_INLINE void ctl_matrix4_set(ctl_matrix4_t* mat, int row, int col, ctrl_gt value)
{
    mat->dat[row * 4 + col] = value;
}

/**
 * @brief Clears a 4x4 matrix, setting all its elements to zero.
 * @param[out] matrix Pointer to the matrix to be cleared.
 */
GMP_STATIC_INLINE void ctl_matrix4_clear(ctl_matrix4_t* matrix)
{
    int i;

    for (i = 0; i < 16; ++i)
        matrix->dat[i] = 0;
}

/**
 * @brief Sets a 4x4 matrix to the identity matrix.
 * @param[out] matrix Pointer to the matrix to be set.
 */
GMP_STATIC_INLINE void ctl_matrix4_set_identity(ctl_matrix4_t* matrix)
{
    ctl_matrix4_clear(matrix);
    matrix->dat[0] = 1;
    matrix->dat[5] = 1;
    matrix->dat[10] = 1;
    matrix->dat[15] = 1;
}

/**
 * @brief Copies the contents of one 4x4 matrix to another.
 * @param[out] dup Pointer to the destination matrix.
 * @param[in]  src Pointer to the source matrix.
 */
GMP_STATIC_INLINE void ctl_matrix4_copy(ctl_matrix4_t* dup, const ctl_matrix4_t* src)
{
    int i;

    for (i = 0; i < 16; ++i)
    {
        dup->dat[i] = src->dat[i];
    }
}

/**
 * @brief Adds two 4x4 matrices.
 * @param a The first matrix.
 * @param b The second matrix.
 * @return The resulting matrix (a + b).
 */
GMP_STATIC_INLINE ctl_matrix4_t ctl_matrix4_add(ctl_matrix4_t a, ctl_matrix4_t b)
{
    ctl_matrix4_t result;
    int i;

    for (i = 0; i < 16; ++i)
    {
        result.dat[i] = a.dat[i] + b.dat[i];
    }
    return result;
}

/**
 * @brief Subtracts one 4x4 matrix from another.
 * @param a The minuend matrix.
 * @param b The subtrahend matrix.
 * @return The resulting matrix (a - b).
 */
GMP_STATIC_INLINE ctl_matrix4_t ctl_matrix4_sub(ctl_matrix4_t a, ctl_matrix4_t b)
{
    ctl_matrix4_t result;
    int i;

    for (i = 0; i < 16; ++i)
    {
        result.dat[i] = a.dat[i] - b.dat[i];
    }
    return result;
}

/**
 * @brief Multiplies a 4x4 matrix by a scalar value.
 * @param mat The matrix to be scaled.
 * @param scalar The scalar value.
 * @return The resulting scaled matrix.
 */
GMP_STATIC_INLINE ctl_matrix4_t ctl_matrix4_scale(ctl_matrix4_t mat, ctrl_gt scalar)
{
    ctl_matrix4_t result;
    int i;

    for (i = 0; i < 16; ++i)
    {
        result.dat[i] = mat.dat[i] * scalar;
    }
    return result;
}

/**
 * @brief Multiplies two 4x4 matrices.
 * @param a The first matrix.
 * @param b The second matrix.
 * @return The resulting matrix (a * b).
 */
GMP_STATIC_INLINE ctl_matrix4_t ctl_matrix4_mul(ctl_matrix4_t a, ctl_matrix4_t b)
{
    ctl_matrix4_t result;
    int i, j, k;

    for (i = 0; i < 4; ++i)
    {
        for (j = 0; j < 4; ++j)
        {
            ctrl_gt sum = 0;
            for (k = 0; k < 4; ++k)
            {
                sum += ctl_matrix4_get(&a, i, k) * ctl_matrix4_get(&b, k, j);
            }
            ctl_matrix4_set(&result, i, j, sum);
        }
    }
    return result;
}

/**
 * @brief Multiplies a 4x4 matrix by a 4D column vector.
 * @param mat The matrix.
 * @param vec The vector.
 * @return The resulting transformed vector (mat * vec).
 */
GMP_STATIC_INLINE ctl_vector4_t ctl_matrix4_mul_vector(ctl_matrix4_t mat, ctl_vector4_t vec)
{
    ctl_vector4_t result;
    int i, j;

    for (i = 0; i < 4; ++i)
    {
        ctrl_gt sum = 0;
        for (j = 0; j < 4; ++j)
        {
            sum += ctl_matrix4_get(&mat, i, j) * vec.dat[j];
        }
        result.dat[i] = sum;
    }
    return result;
}

/**
 * @brief Calculates the determinant of a 4x4 matrix.
 * @param mat The input matrix.
 * @return The determinant of the matrix.
 */
GMP_STATIC_INLINE ctrl_gt ctl_matrix4_det(ctl_matrix4_t mat)
{
    ctrl_gt det = 0;
    det += mat.dat[0] * (mat.dat[5] * (mat.dat[10] * mat.dat[15] - mat.dat[11] * mat.dat[14]) -
                         mat.dat[6] * (mat.dat[9] * mat.dat[15] - mat.dat[11] * mat.dat[13]) +
                         mat.dat[7] * (mat.dat[9] * mat.dat[14] - mat.dat[10] * mat.dat[13]));
    det -= mat.dat[1] * (mat.dat[4] * (mat.dat[10] * mat.dat[15] - mat.dat[11] * mat.dat[14]) -
                         mat.dat[6] * (mat.dat[8] * mat.dat[15] - mat.dat[11] * mat.dat[12]) +
                         mat.dat[7] * (mat.dat[8] * mat.dat[14] - mat.dat[10] * mat.dat[12]));
    det += mat.dat[2] * (mat.dat[4] * (mat.dat[9] * mat.dat[15] - mat.dat[11] * mat.dat[13]) -
                         mat.dat[5] * (mat.dat[8] * mat.dat[15] - mat.dat[11] * mat.dat[12]) +
                         mat.dat[7] * (mat.dat[8] * mat.dat[13] - mat.dat[9] * mat.dat[12]));
    det -= mat.dat[3] * (mat.dat[4] * (mat.dat[9] * mat.dat[14] - mat.dat[10] * mat.dat[13]) -
                         mat.dat[5] * (mat.dat[8] * mat.dat[14] - mat.dat[10] * mat.dat[12]) +
                         mat.dat[6] * (mat.dat[8] * mat.dat[13] - mat.dat[9] * mat.dat[12]));
    return det;
}

/**
 * @brief Calculates the inverse of a 4x4 matrix.
 * @param mat The matrix to be inverted.
 * @return The inverse of the matrix. Returns a zero matrix if the determinant is zero.
 */
GMP_STATIC_INLINE ctl_matrix4_t ctl_matrix4_inv(ctl_matrix4_t mat)
{
    ctl_matrix4_t result;
    ctrl_gt det = ctl_matrix4_det(mat);

    if (fabs(det) < 1e-9) // Avoid division by zero
    {
        ctl_matrix4_clear(&result);
        return result;
    }

    ctrl_gt inv_det = 1.0f / det;

    result.dat[0] = (mat.dat[5] * (mat.dat[10] * mat.dat[15] - mat.dat[11] * mat.dat[14]) -
                     mat.dat[6] * (mat.dat[9] * mat.dat[15] - mat.dat[11] * mat.dat[13]) +
                     mat.dat[7] * (mat.dat[9] * mat.dat[14] - mat.dat[10] * mat.dat[13])) *
                    inv_det;
    result.dat[1] = -(mat.dat[1] * (mat.dat[10] * mat.dat[15] - mat.dat[11] * mat.dat[14]) -
                      mat.dat[2] * (mat.dat[9] * mat.dat[15] - mat.dat[11] * mat.dat[13]) +
                      mat.dat[3] * (mat.dat[9] * mat.dat[14] - mat.dat[10] * mat.dat[13])) *
                    inv_det;
    result.dat[2] = (mat.dat[1] * (mat.dat[6] * mat.dat[15] - mat.dat[7] * mat.dat[14]) -
                     mat.dat[2] * (mat.dat[5] * mat.dat[15] - mat.dat[7] * mat.dat[13]) +
                     mat.dat[3] * (mat.dat[5] * mat.dat[14] - mat.dat[6] * mat.dat[13])) *
                    inv_det;
    result.dat[3] = -(mat.dat[1] * (mat.dat[6] * mat.dat[11] - mat.dat[7] * mat.dat[10]) -
                      mat.dat[2] * (mat.dat[5] * mat.dat[11] - mat.dat[7] * mat.dat[9]) +
                      mat.dat[3] * (mat.dat[5] * mat.dat[10] - mat.dat[6] * mat.dat[9])) *
                    inv_det;
    result.dat[4] = -(mat.dat[4] * (mat.dat[10] * mat.dat[15] - mat.dat[11] * mat.dat[14]) -
                      mat.dat[6] * (mat.dat[8] * mat.dat[15] - mat.dat[11] * mat.dat[12]) +
                      mat.dat[7] * (mat.dat[8] * mat.dat[14] - mat.dat[10] * mat.dat[12])) *
                    inv_det;
    result.dat[5] = (mat.dat[0] * (mat.dat[10] * mat.dat[15] - mat.dat[11] * mat.dat[14]) -
                     mat.dat[2] * (mat.dat[8] * mat.dat[15] - mat.dat[11] * mat.dat[12]) +
                     mat.dat[3] * (mat.dat[8] * mat.dat[14] - mat.dat[10] * mat.dat[12])) *
                    inv_det;
    result.dat[6] = -(mat.dat[0] * (mat.dat[6] * mat.dat[15] - mat.dat[7] * mat.dat[14]) -
                      mat.dat[2] * (mat.dat[4] * mat.dat[15] - mat.dat[7] * mat.dat[12]) +
                      mat.dat[3] * (mat.dat[4] * mat.dat[14] - mat.dat[6] * mat.dat[12])) *
                    inv_det;
    result.dat[7] = (mat.dat[0] * (mat.dat[6] * mat.dat[11] - mat.dat[7] * mat.dat[10]) -
                     mat.dat[2] * (mat.dat[4] * mat.dat[11] - mat.dat[7] * mat.dat[8]) +
                     mat.dat[3] * (mat.dat[4] * mat.dat[10] - mat.dat[6] * mat.dat[8])) *
                    inv_det;
    result.dat[8] = (mat.dat[4] * (mat.dat[9] * mat.dat[15] - mat.dat[11] * mat.dat[13]) -
                     mat.dat[5] * (mat.dat[8] * mat.dat[15] - mat.dat[11] * mat.dat[12]) +
                     mat.dat[7] * (mat.dat[8] * mat.dat[13] - mat.dat[9] * mat.dat[12])) *
                    inv_det;
    result.dat[9] = -(mat.dat[0] * (mat.dat[9] * mat.dat[15] - mat.dat[11] * mat.dat[13]) -
                      mat.dat[1] * (mat.dat[8] * mat.dat[15] - mat.dat[11] * mat.dat[12]) +
                      mat.dat[3] * (mat.dat[8] * mat.dat[13] - mat.dat[9] * mat.dat[12])) *
                    inv_det;
    result.dat[10] = (mat.dat[0] * (mat.dat[5] * mat.dat[15] - mat.dat[7] * mat.dat[13]) -
                      mat.dat[1] * (mat.dat[4] * mat.dat[15] - mat.dat[7] * mat.dat[12]) +
                      mat.dat[3] * (mat.dat[4] * mat.dat[13] - mat.dat[5] * mat.dat[12])) *
                     inv_det;
    result.dat[11] = -(mat.dat[0] * (mat.dat[5] * mat.dat[11] - mat.dat[7] * mat.dat[9]) -
                       mat.dat[1] * (mat.dat[4] * mat.dat[11] - mat.dat[7] * mat.dat[8]) +
                       mat.dat[3] * (mat.dat[4] * mat.dat[9] - mat.dat[5] * mat.dat[8])) *
                     inv_det;
    result.dat[12] = -(mat.dat[4] * (mat.dat[9] * mat.dat[14] - mat.dat[10] * mat.dat[13]) -
                       mat.dat[5] * (mat.dat[8] * mat.dat[14] - mat.dat[10] * mat.dat[12]) +
                       mat.dat[6] * (mat.dat[8] * mat.dat[13] - mat.dat[9] * mat.dat[12])) *
                     inv_det;
    result.dat[13] = (mat.dat[0] * (mat.dat[9] * mat.dat[14] - mat.dat[10] * mat.dat[13]) -
                      mat.dat[1] * (mat.dat[8] * mat.dat[14] - mat.dat[10] * mat.dat[12]) +
                      mat.dat[2] * (mat.dat[8] * mat.dat[13] - mat.dat[9] * mat.dat[12])) *
                     inv_det;
    result.dat[14] = -(mat.dat[0] * (mat.dat[5] * mat.dat[14] - mat.dat[6] * mat.dat[13]) -
                       mat.dat[1] * (mat.dat[4] * mat.dat[14] - mat.dat[6] * mat.dat[12]) +
                       mat.dat[2] * (mat.dat[4] * mat.dat[13] - mat.dat[5] * mat.dat[12])) *
                     inv_det;
    result.dat[15] = (mat.dat[0] * (mat.dat[5] * mat.dat[10] - mat.dat[6] * mat.dat[9]) -
                      mat.dat[1] * (mat.dat[4] * mat.dat[10] - mat.dat[6] * mat.dat[8]) +
                      mat.dat[2] * (mat.dat[4] * mat.dat[9] - mat.dat[5] * mat.dat[8])) *
                     inv_det;

    return result;
}

/**
 * @brief Transposes a 4x4 matrix.
 * @param mat The matrix to be transposed.
 * @return The transposed matrix.
 */
GMP_STATIC_INLINE ctl_matrix4_t ctl_matrix4_trans(ctl_matrix4_t mat)
{
    ctl_matrix4_t result;
    int i, j;

    for (i = 0; i < 4; ++i)
    {
        for (j = 0; j < 4; ++j)
        {
            ctl_matrix4_set(&result, i, j, ctl_matrix4_get(&mat, j, i));
        }
    }
    return result;
}

/**
 * @brief Calculates the trace of a 4x4 matrix (sum of diagonal elements).
 * @param mat The input matrix.
 * @return The trace of the matrix.
 */
GMP_STATIC_INLINE ctrl_gt ctl_matrix4_trace(ctl_matrix4_t mat)
{
    return mat.dat[0] + mat.dat[5] + mat.dat[10] + mat.dat[15];
}

/** @} */ // end of MC_MATRIX4 group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_CTL_MATRIX4_H_
