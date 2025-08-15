/**
 * @file matrix2.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a 2x2 matrix type and related mathematical operations.
 * @version 0.4
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a standard implementation for 2x2 matrix arithmetic,
 * which is fundamental for linear transformations in 2D space.
 * The matrix elements are stored in row-major order: [m00, m01, m10, m11].
 */

#ifndef _FILE_GMP_CTL_MATRIX2_H_
#define _FILE_GMP_CTL_MATRIX2_H_

#include <ctl/math_block/complex_lite/complex.h> // Depends on the complex number module
#include <ctl/math_block/vector_lite/vector2.h>  // Depends on the 2D vector module
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* 2x2 Matrix Math                                                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_MATRIX2 2x2 Matrix Math
 * @ingroup MC_LINEAR_ALGEBRA
 * @brief A collection of types and functions for 2x2 matrix arithmetic.
 * @{
 */

/**
 * @brief Data structure for representing a 2x2 matrix.
 * The elements are stored in row-major order:
 * | dat[0]  dat[1] |
 * | dat[2]  dat[3] |
 */
typedef struct _tag_ctl_matrix2_t
{
    ctrl_gt dat[4];
} ctl_matrix2_t;

/**
 * @brief Gets an element from the matrix at the specified row and column.
 * @param[in] mat Pointer to the matrix.
 * @param[in] row The row index (0 or 1).
 * @param[in] col The column index (0 or 1).
 * @return The value of the element at (row, col).
 */
GMP_STATIC_INLINE ctrl_gt ctl_matrix2_get(const ctl_matrix2_t* mat, int row, int col)
{
    return mat->dat[row * 2 + col];
}

/**
 * @brief Sets an element in the matrix at the specified row and column.
 * @param[out] mat Pointer to the matrix.
 * @param[in] row The row index (0 or 1).
 * @param[in] col The column index (0 or 1).
 * @param[in] value The new value for the element.
 */
GMP_STATIC_INLINE void ctl_matrix2_set(ctl_matrix2_t* mat, int row, int col, ctrl_gt value)
{
    mat->dat[row * 2 + col] = value;
}

/**
 * @brief Clears a 2x2 matrix, setting all its elements to zero.
 * @param[out] matrix Pointer to the matrix to be cleared.
 */
GMP_STATIC_INLINE void ctl_matrix2_clear(ctl_matrix2_t* matrix)
{
    matrix->dat[0] = 0;
    matrix->dat[1] = 0;
    matrix->dat[2] = 0;
    matrix->dat[3] = 0;
}

/**
 * @brief Sets a 2x2 matrix to the identity matrix.
 * @param[out] matrix Pointer to the matrix to be set.
 */
GMP_STATIC_INLINE void ctl_matrix2_set_identity(ctl_matrix2_t* matrix)
{
    matrix->dat[0] = 1;
    matrix->dat[1] = 0;
    matrix->dat[2] = 0;
    matrix->dat[3] = 1;
}

/**
 * @brief Copies the contents of one 2x2 matrix to another.
 * @param[out] dup Pointer to the destination matrix.
 * @param[in]  src Pointer to the source matrix.
 */
GMP_STATIC_INLINE void ctl_matrix2_copy(ctl_matrix2_t* dup, ctl_matrix2_t* src)
{
    dup->dat[0] = src->dat[0];
    dup->dat[1] = src->dat[1];
    dup->dat[2] = src->dat[2];
    dup->dat[3] = src->dat[3];
}

/**
 * @brief Adds two 2x2 matrices.
 * @param a The first matrix.
 * @param b The second matrix.
 * @return The resulting matrix (a + b).
 */
GMP_STATIC_INLINE ctl_matrix2_t ctl_matrix2_add(ctl_matrix2_t a, ctl_matrix2_t b)
{
    ctl_matrix2_t result;
    result.dat[0] = a.dat[0] + b.dat[0];
    result.dat[1] = a.dat[1] + b.dat[1];
    result.dat[2] = a.dat[2] + b.dat[2];
    result.dat[3] = a.dat[3] + b.dat[3];
    return result;
}

/**
 * @brief Subtracts one 2x2 matrix from another.
 * @param a The minuend matrix.
 * @param b The subtrahend matrix.
 * @return The resulting matrix (a - b).
 */
GMP_STATIC_INLINE ctl_matrix2_t ctl_matrix2_sub(ctl_matrix2_t a, ctl_matrix2_t b)
{
    ctl_matrix2_t result;
    result.dat[0] = a.dat[0] - b.dat[0];
    result.dat[1] = a.dat[1] - b.dat[1];
    result.dat[2] = a.dat[2] - b.dat[2];
    result.dat[3] = a.dat[3] - b.dat[3];
    return result;
}

/**
 * @brief Multiplies a 2x2 matrix by a scalar value.
 * @param mat The matrix to be scaled.
 * @param scalar The scalar value.
 * @return The resulting scaled matrix.
 */
GMP_STATIC_INLINE ctl_matrix2_t ctl_matrix2_scale(ctl_matrix2_t mat, ctrl_gt scalar)
{
    ctl_matrix2_t result;
    result.dat[0] = mat.dat[0] * scalar;
    result.dat[1] = mat.dat[1] * scalar;
    result.dat[2] = mat.dat[2] * scalar;
    result.dat[3] = mat.dat[3] * scalar;
    return result;
}

/**
 * @brief Multiplies two 2x2 matrices.
 * @param a The first matrix.
 * @param b The second matrix.
 * @return The resulting matrix (a * b).
 */
GMP_STATIC_INLINE ctl_matrix2_t ctl_matrix2_mul(ctl_matrix2_t a, ctl_matrix2_t b)
{
    ctl_matrix2_t result;
    result.dat[0] = a.dat[0] * b.dat[0] + a.dat[1] * b.dat[2];
    result.dat[1] = a.dat[0] * b.dat[1] + a.dat[1] * b.dat[3];
    result.dat[2] = a.dat[2] * b.dat[0] + a.dat[3] * b.dat[2];
    result.dat[3] = a.dat[2] * b.dat[1] + a.dat[3] * b.dat[3];
    return result;
}

/**
 * @brief Multiplies a 2x2 matrix by a 2D column vector.
 * @param mat The matrix.
 * @param vec The vector.
 * @return The resulting transformed vector (mat * vec).
 */
GMP_STATIC_INLINE ctl_vector2_t ctl_matrix2_mul_vector(ctl_matrix2_t mat, ctl_vector2_t vec)
{
    ctl_vector2_t result;
    result.dat[0] = mat.dat[0] * vec.dat[0] + mat.dat[1] * vec.dat[1];
    result.dat[1] = mat.dat[2] * vec.dat[0] + mat.dat[3] * vec.dat[1];
    return result;
}

/**
 * @brief Calculates the determinant of a 2x2 matrix.
 * @f[
 * det(M) = ad - bc
 * @f]
 * @param mat The input matrix.
 * @return The determinant of the matrix.
 */
GMP_STATIC_INLINE ctrl_gt ctl_matrix2_det(ctl_matrix2_t mat)
{
    return mat.dat[0] * mat.dat[3] - mat.dat[1] * mat.dat[2];
}

/**
 * @brief Calculates the inverse of a 2x2 matrix.
 * @param mat The matrix to be inverted.
 * @return The inverse of the matrix. Returns a zero matrix if the determinant is zero.
 */
GMP_STATIC_INLINE ctl_matrix2_t ctl_matrix2_inv(ctl_matrix2_t mat)
{
    ctl_matrix2_t result;
    ctrl_gt det = ctl_matrix2_det(mat);

    if (fabs(det) < 1e-9) // Avoid division by zero
    {
        ctl_matrix2_clear(&result);
        return result;
    }

    ctrl_gt inv_det = 1.0f / det;
    result.dat[0] = mat.dat[3] * inv_det;
    result.dat[1] = -mat.dat[1] * inv_det;
    result.dat[2] = -mat.dat[2] * inv_det;
    result.dat[3] = mat.dat[0] * inv_det;
    return result;
}

/**
 * @brief Transposes a 2x2 matrix.
 * @param mat The matrix to be transposed.
 * @return The transposed matrix.
 */
GMP_STATIC_INLINE ctl_matrix2_t ctl_matrix2_trans(ctl_matrix2_t mat)
{
    ctl_matrix2_t result;
    result.dat[0] = mat.dat[0];
    result.dat[1] = mat.dat[2];
    result.dat[2] = mat.dat[1];
    result.dat[3] = mat.dat[3];
    return result;
}

/**
 * @brief Calculates the trace of a 2x2 matrix (sum of diagonal elements).
 * @param mat The input matrix.
 * @return The trace of the matrix.
 */
GMP_STATIC_INLINE ctrl_gt ctl_matrix2_trace(ctl_matrix2_t mat)
{
    return mat.dat[0] + mat.dat[3];
}

/**
 * @brief Performs a congruent transformation on matrix A with matrix P.
 *
 * This calculates the result of \f$ P^T A P \f$.
 *
 * @param a The matrix to be transformed.
 * @param p The transformation matrix.
 * @return The resulting matrix from the congruent transformation.
 */
GMP_STATIC_INLINE ctl_matrix2_t ctl_matrix2_congruent(ctl_matrix2_t a, ctl_matrix2_t p)
{
    ctl_matrix2_t p_t = ctl_matrix2_trans(p);
    ctl_matrix2_t temp = ctl_matrix2_mul(p_t, a);
    return ctl_matrix2_mul(temp, p);
}

/**
 * @brief Calculates the eigenvalues of a 2x2 matrix.
 *
 * It solves the characteristic equation: \f$ \lambda^2 - \text{trace}(A)\lambda + \text{det}(A) = 0 \f$.
 * The eigenvalues can be real or a complex conjugate pair.
 *
 * @param[in]  mat The input matrix.
 * @param[out] eigenvalue1 Pointer to the first complex number to store an eigenvalue.
 * @param[out] eigenvalue2 Pointer to the second complex number to store an eigenvalue.
 */
GMP_STATIC_INLINE void ctl_matrix2_eigenvalues_complex(ctl_matrix2_t mat, ctl_complex_t* eigenvalue1,
                                                       ctl_complex_t* eigenvalue2)
{
    ctrl_gt trace = ctl_matrix2_trace(mat);
    ctrl_gt det = ctl_matrix2_det(mat);
    ctrl_gt discriminant = trace * trace - 4 * det;

    if (discriminant >= 0)
    {
        // Real eigenvalues
        ctrl_gt sqrt_discriminant = ctl_sqrt(discriminant);
        eigenvalue1->real = (trace + sqrt_discriminant) / 2.0f;
        eigenvalue1->imag = 0.0f;
        eigenvalue2->real = (trace - sqrt_discriminant) / 2.0f;
        eigenvalue2->imag = 0.0f;
    }
    else
    {
        // Complex conjugate eigenvalues
        ctrl_gt sqrt_abs_discriminant = ctl_sqrt(-discriminant);
        eigenvalue1->real = trace / 2.0f;
        eigenvalue1->imag = sqrt_abs_discriminant / 2.0f;
        eigenvalue2->real = trace / 2.0f;
        eigenvalue2->imag = -sqrt_abs_discriminant / 2.0f;
    }
}

/**
 * @brief Calculates the real eigenvectors of a 2x2 matrix for given real eigenvalues.
 *
 * @param[in]  mat The input matrix.
 * @param[in]  eigenvalues A 2D vector containing the two real eigenvalues.
 * @param[out] eigenvector1 Pointer to a 2D vector to store the first eigenvector.
 * @param[out] eigenvector2 Pointer to a 2D vector to store the second eigenvector.
 */
GMP_STATIC_INLINE void ctl_matrix2_eigenvectors_real(ctl_matrix2_t mat, ctl_vector2_t eigenvalues,
                                                     ctl_vector2_t* eigenvector1, ctl_vector2_t* eigenvector2)
{
    ctrl_gt a = mat.dat[0];
    ctrl_gt b = mat.dat[1];
    ctrl_gt c = mat.dat[2];
    ctrl_gt d = mat.dat[3];

    // Calculate eigenvector for the first eigenvalue
    if (fabs(c) > 1e-9)
    {
        eigenvector1->dat[0] = eigenvalues.dat[0] - d;
        eigenvector1->dat[1] = c;
    }
    else if (fabs(b) > 1e-9)
    {
        eigenvector1->dat[0] = b;
        eigenvector1->dat[1] = eigenvalues.dat[0] - a;
    }
    else
    { // Diagonal matrix
        eigenvector1->dat[0] = 1;
        eigenvector1->dat[1] = 0;
    }

    // Calculate eigenvector for the second eigenvalue
    if (fabs(c) > 1e-9)
    {
        eigenvector2->dat[0] = eigenvalues.dat[1] - d;
        eigenvector2->dat[1] = c;
    }
    else if (fabs(b) > 1e-9)
    {
        eigenvector2->dat[0] = b;
        eigenvector2->dat[1] = eigenvalues.dat[1] - a;
    }
    else
    { // Diagonal matrix
        eigenvector2->dat[0] = 0;
        eigenvector2->dat[1] = 1;
    }

    // Normalize the eigenvectors
    *eigenvector1 = ctl_vector2_normalize(eigenvector1);
    *eigenvector2 = ctl_vector2_normalize(eigenvector2);
}

/** @} */ // end of MC_MATRIX2 group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_CTL_MATRIX2_H_
