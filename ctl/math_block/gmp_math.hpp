/**
 * @file gmp_math.hpp
 * @brief C++ entry point for GMP control math and generic lite algebra.
 */

#ifndef GMP_CTL_MATH_HPP
#define GMP_CTL_MATH_HPP

#include <ctl/math_block/gmp_math.h>
#include <ctl/math_block/vector_lite/vector.hpp>
#include <ctl/math_block/matrix_lite/matrix.hpp>

namespace ctl
{
namespace math
{

typedef vector_lite<ctrl_gt, 2U> ctrl_vector2;
typedef vector_lite<ctrl_gt, 3U> ctrl_vector3;
typedef vector_lite<ctrl_gt, 4U> ctrl_vector4;

typedef matrix_lite<ctrl_gt, 2U, 2U> ctrl_matrix2;
typedef matrix_lite<ctrl_gt, 3U, 3U> ctrl_matrix3;
typedef matrix_lite<ctrl_gt, 4U, 4U> ctrl_matrix4;

} // namespace math
} // namespace ctl

#endif // GMP_CTL_MATH_HPP
