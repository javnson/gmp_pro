/**
 * @file gmp_math.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Master include file for the GMP Control Math Library.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file serves as the main entry point for the entire math library.
 * It handles the selection of the underlying numerical representation (e.g.,
 * float, double, fixed-point) based on project-level definitions and
 * includes all necessary math modules.
 *
 * This header is deliberately independent from gmp_core.h.  It only consumes
 * GMP's configuration/type contract, so control algorithms do not pull in
 * CSP, peripheral services, lifecycle hooks, or the GMP runtime framework.
 */

#ifndef _FILE_GMP_CTL_MATH_H_
#define _FILE_GMP_CTL_MATH_H_

#if defined(GMP_CTL_PORTABLE)
#include <ctl/portable/gmp_ctl_portable.h>
#else
#include <gmp_type.h>
#endif

#include <ctl/ctl.config.h>

/* CTL assertions are part of the algorithm contract, not the GMP runtime.
 * Full GMP builds may define this macro in gmp_base.h before reaching this
 * header; standalone math builds use the standard assertion service exposed
 * by gmp_type.h (or by the portable contract). */
#ifndef gmp_ctl_assert
#if defined(DISABLE_CTL_LIB_ASSERT)
#define gmp_ctl_assert(assert_condition) ((void)(assert_condition))
#else
#define gmp_ctl_assert(assert_condition) assert(assert_condition)
#endif
#endif

/**
 * @defgroup CTL_MATH Control Math Library
 * @brief A comprehensive library for numerical computation in control systems.
 *
 * This library provides a flexible and extensible set of mathematical tools,
 * including linear algebra (vectors, matrices), complex numbers, quaternions,
 * coordinate transformations, and a type-generic abstraction layer (`ctrl_gt`)
 * that can be mapped to different numerical backends like float, double, or
 * fixed-point libraries.
 */

/**
 * @defgroup MC_CTRL_GT_IMPL ctrl_gt Implementations
 * @ingroup CTL_MATH
 * @brief Different underlying numerical implementations for the `ctrl_gt` type.
 */

/**
 * @defgroup MC_EXT_TYPES CTL Extension Types
 * @ingroup CTL_MATH
 * @brief Extension data type definitions for the math library.
 */

/**
 * @defgroup MC_LINEAR_ALGEBRA Linear Algebra
 * @ingroup CTL_MATH
 * @brief Modules for vector, matrix, complex, and quaternion operations.
 */

/**
 * @defgroup MC_CONSTANTS Mathematical Constants
 * @ingroup CTL_MATH
 * @brief Defines various mathematical and physical constants.
 */

/**
 * @defgroup MC_COORDINATE Coordinate functions
 * @ingroup CTL_MATH
 * @brief Defines coordinate transform functions.
 */

/*---------------------------------------------------------------------------*/
/* Backend Math Library Selection                                            */
/*---------------------------------------------------------------------------*/

// Special support for TI C2000 CLA
#if defined __TMS320C28XX_CLA__
#undef SPECIFY_CTRL_GT_TYPE
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_CLA_LIBRARY
#endif

#if SPECIFY_CTRL_GT_TYPE == USING_FIXED_TI_IQ_LIBRARY
#include <ctl/math_block/ctrl_gt/iqmath_macros.h>
#elif SPECIFY_CTRL_GT_TYPE == USING_FIXED_ARM_CMSIS_Q_LIBRARY
#include <ctl/math_block/ctrl_gt/arm_cmsis_macros.h>
#elif (SPECIFY_CTRL_GT_TYPE == USING_FLOAT_TI_IQ_LIBRRARY)
#include <ctl/math_block/ctrl_gt/float_macros.h>
#elif (SPECIFY_CTRL_GT_TYPE == USING_FLOAT_FPU)
#include <ctl/math_block/ctrl_gt/float_macros.h>
#elif (SPECIFY_CTRL_GT_TYPE == USING_DOUBLE_FPU)
#include <ctl/math_block/ctrl_gt/double_macros.h>
#elif (SPECIFY_CTRL_GT_TYPE == USING_QFPLIB_FLOAT)
#include <ctl/math_block/ctrl_gt/qfp_float_macros.h>
#elif (SPECIFY_CTRL_GT_TYPE == USING_FLOAT_CLA_LIBRARY)
#include <ctl/math_block/ctrl_gt/cla_macros.h>
#elif (SPECIFY_CTRL_GT_TYPE == USING_CSP_MATH_LIBRARY)
#include <csp.math.h>
#else
// Default to standard float implementation
#include <ctl/math_block/ctrl_gt/float_macros.h>
#endif

/*---------------------------------------------------------------------------*/
/* Core Type Definitions                                                     */
/*---------------------------------------------------------------------------*/

/** @addtogroup MC_CORE_TYPES GMP CTL Core Type definition
 * @brief This section provide ctrl_gt and paramter_gt types definition.
 * @ingroup CTL_MATH
 * @{
 */

#if (SPECIFY_CTRL_GT_TYPE == USING_FIXED_TI_IQ_LIBRARY)
#define GMP_PORT_CTRL_T _iq
#elif (SPECIFY_CTRL_GT_TYPE == USING_FLOAT_TI_IQ_LIBRRARY)
#define GMP_PORT_CTRL_T float32_t
#elif (SPECIFY_CTRL_GT_TYPE == USING_DOUBLE_FPU)
#define GMP_PORT_CTRL_T double
#else // Default to float for FPU, CLA, QFP, etc.
#define GMP_PORT_CTRL_T float
#endif

/**
 * @brief Generic data type for control calculations.
 * Its underlying type (e.g., float, double, _iq) is determined at compile time.
 */
typedef GMP_PORT_CTRL_T ctrl_gt;

/**
 * @brief Compile-time capability flag for algorithms that require a floating-point ctrl_gt.
 * @details A CSP-specific math backend may define this macro before including
 * this header. Unknown CSP backends default to non-floating for safety.
 */
#ifndef CTL_CTRL_GT_IS_FLOATING_POINT
#if (SPECIFY_CTRL_GT_TYPE == USING_FIXED_TI_IQ_LIBRARY) ||                                                     \
    (SPECIFY_CTRL_GT_TYPE == USING_FIXED_ARM_CMSIS_Q_LIBRARY) || (SPECIFY_CTRL_GT_TYPE == USING_CSP_MATH_LIBRARY)
#define CTL_CTRL_GT_IS_FLOATING_POINT 0
#else
#define CTL_CTRL_GT_IS_FLOATING_POINT 1
#endif
#endif

#if (SPECIFY_PARAMETER_GT_TYPE == USING_DOUBLE_FPU)
#define GMP_PORT_PARAMETER_T double
#else // Default to float for all other types
#define GMP_PORT_PARAMETER_T float
#endif

/**
 * @brief Generic data type for system parameters.
 * This is typically a high-precision floating-point type.
 */
typedef GMP_PORT_PARAMETER_T parameter_gt;

/**
 * @brief Convert a native C arithmetic value to the parameter domain.
 * @details "real" denotes a C literal or native arithmetic expression at a
 * configuration boundary; it is deliberately not another stored GMP type.
 */
#ifndef real2param
#define real2param(value) ((parameter_gt)(value))
#endif

/** @brief Convert a native C arithmetic value to the real-time domain. */
#ifndef real2ctrl
#define real2ctrl(value) ((ctrl_gt)float2ctrl(value))
#endif

/**
 * @brief Convert an initialization/parameter value into the real-time type.
 * @details Perform this conversion while configuring an object and cache the
 * result as ctrl_gt instead of converting parameter_gt in every ISR step.
 */
#ifndef param2ctrl
#define param2ctrl(value) real2ctrl(value)
#endif

/** @brief Convert a real-time value for slow diagnostics or parameter tools. */
#ifndef ctrl2param
#define ctrl2param(value) ((parameter_gt)ctrl2float(value))
#endif

/* Compatibility spellings retained for existing applications. */
#ifndef parameter2ctrl
#define parameter2ctrl(value) param2ctrl(value)
#endif
#ifndef ctrl2parameter
#define ctrl2parameter(value) ctrl2param(value)
#endif

#define GMP_CTL_MATH_TYPES_AVAILABLE 1

/** @} */ // end of MC_CORE_TYPES group

/*---------------------------------------------------------------------------*/
/* Math Module Includes                                                      */
/*---------------------------------------------------------------------------*/

// Patches and constants
#include <ctl/math_block/parameter_gt/parameter_math.h>
#include <ctl/math_block/const/math_ctrl_const.h>
#include <ctl/math_block/const/math_param_const.h>
#include <ctl/math_block/ctrl_gt/ctrl_gt_patch.h>

// Linear algebra types
#include <ctl/math_block/matrix_lite/matrix2.h>
#include <ctl/math_block/matrix_lite/matrix3.h>
#include <ctl/math_block/matrix_lite/matrix4.h>
#include <ctl/math_block/vector_lite/vector2.h>
#include <ctl/math_block/vector_lite/vector3.h>
#include <ctl/math_block/vector_lite/vector4.h>

#include <ctl/math_block/complex_lite/complex.h>
#include <ctl/math_block/complex_lite/quaternion.h>

// Coordinate transformations
#include <ctl/math_block/coordinate/coord_trans.h>

#endif // _FILE_GMP_CTL_MATH_H_
