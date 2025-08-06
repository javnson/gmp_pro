/**
 * @file math_param_const.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a set of common mathematical constants with the 'parameter_gt' type.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a centralized set of high-precision mathematical constants,
 * typed as `parameter_gt` for use in various calculations.
 */

#ifndef FILE_MATH_CONST_PARAM_H_
#define FILE_MATH_CONST_PARAM_H_

/*---------------------------------------------------------------------------*/
/* Typed Mathematical Constants                                              */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_PARAM_CONSTANTS Typed Mathematical Constants
 * @ingroup MC_CONSTANTS
 * @brief A collection of common mathematical constants cast to the `parameter_gt` type.
 *
 * These constants are provided to ensure type consistency in floating-point
 * calculations throughout the library.
 * @{
 */

#ifndef CTL_PARAM_CONST_PI
#define CTL_PARAM_CONST_PI ((parameter_gt)(3.141592653589793)) /**< @brief The constant Pi (¦Ð). */
#endif

#ifndef CTL_PARAM_CONST_E
#define CTL_PARAM_CONST_E ((parameter_gt)(2.718281828459045)) /**< @brief Euler's number (e). */
#endif

#ifndef CTL_PARAM_CONST_SQRT2
#define CTL_PARAM_CONST_SQRT2 ((parameter_gt)(1.414213562373095)) /**< @brief The square root of 2. */
#endif

#ifndef CTL_PARAM_CONST_SQRT3
#define CTL_PARAM_CONST_SQRT3 ((parameter_gt)(1.732050807568877)) /**< @brief The square root of 3. */
#endif

#ifndef CTL_PARAM_CONST_1_OVER_SQRT2
#define CTL_PARAM_CONST_1_OVER_SQRT2 ((parameter_gt)(0.7071067811865475)) /**< @brief The constant 1/sqrt(2). */
#endif

#ifndef CTL_PARAM_CONST_LN2
#define CTL_PARAM_CONST_LN2 ((parameter_gt)(0.6931471805599453)) /**< @brief The natural logarithm of 2. */
#endif

#ifndef CTL_PARAM_CONST_LN10
#define CTL_PARAM_CONST_LN10 ((parameter_gt)(2.302585092994046)) /**< @brief The natural logarithm of 10. */
#endif

#ifndef CTL_PARAM_CONST_GOLDEN_RATIO
#define CTL_PARAM_CONST_GOLDEN_RATIO ((parameter_gt)(1.618033988749895)) /**< @brief The golden ratio (¦Õ). */
#endif

/** @} */ // end of MC_PARAM_CONSTANTS group

#endif // FILE_MATH_CONST_PARAM_H_
