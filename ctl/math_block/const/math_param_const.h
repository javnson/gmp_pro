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

#define CTL_PARAM_CONST_ZERO             real2param(0.0)
#define CTL_PARAM_CONST_1                real2param(1.0)
#define CTL_PARAM_CONST_2                real2param(2.0)
#define CTL_PARAM_CONST_3                real2param(3.0)
#define CTL_PARAM_CONST_4                real2param(4.0)
#define CTL_PARAM_CONST_5                real2param(5.0)
#define CTL_PARAM_CONST_7                real2param(7.0)
#define CTL_PARAM_CONST_9                real2param(9.0)
#define CTL_PARAM_CONST_10               real2param(10.0)
#define CTL_PARAM_CONST_11               real2param(11.0)
#define CTL_PARAM_CONST_13               real2param(13.0)
#define CTL_PARAM_CONST_15               real2param(15.0)
#define CTL_PARAM_CONST_1_OVER_2         real2param(0.5)
#define CTL_PARAM_CONST_1_OVER_3         real2param(0.333333333333333333333333333333)
#define CTL_PARAM_CONST_1_OVER_4         real2param(0.25)
#define CTL_PARAM_CONST_1_OVER_10        real2param(0.1)
#define CTL_PARAM_CONST_1_OVER_20        real2param(0.05)
#define CTL_PARAM_CONST_2_OVER_3         real2param(0.666666666666666666666666666667)
#define CTL_PARAM_CONST_9_OVER_10        real2param(0.9)
#define CTL_PARAM_CONST_3_OVER_2         real2param(1.5)
#define CTL_PARAM_CONST_PI               real2param(3.141592653589793238462643383279)
#define CTL_PARAM_CONST_2PI              real2param(6.283185307179586476925286766559)
#define CTL_PARAM_CONST_1_OVER_2PI       real2param(0.159154943091895335768883763373)
#define CTL_PARAM_CONST_2_OVER_PI        real2param(0.636619772367581343075535053490)
#define CTL_PARAM_CONST_E                real2param(2.718281828459045235360287471353)
#define CTL_PARAM_CONST_SQRT2            real2param(1.414213562373095048801688724210)
#define CTL_PARAM_CONST_1_OVER_SQRT2     real2param(0.707106781186547524400844362105)
#define CTL_PARAM_CONST_SQRT3            real2param(1.732050807568877293527446341506)
#define CTL_PARAM_CONST_1_OVER_SQRT3     real2param(0.577350269189625764509148780502)
#define CTL_PARAM_CONST_SQRT3_OVER_2     real2param(0.866025403784438646763723170753)
#define CTL_PARAM_CONST_SQRT3_OVER_SQRT2 real2param(1.224744871391589049098642037353)
#define CTL_PARAM_CONST_2_SQRT_6         real2param(4.898979485566356196394568149412)
#define CTL_PARAM_CONST_100PI_OVER_3     real2param(104.71975511965977461542144610932)
#define CTL_PARAM_CONST_LN2              real2param(0.693147180559945309417232121458)
#define CTL_PARAM_CONST_LN10             real2param(2.302585092994045684017991454684)
#define CTL_PARAM_CONST_GOLDEN_RATIO     real2param(1.618033988749894848204586834366)

/* Default zero-comparison threshold for the selected parameter domain. */
#if (SPECIFY_PARAMETER_GT_TYPE == USING_DOUBLE_FPU)
#define CTL_PARAM_CONST_EPSILON real2param(1.0e-12)
#else
#define CTL_PARAM_CONST_EPSILON real2param(1.0e-6)
#endif

/* Historical names retained for generated and downstream projects. */
#define CTL_CONST_PARAM_3_OVER_2     CTL_PARAM_CONST_3_OVER_2
#define CTL_CONST_PARAM_2_SQRT_6     CTL_PARAM_CONST_2_SQRT_6
#define CTL_CONST_PARAM_100PI_OVER_3 CTL_PARAM_CONST_100PI_OVER_3
#define CTL_CONST_PARAM_SQRT_2       CTL_PARAM_CONST_SQRT2

/** @} */ // end of MC_PARAM_CONSTANTS group

#endif // FILE_MATH_CONST_PARAM_H_
