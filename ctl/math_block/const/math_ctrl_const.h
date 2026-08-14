/**
 * @file math_ctrl_const.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a set of common mathematical constants for motor control algorithms.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file centralizes frequently used mathematical and physical constants
 * to ensure consistency and precision across the control library.
 */

#ifndef _FILE_FIXED_CONST_PARAM_H_
#define _FILE_FIXED_CONST_PARAM_H_

/*---------------------------------------------------------------------------*/
/* Mathematical Constants                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_MATH_CONSTANTS Mathematical Constants
 * @ingroup MC_CONSTANTS
 * @brief A collection of common mathematical constants used in motor control.
 * @{
 */

// --- General Purpose Constants ---

#define CTL_CTRL_CONST_ZERO          real2ctrl(0.0)
#define CTL_CTRL_CONST_1             real2ctrl(1.0)
#define CTL_CTRL_CONST_2             real2ctrl(2.0)
#define CTL_CTRL_CONST_3             real2ctrl(3.0)
#define CTL_CTRL_CONST_4             real2ctrl(4.0)
#define CTL_CTRL_CONST_1_OVER_2      real2ctrl(0.5)
#define CTL_CTRL_CONST_1_OVER_3      real2ctrl(0.333333333333333333333333333333)
#define CTL_CTRL_CONST_1_OVER_4      real2ctrl(0.25)
#define CTL_CTRL_CONST_1_OVER_10     real2ctrl(0.1)
#define CTL_CTRL_CONST_2_OVER_3      real2ctrl(0.666666666666666666666666666667)
#define CTL_CTRL_CONST_3_OVER_2      real2ctrl(1.5)
#define CTL_CTRL_CONST_PI            real2ctrl(3.141592653589793238462643383279)
#define CTL_CTRL_CONST_2_PI          real2ctrl(6.283185307179586476925286766559)
#define CTL_CTRL_CONST_1_OVER_2PI    real2ctrl(0.159154943091895335768883763373)
#define CTL_CTRL_CONST_2_OVER_PI     real2ctrl(0.636619772367581343075535053490)
#define CTL_CTRL_CONST_E             real2ctrl(2.718281828459045235360287471353)
#define CTL_CTRL_CONST_SQRT_2        real2ctrl(1.414213562373095048801688724210)
#define CTL_CTRL_CONST_1_OVER_SQRT2  real2ctrl(0.707106781186547524400844362105)
#define CTL_CTRL_CONST_SQRT_3        real2ctrl(1.732050807568877293527446341506)
#define CTL_CTRL_CONST_1_OVER_SQRT3  real2ctrl(0.577350269189625764509148780502)
#define CTL_CTRL_CONST_SQRT_3_OVER_2 real2ctrl(0.866025403784438646763723170753)

/* Default zero-comparison threshold for the selected control domain. */
#if (SPECIFY_CTRL_GT_TYPE == USING_FIXED_TI_IQ_LIBRARY)
#define CTL_CTRL_CONST_EPSILON ((ctrl_gt)1)
#elif (SPECIFY_CTRL_GT_TYPE == USING_DOUBLE_FPU)
#define CTL_CTRL_CONST_EPSILON real2ctrl(1.0e-12)
#else
#define CTL_CTRL_CONST_EPSILON real2ctrl(1.0e-6)
#endif

#ifndef CTL_EPSILON
#define CTL_EPSILON CTL_CTRL_CONST_EPSILON
#endif

// --- Clarke/Park Transformation Constants ---

/**
 * @brief Constant for Clarke transform (ABC to Alpha): 2/3.
 */
#define CTL_CTRL_CONST_ABC2AB_ALPHA CTL_CTRL_CONST_2_OVER_3

/**
 * @brief Constant for Clarke transform (ABC to Beta): 1/sqrt(3).
 */
#define CTL_CTRL_CONST_ABC2AB_BETA CTL_CTRL_CONST_1_OVER_SQRT3

/**
 * @brief Constant for Clarke transform (zero sequence): 1/3.
 */
#define CTL_CTRL_CONST_ABC2AB_GAMMA CTL_CTRL_CONST_1_OVER_3

/**
 * @brief Constant for inverse Clarke transform (Alpha/Beta to ABC): sqrt(3)/2.
 */
#define CTL_CTRL_CONST_AB2ABC_ALPHA CTL_CTRL_CONST_SQRT_3_OVER_2

/**
 * @brief Constant for power-invariant Clarke transform: 2/sqrt(3).
 */
#define CTL_CTRL_CONST_AB02AB_ALPHA real2ctrl(1.154700538379251529018297561003)

/** @} */ // end of MC_MATH_CONSTANTS group

#endif // _FILE_FIXED_CONST_PARAM_H_
