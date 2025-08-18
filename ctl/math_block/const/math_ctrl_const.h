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

#define CTL_CTRL_CONST_PI            (float2ctrl(3.1415926535897932))     /**< @brief The constant Pi (¦Ð). */
#define CTL_CTRL_CONST_2_PI          (float2ctrl(6.2831853071795865))     /**< @brief The constant 2*Pi (2¦Ð). */
#define CTL_CTRL_CONST_1_OVER_2PI    (float2ctrl(1 / 6.2831853071795865)) /**< @brief The constant 1/(2*Pi). */
#define CTL_CTRL_CONST_SQRT_3        (float2ctrl(1.73205080756888))       /**< @brief The constant sqrt(3). */
#define CTL_CTRL_CONST_1_OVER_SQRT3  (float2ctrl(0.5773502691896))        /**< @brief The constant 1/sqrt(3). */
#define CTL_CTRL_CONST_SQRT_3_OVER_2 (float2ctrl(0.8660254038))           /**< @brief The constant sqrt(3)/2. */
#define CTL_CTRL_CONST_1             (float2ctrl(1.0))                    /**< @brief The constant 1.0. */
#define CTL_CTRL_CONST_1_OVER_2      (float2ctrl(0.5))                    /**< @brief The constant 0.5. */
#define CTL_CTRL_CONST_3_OVER_2      (float2ctrl(1.5))                    /**< @brief The constant 1.5. */

// --- Clarke/Park Transformation Constants ---

/**
 * @brief Constant for Clarke transform (ABC to Alpha): 2/3.
 */
#define CTL_CTRL_CONST_ABC2AB_ALPHA (float2ctrl(0.666666666666667))

/**
 * @brief Constant for Clarke transform (ABC to Beta): 1/sqrt(3).
 */
#define CTL_CTRL_CONST_ABC2AB_BETA (float2ctrl(0.577350269189626))

/**
 * @brief Constant for Clarke transform (zero sequence): 1/3.
 */
#define CTL_CTRL_CONST_ABC2AB_GAMMA (float2ctrl(0.333333333333334))

/**
 * @brief Constant for inverse Clarke transform (Alpha/Beta to ABC): sqrt(3)/2.
 */
#define CTL_CTRL_CONST_AB2ABC_ALPHA (float2ctrl(0.8660254))

/**
 * @brief Constant for power-invariant Clarke transform: 2/sqrt(3).
 */
#define CTL_CTRL_CONST_AB02AB_ALPHA (float2ctrl(1.154700538379252))

/** @} */ // end of MC_MATH_CONSTANTS group

#endif // _FILE_FIXED_CONST_PARAM_H_
