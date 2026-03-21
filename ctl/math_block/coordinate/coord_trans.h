/**
 * @file coord_trans.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides coordinate transformation functions for motor control.
 * @version 1.1
 * @date 2025-07-23
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file contains essential coordinate transformations used in Field-Oriented
 * Control (FOC), including Clarke, Park, and their inverse transformations, as
 * well as SVPWM calculation routines.
 */

#ifndef _FILE_COORD_TRANS_H_
#define _FILE_COORD_TRANS_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup MC_COORD_TRANSFORMATIONS Coordinate Transformations
 * @ingroup CTL_MATH
 * @brief A collection of functions for coordinate system transformations.
 * @{
 */

	#include <ctl/math_block/coordinate/coordinate.h>
	#include <ctl/math_block/coordinate/Clarke.h>
	#include <ctl/math_block/coordinate/Park.h>
	#include <ctl/math_block/coordinate/Park_neg.h>
	#include <ctl/math_block/coordinate/svpwm.h>


/** @} */ // end of MC_COORD_TRANSFORMATIONS group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_COORD_TRANS_H_
