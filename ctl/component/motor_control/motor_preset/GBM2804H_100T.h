/**
 * @file GBM2804H_100T.h
 * @brief Defines the parameters for the GBM2804H-100T Brushless Gimbal Motor (PMSM).
 * @details This file contains the electrical, mechanical, and operational parameters
 * for the specified Permanent Magnet Synchronous Motor. These macros are intended
 * to be used throughout the motor control application to configure various
 * algorithms and safety limits.
 *
 * @note Information Source: https://zhuanlan.zhihu.com/p/545688192
 */

#ifndef _FILE_MOTOR_PARAM_GBM2804H_100T_H_
#define _FILE_MOTOR_PARAM_GBM2804H_100T_H_

#include <ctl/component/motor_control/basic/motor_unit_calculator.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Parameter Definitions for PMSM (GBM2804H-100T)                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSM_GBM2804H_PARAMETERS Motor Parameters (GBM2804H-100T)
 * @brief Contains all parameter definitions for the specified PMSM.
 * @{
 */

//================================================================================
// Motor & System Identification
//================================================================================
#define MOTOR_TYPE PMSM_MOTOR ///< Specifies the motor type as a Permanent Magnet Synchronous Motor.

//================================================================================
// Electrical Parameters
//================================================================================
#define MOTOR_PARAM_RS ((4.7))     ///< Stator resistance per phase (Ohm).
#define MOTOR_PARAM_LS ((0.96e-3)) ///< Stator inductance per phase (H). Note: Ld = Lq = Ls for a non-salient PMSM.
#define MOTOR_PARAM_FLUX                                                                                               \
    ((MOTOR_PARAM_CALCULATE_FLUX_BY_KV(                                                                                \
        MOTOR_PARAM_KV, MOTOR_PARAM_POLE_PAIRS))) ///< Permanent magnet flux linkage (Wb), calculated from Kv.

//================================================================================
// Mechanical Parameters
//================================================================================
#define MOTOR_PARAM_POLE_PAIRS ((7))     ///< Number of pole pairs in the motor.
#define MOTOR_PARAM_INERTIA    ((497.0)) ///< Total rotor inertia (g*cm^2).
#define MOTOR_PARAM_FRICTION   ((755.0)) ///< Viscous friction coefficient (uN*m*s/rad).

//================================================================================
// Characteristic Constants
//================================================================================
#define MOTOR_PARAM_KV  ((206.2)) ///< Motor velocity constant (RPM/V).
#define MOTOR_PARAM_EMF ((4.85))  ///< Back-EMF constant (V/kRPM).

//================================================================================
// Rated Operating Parameters
//================================================================================
#define MOTOR_PARAM_RATED_VOLTAGE   ((10.0))  ///< Rated operating voltage (V).
#define MOTOR_PARAM_RATED_CURRENT   ((4.5))   ///< Rated phase current (A, Peak).
#define MOTOR_PARAM_NO_LOAD_CURRENT ((0.01))  ///< No-load phase current (A, Peak).
#define MOTOR_PARAM_RATED_FREQUENCY ((250.0)) ///< Rated operating frequency (Hz).

//================================================================================
// Absolute Maximum Ratings & Limits
//================================================================================
#define MOTOR_PARAM_MAX_SPEED      ((2180))  ///< Maximum allowable speed (RPM).
#define MOTOR_PARAM_MAX_TORQUE     ((0.981)) ///< Maximum intermittent torque (N*m).
#define MOTOR_PARAM_MAX_DC_VOLTAGE ((14.2))  ///< Maximum allowable DC bus voltage (V).
#define MOTOR_PARAM_MAX_PH_CURRENT ((5.0))   ///< Maximum allowable phase current (A, Peak).

/** @} */ // end of PMSM_GBM2804H_PARAMETERS group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MOTOR_PARAM_GBM2804H_100T_H_
