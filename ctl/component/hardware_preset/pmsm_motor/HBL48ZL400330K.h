/**
 * @file HBL48ZL400330K.h
 * @brief Defines the parameters for the HBL48ZL400330K Brushless DC Motor (PMSM).
 * @details This file contains the electrical, mechanical, and operational parameters
 * for the specified Permanent Magnet Synchronous Motor. These macros are intended
 * to be used throughout the motor control application to configure various
 * algorithms and safety limits.
 */

#ifndef _FILE_HBL48ZL400330K_H_
#define _FILE_HBL48ZL400330K_H_

#include <ctl/component/motor_control/basic/motor_unit_calculator.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Parameter Definitions for PMSM (HBL48ZL400330K)                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSM_HBL48ZL400330K_PARAMETERS Motor Parameters (HBL48ZL400330K)
 * @ingroup CTL_MC_PRESET
 * @brief Contains all parameter definitions for the specified PMSM.
 * @{
 */

//================================================================================
// Motor & System Identification
//================================================================================
#define MOTOR_TYPE                PMSM_MOTOR  ///< Specifies the motor type as a Permanent Magnet Synchronous Motor.
#define MOTOR_ENCODER_TYPE        QEP_ENCODER ///< Specifies the type of encoder used with this motor.
#define MOTOR_ENCODER_LINE_NUMBER ((2500))    ///< The number of lines (PPR) for the quadrature encoder.

//================================================================================
// Electrical Parameters
//================================================================================
/**
 * @brief Stator resistance per phase (Ohm).
 * @note The datasheet value of 0.4 Ohm is likely line-to-line, so it is divided by 2
 * to get the phase resistance for the star-connected model.
 */
#define MOTOR_PARAM_RS ((0.4 / 2.0))

/**
 * @brief Stator inductance per phase (H).
 * @note The datasheet value of 0.6mH is likely line-to-line, so it is divided by 2
 * to get the phase inductance for the star-connected model.
 */
#define MOTOR_PARAM_LS ((0.6e-3 / 2.0))

/**
 * @brief Permanent magnet flux linkage (Wb).
 * @note This value is calculated from the back-EMF constant.
 */
#define MOTOR_PARAM_FLUX ((MOTOR_PARAM_CALCULATE_FLUX_BY_EMF(MOTOR_PARAM_EMF)))

//================================================================================
// Mechanical Parameters
//================================================================================
#define MOTOR_PARAM_POLE_PAIRS ((8))    ///< Number of pole pairs in the motor.
#define MOTOR_PARAM_INERTIA    ((0.45)) ///< Total rotor inertia (kg*cm^2).
#define MOTOR_PARAM_FRICTION   ((0.55)) ///< Viscous friction coefficient (mN*m*s/rad).

//================================================================================
// Characteristic Constants
//================================================================================
#define MOTOR_PARAM_EMF ((10.5)) ///< Back-EMF constant (V_LN_RMS / kRPM).

//================================================================================
// Rated Operating Parameters
//================================================================================
#define MOTOR_PARAM_RATED_VOLTAGE   ((48.0)) ///< Rated operating voltage (V).
#define MOTOR_PARAM_RATED_SPEED     ((3000)) ///< Rated operating speed (RPM).
#define MOTOR_PARAM_RATED_CURRENT   ((11.0)) ///< Rated phase current (A, Peak).
#define MOTOR_PARAM_RATED_TORQUE    ((1.27)) ///< Rated continuous torque (N*m).
#define MOTOR_PARAM_NO_LOAD_CURRENT ((0.25)) ///< No-load phase current (A, Peak).

//================================================================================
// Absolute Maximum Ratings & Limits
//================================================================================
#define MOTOR_PARAM_MAX_SPEED      ((3500)) ///< Maximum allowable speed (RPM).
#define MOTOR_PARAM_MAX_TORQUE     ((3.81)) ///< Maximum intermittent torque (N*m).
#define MOTOR_PARAM_MAX_DC_VOLTAGE ((54.0)) ///< Maximum allowable DC bus voltage (V).
#define MOTOR_PARAM_MAX_PH_CURRENT ((33.0)) ///< Maximum allowable phase current (A, Peak).

/** @} */ // end of PMSM_HBL48ZL400330K_PARAMETERS group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_HBL48ZL400330K_H_
