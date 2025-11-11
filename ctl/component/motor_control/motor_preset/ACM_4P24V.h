/**
 * @file ACM_4P24V.h
 * @brief Defines the parameters for a specific AC Induction Motor (ACM).
 * @details This file contains the electrical, mechanical, and operational parameters
 * for a 24V, 4-pole AC induction motor. These macros are intended to be used
 * throughout the motor control application to configure various algorithms
 * and safety limits.
 */

#ifndef _FILE_ACM_4P24V_H_
#define _FILE_ACM_4P24V_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Parameter Definitions for AC Induction Motor (4-Pole, 24V)                */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup ACM_4P24V_PARAMETERS Motor Parameters (ACM 4-Pole 24V)
 * @ingroup CTL_MC_PRESET
 * @brief Contains all parameter definitions for the specified AC Induction Motor.
 * @{
 */

//================================================================================
// Motor & System Identification
//================================================================================
#define MOTOR_TYPE   INDUCTION_MOTOR ///< Specifies the motor type as an Induction Motor.
#define ENCODER_TYPE QEP_INC_ENCODER ///< Specifies the type of encoder used with this motor.

//================================================================================
// Electrical Parameters
//================================================================================
#define MOTOR_PARAM_RS  ((0.329))  ///< Stator resistance per phase (Ohm).
#define MOTOR_PARAM_L1S ((0.0003)) ///< Stator leakage inductance (H).
#define MOTOR_PARAM_RR  ((0.44))   ///< Rotor resistance per phase, referred to the stator (Ohm).
#define MOTOR_PARAM_L1R ((0.0005)) ///< Rotor leakage inductance, referred to the stator (H).
#define MOTOR_PARAM_LM  ((0.0012)) ///< Main magnetizing inductance (H).

//================================================================================
// Mechanical Parameters
//================================================================================
#define MOTOR_PARAM_POLE_PAIRS ((2))       ///< Number of pole pairs in the motor.
#define MOTOR_PARAM_INERTIA    ((0.001))   ///< Total rotor inertia (kg*m^2).
#define MOTOR_PARAM_FRICTION   ((0.00001)) ///< Viscous friction coefficient (N*m*s/rad).

//================================================================================
// Rated Operating Parameters
//================================================================================
#define MOTOR_PARAM_RATED_VOLTAGE   ((24.0)) ///< Rated line-to-line voltage (V).
#define MOTOR_PARAM_RATED_CURRENT   ((8.5))  ///< Rated phase current (A, RMS).
#define MOTOR_PARAM_NO_LOAD_CURRENT ((0.5))  ///< No-load phase current (A, RMS).
#define MOTOR_PARAM_RATED_FREQUENCY ((50.0)) ///< Rated operating frequency (Hz).
#define MOTOR_PARAM_MAX_SPEED       ((1450)) ///< Rated (or maximum continuous) speed (RPM).

//================================================================================
// Absolute Maximum Ratings & Limits
//================================================================================
#define MOTOR_PARAM_MAX_TORQUE     ((0.981)) ///< Maximum intermittent torque (N*m).
#define MOTOR_PARAM_MAX_DC_VOLTAGE ((36.0))  ///< Maximum allowable DC bus voltage (V).
#define MOTOR_PARAM_MAX_PH_CURRENT ((10.0))  ///< Maximum allowable phase current (A, RMS).

/** @} */ // end of ACM_4P24V_PARAMETERS group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ACM_4P24V_H_
