/**
 * @file TYI_5008_KV335.h
 * @brief Defines the parameters for the TYI-5008 KV335 Brushless Motor (PMSM).
 * @details This file contains the electrical, mechanical, and operational parameters
 * for the specified Permanent Magnet Synchronous Motor, commonly used in UAVs
 * and gimbals. These macros are intended to be used throughout the motor
 * control application to configure various algorithms and safety limits.
 */

#ifndef _FILE_MOTOR_PARAM_TYI5008_KV335_H_
#define _FILE_MOTOR_PARAM_TYI5008_KV335_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Parameter Definitions for PMSM (TYI-5008 KV335)                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSM_TYI5008_PARAMETERS Motor Parameters (TYI-5008 KV335)
 * @ingroup CTL_MC_PRESET
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
#define MOTOR_PARAM_RS   ((65.0e-3f)) ///< Stator resistance per phase (Ohm).
#define MOTOR_PARAM_LS   ((25.0e-6f)) ///< Stator inductance per phase (H). Assumes Ld = Lq.
#define MOTOR_PARAM_FLUX ((0.0023f))  ///< Permanent magnet flux linkage (Wb).

//================================================================================
// Mechanical Parameters
//================================================================================
#define MOTOR_PARAM_POLE_PAIRS ((7)) ///< Number of pole pairs in the motor.

//================================================================================
// Characteristic Constants
//================================================================================
/**
 * @brief Motor velocity constant (RPM/V).
 * @note This value may not be perfectly consistent with the provided flux linkage
 * under all measurement conditions.
 */
#define MOTOR_PARAM_KV ((335.0f))

/**
 * @brief Back-EMF constant (V/kRPM).
 * @note This value may not be perfectly consistent with the provided flux linkage.
 */
#define MOTOR_PARAM_EMF ((4.85f))

//================================================================================
// Rated Operating Parameters
//================================================================================
#define MOTOR_PARAM_RATED_VOLTAGE   ((24.0f)) ///< Rated operating voltage (V).
#define MOTOR_PARAM_NO_LOAD_CURRENT ((0.5f))  ///< No-load phase current (A, Peak).
#define MOTOR_PARAM_RATED_FREQUENCY ((933..33333f))

//================================================================================
// Absolute Maximum Ratings & Limits
//================================================================================
#define MOTOR_PARAM_MAX_SPEED      ((8000)) ///< Maximum allowable speed (RPM).
#define MOTOR_PARAM_MAX_TORQUE     ((0.4))  ///< Maximum intermittent torque (N*m).
#define MOTOR_PARAM_MAX_DC_VOLTAGE ((25.2)) ///< Maximum allowable DC bus voltage (V), likely for a 6S LiPo battery.
#define MOTOR_PARAM_MAX_PH_CURRENT ((27.0)) ///< Maximum allowable phase current (A, Peak).

/** @} */ // end of PMSM_TYI5008_PARAMETERS group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif //_FILE_MOTOR_PARAM_TYI5008_KV335_H_
