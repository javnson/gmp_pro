/**
 * @file PMSRM_4P_15KW520V.h
 * @brief Defines parameters for a 15kW Permanent Magnet Synchronous Reluctance Motor (PMSRM).
 * @details This file contains the electrical, mechanical, and operational parameters
 * for a specific high-power, salient-pole PMSM. These macros are intended
 * to be used throughout the motor control application to configure various
 * algorithms (like MTPA) and safety limits.
 *
 * @note Information Source: Lin Minyi (lammanyee@zju.edu.cn)
 */

#ifndef _FILE_PMSRM_4P_15KW520V_H_
#define _FILE_PMSRM_4P_15KW520V_H_

#include <ctl/component/motor_control/basic/motor_unit_calculator.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Parameter Definitions for PMSRM (4-Pole, 15kW, 520V)                      */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSRM_4P_15KW_PARAMETERS Motor Parameters (PMSRM 4-Pole 15kW)
 * @brief Contains all parameter definitions for the specified PMSRM.
 * @{
 */

//================================================================================
// Motor & System Identification
//================================================================================
#define MOTOR_TYPE         PMSM_MOTOR ///< Specifies the motor type as a Permanent Magnet Synchronous Motor.
#define MOTOR_ENCODER_TYPE RS485_ABS  ///< Specifies the encoder type as an RS485 absolute encoder.

//================================================================================
// Electrical Parameters
//================================================================================
#define MOTOR_PARAM_RS   ((0.1215))   ///< Stator resistance per phase (Ohm).
#define MOTOR_PARAM_LD   ((3.03e-3))  ///< D-axis inductance (H).
#define MOTOR_PARAM_LQ   ((20.19e-3)) ///< Q-axis inductance (H). Note: Lq > Ld indicates a salient-pole machine.
#define MOTOR_PARAM_FLUX ((0.44))     ///< Permanent magnet flux linkage (Wb).

//================================================================================
// Mechanical Parameters
//================================================================================
#define MOTOR_PARAM_POLE_PAIRS ((2))     ///< Number of pole pairs in the motor.
#define MOTOR_PARAM_INERTIA    ((0.018)) ///< Total rotor inertia (kg*m^2).

//================================================================================
// Characteristic Constants (Calculated)
//================================================================================
/**
 * @brief Motor velocity constant (RPM/V), calculated from flux linkage.
 */
#define MOTOR_PARAM_KV ((MOTOR_PARAM_CALCULATE_KV_BY_FLUX(MOTOR_PARAM_FLUX, MOTOR_PARAM_POLE_PAIRS)))

/**
 * @brief Back-EMF constant (V_LN_RMS / kRPM), calculated from flux linkage.
 */
#define MOTOR_PARAM_EMF ((MOTOR_PARAM_CALCULATE_EMF_BY_FLUX(MOTOR_PARAM_FLUX, MOTOR_PARAM_POLE_PAIRS)))

//================================================================================
// Rated Operating Parameters
//================================================================================
#define MOTOR_PARAM_RATED_VOLTAGE   ((500.0)) ///< Rated operating voltage (V).
#define MOTOR_PARAM_RATED_CURRENT   ((32.09)) ///< Rated phase current (A, Peak).
#define MOTOR_PARAM_RATED_FREQUENCY ((100.0)) ///< Rated operating frequency at rated speed (Hz).
#define MOTOR_PARAM_RATED_SPEED     ((3000))  ///< Rated operating speed (RPM).
#define MOTOR_PARAM_RATED_TORQUE    ((47.8))  ///< Rated continuous torque (N*m).

//================================================================================
// Absolute Maximum Ratings & Limits
//================================================================================
#define MOTOR_PARAM_MAX_SPEED      ((3000)) ///< Maximum allowable speed (RPM).
#define MOTOR_PARAM_MAX_PH_CURRENT ((20.0)) ///< Maximum allowable phase current (A, Peak).
/**
 * @warning The maximum DC bus voltage is defined as 150V, which seems unusually
 * low for a motor rated at 500V. Please verify this value.
 */
#define MOTOR_PARAM_MAX_DC_VOLTAGE ((150.0))

/** @} */ // end of PMSRM_4P_15KW_PARAMETERS group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSRM_4P_15KW520V_H_
