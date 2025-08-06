/**
 * @file motor_unit_calculator.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines constants and macros for motor parameter calculations.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a collection of predefined constants and helper macros
 * for motor types, encoder types, and physical parameter conversions.
 */

#ifndef _FILE_MOTOR_UNIT_CALCULATOR_H_
#define _FILE_MOTOR_UNIT_CALCULATOR_H_

/**
 * @defgroup MC_DEFINES Motor Control Definitions
 * @brief A collection of common definitions, constants, and macros for motor control.
 */

/*---------------------------------------------------------------------------*/
/* Motor Type Definitions                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_MOTOR_TYPES Motor Type Definitions
 * @ingroup MC_DEFINES
 * @brief Defines unique identifiers for different types of motors.
 * @{
 */

#define PMSM_MOTOR      ((1)) /**< @brief Identifier for Permanent Magnet Synchronous Motors (PMSM). */
#define INDUCTION_MOTOR ((2)) /**< @brief Identifier for Induction Motors. */

/** @} */ // end of MC_MOTOR_TYPES group

/*---------------------------------------------------------------------------*/
/* Encoder Type Definitions                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ENCODER_TYPES Encoder Type Definitions
 * @ingroup MC_DEFINES
 * @brief Defines unique identifiers for different types of encoders.
 * @{
 */

#define QEP_INC_ENCODER ((1)) /**< @brief Identifier for Quadrature Encoder Pulse (QEP) incremental encoders. */

/** @} */ // end of MC_ENCODER_TYPES group

/*---------------------------------------------------------------------------*/
/* Physical Constants and Formulas                                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_PHYSICAL_CONSTANTS Physical Constants and Formulas
 * @ingroup MC_DEFINES
 * @brief Defines physical constants and macros for parameter calculations.
 * @{
 */

/**
 * @brief The constant value for the square root of 3.
 */
#define MTR_CONST_SQRT_3 (1.732050807568877)

/**
 * @brief Calculates the motor velocity constant (Kv) from flux.
 *
 * @note This macro depends on a `PI` constant which must be defined elsewhere.
 * The units of the parameters must be consistent for a meaningful result.
 *
 * @param flux The magnetic flux linkage in Weber (Wb).
 * @param pole_pair The number of motor pole pairs.
 * @return The calculated Kv value (typically in RPM/Volt).
 */
#define MOTOR_PARAM_CALCULATE_KV_BY_FLUX(flux, pole_pair) ((flux * 10.0 * MTR_CONST_SQRT_3 / PI / pole_pair))

/**
 * @brief Calculates the motor's magnetic flux linkage from the velocity constant (Kv).
 *
 * @note This macro depends on a `PI` constant which must be defined elsewhere.
 * The units of the parameters must be consistent for a meaningful result.
 *
 * @param Kv The motor velocity constant (typically in RPM/Volt).
 * @param pole_pair The number of motor pole pairs.
 * @return The calculated magnetic flux linkage in Weber (Wb).
 */
#define MOTOR_PARAM_CALCULATE_FLUX_BY_KV(Kv, pole_pair) ((Kv * PI * pole_pair / 10.0 / MTR_CONST_SQRT_3))

/** @} */ // end of MC_PHYSICAL_CONSTANTS group

#endif // _FILE_MOTOR_UNIT_CALCULATOR_H_
