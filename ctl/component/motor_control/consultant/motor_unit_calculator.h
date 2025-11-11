/**
 * @file motor_unit_calculator.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines constants and macros for motor parameter and unit conversions.
 * @details This file provides a collection of predefined constants, motor/encoder type
 * definitions, and helper macros for physical parameter and unit conversions essential
 * for motor control algorithms.
 * @version 0.2
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_MOTOR_UNIT_CALCULATOR_H_
#define _FILE_MOTOR_UNIT_CALCULATOR_H_

/**
 * @defgroup MC_DEFINES Motor Control Definitions
 * @ingroup CTL_MC_COMPONENT
 * @brief A collection of common definitions, constants, and macros for motor control.
 */

/*---------------------------------------------------------------------------*/
/* Motor & Encoder Type Definitions                                          */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_TYPES Motor & Encoder Type Definitions
 * @ingroup MC_DEFINES
 * @brief Defines unique identifiers for different types of motors and encoders.
 * @{
 */

#define PMSM_MOTOR      (1) /**< @brief Identifier for Permanent Magnet Synchronous Motors (PMSM). */
#define INDUCTION_MOTOR (2) /**< @brief Identifier for Induction Motors. */

#define QEP_INC_ENCODER (1) /**< @brief Identifier for Quadrature Encoder Pulse (QEP) incremental encoders. */

/** @} */ // end of MC_TYPES group

/*---------------------------------------------------------------------------*/
/* Physical Constants and Unit Conversion Formulas                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_PHYSICAL_UNITS Physical Constants and Unit Conversions
 * @ingroup MC_DEFINES
 * @brief Defines physical constants and macros for unit and parameter calculations.
 * @{
 */

// --- Mathematical and Physical Constants ---
//#define M_PI          (3.141592653589793) /**< @brief The constant PI. */
//#define M_2_PI        (6.283185307179586) /**< @brief The constant 2*PI. */
//#define M_1_DIV_SQRT3 (0.577350269189626) /**< @brief The constant 1/sqrt(3). */
//#define M_SQRT3       (1.732050807568877) /**< @brief The constant sqrt(3). */
//#define M_2_DIV_3     (0.666666666666667) /**< @brief The constant 2/3. */

// --- Angular Velocity Conversions ---

/**
 * @brief Converts mechanical rotational speed from RPM (Revolutions Per Minute) to rad/s.
 * @details Formula: rad/s = RPM * 2 * PI / 60
 * @param rpm Speed in RPM.
 * @return Speed in rad/s.
 */
#define MTR_RPM_TO_RADS(rpm) ((rpm) * M_2_PI / 60.0)

/**
 * @brief Converts mechanical rotational speed from rad/s to RPM (Revolutions Per Minute).
 * @details Formula: RPM = rad/s * 60 / (2 * PI)
 * @param rads Speed in rad/s.
 * @return Speed in RPM.
 */
#define MTR_RADS_TO_RPM(rads) ((rads) * 60.0 / M_2_PI)

/**
 * @brief Converts mechanical rotational speed from Hz (Revolutions Per Second) to RPM.
 * @details Formula: RPM = Hz * 60
 * @param hz Speed in Hz.
 * @return Speed in RPM.
 */
#define MTR_HZ_TO_RPM(hz) ((hz) * 60.0)

/**
 * @brief Converts mechanical rotational speed from RPM to Hz (Revolutions Per Second).
 * @details Formula: Hz = RPM / 60
 * @param rpm Speed in RPM.
 * @return Speed in Hz.
 */
#define MTR_RPM_TO_HZ(rpm) ((rpm) / 60.0)

// --- Electrical vs. Mechanical Speed Conversions ---

/**
 * @brief Converts electrical frequency (Hz) to mechanical speed (rad/s).
 * @details Formula: rad/s_mech = (Hz_elec * 2 * PI) / pole_pairs
 * @param hz_elec Electrical frequency in Hz.
 * @param pp Number of motor pole pairs.
 * @return Mechanical speed in rad/s.
 */
#define MTR_ELEC_HZ_TO_MECH_RADS(hz_elec, pp) (((hz_elec) * M_2_PI) / (pp))

/**
 * @brief Converts mechanical speed (rad/s) to electrical frequency (Hz).
 * @details Formula: Hz_elec = (rad/s_mech * pole_pairs) / (2 * PI)
 * @param rads_mech Mechanical speed in rad/s.
 * @param pp Number of motor pole pairs.
 * @return Electrical frequency in Hz.
 */
#define MTR_MECH_RADS_TO_ELEC_HZ(rads_mech, pp) (((rads_mech) * (pp)) / M_2_PI)

/**
 * @brief Converts electrical speed (rad/s) to mechanical speed (rad/s).
 * @details Formula: rad/s_mech = rad/s_elec / pole_pairs
 * @param rads_elec Electrical speed in rad/s.
 * @param pp Number of motor pole pairs.
 * @return Mechanical speed in rad/s.
 */
#define MTR_ELEC_RADS_TO_MECH_RADS(rads_elec, pp) ((rads_elec) / (pp))

/**
 * @brief Converts mechanical speed (rad/s) to electrical speed (rad/s).
 * @details Formula: rad/s_elec = rad/s_mech * pole_pairs
 * @param rads_mech Mechanical speed in rad/s.
 * @param pp Number of motor pole pairs.
 * @return Electrical speed in rad/s.
 */
#define MTR_MECH_RADS_TO_ELEC_RADS(rads_mech, pp) ((rads_mech) * (pp))

// --- Motor Constant Conversions (Flux, Kv, Kt) ---

/**
 * @brief Calculates motor torque constant Kt (in Nm/A) from flux linkage.
 * @details For a PMSM: Kt = 1.5 * pole_pairs * Flux
 * @param flux The magnetic flux linkage of one phase in Weber (Wb).
 * @param pp The number of motor pole pairs.
 * @return The calculated torque constant Kt in Nm/A.
 */
#define MTR_FLUX_TO_KT(flux, pp) (1.5 * (pp) * (flux))

/**
 * @brief Calculates motor back-EMF constant Kv (in V/(rad/s)) from flux linkage.
 * @details For a PMSM: Kv = pole_pairs * Flux (line-to-neutral).
 * @param flux The magnetic flux linkage of one phase in Weber (Wb).
 * @param pp The number of motor pole pairs.
 * @return The calculated back-EMF constant Kv in V/(rad/s).
 */
#define MTR_FLUX_TO_KV_RADS(flux, pp) ((pp) * (flux))

/**
 * @brief Calculates motor back-EMF constant Kv (in RPM/V) from flux linkage.
 * @details This converts Kv from V/(rad/s) to the more common RPM/V unit.
 * @param flux The magnetic flux linkage of one phase in Weber (Wb).
 * @param pp The number of motor pole pairs.
 * @return The calculated back-EMF constant Kv in RPM/V.
 */
#define MTR_FLUX_TO_KV_RPM(flux, pp) (MTR_RADS_TO_RPM(1.0 / MTR_FLUX_TO_KV_RADS(flux, pp)))

/** @} */ // end of MC_PHYSICAL_UNITS group

#endif // _FILE_MOTOR_UNIT_CALCULATOR_H_
