/**
 * @file motor_per_unit_consultant.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a consultant module to calculate base values for a motor's per-unit system.
 * @details The per-unit system simplifies analysis and control of power systems by normalizing
 * all quantities with respect to a common set of base values. This module takes fundamental
 * motor ratings (power, voltage, frequency) and computes a consistent set of base values
 * for current, impedance, flux, torque, etc.
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_MOTOR_PER_UNIT_CONSULTANT_H_
#define _FILE_MOTOR_PER_UNIT_CONSULTANT_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup MOTOR_PER_UNIT_SYSTEM Motor Per-Unit System Consultant
 * @brief A module to establish base quantities for per-unit calculations in motor control.
 */

/*---------------------------------------------------------------------------*/
/* Per-Unit System Consultant                                                */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup MOTOR_PER_UNIT_SYSTEM
 * @{
 */

/**
 * @brief Data structure holding all base quantities for the per-unit system.
 */
typedef struct _tag_per_unit_consultant_t
{
    //
    // Primary Input Ratings
    //
    parameter_gt base_power;   /**< Base power per phase (P_base) in Watts [W]. */
    parameter_gt base_voltage; /**< Base phase voltage (V_base_rms) in Volts [V]. */
    parameter_gt base_freq;    /**< Base electrical frequency (f_base) in Hertz [Hz]. */
    uint32_t pole_pairs;       /**< Number of motor pole pairs (p). */
    uint32_t phases;           /**< Number of motor phases (typically 3). */

    //
    // Derived Electrical Base Values
    //
    parameter_gt base_current;      /**< Base phase current (I_base_rms) in Amperes [A]. */
    parameter_gt base_inst_current; /**< Base instantaneous current (I_base_peak) in Amperes [A]. */
    parameter_gt base_inst_voltage; /**< Base instantaneous voltage (V_base_peak) in Volts [V]. */
    parameter_gt base_omega;        /**< Base electrical angular velocity (¦Ø_e_base) in rad/s. */
    parameter_gt base_impedence;    /**< Base impedance (Z_base) in Ohms [¦¸]. */
    parameter_gt base_inductance;   /**< Base inductance (L_base) in Henrys [H]. */
    parameter_gt base_capacitance;  /**< Base capacitance (C_base) in Farads [F]. */
    parameter_gt base_flux;         /**< Base flux linkage (¦×_base) in Webers [Wb]. */

    //
    // Derived Mechanical Base Values
    //
    parameter_gt base_torque;     /**< Base torque (T_base) in Newton-meters [N¡¤m]. */
    parameter_gt base_speed;      /**< Base mechanical angular velocity (¦Ø_m_base) in rad/s. */
    parameter_gt base_speed_krpm; /**< Base mechanical speed in thousands of RPM [krpm]. */

} ctl_per_unit_consultant_t;

/**
 * @brief Sets up the per-unit consultant object by calculating all base values.
 * @param pu Pointer to the `ctl_per_unit_consultant_t` object to be initialized.
 * @param pole_pairs Number of motor pole pairs.
 * @param phases Number of motor phases.
 * @param rated_power Total rated power of the motor in Watts [W].
 * @param rated_voltage_phase_rms Rated RMS phase voltage in Volts [V].
 * @param rated_freq Rated electrical frequency in Hertz [Hz].
 */
void ctl_setup_per_unit_consultant_by_puf(ctl_per_unit_consultant_t* pu, uint32_t pole_pairs, uint32_t phases,
                                          parameter_gt rated_power, parameter_gt rated_voltage_phase_rms,
                                          parameter_gt rated_freq);

/**
 * @brief Helper function to convert horsepower to Watts.
 * @param hp Power in horsepower.
 * @return Power in Watts.
 */
parameter_gt ctl_helper_hp2power(parameter_gt hp);

/**
 * @brief Helper function to convert line-to-line RMS voltage to phase RMS voltage for a Wye-connected motor.
 * @param wye_value Line-to-line RMS voltage.
 * @return Phase RMS voltage.
 */
GMP_STATIC_INLINE parameter_gt ctl_helper_wye_line_voltage_to_phase(parameter_gt wye_value)
{
    return wye_value / CTL_PARAM_CONST_SQRT3;
}

/**
 * @brief Helper function to convert line-to-line RMS voltage to phase RMS voltage for a Delta-connected motor.
 * @param delta_value Line-to-line RMS voltage.
 * @return Phase RMS voltage (which is the same as line-to-line for Delta).
 */
GMP_STATIC_INLINE parameter_gt ctl_helper_delta_line_voltage_to_phase(parameter_gt delta_value)
{
    return delta_value;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MOTOR_PER_UNIT_CONSULTANT_H_
