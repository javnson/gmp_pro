/**
 * @file motor_per_unit_consultant.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a consultant module to calculate base values for a motor's per-unit system.
 * @details The per-unit system simplifies analysis and control of power systems by normalizing
 * all quantities with respect to a common set of base values. This module takes fundamental
 * motor ratings (power, voltage, frequency) and computes a consistent set of base values
 * for current, impedance, flux, torque, etc.
 * @version 0.2
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
    parameter_gt base_omega;        /**< Base electrical angular velocity (omega_e_base) in rad/s. */
    parameter_gt base_impedence;    /**< Base impedance (Z_base) in Ohms [Ohm]. */
    parameter_gt base_inductance;   /**< Base inductance (L_base) in Henrys [H]. */
    parameter_gt base_capacitance;  /**< Base capacitance (C_base) in Farads [F]. */
    parameter_gt base_flux;         /**< Base flux linkage (psi_base) in Webers [Wb]. */

    //
    // Derived Mechanical Base Values
    //
    parameter_gt base_torque;     /**< Base torque (T_base) in Newton-meters [Nm]. */
    parameter_gt base_speed;      /**< Base mechanical angular velocity (omega_m_base) in rad/s. */
    parameter_gt base_speed_krpm; /**< Base mechanical speed in thousands of RPM [krpm]. */

} ctl_per_unit_consultant_t;

/**
 * @brief Sets up the per-unit consultant for a SYNCHRONOUS motor.
 * @param pu Pointer to the `ctl_per_unit_consultant_t` object to be initialized.
 * @param pole_pairs Number of motor pole pairs.
 * @param phases Number of motor phases.
 * @param rated_power Total rated power of the motor in Watts [W].
 * @param rated_voltage_phase_rms Rated RMS phase voltage in Volts [V].
 * @param rated_freq Rated electrical frequency in Hertz [Hz]. For sync motors, this corresponds to the rated speed.
 */
void ctl_init_per_unit_consultant_pmsm(ctl_per_unit_consultant_t* pu, uint32_t pole_pairs, uint32_t phases,
                                       parameter_gt rated_power, parameter_gt rated_voltage_phase_rms,
                                       parameter_gt rated_freq);

/**
 * @brief Sets up the per-unit consultant for an ASYNCHRONOUS (Induction) motor.
 * @param pu Pointer to the `ctl_per_unit_consultant_t` object to be initialized.
 * @param pole_pairs Number of motor pole pairs.
 * @param phases Number of motor phases.
 * @param rated_power Total rated power of the motor in Watts [W].
 * @param rated_voltage_phase_rms Rated RMS phase voltage in Volts [V].
 * @param synchronous_freq The synchronous electrical frequency in Hz.
 * @param rated_spd_krpm The rated speed for induction motor in krpm.
 */
void ctl_init_per_unit_consultant_acm(ctl_per_unit_consultant_t* pu, uint32_t pole_pairs, uint32_t phases,
                                      parameter_gt rated_power, parameter_gt rated_voltage_phase_rms,
                                      parameter_gt synchronous_freq, parameter_gt rated_spd_krpm);

/**
 * @brief Helper function to convert horsepower to Watts.
 * @param hp Power in horsepower.
 * @return Power in Watts.
 */
GMP_STATIC_INLINE parameter_gt ctl_helper_horsepower2watt(parameter_gt hp)
{
    return (parameter_gt)746.0 * hp * 1000.0;
}

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

/*---------------------------------------------------------------------------*/
/* Consultant Getter Functions                                               */
/*---------------------------------------------------------------------------*/

/**
 * @name Per-Unit Base Value Accessors
 * @brief A set of inline functions to safely retrieve base values from the consultant object.
 * @{
 */

//
// Accessors for Primary Input Ratings
//

/** @brief Gets the base power per phase (P_base) in Watts [W]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_power(ctl_per_unit_consultant_t* pu)
{
    return pu->base_power;
}

/** @brief Gets the base phase voltage (V_base_rms) in Volts [V]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_voltage(ctl_per_unit_consultant_t* pu)
{
    return pu->base_voltage;
}

/** @brief Gets the base electrical frequency (f_base) in Hertz [Hz]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_frequency(ctl_per_unit_consultant_t* pu)
{
    return pu->base_freq;
}

/** @brief Gets the number of motor pole pairs (p). */
GMP_STATIC_INLINE uint32_t ctl_consult_pole_pairs(ctl_per_unit_consultant_t* pu)
{
    return pu->pole_pairs;
}

/** @brief Gets the number of motor phases. */
GMP_STATIC_INLINE uint32_t ctl_consult_phases(ctl_per_unit_consultant_t* pu)
{
    return pu->phases;
}

//
// Accessors for Derived Electrical Base Values
//

/** @brief Gets the base phase current (I_base_rms) in Amperes [A]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_current(ctl_per_unit_consultant_t* pu)
{
    return pu->base_current;
}

/** @brief Gets the base instantaneous peak current (I_base_peak) in Amperes [A]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_peak_current(ctl_per_unit_consultant_t* pu)
{
    return pu->base_inst_current;
}

/** @brief Gets the base instantaneous peak voltage (V_base_peak) in Volts [V]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_peak_voltage(ctl_per_unit_consultant_t* pu)
{
    return pu->base_inst_voltage;
}

/** @brief Gets the base electrical angular velocity (¦Ø_e_base) in rad/s. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_electrical_omega(ctl_per_unit_consultant_t* pu)
{
    return pu->base_omega;
}

/** @brief Gets the base impedance (Z_base) in Ohms [¦¸]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_impedance(ctl_per_unit_consultant_t* pu)
{
    return pu->base_impedence;
}

/** @brief Gets the base inductance (L_base) in Henrys [H]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_inductance(ctl_per_unit_consultant_t* pu)
{
    return pu->base_inductance;
}

/** @brief Gets the base capacitance (C_base) in Farads [F]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_capacitance(ctl_per_unit_consultant_t* pu)
{
    return pu->base_capacitance;
}

/** @brief Gets the base flux linkage (¦·_base) in Webers [Wb]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_flux(ctl_per_unit_consultant_t* pu)
{
    return pu->base_flux;
}

//
// Accessors for Derived Mechanical Base Values
//

/** @brief Gets the base torque (T_base) in Newton-meters [Nm]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_torque(ctl_per_unit_consultant_t* pu)
{
    return pu->base_torque;
}

/** @brief Gets the base mechanical angular velocity (omega_m_base) in rad/s. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_mechanical_speed(ctl_per_unit_consultant_t* pu)
{
    return pu->base_speed;
}

/** @brief Gets the base mechanical speed in thousands of RPM [krpm]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_base_mechanical_speed_krpm(ctl_per_unit_consultant_t* pu)
{
    return pu->base_speed_krpm;
}

/** 
 * @} 
 */

/*---------------------------------------------------------------------------*/
/* Per-Unit Conversion Functions                                             */
/*---------------------------------------------------------------------------*/

/**
 * @name Per-Unit Value Converters
 * @brief A set of inline functions to convert physical values to per-unit and vice versa.
 * @{
 */

//
// Impedance / Resistance / Reactance Conversions
//

/** @brief Converts an impedance/resistance value [¦¸] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_Z_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt Z_ohm)
{
    return Z_ohm / pu->base_impedence;
}

/** @brief Converts an impedance/resistance per-unit value to its physical value [¦¸]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_Z_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt Z_pu)
{
    return Z_pu * pu->base_impedence;
}

//
// Voltage Conversions
//

/** @brief Converts an RMS voltage [V] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_V_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt V_rms)
{
    return V_rms / pu->base_voltage;
}

/** @brief Converts a voltage per-unit value to its physical RMS value [V]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_V_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt V_pu)
{
    return V_pu * pu->base_voltage;
}

/** @brief Converts a peak voltage [V] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_Vpeak_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt V_peak)
{
    return V_peak / pu->base_inst_voltage;
}

/** @brief Converts a voltage per-unit value to its physical peak value [V]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_Vpeak_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt V_pu)
{
    return V_pu * pu->base_inst_voltage;
}

//
// Current Conversions
//

/** @brief Converts an RMS current [A] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_I_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt I_rms)
{
    return I_rms / pu->base_current;
}

/** @brief Converts a current per-unit value to its physical RMS value [A]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_I_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt I_pu)
{
    return I_pu * pu->base_current;
}

/** @brief Converts a peak current [A] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_Ipeak_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt I_peak)
{
    return I_peak / pu->base_inst_current;
}

/** @brief Converts a current per-unit value to its physical peak value [A]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_Ipeak_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt I_pu)
{
    return I_pu * pu->base_inst_current;
}

//
// Inductance Conversions
//

/** @brief Converts an inductance value [H] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_L_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt L_H)
{
    return L_H / pu->base_inductance;
}

/** @brief Converts an inductance per-unit value to its physical value [H]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_L_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt L_pu)
{
    return L_pu * pu->base_inductance;
}

//
// Flux Linkage Conversions
//

/** @brief Converts a flux linkage value [Wb] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_Flux_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt Flux_Wb)
{
    return Flux_Wb / pu->base_flux;
}

/** @brief Converts a flux linkage per-unit value to its physical value [Wb]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_Flux_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt Flux_pu)
{
    return Flux_pu * pu->base_flux;
}

//
// Torque Conversions
//

/** @brief Converts a torque value [Nm] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_T_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt T_Nm)
{
    return T_Nm / pu->base_torque;
}

/** @brief Converts a torque per-unit value to its physical value [Nm]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_T_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt T_pu)
{
    return T_pu * pu->base_torque;
}

//
// Speed and Frequency Conversions
//

/** @brief Converts an electrical angular velocity [rad/s] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_omega_e_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt omega_e_rads)
{
    return omega_e_rads / pu->base_omega;
}

/** @brief Converts an electrical omega per-unit value to its physical value [rad/s]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_omega_e_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt omega_e_pu)
{
    return omega_e_pu * pu->base_omega;
}

/** @brief Converts a mechanical angular velocity [rad/s] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_omega_m_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt omega_m_rads)
{
    return omega_m_rads / pu->base_speed;
}

/** @brief Converts a mechanical omega per-unit value to its physical value [rad/s]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_omega_m_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt omega_m_pu)
{
    return omega_m_pu * pu->base_speed;
}

/** @brief Converts a mechanical speed [krpm] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_speed_krpm_to_pu(ctl_per_unit_consultant_t* pu, parameter_gt speed_krpm)
{
    return speed_krpm / pu->base_speed_krpm;
}

/** @brief Converts a speed per-unit value to its physical value [krpm]. */
GMP_STATIC_INLINE parameter_gt ctl_consult_speed_krpm_to_phy(ctl_per_unit_consultant_t* pu, parameter_gt speed_pu)
{
    return speed_pu * pu->base_speed_krpm;
}

/** 
 * @} 
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MOTOR_PER_UNIT_CONSULTANT_H_
