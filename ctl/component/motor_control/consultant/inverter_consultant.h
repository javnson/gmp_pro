/**
 * @file consultant_inverter.h
 * @brief Implements the Inverter (Power Converter) physical model.
 *
 * @version 1.0
 * @date 2024-10-27
 */

#ifndef _FILE_CONSULTANT_INVERTER_H_
#define _FILE_CONSULTANT_INVERTER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Consultant: Inverter Physical Model                                       */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CONSULTANT_INVERTER Inverter Model Consultant
 * @brief Manages the physical capabilities and limits of the power converter.
 * @details Provides the actual voltage limits based on real-time DC bus measurements,
 * switching frequency characteristics, and hardware current limits. Crucial for 
 * anti-windup clamping and dead-time compensation.
 * @{
 */

/**
 * @brief Standardized structure for Inverter physical parameters.
 */
typedef struct _tag_consultant_inverter
{
    // --- Hardware Nameplate & Ratings ---
    parameter_gt nominal_v_bus; //!< Nominal DC bus voltage (V).
    parameter_gt max_i_peak;    //!< Absolute maximum peak phase current allowed by IGBT/SiC (A).

    // --- Timing & Frequency ---
    parameter_gt f_pwm;         //!< PWM switching frequency (Hz).
    parameter_gt f_ctrl;        //!< Control loop execution frequency (Hz).
    parameter_gt dead_time_sec; //!< Hardware dead-time (seconds).

    // --- Modulation Characteristics ---
    parameter_gt max_modulation_idx; //!< Max utilization of Vbus. e.g., 0.57735 (1/sqrt(3)) for linear SVPWM.

    // --- Dynamic Derived States (Calculated in real-time) ---
    parameter_gt actual_v_bus;     //!< The dynamically measured DC bus voltage (V).
    parameter_gt v_phase_max_peak; //!< The dynamic maximum phase voltage peak (V).

    // --- Pre-calculated Constants ---
    parameter_gt ts_ctrl;            //!< Control period (1 / f_ctrl) [seconds].
    parameter_gt v_err_deadtime_fac; //!< Dead-time error factor: (dead_time * f_pwm).

} ctl_consultant_inverter_t;

//================================================================================
// Function Prototypes & Inline APIs
//================================================================================

/**
 * @brief Initializes the Inverter model and validates hardware limits.
 * @param[out] inv Pointer to the Inverter consultant instance.
 * @param[in]  v_bus_nom Nominal DC bus voltage (V).
 * @param[in]  i_max_peak Maximum allowed peak current (A).
 * @param[in]  f_pwm PWM switching frequency (Hz).
 * @param[in]  f_ctrl Control loop execution frequency (Hz).
 * @param[in]  dt_sec Hardware dead-time (seconds).
 * @param[in]  m_max Max modulation index (e.g., 0.57735f for linear SVPWM).
 */
void ctl_consultant_inverter_init(ctl_consultant_inverter_t* inv, parameter_gt v_bus_nom, parameter_gt i_max_peak,
                                  parameter_gt f_pwm, parameter_gt f_ctrl, parameter_gt dt_sec, parameter_gt m_max);

/**
 * @brief Dynamically updates the actual DC bus voltage and recalculates output limits.
 * @details Must be called periodically (e.g., in a background task or at the start 
 * of the ISR) if the DC bus is fluctuating.
 * @param[in,out] inv Pointer to the Inverter consultant instance.
 * @param[in]     measured_v_bus The real-time ADC measurement of the DC bus (V).
 */
GMP_STATIC_INLINE void ctl_consultant_inverter_update_vbus(ctl_consultant_inverter_t* inv, parameter_gt measured_v_bus)
{
    // Prevent negative or absurdly low bus voltages from crashing algorithms
    inv->actual_v_bus = (measured_v_bus > 1.0f) ? measured_v_bus : 1.0f;

    // Recalculate dynamic phase voltage limit: V_max_peak = V_bus * M_max
    inv->v_phase_max_peak = inv->actual_v_bus * inv->max_modulation_idx;
}

/**
 * @brief Calculates the physical voltage error magnitude caused by dead-time.
 * @details The error voltage magnitude is approximately: V_err = V_bus * (T_dead / T_pwm).
 * This value can be used by the current controller to inject a feedforward compensation
 * voltage based on the current polarity.
 * @param[in] inv Pointer to the Inverter consultant instance.
 * @return parameter_gt The magnitude of the dead-time error voltage (V).
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_inverter_calc_deadtime_v_err(const ctl_consultant_inverter_t* inv)
{
    return inv->actual_v_bus * inv->v_err_deadtime_fac;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CONSULTANT_INVERTER_H_
