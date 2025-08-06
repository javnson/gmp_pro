/**
 * @file spfc.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implements a single-phase Power Factor Correction (PFC) controller.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details This file provides a dual-loop controller for a single-phase boost-type
 * PFC converter. The control strategy aims to maintain a stable DC bus voltage
 * while forcing the input AC current to be sinusoidal and in phase with the
 * input voltage, thus achieving a high power factor.
 *
 * The controller consists of:
 * 1. An outer voltage loop that regulates the DC bus voltage. Its output sets the
 * amplitude for the current reference.
 * 2. An inner current loop that shapes the inductor current to follow a rectified
 * sinusoidal reference, which is synchronized with the input voltage.
 */

#ifndef _FILE_GMP_CTL_SPFC_H_
#define _FILE_GMP_CTL_SPFC_H_

#include <ctl/component/interface/interface_base.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup spfc_api Single Phase PFC API
 * @brief Controller for single-phase Power Factor Correction.
 * @{
 * @ingroup CTL_DP_LIB
 */

/**
 * @brief Data structure for the single-phase PFC controller.
 */
typedef struct _tag_spfc_type
{
    /*-- Interfaces --*/
    adc_ift* adc_rect_u; /**< ADC interface for the rectified grid voltage. */
    adc_ift* adc_rect_i; /**< ADC interface for the inductor current (rectified grid current). */
    adc_ift* adc_dc_u;   /**< ADC interface for the DC bus output voltage. */

    /*-- Output --*/
    ctrl_gt boost_modulation; /**< Final calculated PWM duty cycle for the Boost stage. */

    /*-- Set-points and References --*/
    ctrl_gt voltage_set; /**< Target reference for the DC bus voltage. */
    ctrl_gt current_set; /**< Amplitude reference for the current loop (output of voltage loop). */
    ctrl_gt current_ref; /**< Instantaneous reference for the inner current loop. */

    /*-- Submodules --*/
    ctl_pid_t current_ctrl;     /**< PID controller for the inner current loop. */
    ctl_pid_t voltage_ctrl; /**< PID controller for the outer voltage loop. */

    /*-- Control Flags --*/
    fast_gt flag_enable; /**< Master enable flag for the PFC controller. */

} spfc_t;

/**
 * @brief Initializes the single-phase PFC controller.
 * @param[out] pfc Pointer to the PFC controller instance.
 * @param[in] voltage_kp Proportional gain for the voltage PID controller.
 * @param[in] voltage_Ti Integral time constant for the voltage PID controller.
 * @param[in] voltage_Td Derivative time constant for the voltage PID controller.
 * @param[in] current_kp Proportional gain for the current PID controller.
 * @param[in] current_Ti Integral time constant for the current PID controller.
 * @param[in] current_Td Derivative time constant for the current PID controller.
 * @param[in] fs Controller execution frequency in Hz.
 */
void ctl_init_spfc_ctrl(spfc_t* pfc, parameter_gt voltage_kp, parameter_gt voltage_Ti, parameter_gt voltage_Td,
                        parameter_gt current_kp, parameter_gt current_Ti, parameter_gt current_Td, parameter_gt fs);

/**
 * @brief Attaches the PFC controller to the physical ADC input interfaces.
 * @param[out] pfc Pointer to the PFC controller instance.
 * @param[in] rect_u Pointer to the ADC interface for the rectified grid voltage.
 * @param[in] rect_i Pointer to the ADC interface for the inductor current.
 * @param[in] dc_u Pointer to the ADC interface for the DC bus voltage.
 */
void ctl_attach_spfc_input(spfc_t* pfc, adc_ift* rect_u, adc_ift* rect_i, adc_ift* dc_u);

/**
 * @brief Clears the internal states of the PFC PID controllers.
 * @param[in,out] pfc Pointer to the PFC controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_pfc_ctrl(spfc_t* pfc)
{
    ctl_clear_pid(&pfc->current_ctrl);
    ctl_clear_pid(&pfc->voltage_ctrl);
}

/**
 * @brief Executes one step of the single-phase PFC control logic.
 * @param[in,out] pfc Handle of the PFC controller.
 * @return The calculated PWM duty cycle for the Boost converter.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_spfc_ctrl(spfc_t* pfc)
{
    if (!pfc->flag_enable)
    {
        pfc->boost_modulation = 0; // Turn off PWM if disabled
        return pfc->boost_modulation;
    }

    // Read sensor values from interfaces
    ctrl_gt rectifier_voltage = pfc->adc_rect_u->value;
    ctrl_gt rectifier_current = pfc->adc_rect_i->value;
    ctrl_gt dc_voltage = pfc->adc_dc_u->value;

    // Outer DC Bus voltage controller: calculates the required current amplitude.
    pfc->current_set = ctl_step_pid_ser(&pfc->voltage_ctrl, pfc->voltage_set - dc_voltage);

    // Generate the instantaneous current reference by shaping the amplitude with the rectified voltage waveform.
    pfc->current_ref = ctl_mul(pfc->current_set, rectifier_voltage);

    // Inner Current controller: forces the inductor current to follow the reference.
    pfc->boost_modulation = ctl_step_pid_ser(&pfc->current_ctrl, pfc->current_ref - rectifier_current);

    // Return the calculated modulation duty cycle
    return pfc->boost_modulation;
}

/**
 * @brief Sets the target DC bus voltage.
 * @param[in,out] pfc Pointer to the PFC controller instance.
 * @param[in] voltage_set The desired DC bus voltage.
 */
GMP_STATIC_INLINE void ctl_set_spfc_voltage(spfc_t* pfc, ctrl_gt voltage_set)
{
    pfc->voltage_set = voltage_set;
}

/**
 * @brief Enables the PFC controller.
 * @param[in,out] pfc Pointer to the PFC controller instance.
 */
GMP_STATIC_INLINE void ctl_enable_spfc_ctrl(spfc_t* pfc)
{
    pfc->flag_enable = 1;
}

/**
 * @brief Disables the PFC controller.
 * @param[in,out] pfc Pointer to the PFC controller instance.
 */
GMP_STATIC_INLINE void ctl_disable_spfc_ctrl(spfc_t* pfc)
{
    pfc->flag_enable = 0;
}

/** @} */ // end of spfc_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_CTL_SPFC_H_
