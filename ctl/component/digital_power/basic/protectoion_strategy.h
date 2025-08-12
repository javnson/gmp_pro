/**
 * @file protection_strategy.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Provides standard protection strategies for DC/DC converters.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _FILE_PROTECTION_STRATEGY_H_
#define _FILE_PROTECTION_STRATEGY_H_

#include <ctl/component/intrinsic/discrete/discrete_filter.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup protection_strategy_api DC/DC Protection Strategy API
 * @brief Contains standard protection modules for power converters.
 * @details This file implements a standard "brick-wall" protection scheme against
 * over-voltage, over-current, and over-power conditions.
 * @{
 * @ingroup CTL_DP_LIB
 */

/*---------------------------------------------------------------------------*/
/* Standard VIP (Voltage, Current, Power) Protection                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the standard VIP "brick-wall" protection module.
 * @details This module trips a persistent error flag if the output voltage, current,
 * or power exceeds their configured maximum limits.
 */
typedef struct _tag_std_vip_protection_type
{
    /*-- Interfaces --*/
    adc_ift* adc_uo; /**< ADC interface for the output voltage. */
    adc_ift* adc_io; /**< ADC interface for the output current. */

    /*-- Output --*/
    fast_gt flag_error; /**< Error flag. Set to 1 when a protection limit is exceeded. */

    /*-- Protection Parameters --*/
    ctrl_gt voltage_max; /**< Maximum permissible output voltage. */
    ctrl_gt current_max; /**< Maximum permissible output current. */
    ctrl_gt power_max;   /**< Maximum permissible output power. */

    /*-- Intermediate Variables --*/
    ctrl_gt uout; /**< Filtered output voltage. */
    ctrl_gt iout; /**< Filtered output current. */
    ctrl_gt pout; /**< Calculated and filtered output power. */

    /*-- Filter Modules --*/
    ctl_low_pass_filter_t power_filter;   /**< Low-pass filter for the power calculation. */
    ctl_low_pass_filter_t voltage_filter; /**< Low-pass filter for the voltage measurement. */
    ctl_low_pass_filter_t current_filter; /**< Low-pass filter for the current measurement. */

} std_vip_protection_t;

/**
 * @brief Initializes the VIP protection module.
 * @param[out] obj Pointer to the VIP protection instance.
 * @param[in] power_f_cut Cutoff frequency for the power measurement filter.
 * @param[in] voltage_f_cut Cutoff frequency for the voltage measurement filter.
 * @param[in] current_f_cut Cutoff frequency for the current measurement filter.
 * @param[in] v_max Maximum voltage limit.
 * @param[in] v_base Base voltage for per-unit conversion (if applicable).
 * @param[in] i_max Maximum current limit.
 * @param[in] i_base Base current for per-unit conversion (if applicable).
 * @param[in] p_max Maximum power limit.
 * @param[in] fs Sampling frequency of the controller.
 */
void ctl_init_vip_protection(std_vip_protection_t* obj, parameter_gt power_f_cut, parameter_gt voltage_f_cut,
                             parameter_gt current_f_cut, parameter_gt v_max, parameter_gt v_base, parameter_gt i_max,
                             parameter_gt i_base, parameter_gt p_max, parameter_gt fs);

/**
 * @brief Attaches the protection module to the physical ADC input interfaces.
 * @param[out] obj Pointer to the VIP protection instance.
 * @param[in] uo Pointer to the ADC interface for the output voltage.
 * @param[in] io Pointer to the ADC interface for the output current.
 */
void ctl_attach_vip_protection(std_vip_protection_t* obj, adc_ift* uo, adc_ift* io);

/**
 * @brief Clears a latched error flag.
 * @details This function must be called to reset the protection module after a fault has occurred.
 * @param[out] obj Pointer to the VIP protection instance.
 */
GMP_STATIC_INLINE void ctl_clear_vip_protection_error(std_vip_protection_t* obj)
{
    obj->flag_error = 0;
}

/**
 * @brief Executes one step of the VIP protection logic.
 * @details This function filters the inputs, calculates power, and checks against the limits.
 * If a limit is exceeded, it sets a persistent error flag.
 * @param[in,out] obj Pointer to the VIP protection instance.
 * @return The current state of the error flag (0 for no error, 1 for error).
 */
GMP_STATIC_INLINE fast_gt ctl_step_vip_protection(std_vip_protection_t* obj)
{
    // If an error has already occurred, latch it and do nothing further.
    if (obj->flag_error != 0)
    {
        return obj->flag_error;
    }

    // Filter the current and voltage measurements.
    obj->iout = ctl_step_lowpass_filter(&obj->current_filter, obj->adc_io->value);
    obj->uout = ctl_step_lowpass_filter(&obj->voltage_filter, obj->adc_uo->value);

    // Calculate and filter the output power.
    obj->pout = ctl_step_lowpass_filter(&obj->power_filter, ctl_mul(obj->iout, obj->uout));

    // Check if any of the protection limits have been exceeded.
    if ((obj->iout > obj->current_max) || (obj->uout > obj->voltage_max) || (obj->pout > obj->power_max))
    {
        obj->flag_error = 1;
    }

    return obj->flag_error;
}

/*---------------------------------------------------------------------------*/
/* Foldback Overcurrent Protection                                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a foldback current protection module.
 * @details This module implements a foldback overcurrent protection scheme.
 * When the output voltage drops due to an overload, the current limit is proportionally
 * reduced, minimizing power dissipation during fault conditions like a short circuit.
 * The output of this module is a dynamic current limit reference, intended to be
 * fed into the current control loop.
 */
typedef struct _tag_std_foldback_protection_type
{
    /*-- Interfaces --*/
    adc_ift* adc_uo; /**< ADC interface for the output voltage. */
    adc_ift* adc_io; /**< ADC interface for the output current. */

    /*-- Output --*/
    ctrl_gt current_limit_output; /**< Calculated dynamic current limit. This is the main output. */
    fast_gt flag_overcurrent;     /**< Flag indicating if the measured current exceeds the dynamic limit. */

    /*-- Protection Parameters --*/
    ctrl_gt voltage_rated;         /**< Normal operating output voltage, where foldback starts. */
    ctrl_gt current_max;           /**< Maximum current limit at or above the rated voltage. */
    ctrl_gt current_short_circuit; /**< Current limit when the output is fully shorted (Vout -> 0). */

    /*-- Intermediate Variables --*/
    ctrl_gt uout; /**< Filtered output voltage. */
    ctrl_gt iout; /**< Filtered output current. */

    /*-- Private Calculation Variables --*/
    ctrl_gt slope; /**< The slope of the foldback curve (calculated once at init). */

    /*-- Filter Modules --*/
    ctl_low_pass_filter_t voltage_filter; /**< Low-pass filter for the voltage measurement. */
    ctl_low_pass_filter_t current_filter; /**< Low-pass filter for the current measurement. */

} std_foldback_protection_t;

/**
 * @brief Initializes the foldback protection module.
 * @param[out] obj Pointer to the foldback protection instance.
 * @param[in] voltage_f_cut Cutoff frequency for the voltage measurement filter.
 * @param[in] current_f_cut Cutoff frequency for the current measurement filter.
 * @param[in] v_rated The rated output voltage. Foldback begins when the voltage drops below this level.
 * @param[in] i_max The maximum current limit under normal operation (at v_rated).
 * @param[in] i_short The desired current limit during a full short-circuit (at v_out = 0).
 * @param[in] fs Sampling frequency of the controller.
 */
void ctl_init_foldback_protection(std_foldback_protection_t* obj, parameter_gt voltage_f_cut,
                                  parameter_gt current_f_cut, parameter_gt v_rated, parameter_gt i_max,
                                  parameter_gt i_short, parameter_gt fs);

/**
 * @brief Attaches the foldback protection module to the physical ADC input interfaces.
 * @param[out] obj Pointer to the foldback protection instance.
 * @param[in] uo Pointer to the ADC interface for the output voltage.
 * @param[in] io Pointer to the ADC interface for the output current.
 */
void ctl_attach_foldback_protection(std_foldback_protection_t* obj, adc_ift* uo, adc_ift* io);

/**
 * @brief Clears the overcurrent state flag.
 * @details Call this function to acknowledge and reset the overcurrent flag.
 * @param[out] obj Pointer to the foldback protection instance.
 */
GMP_STATIC_INLINE void ctl_clear_foldback_state(std_foldback_protection_t* obj)
{
    obj->flag_overcurrent = 0;
}

/**
 * @brief Executes one step of the foldback protection logic.
 * @details This function filters the inputs and calculates the dynamic current limit based on the
 * measured output voltage. It also sets a flag if the measured current exceeds this limit.
 * @param[in,out] obj Pointer to the foldback protection instance.
 * @return The calculated dynamic current limit for the current control loop.
 */
GMP_STATIC_INLINE fast_gt ctl_step_foldback_protection(std_foldback_protection_t* obj)
{
    // Filter the current and voltage measurements.
    obj->iout = ctl_step_lowpass_filter(&obj->current_filter, obj->adc_io->value);
    obj->uout = ctl_step_lowpass_filter(&obj->voltage_filter, obj->adc_uo->value);

    // Calculate the new dynamic current limit based on the output voltage.
    if (obj->uout >= obj->voltage_rated)
    {
        // Normal operation: limit is the maximum allowed current.
        obj->current_limit_output = obj->current_max;
    }
    else if (obj->uout > 0)
    {
        // Foldback region: linearly decrease the current limit as voltage drops.
        // Equation of the line: I_limit = slope * V_out + I_short_circuit
        obj->current_limit_output = ctl_mac(obj->slope, obj->uout, obj->current_short_circuit);
    }
    else
    {
        // Short-circuit condition: limit is the minimum short-circuit current.
        obj->current_limit_output = obj->current_short_circuit;
    }

    // Ensure the calculated limit does not exceed the absolute maximum.
    if (obj->current_limit_output > obj->current_max)
    {
        obj->current_limit_output = obj->current_max;
    }

    // Check if the actual measured current is exceeding the newly calculated limit.
    if (obj->iout > obj->current_limit_output)
    {
        obj->flag_overcurrent = 1;
    }

    // The primary output of this function is the dynamic limit itself.
    return obj->current_limit_output;
}

/**
 * @brief this function will get foldback protect current limit result.
 */
ctrl_gt ctl_get_foldback_protection_current_limit(std_foldback_protection_t* obj)
{
    return obj->current_limit_output;
}

/** @} */ // end of protection_strategy_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PROTECTION_STRATEGY_H_
