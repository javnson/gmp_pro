/**
 * @file protection_strategy.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Provides standard protection strategies for DC/DC converters.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details This file implements a standard "brick-wall" protection scheme against
 * over-voltage, over-current, and over-power conditions.
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
GMP_STATIC_INLINE
void ctl_clear_vip_protection_error(std_vip_protection_t* obj)
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
GMP_STATIC_INLINE
fast_gt ctl_step_vip_protection(std_vip_protection_t* obj)
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
/* Foldback Overcurrent Protection (Incomplete)                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a foldback current protection module.
 * @note This feature is currently incomplete.
 */
typedef struct _tag_std_foldback_protection_type
{
    adc_ift* adc_uo; /**< ADC interface for the output voltage. */
    adc_ift* adc_io; /**< ADC interface for the output current. */

} std_foldback_protection_t;

/** @} */ // end of protection_strategy_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PROTECTION_STRATEGY_H_
