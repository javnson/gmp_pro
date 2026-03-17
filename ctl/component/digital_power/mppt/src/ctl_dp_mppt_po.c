/**
 * @file ctl_dp_mppt.c
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implementation file for the MPPT algorithm modules.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details This file contains the function definitions for initializing the
 * MPPT controllers.
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// P&O MPPT Control
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/mppt/PnO_algorithm.h>

void ctl_init_mppt_PnO_algo(mppt_PnO_algo_t* _mppt_obj, parameter_gt voltage_0, parameter_gt searching_range_max,
                            parameter_gt searching_range_min, parameter_gt searching_step_max,
                            parameter_gt searching_step_min, parameter_gt attenuation_time, parameter_gt freq_mppt,
                            parameter_gt freq_ctrl)
{
    // Initialize the low-pass filter for power measurement.
    // A common rule of thumb is to set the filter's cutoff frequency higher than the MPPT frequency.
    ctl_init_lp_filter(&_mppt_obj->power_filter, freq_ctrl, freq_mppt * 6.0f);

    // Initialize the divider to run the MPPT algorithm at the specified frequency.
    ctl_init_divider(&_mppt_obj->divider, (uint32_t)(freq_ctrl / freq_mppt));

    // Set initial and boundary conditions
    _mppt_obj->v_ref = voltage_0;
    _mppt_obj->searching_step_max = searching_step_max;
    _mppt_obj->searching_step_min = searching_step_min;
    _mppt_obj->max_voltage_limit = searching_range_max;
    _mppt_obj->min_voltage_limit = searching_range_min;

    // Calculate the attenuation factor for the adaptive step size based on the time constant.
    _mppt_obj->inc_attenuation = ctl_helper_lp_filter(freq_mppt, 1.0f / attenuation_time);

    // Clear all internal states to ensure a clean start.
    ctl_clear_mppt_PnO_algo(_mppt_obj);

    // Set the default operating state.
    ctl_disable_mppt_PnO_algo(_mppt_obj);
    ctl_enable_adaptive_step_size(_mppt_obj);
}

//////////////////////////////////////////////////////////////////////////
// INC MPPT Control
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/mppt/INC_algorithm.h>

void ctl_init_mppt_inc_algo(mppt_inc_algo_t* _mppt_obj, adc_ift* u_in, adc_ift* i_in, parameter_gt voltage_0,
                            parameter_gt voltage_step, parameter_gt max_voltage, parameter_gt min_voltage)
{
    // Attach ADC interfaces
    _mppt_obj->adc_u = u_in;
    _mppt_obj->adc_i = i_in;

    // Set initial and boundary conditions
    _mppt_obj->v_ref = voltage_0;
    _mppt_obj->voltage_increment = voltage_step;
    _mppt_obj->max_voltage_limit = max_voltage;
    _mppt_obj->min_voltage_limit = min_voltage;

    _mppt_obj->last_voltage = 0;
    _mppt_obj->last_current = 0;
    _mppt_obj->current_power = 0;
    _mppt_obj->flag_enable_mppt = 0;

    // Clear all internal states to ensure a clean start.
    ctl_clear_mppt_inc_algo(_mppt_obj);

    // Set the default operating state.
    ctl_disable_mppt_inc_algo(_mppt_obj);
}
