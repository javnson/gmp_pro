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
