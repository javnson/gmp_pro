/**
 * @file INC_algorithm.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implements the Incremental Conductance (INC) MPPT algorithm.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details This file provides an implementation of the Incremental Conductance algorithm
 * for Maximum Power Point Tracking (MPPT). The INC algorithm offers improved tracking
 * under rapidly changing atmospheric conditions compared to the P&O method.
 *
 * The core principle is that the derivative of power with respect to voltage (dP/dV)
 * is zero at the MPP. The algorithm checks the relationship between the incremental
 * conductance (dI/dV) and the instantaneous conductance (I/V) to determine the
 * direction in which to adjust the voltage.
 * - dI/dV > -I/V : Left of MPP, increase voltage.
 * - dI/dV < -I/V : Right of MPP, decrease voltage.
 * - dI/dV = -I/V : At MPP, keep voltage constant.
 *
 * Reference: https://ww2.mathworks.cn/discovery/mppt-algorithm.html
 */

#ifndef _FILE_INC_ALGORITHM_H_
#define _FILE_INC_ALGORITHM_H_

#include <ctl/component/interface/interface_base.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @addtogroup mppt_api
 * @{
 * @ingroup CTL_DP_LIB
 */

/**
 * @brief Data structure for the Incremental Conductance (INC) MPPT algorithm.
 */
typedef struct _tag_mppt_inc_algo
{
    /*-- Interfaces --*/
    adc_ift* adc_u; /**< ADC interface for the panel voltage. */
    adc_ift* adc_i; /**< ADC interface for the panel current. */

    /*-- Output --*/
    ctrl_gt v_ref; /**< The calculated output voltage reference for the power converter. */

    /*-- Parameters --*/
    ctrl_gt voltage_increment; /**< The fixed voltage perturbation step size. */
    ctrl_gt max_voltage_limit; /**< The upper limit for the output voltage reference. */
    ctrl_gt min_voltage_limit; /**< The lower limit for the output voltage reference. */

    /*-- Intermediate Variables --*/
    ctrl_gt last_voltage;  /**< Voltage measured in the previous MPPT step. */
    ctrl_gt last_current;  /**< Current measured in the previous MPPT step. */
    ctrl_gt current_power; /**< Power calculated in the current step. */

    /*-- Control Flags --*/
    fast_gt flag_enable_mppt; /**< Master enable flag for the MPPT algorithm. */

} mppt_inc_algo_t;

/**
 * @brief Initializes the Incremental Conductance MPPT algorithm module.
 * @param[out] _mppt Pointer to the MPPT algorithm instance.
 * @param[in] u_in Pointer to the voltage ADC interface.
 * @param[in] i_in Pointer to the current ADC interface.
 * @param[in] voltage_0 The initial voltage reference output by the algorithm.
 * @param[in] voltage_step The fixed perturbation step size.
 * @param[in] max_voltage The absolute maximum voltage limit.
 * @param[in] min_voltage The absolute minimum voltage limit.
 */
void ctl_init_mppt_inc_algo(mppt_inc_algo_t* _mppt, adc_ift* u_in, adc_ift* i_in, parameter_gt voltage_0,
                            parameter_gt voltage_step, parameter_gt max_voltage, parameter_gt min_voltage);

/**
 * @brief Clears the internal states of the INC MPPT algorithm.
 * @param[in,out] mppt Pointer to the MPPT algorithm instance.
 */
GMP_STATIC_INLINE void ctl_clear_mppt_inc_algo(mppt_inc_algo_t* _mppt)
{
    _mppt->last_voltage = 0;
    _mppt->last_current = 0;
    _mppt->current_power = 0;
}

/**
 * @brief Executes one step of the Incremental Conductance MPPT algorithm.
 * @details This function should be called periodically. It calculates the incremental
 * and instantaneous conductance to determine the direction of voltage change needed
 * to track the maximum power point.
 * @param[in,out] _mppt_obj Pointer to the MPPT algorithm instance.
 * @return The updated voltage reference (`v_ref`) to be used by the power converter.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_mppt_inc_algo(mppt_inc_algo_t* _mppt_obj)
{
    ctrl_gt current_voltage = _mppt_obj->adc_u->value;
    ctrl_gt current_current = _mppt_obj->adc_i->value;

    // MPPT algorithm logic
    if (_mppt_obj->flag_enable_mppt)
    {
        ctrl_gt delta_voltage = current_voltage - _mppt_obj->last_voltage;
        ctrl_gt delta_current = current_current - _mppt_obj->last_current;

        // Check for dV=0 edge case
        if (delta_voltage == 0)
        {
            if (delta_current > 0)
            {
                // Power increased, keep moving in the same direction (increase v_ref)
                _mppt_obj->v_ref += _mppt_obj->voltage_increment;
            }
            else if (delta_current < 0)
            {
                // Power decreased, reverse direction (decrease v_ref)
                _mppt_obj->v_ref -= _mppt_obj->voltage_increment;
            }
        }
        else // Standard INC logic for dV != 0
        {
            // The core condition is dI/dV == -I/V at MPP.
            // Rearranging gives I + V * (dI/dV) == 0.
            // We check the sign of this expression to determine direction.
            ctrl_gt inc_cond = current_current + ctl_mul(ctl_div(delta_current, delta_voltage), current_voltage);

            if (inc_cond > 0)
            {
                // dI/dV > -I/V, left of MPP, increase voltage
                _mppt_obj->v_ref += _mppt_obj->voltage_increment;
            }
            else if (inc_cond < 0)
            {
                // dI/dV < -I/V, right of MPP, decrease voltage
                _mppt_obj->v_ref -= _mppt_obj->voltage_increment;
            }
            // if inc_cond == 0, we are at MPP, do nothing.
        }

        // Apply voltage limits
        _mppt_obj->v_ref = ctl_sat(_mppt_obj->v_ref, _mppt_obj->max_voltage_limit, _mppt_obj->min_voltage_limit);
    }

    // Update state for the next iteration
    _mppt_obj->last_voltage = current_voltage;
    _mppt_obj->last_current = current_current;

    // Return the calculated voltage reference
    return _mppt_obj->v_ref;
}

/**
 * @brief Enables the MPPT algorithm.
 * @param[in,out] _mppt_obj Pointer to the MPPT algorithm instance.
 */
GMP_STATIC_INLINE void ctl_enable_mppt_inc_algo(mppt_inc_algo_t* _mppt_obj)
{
    _mppt_obj->flag_enable_mppt = 1;
}

/**
 * @brief Disables the MPPT algorithm.
 * @param[in,out] _mppt_obj Pointer to the MPPT algorithm instance.
 */
GMP_STATIC_INLINE void ctl_disable_mppt_inc_algo(mppt_inc_algo_t* _mppt_obj)
{
    _mppt_obj->flag_enable_mppt = 0;
}

/** @} */ // end of mppt_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_INC_ALGORITHM_H_
