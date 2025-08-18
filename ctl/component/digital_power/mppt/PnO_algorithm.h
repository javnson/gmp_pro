/**
 * @file PnO_algorithm.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implements the Perturb and Observe (P&O) MPPT algorithm with adaptive step size.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _FILE_PNO_ALGORITHM_H_
#define _FILE_PNO_ALGORITHM_H_

#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/basic/divider.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup mppt_api MPPT Algorithms API
 * @brief Contains algorithms for Maximum Power Point Tracking.
 * @{
 * @ingroup CTL_DP_LIB
 */

/**
 * @brief A small epsilon value to create a dead-zone around zero for power change.
 * @details This helps prevent the algorithm from reacting to negligible power fluctuations
 * caused by noise or floating-point inaccuracies.
 */
#define CTL_MPPT_PNO_EPSILON (float2ctrl(-0.000001))

/**
 * @brief Data structure for the Perturb and Observe (P&O) MPPT algorithm.
 * @details This file provides a robust implementation of the P&O algorithm for
 * Maximum Power Point Tracking (MPPT) in photovoltaic systems. It includes an
 * adaptive step size mechanism to improve both dynamic tracking speed and
 * steady-state efficiency by reducing oscillations around the MPP.
 *
 * The algorithm works by periodically perturbing the operating voltage of the
 * solar panel and observing the resulting change in power.
 *
 * Reference: https://ww2.mathworks.cn/discovery/mppt-algorithm.html
 */
typedef struct _tag_mppt_PnO_algo
{
    /*-- Output --*/
    ctrl_gt v_ref; /**< The calculated output voltage reference for the power converter. */

    /*-- Parameters --*/
    ctrl_gt searching_step_max; /**< The maximum voltage perturbation step size. */
    ctrl_gt searching_step_min; /**< The minimum voltage perturbation step size (for steady-state). */
    ctrl_gt inc_attenuation;    /**< Attenuation factor for adaptive step size convergence. */
    ctrl_gt max_voltage_limit;  /**< The upper limit for the output voltage reference. */
    ctrl_gt min_voltage_limit;  /**< The lower limit for the output voltage reference. */

    /*-- Intermediate Variables --*/
    ctrl_gt last_power;        /**< Power measured in the previous MPPT step. */
    ctrl_gt current_power;     /**< Power measured in the current MPPT step. */
    ctrl_gt last_voltage;      /**< Voltage measured in the previous MPPT step. */
    ctrl_gt delta_power;       /**< Change in power between steps (P_k - P_{k-1}). */
    ctrl_gt delta_voltage;     /**< Change in voltage between steps (V_k - V_{k-1}). */
    ctrl_gt voltage_increment; /**< The current voltage perturbation step size. */
    uint32_t mppt_record;      /**< A bitmask history of recent perturbation directions (0=down, 1=up). */

    /*-- Modules --*/
    ctl_low_pass_filter_t power_filter; /**< Low-pass filter for the calculated power to reduce noise. */
    ctl_divider_t divider;              /**< Divider to execute the MPPT algorithm at a lower frequency than the ISR. */

    /*-- Control Flags --*/
    fast_gt flag_enable_mppt;               /**< Master enable flag for the MPPT algorithm. */
    fast_gt flag_enable_adaptive_step_size; /**< Flag to enable/disable the adaptive step size feature. */

} mppt_PnO_algo_t;

/**
 * @brief Initializes the P&O MPPT algorithm module.
 * @param[out] _mppt_obj Pointer to the MPPT algorithm instance.
 * @param[in] voltage_0 The initial voltage reference output by the algorithm.
 * @param[in] searching_range_max The absolute maximum voltage limit.
 * @param[in] searching_range_min The absolute minimum voltage limit.
 * @param[in] searching_step_max The maximum perturbation step size, used for fast tracking.
 * @param[in] searching_step_min The minimum perturbation step size, used for steady-state fine-tuning.
 * @param[in] attenuation_time Time constant for the convergence of the adaptive step size.
 * @param[in] freq_mppt The desired execution frequency of the MPPT algorithm (e.g., 50 Hz).
 * @param[in] freq_ctrl The frequency of the main control ISR that calls this function.
 */
void ctl_init_mppt_PnO_algo(mppt_PnO_algo_t* _mppt_obj, parameter_gt voltage_0, parameter_gt searching_range_max,
                            parameter_gt searching_range_min, parameter_gt searching_step_max,
                            parameter_gt searching_step_min, parameter_gt attenuation_time, parameter_gt freq_mppt,
                            parameter_gt freq_ctrl);

/**
 * @brief Executes one step of the P&O MPPT algorithm.
 * @details This function should be called periodically. It calculates the current power,
 * compares it with the previous power, and adjusts the output voltage reference (`v_ref`)
 * accordingly to track the maximum power point.
 * @param[in,out] mppt Pointer to the MPPT algorithm instance.
 * @param[in] current_voltage The current measured voltage from the solar panel.
 * @param[in] current_current The current measured current from the solar panel.
 * @return The updated voltage reference (`v_ref`) to be used by the power converter.
 */
GMP_STATIC_INLINE
ctrl_gt ctl_step_mppt_PnO_algo(mppt_PnO_algo_t* mppt, ctrl_gt current_voltage, ctrl_gt current_current)
{
    // Calculate and filter the current power
    mppt->current_power = ctl_step_lowpass_filter(&mppt->power_filter, ctl_mul(current_voltage, current_current));

    // This block only executes when the divider triggers, i.e., at the MPPT frequency
    if (mppt->flag_enable_mppt && ctl_step_divider(&mppt->divider))
    {
        // Calculate power and voltage deltas
        mppt->delta_power = mppt->current_power - mppt->last_power;
        mppt->delta_voltage = current_voltage - mppt->last_voltage;

        // --- Core P&O Logic ---
        if (mppt->delta_power > CTL_MPPT_PNO_EPSILON) // Power is increasing, continue in the same direction
        {
            if (mppt->delta_voltage > 0)
            {
                mppt->v_ref += mppt->voltage_increment; // Increase V_ref
                mppt->mppt_record = (mppt->mppt_record << 1) + 1;
            }
            else
            {
                mppt->v_ref -= mppt->voltage_increment; // Decrease V_ref
                mppt->mppt_record = mppt->mppt_record << 1;
            }
        }
        else // Power is decreasing, reverse the direction
        {
            if (mppt->delta_voltage > 0)
            {
                mppt->v_ref -= mppt->voltage_increment; // Decrease V_ref
                mppt->mppt_record = mppt->mppt_record << 1;
            }
            else
            {
                mppt->v_ref += mppt->voltage_increment; // Increase V_ref
                mppt->mppt_record = (mppt->mppt_record << 1) + 1;
            }
        }

        // --- Adaptive Step Size Logic ---
        if (mppt->flag_enable_adaptive_step_size)
        {
            // If oscillating around MPP, decrease the step size for fine-tuning
            if (((mppt->mppt_record & 0x07) == 2) || ((mppt->mppt_record & 0x07) == 5)) // Pattern 010 or 101
            {
                mppt->voltage_increment = ctl_mul(mppt->searching_step_min, mppt->inc_attenuation) +
                                          ctl_mul(mppt->voltage_increment, CTL_CTRL_CONST_1 - mppt->inc_attenuation);
            }
            // Advanced oscillation detection and correction
            else if (((mppt->mppt_record & 0x0F) == 12)) // Pattern 1100
            {
                mppt->v_ref += mppt->voltage_increment; // Backtrack
                mppt->voltage_increment = ctl_mul(mppt->searching_step_min, mppt->inc_attenuation) +
                                          ctl_mul(mppt->voltage_increment, CTL_CTRL_CONST_1 - mppt->inc_attenuation);
                mppt->v_ref -= mppt->voltage_increment; // Restore with smaller step
            }
            else if (((mppt->mppt_record & 0x0F) == 3)) // Pattern 0011
            {
                mppt->v_ref -= mppt->voltage_increment; // Backtrack
                mppt->voltage_increment = ctl_mul(mppt->searching_step_min, mppt->inc_attenuation) +
                                          ctl_mul(mppt->voltage_increment, CTL_CTRL_CONST_1 - mppt->inc_attenuation);
                mppt->v_ref += mppt->voltage_increment; // Restore with smaller step
            }
            // If moving consistently in one direction, increase step size for fast tracking
            else if (((mppt->mppt_record & 0x0F) == 0x0F) || ((mppt->mppt_record & 0x0F) == 0x00))
            {
                mppt->voltage_increment = mppt->searching_step_max;
            }
        }

        // Apply voltage limits
        mppt->v_ref = ctl_sat(mppt->v_ref, mppt->max_voltage_limit, mppt->min_voltage_limit);

        // Update state for the next iteration
        mppt->last_power = mppt->current_power;
        mppt->last_voltage = current_voltage;
    }

    return mppt->v_ref;
}

/**
 * @brief Clears the internal states of the MPPT algorithm.
 * @details Resets power history, filters, and sets the step size back to maximum.
 * @param[in,out] mppt Pointer to the MPPT algorithm instance.
 */
GMP_STATIC_INLINE
void ctl_clear_mppt_PnO_algo(mppt_PnO_algo_t* mppt)
{
    ctl_clear_lowpass_filter(&mppt->power_filter);
    ctl_clear_divider(&mppt->divider);

    mppt->voltage_increment = mppt->searching_step_max;
    mppt->last_power = 0;
    mppt->current_power = 0;
    mppt->last_voltage = 0;
    mppt->delta_power = 0;
    mppt->delta_voltage = 0;
    mppt->mppt_record = 0;
}

/**
 * @brief Enables the MPPT algorithm.
 * @param[in,out] mppt Pointer to the MPPT algorithm instance.
 */
GMP_STATIC_INLINE
void ctl_enable_mppt_PnO_algo(mppt_PnO_algo_t* mppt)
{
    mppt->flag_enable_mppt = 1;
}

/**
 * @brief Disables the MPPT algorithm.
 * @param[in,out] mppt Pointer to the MPPT algorithm instance.
 */
GMP_STATIC_INLINE
void ctl_disable_mppt_PnO_algo(mppt_PnO_algo_t* mppt)
{
    mppt->flag_enable_mppt = 0;
}

/**
 * @brief Enables the adaptive step size feature.
 * @param[in,out] mppt Pointer to the MPPT algorithm instance.
 */
GMP_STATIC_INLINE
void ctl_enable_adaptive_step_size(mppt_PnO_algo_t* mppt)
{
    mppt->flag_enable_adaptive_step_size = 1;
}

/**
 * @brief Disables the adaptive step size feature, using a fixed step size.
 * @param[in,out] mppt Pointer to the MPPT algorithm instance.
 */
GMP_STATIC_INLINE
void ctl_disable_adaptive_step_size(mppt_PnO_algo_t* mppt)
{
    mppt->flag_enable_adaptive_step_size = 0;
}

/** @} */ // end of mppt_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PNO_ALGORITHM_H_
