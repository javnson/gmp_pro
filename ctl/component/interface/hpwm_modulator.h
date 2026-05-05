/**
 * @file sp_modulation.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implements single-phase, unipolar SPWM for an H-bridge with dead-time compensation.
 * @note **Positive Current (inverter_current > 0):**
 * 
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _FILE_SINGLE_PHASE_MODULATION_H_
#define _FILE_SINGLE_PHASE_MODULATION_H_

#include <ctl/component/interface/interface_base.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup sp_modulation_api Single-Phase Modulation API
 * @brief Generates PWM signals for a single-phase H-bridge inverter.
 * @details This module generates two PWM compare values for a standard single-phase
 * H-bridge inverter using unipolar Sine Pulse Width Modulation (SPWM). It accepts a
 * modulation signal from -1.0 to 1.0.
 *
 * It also includes a dead-time compensation feature that adjusts the PWM duty cycles
 * based on the direction of the output current. This minimizes voltage distortion
 * caused by the blanking time inserted by the PWM hardware.

 * @{
 * @ingroup CTL_DP_LIB
 */

/**
 * @brief Data structure for the single-phase H-bridge modulation module.
 */
typedef struct _tag_single_phase_H_modulation
{
    /*-- Outputs --*/
    pwm_gt phase_L; /**< The calculated PWM compare value for the 'L' phase leg. */
    pwm_gt phase_N; /**< The calculated PWM compare value for the 'N' phase leg. */

    /*-- Parameters --*/
    pwm_gt pwm_full_scale;    /**< The maximum value of the PWM counter (e.g., timer period). */
    pwm_gt pwm_deadband;      /**< The dead-time compensation value, in PWM timer ticks. */
    ctrl_gt current_deadband; /**< A small deadband for the current direction detection to prevent chattering. */

    /*-- Internal State --*/
    fast_gt current_dir;        /**< The detected direction of the output current (-1, 0, or 1). */
    fast_gt flag_enable_dbcomp; /**< Enable DB compesator */

} single_phase_H_modulation_t;

/**
 * @brief Initializes the single-phase H-bridge modulation module.
 * @param[out] bridge Handle of the modulation object.
 * @param[in] pwm_full_scale The maximum value of the PWM counter (timer period).
 * @param[in] pwm_deadband The dead-time value in timer ticks to be compensated.
 * @param[in] current_deadband A small threshold for current direction detection.
 */
void ctl_init_single_phase_H_modulation(single_phase_H_modulation_t* bridge, pwm_gt pwm_full_scale, pwm_gt pwm_deadband,
                                        ctrl_gt current_deadband);

/**
 * @brief Clears the internal state of the modulation module.
 * @param[out] bridge Handle of the modulation object.
 */
GMP_STATIC_INLINE void ctl_clear_single_phase_H_modulation(single_phase_H_modulation_t* bridge)
{
    bridge->phase_L = 0;
    bridge->phase_N = 0;
    bridge->current_dir = 0;
}

/**
 * @brief Executes one step of the modulation calculation with dead-time compensation.
 * 
 * @note **Current Direction & Dead-Time Convention:**
 * - **Positive Current (inverter_current > 0):** Defined as current flowing OUT of the 
 *   'N' phase leg and INTO the 'L' phase leg (i.e., Inverter sourcing power to the load/grid).
 * - **Compensation Logic:** When current flows OUT of a leg, the freewheeling diode of the 
 *   lower switch conducts during the dead-time, clamping the output to the negative bus. 
 *   This causes the actual output voltage pulse to be narrower than the ideal pulse. 
 *   Therefore, the algorithm INCREASES the duty cycle of the sourcing leg (Phase N) 
 *   and DECREASES the duty cycle of the sinking leg (Phase L) to compensate for the lost volt-seconds.
 * - **PWM Logic:** Assumes standard positive logic where a larger compare value (duty cycle) 
 *   results in a wider high-level output pulse.
 * 
 * @param[out] bridge Handle of the modulation object.
 * @param[in] u_target The target output voltage modulation index (-1.0 to 1.0).
 * @param[in] inverter_current The measured instantaneous output current.
 */
GMP_STATIC_INLINE void ctl_step_single_phase_H_modulation(single_phase_H_modulation_t* bridge, ctrl_gt u_target,
                                                          ctrl_gt inverter_current)
{
    // 1. Detect current direction with hysteresis to prevent chattering near zero crossing.
    if (inverter_current > bridge->current_deadband)
    {
        bridge->current_dir = 1; // Positive current
    }
    else if (inverter_current < -bridge->current_deadband)
    {
        bridge->current_dir = -1; // Negative current
    }
    // If within the deadband, keep the previous direction.

    // 2. Calculate ideal unipolar PWM duty cycles for each leg.
    // Duty_L = (1 - u_target) / 2
    // Duty_N = (1 + u_target) / 2
    ctrl_gt modulate_target_L = ctl_sat(ctl_div2(-u_target + float2ctrl(1)), float2ctrl(1), 0);
    ctrl_gt modulate_target_N = ctl_sat(ctl_div2(u_target + float2ctrl(1), float2ctrl(1), 0);

    // 3. Apply dead-time compensation based on current direction.
    int32_t calc_L = (int32_t)pwm_mul(modulate_target_L, bridge->pwm_full_scale);
    int32_t calc_N = (int32_t)pwm_mul(modulate_target_N, bridge->pwm_full_scale);
    int32_t deadband = (int32_t)bridge->pwm_deadband;

    if (bridge->flag_enable_dbcomp)
    {
        if (bridge->current_dir == 1) // Positive current: Phase N is sourcing, Phase L is sinking
        {
            calc_L -= deadband; // L needs less duty
            calc_N += deadband; // N needs more duty
        }
        else if (bridge->current_dir == -1) // Negative current: Phase L is sourcing, Phase N is sinking
        {
            calc_L += deadband; // L needs more duty
            calc_N -= deadband; // N needs less duty
        }
    }

    // 4. Saturate safely and write back to unsigned types
    if (calc_L > (int32_t)bridge->pwm_full_scale)
        calc_L = (int32_t)bridge->pwm_full_scale;
    if (calc_L < 0)
        calc_L = 0;

    if (calc_N > (int32_t)bridge->pwm_full_scale)
        calc_N = (int32_t)bridge->pwm_full_scale;
    if (calc_N < 0)
        calc_N = 0;

    bridge->phase_L = (pwm_gt)calc_L;
    bridge->phase_N = (pwm_gt)calc_N;
}

/**
 * @brief Gets the calculated PWM compare value for the 'L' phase leg.
 * @param[in] bridge Handle of the modulation object.
 * @return The PWM compare value for the 'L' phase.
 */
GMP_STATIC_INLINE pwm_gt ctl_get_single_phase_modulation_L_phase(single_phase_H_modulation_t* bridge)
{
    return bridge->phase_L;
}

/**
 * @brief Gets the calculated PWM compare value for the 'N' phase leg.
 * @param[in] bridge Handle of the modulation object.
 * @return The PWM compare value for the 'N' phase.
 */
GMP_STATIC_INLINE pwm_gt ctl_get_single_phase_modulation_N_phase(single_phase_H_modulation_t* bridge)
{
    return bridge->phase_N;
}

/** @} */ // end of sp_modulation_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SINGLE_PHASE_MODULATION_H_
