/**
 * @file hpwm_modulator.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implements single-phase, unipolar SPWM for an H-bridge with dead-time compensation.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _FILE_SINGLE_PHASE_MODULATION_H_
#define _FILE_SINGLE_PHASE_MODULATION_H_

#include <ctl/math_block/gmp_math.h>
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
 * modulation signal from -1.0 to 1.0, where a positive target commands a
 * positive L-to-N bridge voltage.
 *
 * It also includes a dead-time compensation feature that adjusts the PWM compare
 * commands based on the direction of the output current. This minimizes voltage
 * distortion caused by the blanking time inserted by the PWM hardware.

 * @{
 * @ingroup CTL_DP_LIB
 */

/**
 * @brief Data structure for the single-phase H-bridge modulation module.
 */
typedef struct _tag_single_phase_H_modulation
{
    /*-- Outputs --*/
    pwm_gt phase_L; /**< PWM timer compare command for the 'L' phase leg. */
    pwm_gt phase_N; /**< PWM timer compare command for the 'N' phase leg. */

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
    // Both legs at 50% produce zero differential bridge voltage and avoid a
    // full-scale pulse when the trip-zone latch is released.
    bridge->phase_L = bridge->pwm_full_scale / 2;
    bridge->phase_N = bridge->pwm_full_scale / 2;
    bridge->current_dir = 0;
}

/**
 * @brief Executes one step of the modulation calculation with dead-time compensation.
 * 
 * @note **Current Direction & Dead-Time Convention:**
 * - **Voltage Convention:** Positive bridge voltage is the L terminal voltage relative to N.
 * - **Positive Current (inverter_current > 0):** Current leaves the inverter through the L
 *   terminal, flows into the load/grid, and returns through the N terminal.
 * - **Compare Convention:** `phase_L` and `phase_N` are timer compare commands, not abstract
 *   high-side duty ratios. This implementation assumes the center-aligned action-qualifier
 *   convention used by the supported SINV targets: decreasing CMP widens the corresponding
 *   high-side pulse, while increasing CMP narrows it. A platform with the opposite timer
 *   polarity must invert the compare mapping in its peripheral binding.
 * - **Compensation Logic:** For positive current, Phase L is the sourcing leg and Phase N is
 *   the returning leg. The algorithm decreases the L compare command and increases the N
 *   compare command to restore positive L-to-N volt-seconds lost during dead time. The
 *   adjustments are reversed for negative current.
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

    // 2. Calculate normalized unipolar PWM compare commands for each leg.
    // Compare_L = (1 - u_target) / 2
    // Compare_N = (1 + u_target) / 2
    ctrl_gt modulate_target_L = ctl_sat(ctl_div2(-u_target + float2ctrl(1)), float2ctrl(1), 0);
    ctrl_gt modulate_target_N = ctl_sat(ctl_div2(u_target + float2ctrl(1)), float2ctrl(1), 0);

    // 3. Apply dead-time compensation based on current direction.
    int32_t calc_L = (int32_t)pwm_mul(modulate_target_L, bridge->pwm_full_scale);
    int32_t calc_N = (int32_t)pwm_mul(modulate_target_N, bridge->pwm_full_scale);
    int32_t deadband = (int32_t)bridge->pwm_deadband;

    if (bridge->flag_enable_dbcomp)
    {
        if (bridge->current_dir == 1) // Positive current: Phase L is sourcing, Phase N is returning
        {
            calc_L -= deadband; // Smaller L compare widens its high-side pulse
            calc_N += deadband; // Larger N compare narrows its high-side pulse
        }
        else if (bridge->current_dir == -1) // Negative current: Phase N is sourcing, Phase L is returning
        {
            calc_L += deadband; // Larger L compare narrows its high-side pulse
            calc_N -= deadband; // Smaller N compare widens its high-side pulse
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
