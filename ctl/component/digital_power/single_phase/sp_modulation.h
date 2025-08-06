/**
 * @file sp_modulation.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implements single-phase, unipolar SPWM for an H-bridge with dead-time compensation.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details This module generates two PWM compare values for a standard single-phase
 * H-bridge inverter using unipolar Sine Pulse Width Modulation (SPWM). It accepts a
 * modulation signal from -1.0 to 1.0.
 *
 * It also includes a dead-time compensation feature that adjusts the PWM duty cycles
 * based on the direction of the output current. This minimizes voltage distortion
 * caused by the blanking time inserted by the PWM hardware.
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
 * @{
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
    fast_gt current_dir; /**< The detected direction of the output current (-1, 0, or 1). */

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
GMP_STATIC_INLINE
void ctl_clear_single_phase_H_modulation(single_phase_H_modulation_t* bridge)
{
    bridge->phase_L = 0;
    bridge->phase_N = 0;
    bridge->current_dir = 0;
}

/**
 * @brief Executes one step of the modulation calculation.
 * @details This function calculates the PWM compare values for both legs of the H-bridge
 * based on the target voltage and applies dead-time compensation based on the current direction.
 * @param[out] bridge Handle of the modulation object.
 * @param[in] u_target The target output voltage modulation index (-1.0 to 1.0).
 * @param[in] inverter_current The measured instantaneous output current.
 */
GMP_STATIC_INLINE
void ctl_step_single_phase_H_modulation(single_phase_H_modulation_t* bridge, ctrl_gt u_target, ctrl_gt inverter_current)
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
    ctrl_gt modulate_target_L = ctl_sat(ctl_div2(-u_target + float2ctrl(1)), 0, float2ctrl(1));
    ctrl_gt modulate_target_N = ctl_sat(ctl_div2(u_target + float2ctrl(1)), 0, float2ctrl(1));

    // 3. Apply dead-time compensation based on current direction.
    if (bridge->current_dir == 1) // Positive current: V_actual < V_ideal, need to increase duty cycle.
    {
        bridge->phase_L = pwm_mul(modulate_target_L, bridge->pwm_full_scale) + bridge->pwm_deadband;
        bridge->phase_N = pwm_mul(modulate_target_N, bridge->pwm_full_scale) - bridge->pwm_deadband;
    }
    else if (bridge->current_dir == -1) // Negative current: V_actual > V_ideal, need to decrease duty cycle.
    {
        bridge->phase_L = pwm_mul(modulate_target_L, bridge->pwm_full_scale) - bridge->pwm_deadband;
        bridge->phase_N = pwm_mul(modulate_target_N, bridge->pwm_full_scale) + bridge->pwm_deadband;
    }
    else // No current or within deadband: no compensation.
    {
        bridge->phase_L = pwm_mul(modulate_target_L, bridge->pwm_full_scale);
        bridge->phase_N = pwm_mul(modulate_target_N, bridge->pwm_full_scale);
    }

    // 4. Saturate the final compare values to ensure they are within the valid PWM range.
    bridge->phase_L = pwm_sat(bridge->phase_L, bridge->pwm_full_scale, 0);
    bridge->phase_N = pwm_sat(bridge->phase_N, bridge->pwm_full_scale, 0);
}

/**
 * @brief Gets the calculated PWM compare value for the 'L' phase leg.
 * @param[in] bridge Handle of the modulation object.
 * @return The PWM compare value for the 'L' phase.
 */
GMP_STATIC_INLINE
pwm_gt ctl_get_single_phase_modulation_L_phase(single_phase_H_modulation_t* bridge)
{
    return bridge->phase_L;
}

/**
 * @brief Gets the calculated PWM compare value for the 'N' phase leg.
 * @param[in] bridge Handle of the modulation object.
 * @return The PWM compare value for the 'N' phase.
 */
GMP_STATIC_INLINE
pwm_gt ctl_get_single_phase_modulation_N_phase(single_phase_H_modulation_t* bridge)
{
    return bridge->phase_N;
}

/** @} */ // end of sp_modulation_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SINGLE_PHASE_MODULATION_H_
