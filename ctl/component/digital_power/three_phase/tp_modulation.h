/**
 * @file tp_modulation.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for three-phase bridge modulation with dead-time compensation.
 * @version 1.0
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2025
 *
 * @defgroup CTL_TP_MODULATION_API Three-Phase Modulation API
 * @{
 * @ingroup CTL_DP_LIB
 * @brief Provides functions for generating three-phase PWM signals from voltage commands,
 * including dead-time compensation based on current direction.
 */

#ifndef _FILE_THREE_PHASE_MODULATION_H_
#define _FILE_THREE_PHASE_MODULATION_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Data structure for the three-phase bridge modulation module.
 */
typedef struct _tag_three_phase_bridge_modulation
{
    //
    // Output
    //
    pwm_gt phase_A; //!< Final PWM compare value for Phase A.
    pwm_gt phase_B; //!< Final PWM compare value for Phase B.
    pwm_gt phase_C; //!< Final PWM compare value for Phase C.

    //
    // Parameters
    //
    pwm_gt pwm_full_scale;    //!< The maximum value of the PWM counter (represents 100% duty cycle).
    pwm_gt pwm_deadband_half; //!< Pre-calculated half of the dead-time value in PWM counts.
    ctrl_gt current_deadband; //!< Current threshold for activating dead-time compensation.

    //
    // Internal State
    //
    fast_gt current_dir_A; //!< Detected direction of Phase A current: 1 for positive, -1 for negative.
    fast_gt current_dir_B; //!< Detected direction of Phase B current: 1 for positive, -1 for negative.
    fast_gt current_dir_C; //!< Detected direction of Phase C current: 1 for positive, -1 for negative.
} three_phase_bridge_modulation_t;

// Forward declaration for the inline function
GMP_STATIC_INLINE void ctl_clear_three_phase_bridge_modulation(three_phase_bridge_modulation_t* bridge);

/**
 * @brief Initializes the three-phase bridge modulation module.
 * @ingroup CTL_TP_MODULATION_API
 *
 * @param[out] bridge Pointer to the `three_phase_bridge_modulation_t` structure.
 * @param[in] pwm_full_scale The maximum value of the PWM counter.
 * @param[in] pwm_deadband The total dead-time value in PWM timer counts.
 * @param[in] current_deadband The current threshold to enable dead-time compensation.
 */
void ctl_init_three_phase_bridge_modulation(three_phase_bridge_modulation_t* bridge, pwm_gt pwm_full_scale,
                                            pwm_gt pwm_deadband, ctrl_gt current_deadband);

/**
 * @brief Clears the internal states of the modulation module.
 * @ingroup CTL_TP_MODULATION_API
 * @param[out] bridge Pointer to the @ref three_phase_bridge_modulation_t structure.
 */
GMP_STATIC_INLINE void ctl_clear_three_phase_bridge_modulation(three_phase_bridge_modulation_t* bridge)
{
    bridge->phase_A = 0;
    bridge->phase_B = 0;
    bridge->phase_C = 0;

    bridge->current_dir_A = 0;
    bridge->current_dir_B = 0;
    bridge->current_dir_C = 0;
}

/**
 * @brief Executes one step of the three-phase modulation algorithm.
 * @ingroup CTL_TP_MODULATION_API
 * @details Converts per-unit voltage commands (-1.0 to 1.0) to PWM compare values,
 * and applies dead-time compensation based on the direction of phase currents.
 *
 * @param[out] bridge Pointer to the @ ref three_phase_bridge_modulation_t structure.
 * @param[in] uA Per-unit voltage command for Phase A.
 * @param[in] uB Per-unit voltage command for Phase B.
 * @param[in] uC Per-unit voltage command for Phase C.
 * @param[in] iA Measured current for Phase A.
 * @param[in] iB Measured current for Phase B.
 * @param[in] iC Measured current for Phase C.
 */
GMP_STATIC_INLINE void ctl_step_three_phase_bridge_modulation(three_phase_bridge_modulation_t* bridge, ctrl_gt uA,
                                                              ctrl_gt uB, ctrl_gt uC, ctrl_gt iA, ctrl_gt iB,
                                                              ctrl_gt iC)
{
    // --- Calculate Raw PWM Values ---
    // Convert voltage command (-1.0 to 1.0) to duty cycle (0 to 1.0), then to raw PWM value.
    bridge->phase_A = pwm_mul(ctl_div2(ctl_add(float2ctrl(1.0f), uA)), bridge->pwm_full_scale);
    bridge->phase_B = pwm_mul(ctl_div2(ctl_add(float2ctrl(1.0f), uB)), bridge->pwm_full_scale);
    bridge->phase_C = pwm_mul(ctl_div2(ctl_add(float2ctrl(1.0f), uC)), bridge->pwm_full_scale);

    // --- Phase A Dead-time Compensation ---
    if (iA > bridge->current_deadband)
    {
        bridge->current_dir_A = 1;
        bridge->phase_A += bridge->pwm_deadband_half;
    }
    else if (iA < -bridge->current_deadband)
    {
        bridge->current_dir_A = -1;
        bridge->phase_A -= bridge->pwm_deadband_half;
    }

    // --- Phase B Dead-time Compensation ---
    if (iB > bridge->current_deadband)
    {
        bridge->current_dir_B = 1;
        bridge->phase_B += bridge->pwm_deadband_half;
    }
    else if (iB < -bridge->current_deadband)
    {
        bridge->current_dir_B = -1;
        bridge->phase_B -= bridge->pwm_deadband_half;
    }

    // --- Phase C Dead-time Compensation ---
    if (iC > bridge->current_deadband)
    {
        bridge->current_dir_C = 1;
        bridge->phase_C += bridge->pwm_deadband_half;
    }
    else if (iC < -bridge->current_deadband)
    {
        bridge->current_dir_C = -1;
        bridge->phase_C -= bridge->pwm_deadband_half;
    }

    // Saturate final PWM values to ensure they are within valid range.
    bridge->phase_A = pwm_sat(bridge->phase_A, bridge->pwm_full_scale, 0);
    bridge->phase_B = pwm_sat(bridge->phase_B, bridge->pwm_full_scale, 0);
    bridge->phase_C = pwm_sat(bridge->phase_C, bridge->pwm_full_scale, 0);
}

/**
 * @brief Gets the final PWM compare value for a specific phase.
 * @ingroup CTL_TP_MODULATION_API
 *
 * @param[in] bridge Pointer to the @ref three_phase_bridge_modulation_t structure.
 * @param[in] index The phase to get the result from (0 for A, 1 for B, 2 for C).
 * @return The final PWM compare value for the selected phase.
 */
GMP_STATIC_INLINE pwm_gt ctl_get_three_phase_modulation_result(three_phase_bridge_modulation_t* bridge, size_gt index)
{
    // CORRECTED: Compared index with integer/enum values instead of PWM values.
    if (index == 0) // phase A
    {
        return bridge->phase_A;
    }
    else if (index == 1) // phase B
    {
        return bridge->phase_B;
    }
    else
    { // Assumes PHASE_C_INDEX
        return bridge->phase_C;
    }
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_MODULATION_H_

/**
 * @}
 */
