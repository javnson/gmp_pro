/**
 * @file ato_pll.h
 * @brief Implements an industrial Angle Tracking Observer (Closed-loop PLL).
 * @details Extracts noise-free velocity and position from phase error signals.
 * Un-nested, pure mathematical engine utilizing standard PID components.
 *
 * @version 1.3
 * @date 2024-10-27
 */

#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#ifndef _FILE_ATO_PLL_H_
#define _FILE_ATO_PLL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Angle Tracking Observer (Software PLL)                                    */
/*---------------------------------------------------------------------------*/

typedef struct _tag_ato_pll_t
{
    // --- Core Sub-modules ---
    ctl_pid_t pi_ctrl; //!< Standard PID controller for tracking loop.

    // --- Scale Factors ---
    ctrl_gt sf_w_to_angle; //!< Scale factor: integrates PU speed to PU angle (W_base * Ts / 2*PI).

    // --- Output States (PU) ---
    ctrl_gt elec_angle_pu; //!< Estimated electrical angle (0.0 ~ 1.0 PU).
    ctrl_gt elec_speed_pu; //!< Estimated electrical speed (PU, 1.0 = W_base).

} ctl_ato_pll_t;

/**
 * @brief Auto-tunes and initializes the Angle Tracking Observer.
 */
void ctl_init_ato_pll(ctl_ato_pll_t* pll, parameter_gt bandwidth_hz, parameter_gt damping_ratio,
                      parameter_gt omega_base, parameter_gt fs, parameter_gt spd_limit_max, parameter_gt spd_limit_min);

/**
 * @brief Clears the internal states of the PLL.
 */
GMP_STATIC_INLINE void ctl_clear_ato_pll(ctl_ato_pll_t* pll)
{
    ctl_clear_pid(&pll->pi_ctrl);
    pll->elec_angle_pu = float2ctrl(0.0f);
    pll->elec_speed_pu = float2ctrl(0.0f);
}

/**
 * @brief Dynamically updates the PLL tracking gains (Gain Scheduling API).
 * @details Allows outer loops to adjust PLL bandwidth based on speed for robustness.
 */
GMP_STATIC_INLINE void ctl_set_ato_pll_gains(ctl_ato_pll_t* pll, ctrl_gt kp_pu_new, ctrl_gt ki_pu_new)
{
    pll->pi_ctrl.kp = kp_pu_new;
    pll->pi_ctrl.ki = ki_pu_new;
}

/**
 * @brief Forces the PLL angle and speed to specific values (Bumpless start).
 */
GMP_STATIC_INLINE void ctl_sync_ato_pll(ctl_ato_pll_t* pll, ctrl_gt angle_pu, ctrl_gt speed_pu)
{
    pll->elec_angle_pu = angle_pu;
    pll->elec_speed_pu = speed_pu;
    pll->pi_ctrl.i_term = speed_pu; // reset integral
}

/**
 * @brief Executes one step of the Angle Tracking Observer.
 */
GMP_STATIC_INLINE void ctl_step_ato_pll(ctl_ato_pll_t* pll, ctrl_gt phase_err)
{
    // 1. Closed-Loop Tracking (Standard PI Controller -> Generates Speed)
    // Saturations and anti-windup are automatically handled by ctl_step_pid_par
    pll->elec_speed_pu = ctl_step_pid_par(&pll->pi_ctrl, phase_err);

    // 2. Integration (Speed -> Generates Angle)
    pll->elec_angle_pu += ctl_mul(pll->elec_speed_pu, pll->sf_w_to_angle);

    // 3. Fast, branchless angle wrapping
    pll->elec_angle_pu = ctrl_mod_1(pll->elec_angle_pu);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ATO_PLL_H_
