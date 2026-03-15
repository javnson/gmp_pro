/**
 * @file ladrc_spd_ctrl.h
 * @brief Implements a 1st-Order LADRC-based Mechanical Speed Controller (PU System).
 *
 * @version 1.0
 * @date 2024-10-26
 *
 */

#ifndef _FILE_LADRC_SPD_CTRL_H_
#define _FILE_LADRC_SPD_CTRL_H_

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/continuous/ladrc1.h>
#include <ctl/component/motor_control/interface/encoder.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* LADRC Speed Controller (Velocity Only)                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup LADRC_SPD_CONTROLLER LADRC Speed Controller (PU System)
 * @brief A pure velocity loop controller using 1st-Order LADRC.
 * @details Replaces the traditional PI velocity loop. It automatically estimates 
 * and compensates for system friction and external load torques using a Linear 
 * Extended State Observer (LESO). Operates entirely in the Per-Unit (PU) domain.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Available operating modes for the LADRC speed controller.
 */
typedef enum
{
    LADRC_SPD_MODE_DISABLE = 0, //!< Controller is disabled, output is 0.
    LADRC_SPD_MODE_ENABLE = 1   //!< Velocity control mode (LADRC active).
} ctl_ladrc_spd_mode_e;

/**
 * @brief Initialization parameters for the LADRC Speed Controller.
 */
typedef struct _tag_ladrc_spd_init
{
    // --- System & Hardware Configurations ---
    parameter_gt fs;        //!< Execution frequency of the inner current loop (Hz).
    uint32_t mech_division; //!< Divider ratio for the mechanical loop.

    parameter_gt inertia;      //!< Total system inertia J (kg*m^2).
    parameter_gt torque_const; //!< Motor torque constant Kt (Nm/A).

    // --- Per-Unit Base Values ---
    parameter_gt omega_base; //!< Base mechanical speed for per-unit conversion (rad/s).
    parameter_gt i_base;     //!< Base current for per-unit conversion (A).

    // --- Safety Limits ---
    parameter_gt speed_limit;       //!< Absolute maximum speed reference (PU).
    parameter_gt speed_slope_limit; //!< Maximum velocity slew rate (PU/s).
    parameter_gt cur_limit;         //!< Absolute maximum current/torque output reference (PU).

    // --- LADRC Tuning Targets ---
    parameter_gt target_wc; //!< Controller bandwidth (Hz) -> Determines tracking speed.
    parameter_gt target_wo; //!< Observer bandwidth (Hz) -> Determines disturbance rejection speed (Typically 3~5x wc).

} ctl_ladrc_spd_init_t;

/**
 * @brief Main structure for the LADRC Speed Controller.
 */
typedef struct _tag_ladrc_spd_ctrl
{
    // --- Interfaces ---
    velocity_ift* spd_if; //!< Velocity feedback interface (PU).

    // --- State Machine ---
    ctl_ladrc_spd_mode_e active_mode; //!< Current operating mode.

    // --- Sub-modules ---
    ctl_ladrc1_t ladrc_core;      //!< Core 1st-order LADRC algorithm.
    ctl_slope_limiter_t vel_traj; //!< Velocity trajectory/profile generator.
    ctl_divider_t div_mech;       //!< Divider to down-sample execution.

    // --- Targets & Limits (PU) ---
    ctrl_gt target_velocity; //!< Raw target velocity (PU).
    ctrl_gt speed_limit;     //!< Dynamic speed saturation limit (PU).
    ctrl_gt cur_limit;       //!< Current/Torque limit (PU).

    // --- Outputs (PU) ---
    ctrl_gt torque_cmd; //!< The final calculated current/torque command (PU).

} ctl_ladrc_spd_ctrl_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_autotuning_ladrc_spd_ctrl(ctl_ladrc_spd_init_t* init, ctl_ladrc_spd_ctrl_t* ctrl);

/**
 * @brief Attaches velocity feedback interface.
 */
GMP_STATIC_INLINE void ctl_attach_ladrc_spd_ctrl(ctl_ladrc_spd_ctrl_t* ctrl, velocity_ift* spd_if)
{
    ctrl->spd_if = spd_if;
}

/**
 * @brief Clears the internal states of the controller.
 */
GMP_STATIC_INLINE void ctl_clear_ladrc_spd_ctrl(ctl_ladrc_spd_ctrl_t* ctrl)
{
    ctl_clear_ladrc1(&ctrl->ladrc_core);
    ctl_clear_slope_limiter(&ctrl->vel_traj);
    ctl_clear_divider(&ctrl->div_mech);
    ctrl->torque_cmd = float2ctrl(0.0f);
}

/**
 * @brief Sets the operating mode with Bumpless Transfer.
 * @param[in] current_iq_fbk The ACTUAL Iq current (PU) measured right now to pre-load the observer.
 */
GMP_STATIC_INLINE void ctl_set_ladrc_spd_mode(ctl_ladrc_spd_ctrl_t* ctrl, ctl_ladrc_spd_mode_e mode,
                                              ctrl_gt current_iq_fbk)
{
    if (ctrl->active_mode == mode)
        return;

    if (mode == LADRC_SPD_MODE_ENABLE)
    {
        // 1. Get current actual speed
        ctrl_gt actual_spd = (ctrl->spd_if != NULL) ? ctrl->spd_if->speed : float2ctrl(0.0f);

        // 2. Bumpless Transfer: Sync trajectory generator
        ctl_set_slope_limiter_current(&ctrl->vel_traj, actual_spd);
        ctrl->target_velocity = actual_spd;

        // 3. Bumpless Transfer: Pre-load LADRC observer states
        // We tell the observer: "Currently we are at `actual_spd`, and we are outputting `current_iq_fbk`"
        ctl_set_ladrc1_states(&ctrl->ladrc_core, actual_spd, current_iq_fbk);
    }
    else
    {
        ctl_clear_ladrc_spd_ctrl(ctrl);
    }

    ctrl->active_mode = mode;
}

/**
 * @brief Sets the target velocity.
 */
GMP_STATIC_INLINE void ctl_set_ladrc_target_velocity(ctl_ladrc_spd_ctrl_t* ctrl, ctrl_gt spd_pu)
{
    ctrl->target_velocity = ctl_sat(spd_pu, ctrl->speed_limit, -ctrl->speed_limit);
}

/**
 * @brief Executes one step of the LADRC speed control loop.
 */
GMP_STATIC_INLINE void ctl_step_ladrc_spd_ctrl(ctl_ladrc_spd_ctrl_t* ctrl)
{
    if (ctrl->active_mode == LADRC_SPD_MODE_DISABLE)
    {
        ctrl->torque_cmd = float2ctrl(0.0f);
        return;
    }

    if (ctl_step_divider(&ctrl->div_mech))
    {
        // ·ÀÓùÐÔ¶ÏÑÔ
        gmp_base_assert(ctrl->spd_if != NULL);

        // 1. Smooth the velocity command
        ctrl_gt smoothed_vel_cmd_pu = ctl_step_slope_limiter(&ctrl->vel_traj, ctrl->target_velocity);

        // 2. Execute 1st-Order LADRC (Everything is in PU, no conversions needed)
        ctrl->torque_cmd = ctl_step_ladrc1(&ctrl->ladrc_core, smoothed_vel_cmd_pu, ctrl->spd_if->speed);
    }
}

/**
 * @brief Gets the calculated current/torque command (PU).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ladrc_spd_cmd(const ctl_ladrc_spd_ctrl_t* ctrl)
{
    return ctrl->torque_cmd;
}

/**
 * @brief Gets the estimated total disturbance equivalent current (PU).
 * @details Can be used to detect sudden load impacts or monitor friction.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ladrc_spd_disturbance(const ctl_ladrc_spd_ctrl_t* ctrl)
{
    return ctl_get_ladrc1_disturbance_pu(&ctrl->ladrc_core);
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_LADRC_SPD_CTRL_H_
