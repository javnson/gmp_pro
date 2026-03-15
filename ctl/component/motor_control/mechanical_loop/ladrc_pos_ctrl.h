/**
 * @file ladrc_pos_ctrl.h
 * @brief Implements a 2nd-Order LADRC-based Mechanical Position Controller (PU System).
 *
 * @version 1.0
 * @date 2024-10-26
 *
 */

#ifndef _FILE_LADRC_POS_CTRL_H_
#define _FILE_LADRC_POS_CTRL_H_

#include <ctl/component/intrinsic/continuous/ladrc2.h>
#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <gmp_core.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* LADRC Position Controller                                                 */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup LADRC_POS_CONTROLLER LADRC Position Controller (PU System)
 * @brief A high-performance position loop controller using 2nd-Order LADRC.
 * @details Replaces the traditional P-PI cascaded position loop. Tracks position, 
 * velocity, and acceleration references directly using full-state feedback and 
 * a 3rd-order LESO to actively compensate for unmodeled friction and load disturbances.
 * Features a dynamic local-frame shift algorithm to absolutely prevent fixed-point overflow.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the LADRC Position Controller.
 */
typedef struct _tag_ladrc_pos_init
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
    parameter_gt cur_limit; //!< Absolute maximum current/torque output reference (PU).

    // --- LADRC Tuning Targets ---
    parameter_gt target_wc; //!< Controller bandwidth (Hz) -> Tracks reference.
    parameter_gt target_wo; //!< Observer bandwidth (Hz) -> Rejects disturbance.

} ctl_ladrc_pos_init_t;

/**
 * @brief Main structure for the LADRC Position Controller.
 */
typedef struct _tag_ladrc_pos_ctrl
{
    // --- Interfaces ---
    rotation_ift* pos_if; //!< Position feedback interface.

    // --- Sub-modules ---
    ctl_ladrc2_t ladrc_core; //!< Core 2nd-order LADRC algorithm.
    ctl_divider_t div_mech;  //!< Divider to down-sample execution.

    // --- Real-time Parameters ---
    ctrl_gt cur_limit;       //!< Current/Torque limit (PU).
    ctrl_gt scale_w_to_revs; //!< Constant to convert PU speed (w_base) to rev/s.

    // --- Local Frame Tracking (Anti-Overflow mechanism) ---
    ctrl_gt prev_fbk_raw;  //!< Previous raw fractional angle from encoder (PU).
    ctrl_gt local_fbk_pos; //!< Continuous unwrapped local feedback position.
    ctrl_gt local_ref_pos; //!< Continuous local reference position.

    // --- Outputs & Status ---
    ctrl_gt torque_cmd;  //!< The final calculated current/torque command (PU).
    fast_gt flag_enable; //!< Enable switch.

} ctl_ladrc_pos_ctrl_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_autotuning_ladrc_pos_ctrl(ctl_ladrc_pos_init_t* init, ctl_ladrc_pos_ctrl_t* ctrl);

/**
 * @brief Attaches position feedback interface.
 */
GMP_STATIC_INLINE void ctl_attach_ladrc_pos_ctrl(ctl_ladrc_pos_ctrl_t* ctrl, rotation_ift* pos_if)
{
    ctrl->pos_if = pos_if;
}

/**
 * @brief Clears the internal states of the controller.
 */
GMP_STATIC_INLINE void ctl_clear_ladrc_pos_ctrl(ctl_ladrc_pos_ctrl_t* ctrl)
{
    ctl_clear_ladrc2(&ctrl->ladrc_core);
    ctl_clear_divider(&ctrl->div_mech);
    ctrl->local_fbk_pos = float2ctrl(0.0f);
    ctrl->local_ref_pos = float2ctrl(0.0f);
    ctrl->prev_fbk_raw = (ctrl->pos_if != NULL) ? ctrl->pos_if->position : float2ctrl(0.0f);
    ctrl->torque_cmd = float2ctrl(0.0f);
}

/**
 * @brief Enables the controller with Bumpless Transfer.
 * @param[in] current_iq_fbk The ACTUAL Iq current (PU) measured right now to pre-load the observer.
 */
GMP_STATIC_INLINE void ctl_enable_ladrc_pos_ctrl(ctl_ladrc_pos_ctrl_t* ctrl, ctrl_gt current_iq_fbk)
{
    if (ctrl->flag_enable)
        return;

    // Reset local tracking frame to absolute zero locally
    ctrl->local_fbk_pos = float2ctrl(0.0f);
    ctrl->local_ref_pos = float2ctrl(0.0f);
    if (ctrl->pos_if != NULL)
        ctrl->prev_fbk_raw = ctrl->pos_if->position;

    // Pre-load LADRC states: Current local pos is 0, velocity is approx 0, output is current Iq
    ctl_set_ladrc2_states(&ctrl->ladrc_core, float2ctrl(0.0f), float2ctrl(0.0f), current_iq_fbk);

    ctrl->flag_enable = 1;
}

/**
 * @brief Disables the controller.
 */
GMP_STATIC_INLINE void ctl_disable_ladrc_pos_ctrl(ctl_ladrc_pos_ctrl_t* ctrl)
{
    ctrl->flag_enable = 0;
    ctrl->torque_cmd = float2ctrl(0.0f);
}

/**
 * @brief Executes one step of the LADRC position control loop.
 * @param[in,out] ctrl            Pointer to the controller.
 * @param[in]     target_revs     Target absolute position (Full revolutions).
 * @param[in]     target_angle_pu Target absolute position (Fractional angle 0~1 PU).
 * @param[in]     ref_vel_pu      Target feedforward velocity (PU, based on w_base).
 * @param[in]     ref_acc_pu      Target feedforward acceleration (PU/s).
 */
GMP_STATIC_INLINE void ctl_step_ladrc_pos_ctrl(ctl_ladrc_pos_ctrl_t* ctrl, int32_t target_revs, ctrl_gt target_angle_pu,
                                               ctrl_gt ref_vel_pu, ctrl_gt ref_acc_pu)
{
    if (!ctrl->flag_enable)
    {
        ctrl->torque_cmd = float2ctrl(0.0f);
        return;
    }

    if (ctl_step_divider(&ctrl->div_mech))
    {
        // ·ÀÓùÐÔ¶ÏÑÔ
        gmp_base_assert(ctrl->pos_if != NULL);

        // ====================================================================
        // 1. Dynamic Local Coordinate Tracking (Zero-Cost Wrap-Around Fix)
        // ====================================================================
        // Get incremental movement of the motor since last step
        ctrl_gt current_raw = ctrl->pos_if->position;
        ctrl_gt delta_fbk = current_raw - ctrl->prev_fbk_raw;

        // Handle physical wrap-around of the fractional angle
        if (delta_fbk > float2ctrl(0.5f))
            delta_fbk -= float2ctrl(1.0f);
        else if (delta_fbk < float2ctrl(-0.5f))
            delta_fbk += float2ctrl(1.0f);

        ctrl->local_fbk_pos += delta_fbk;
        ctrl->prev_fbk_raw = current_raw;

        // Calculate absolute bounded error to anchor the reference safely
        ctrl_gt pos_err_pu = ctl_calc_position_error(target_revs, target_angle_pu, ctrl->pos_if);

        // Pin the local reference strictly to the feedback + error
        ctrl->local_ref_pos = ctrl->local_fbk_pos + pos_err_pu;

        // ====================================================================
        // 2. Anti-Overflow Shift Mechanism
        // ====================================================================
        // If the motor rotates continuously, local_fbk_pos grows. We shift the
        // entire coordinate frame back to near-zero to preserve Q24 precision.
        if (ctrl->local_fbk_pos > float2ctrl(10.0f) || ctrl->local_fbk_pos < float2ctrl(-10.0f))
        {
            ctrl_gt shift = -ctrl->local_fbk_pos;
            ctrl->local_fbk_pos += shift; // Becomes exactly 0
            ctrl->local_ref_pos += shift; // Becomes strictly pos_err_pu
            ctrl->ladrc_core.z1 += shift; // Shift LESO memory! Bumpless!
        }

        // ====================================================================
        // 3. Feedforward Scaling (Convert PU(w_base) to Revs/s)
        // ====================================================================
        ctrl_gt ref_vel_revs = ctl_mul(ref_vel_pu, ctrl->scale_w_to_revs);
        ctrl_gt ref_acc_revs = ctl_mul(ref_acc_pu, ctrl->scale_w_to_revs);

        // ====================================================================
        // 4. Execute 2nd-Order LADRC Core
        // ====================================================================
        ctrl->torque_cmd =
            ctl_step_ladrc2(&ctrl->ladrc_core, ctrl->local_ref_pos, ref_vel_revs, ref_acc_revs, ctrl->local_fbk_pos);
    }
}

/**
 * @brief Gets the calculated current/torque command (PU).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ladrc_pos_cmd(const ctl_ladrc_pos_ctrl_t* ctrl)
{
    return ctrl->torque_cmd;
}

/**
 * @brief Gets the estimated highly-filtered velocity from the LESO (in PU based on w_base).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ladrc_pos_velocity_pu(const ctl_ladrc_pos_ctrl_t* ctrl)
{
    // z2 is in Revs/s. We divide by scale_w_to_revs (which is w_base / 2pi).
    // Note: To avoid division in ISR, we could pre-calculate the inverse,
    // but this is just telemetry, so it's acceptable.
    return ctl_div(ctl_get_ladrc2_velocity(&ctrl->ladrc_core), ctrl->scale_w_to_revs);
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_LADRC_POS_CTRL_H_
