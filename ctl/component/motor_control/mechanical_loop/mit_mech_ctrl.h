/**
 * @file mit_pos_ctrl.h
 * @brief Implements a Full State Feedback (FSF) Position Controller.
 *
 * @version 1.0
 * @date 2024-10-26
 *
 */

#ifndef _FILE_MIT_POS_CTRL_H_
#define _FILE_MIT_POS_CTRL_H_

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* MIT / Full State Feedback Position Controller                             */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MIT_POS_CONTROLLER FSF Position Controller
 * @brief Full State Feedback (PD-like) position controller with feedforward.
 * @details This controller uses actual velocity feedback instead of differentiating 
 * position error, eliminating high-frequency noise. It utilizes pole-placement 
 * techniques to define exact closed-loop bandwidth and damping characteristics.
 * * @par Example Usage:
 * @code
 * // 1. Instantiate
 * ctl_mit_pos_init_t mit_init;
 * ctl_mit_pos_ctrl_t mit_ctrl;
 * * // 2. Provide hardware parameters
 * mit_init.fs_current = 10000.0f;     
 * mit_init.mech_division = 10;        // 1kHz mechanical loop
 * mit_init.inertia = 0.0002f;         // System inertia J (kg*m^2)
 * mit_init.torque_const = 0.5f;       // Motor torque constant Kt (Nm/A)
 * mit_init.cur_limit = 20.0f;         
 * * // 3. Auto-tune based on target dynamics
 * mit_init.target_bw = 30.0f;         // Closed-loop Bandwidth: 30 Hz
 * mit_init.damping_ratio = 1.0f;      // Critically damped (No overshoot)
 * ctl_autotuning_mit_pos_ctrl(&mit_init);
 * * // 4. Initialize and Attach Feedback
 * ctl_init_mit_pos_ctrl(&mit_ctrl, &mit_init);
 * ctl_attach_mit_pos_ctrl(&mit_ctrl, &encoder_pos, &encoder_spd);
 * ctl_enable_mit_pos_ctrl(&mit_ctrl);
 * * // 5. Real-time loop (Inside Main ISR)
 * void Main_ISR(void) {
 * // Get trajectory from S-Curve generator
 * ctrl_gt pos_ref = get_traj_pos();
 * ctrl_gt vel_ref = get_traj_vel();
 * ctrl_gt acc_ref = get_traj_acc(); 
 * * // Execute MIT controller
 * ctl_step_mit_pos_ctrl(&mit_ctrl, pos_ref, vel_ref, acc_ref);
 * * // Output to inner loop
 * target_im = ctl_get_mit_pos_cmd(&mit_ctrl);
 * }
 * @endcode
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the MIT Position Controller.
 */
typedef struct _tag_mit_pos_init
{
    // --- System & Hardware Configurations ---
    parameter_gt fs_current; //!< Execution frequency of the inner current loop (Hz).
    uint32_t mech_division;  //!< Divider ratio for the mechanical loop.

    parameter_gt inertia;      //!< Total system inertia J (kg*m^2).
    parameter_gt torque_const; //!< Motor torque constant Kt (Nm/A).

    // --- Safety Limits ---
    parameter_gt cur_limit; //!< Absolute maximum current/torque output reference (A).

    // --- Tuning Targets ---
    parameter_gt target_bw;     //!< Target closed-loop bandwidth (Hz).
    parameter_gt damping_ratio; //!< Target damping ratio (1.0 for critically damped, 0.707 for fast step).

    // --- Auto-Tuned Gains ---
    parameter_gt k_pp; //!< Position Proportional Gain.
    parameter_gt k_vp; //!< Velocity Proportional Gain (acts as D-term).
    parameter_gt k_ff; //!< Acceleration Feedforward Gain.

} ctl_mit_pos_init_t;

/**
 * @brief Main structure for the MIT Position Controller.
 */
typedef struct _tag_mit_pos_ctrl
{
    // --- Interfaces ---
    rotation_ift* pos_if; //!< Position feedback interface (PU).
    velocity_ift* spd_if; //!< Velocity feedback interface (rad/s).

    // --- Controller Gains ---
    ctrl_gt k_pp; //!< Real-time Position Proportional Gain.
    ctrl_gt k_vp; //!< Real-time Velocity Proportional Gain.
    ctrl_gt k_ff; //!< Real-time Acceleration Feedforward Gain.

    // --- State & Limits ---
    ctrl_gt cur_limit;   //!< Output saturation limit.
    ctrl_gt cur_output;  //!< The final calculated current command.
    fast_gt flag_enable; //!< Enable switch.

    ctl_divider_t div_mech; //!< Divider to down-sample from current loop freq.

} ctl_mit_pos_ctrl_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_autotuning_mit_pos_ctrl(ctl_mit_pos_init_t* init);
void ctl_init_mit_pos_ctrl(ctl_mit_pos_ctrl_t* ctrl, const ctl_mit_pos_init_t* init);

/**
 * @brief Attaches feedback interfaces to the controller.
 */
GMP_STATIC_INLINE void ctl_attach_mit_pos_ctrl(ctl_mit_pos_ctrl_t* ctrl, rotation_ift* pos_if, velocity_ift* spd_if)
{
    ctrl->pos_if = pos_if;
    ctrl->spd_if = spd_if;
}

/**
 * @brief Enables the controller.
 */
GMP_STATIC_INLINE void ctl_enable_mit_pos_ctrl(ctl_mit_pos_ctrl_t* ctrl)
{
    ctrl->flag_enable = 1;
}

/**
 * @brief Disables the controller and zeroes output.
 */
GMP_STATIC_INLINE void ctl_disable_mit_pos_ctrl(ctl_mit_pos_ctrl_t* ctrl)
{
    ctrl->flag_enable = 0;
    ctrl->cur_output = float2ctrl(0.0f);
}

/**
 * @brief Executes one step of the Full State Feedback position controller.
 * @param[in,out] ctrl        Pointer to the controller.
 * @param[in]     target_revs Target position (Full revolutions).
 * @param[in]     target_pu   Target position (Fractional angle 0~1 PU).
 * @param[in]     vel_ref     Target velocity (rad/s).
 * @param[in]     acc_ref     Target acceleration (rad/s^2).
 */
GMP_STATIC_INLINE void ctl_step_mit_pos_ctrl(ctl_mit_pos_ctrl_t* ctrl, int32_t target_revs, ctrl_gt target_pu,
                                             ctrl_gt vel_ref, ctrl_gt acc_ref)
{
    if (!ctrl->flag_enable)
    {
        ctrl->cur_output = float2ctrl(0.0f);
        return;
    }

    if (ctl_step_divider(&ctrl->div_mech))
    {
        // 1. Position Error Calculation
        ctrl_gt pos_err_pu = ctl_calc_position_error(target_revs, target_pu, ctrl->pos_if);

        // Convert PU error to Mechanical Radians for mathematically correct SI calculation
        ctrl_gt pos_err_rad = ctl_mul(pos_err_pu, float2ctrl(CTL_PARAM_CONST_2PI));

        // 2. Velocity Error Calculation (rad/s)
        ctrl_gt vel_err = vel_ref - ctrl->spd_if->speed;

        // 3. Full State Feedback Control Law
        ctrl_gt p_term = ctl_mul(ctrl->k_pp, pos_err_rad);
        ctrl_gt v_term = ctl_mul(ctrl->k_vp, vel_err);
        ctrl_gt ff_term = ctl_mul(ctrl->k_ff, acc_ref);

        ctrl_gt total_cmd = p_term + v_term + ff_term;

        // 4. Output Saturation
        ctrl->cur_output = ctl_sat(total_cmd, ctrl->cur_limit, -ctrl->cur_limit);
    }
}

/**
 * @brief Gets the calculated current/torque command.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mit_pos_cmd(const ctl_mit_pos_ctrl_t* ctrl)
{
    return ctrl->cur_output;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MIT_POS_CTRL_H_
