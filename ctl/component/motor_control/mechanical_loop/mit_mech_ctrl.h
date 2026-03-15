/**
 * @file mit_pos_ctrl.h
 * @brief Implements a Full State Feedback (FSF) Position Controller in Per-Unit (PU) system.
 *
 * @version 1.1
 * @date 2024-10-26
 *
 */

#ifndef _FILE_MIT_POS_CTRL_H_
#define _FILE_MIT_POS_CTRL_H_

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/math_block/gmp_math.h>
#include <gmp_core.h> // 引入断言支持

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* MIT / Full State Feedback Position Controller                             */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MIT_POS_CONTROLLER FSF Position Controller (PU System)
 * @brief Full State Feedback (PD-like) position controller with feedforward using PU values.
 * @details This controller uses actual PU velocity feedback instead of differentiating 
 * position error. All unit conversions (rad, rad/s) are absorbed into the gains during 
 * auto-tuning, allowing the real-time step function to perform zero conversions.
 * * @par Example Usage:
 * @code
 * ctl_mit_pos_init_t mit_init;
 * ctl_mit_pos_ctrl_t mit_ctrl;
 * * mit_init.fs = 10000.0f;     
 * mit_init.mech_division = 10;
 * mit_init.inertia = 0.0002f;         // J (kg*m^2)
 * mit_init.torque_const = 0.5f;       // Kt (Nm/A)
 * mit_init.omega_base = 314.159f;     // Nominal speed (rad/s)
 * mit_init.i_base = 20.0f;            // Nominal current (A)
 * mit_init.cur_limit = 1.0f;          // PU limit
 * * mit_init.target_bw = 30.0f;         // Bandwidth: 30 Hz
 * mit_init.damping_ratio = 1.0f;      
 * ctl_autotuning_mit_pos_ctrl(&mit_init);
 * * ctl_init_mit_pos_ctrl(&mit_ctrl, &mit_init);
 * ctl_attach_mit_pos_ctrl(&mit_ctrl, &encoder_pos, &encoder_spd);
 * ctl_enable_mit_pos_ctrl(&mit_ctrl);
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
    parameter_gt fs;        //!< Execution frequency of the inner current loop (Hz).
    uint32_t mech_division; //!< Divider ratio for the mechanical loop.

    parameter_gt inertia;      //!< Total system inertia J (kg*m^2).
    parameter_gt torque_const; //!< Motor torque constant Kt (Nm/A).

    // --- Per-Unit Base Values ---
    parameter_gt omega_base; //!< Base mechanical speed for per-unit conversion (rad/s).
    parameter_gt i_base;     //!< Base current for per-unit conversion (A).

    // --- Safety Limits ---
    parameter_gt cur_limit; //!< Absolute maximum current/torque output reference (PU).

    // --- Tuning Targets ---
    parameter_gt target_bw;     //!< Target closed-loop bandwidth (Hz).
    parameter_gt damping_ratio; //!< Target damping ratio (1.0 for critically damped).

    // --- Auto-Tuned Gains (PU Space) ---
    parameter_gt k_pp; //!< PU Position Proportional Gain.
    parameter_gt k_vp; //!< PU Velocity Proportional Gain.
    parameter_gt k_ff; //!< PU Acceleration Feedforward Gain.

} ctl_mit_pos_init_t;

/**
 * @brief Main structure for the MIT Position Controller.
 */
typedef struct _tag_mit_pos_ctrl
{
    // --- Interfaces ---
    rotation_ift* pos_if; //!< Position feedback interface (PU).
    velocity_ift* spd_if; //!< Velocity feedback interface (PU).

    // --- Controller Gains (PU) ---
    ctrl_gt k_pp; //!< Real-time Position Proportional Gain.
    ctrl_gt k_vp; //!< Real-time Velocity Proportional Gain.
    ctrl_gt k_ff; //!< Real-time Acceleration Feedforward Gain.

    // --- State & Limits ---
    ctrl_gt cur_limit;   //!< Output saturation limit (PU).
    ctrl_gt cur_output;  //!< The final calculated current command (PU).
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
 * @details 
 * Real-time calculation operates purely in PU domain:
 * Command = K_pp * Pos_err_pu + K_vp * Vel_err_pu + K_ff * Acc_ref_pu
 * @param[in,out] ctrl            Pointer to the controller.
 * @param[in]     target_revs     Target position (Full revolutions).
 * @param[in]     target_angle_pu Target position (Fractional angle 0~1 PU).
 * @param[in]     vel_ref_pu      Target velocity (PU).
 * @param[in]     acc_ref_pu      Target acceleration (PU/s).
 */
GMP_STATIC_INLINE void ctl_step_mit_pos_ctrl(ctl_mit_pos_ctrl_t* ctrl, int32_t target_revs, ctrl_gt target_angle_pu,
                                             ctrl_gt vel_ref_pu, ctrl_gt acc_ref_pu)
{
    if (!ctrl->flag_enable)
    {
        ctrl->cur_output = float2ctrl(0.0f);
        return;
    }

    if (ctl_step_divider(&ctrl->div_mech))
    {
        gmp_base_assert(ctrl->pos_if != NULL);
        gmp_base_assert(ctrl->spd_if != NULL);

        // 1. Safe Position Error Calculation (PU)
        ctrl_gt pos_err_pu = ctl_calc_position_error(target_revs, target_angle_pu, ctrl->pos_if);

        // 2. Velocity Error Calculation (PU)
        ctrl_gt vel_err_pu = vel_ref_pu - ctrl->spd_if->speed;

        // 3. Full State Feedback Control Law (Zero conversions required here)
        ctrl_gt p_term = ctl_mul(ctrl->k_pp, pos_err_pu);
        ctrl_gt v_term = ctl_mul(ctrl->k_vp, vel_err_pu);
        ctrl_gt ff_term = ctl_mul(ctrl->k_ff, acc_ref_pu);

        ctrl_gt total_cmd = p_term + v_term + ff_term;

        // 4. Output Saturation (PU)
        ctrl->cur_output = ctl_sat(total_cmd, ctrl->cur_limit, -ctrl->cur_limit);
    }
}

/**
 * @brief Gets the calculated current/torque command in PU.
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
