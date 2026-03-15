/**
 * @file smc_mech_ctrl.h
 * @brief Implements a Sliding Mode Controller (SMC) for Mechanical Position/Velocity Loop.
 *
 * @version 1.0
 * @date 2024-10-26
 *
 */

#ifndef _FILE_SMC_MECH_CTRL_H_
#define _FILE_SMC_MECH_CTRL_H_

#include <ctl/component/intrinsic/advance/smc.h>
#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/motor_control/interface/encoder.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* SMC Mechanical Controller (Position -> Current)                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup SMC_MECH_CONTROLLER SMC Mechanical Loop Controller
 * @brief Direct Position-to-Current robust controller using Sliding Mode Control.
 * @details Uses a switching sliding surface s = \lambda x_1 + x_2 to drive the 
 * mechanical errors to zero. It offers superior robustness against inertia 
 * mismatches and load torque disturbances compared to traditional PI loops.
 * * @par Example Usage:
 * @code
 * ctl_smc_mech_init_t smc_init;
 * ctl_smc_mech_ctrl_t smc_ctrl;
 * * // Physical Parameters
 * smc_init.fs_current = 10000.0f;
 * smc_init.mech_division = 10;
 * smc_init.inertia = 0.0002f;     // J
 * smc_init.torque_const = 0.5f;   // Kt
 * smc_init.cur_limit = 20.0f;     
 * * // SMC Tuning Targets
 * smc_init.target_bw = 30.0f;           // Determines the sliding surface slope \lambda
 * smc_init.dist_reject_torque = 2.0f;   // Max disturbance torque to reject (determines \rho)
 * ctl_autotuning_smc_mech_ctrl(&smc_init);
 * * // Init & Run
 * ctl_init_smc_mech_ctrl(&smc_ctrl, &smc_init);
 * ctl_attach_smc_mech_ctrl(&smc_ctrl, &encoder_pos, &encoder_spd);
 * ctl_enable_smc_mech_ctrl(&smc_ctrl);
 * * void Main_ISR(void) {
 * ctl_step_smc_mech_ctrl(&smc_ctrl, pos_ref, vel_ref, acc_ref);
 * target_iq = ctl_get_smc_mech_cmd(&smc_ctrl);
 * }
 * @endcode
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the SMC Mechanical Controller.
 */
typedef struct _tag_smc_mech_init
{
    // --- System & Hardware Configurations ---
    parameter_gt fs_current; //!< Execution frequency of the inner current loop (Hz).
    uint32_t mech_division;  //!< Divider ratio for the mechanical loop.

    parameter_gt inertia;      //!< Total system inertia J (kg*m^2).
    parameter_gt torque_const; //!< Motor torque constant Kt (Nm/A).

    // --- Safety Limits ---
    parameter_gt cur_limit; //!< Absolute maximum current/torque output reference (A).

    // --- Tuning Targets ---
    parameter_gt target_bw;          //!< Target sliding surface decay bandwidth (Hz). Determines lambda.
    parameter_gt dist_reject_torque; //!< Max disturbance torque to reject (Nm). Determines switching gain rho.

    // --- Auto-Tuned SMC Gains ---
    parameter_gt lambda; //!< Sliding surface slope (s = \lambda*x1 + x2).
    parameter_gt eta11;  //!< Pos Error gain.
    parameter_gt eta12;  //!< Pos Error gain.
    parameter_gt eta21;  //!< Vel Error gain.
    parameter_gt eta22;  //!< Vel Error gain.
    parameter_gt rho;    //!< Switching gain for sgn(s) term.
    parameter_gt k_ff;   //!< Feedforward gain for external acceleration (J/Kt).

} ctl_smc_mech_init_t;

/**
 * @brief Main structure for the SMC Mechanical Controller.
 */
typedef struct _tag_smc_mech_ctrl
{
    // --- Interfaces ---
    rotation_ift* pos_if; //!< Position feedback interface.
    velocity_ift* spd_if; //!< Velocity feedback interface.

    // --- Controller Modules ---
    ctl_smc_t smc_core;     //!< The core Sliding Mode Controller instance.
    ctl_divider_t div_mech; //!< Divider to down-sample execution.

    // --- Real-time Parameters & States ---
    ctrl_gt k_ff;        //!< Real-time Acceleration Feedforward Gain.
    ctrl_gt cur_limit;   //!< Output saturation limit.
    ctrl_gt cur_output;  //!< The final calculated current command.
    fast_gt flag_enable; //!< Enable switch.

} ctl_smc_mech_ctrl_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_autotuning_smc_mech_ctrl(ctl_smc_mech_init_t* init);
void ctl_init_smc_mech_ctrl(ctl_smc_mech_ctrl_t* ctrl, const ctl_smc_mech_init_t* init);

/**
 * @brief Attaches feedback interfaces to the SMC mechanical controller.
 */
GMP_STATIC_INLINE void ctl_attach_smc_mech_ctrl(ctl_smc_mech_ctrl_t* ctrl, rotation_ift* pos_if, velocity_ift* spd_if)
{
    ctrl->pos_if = pos_if;
    ctrl->spd_if = spd_if;
}

/**
 * @brief Enables the SMC mechanical controller.
 */
GMP_STATIC_INLINE void ctl_enable_smc_mech_ctrl(ctl_smc_mech_ctrl_t* ctrl)
{
    ctrl->flag_enable = 1;
}

/**
 * @brief Disables the SMC controller and zeroes output.
 */
GMP_STATIC_INLINE void ctl_disable_smc_mech_ctrl(ctl_smc_mech_ctrl_t* ctrl)
{
    ctrl->flag_enable = 0;
    ctrl->cur_output = float2ctrl(0.0f);
}

/**
 * @brief Executes one step of the SMC mechanical loop.
 * @param[in,out] ctrl        Pointer to the controller.
 * @param[in]     target_revs Target position (Full revolutions).
 * @param[in]     target_pu   Target position (Fractional angle 0~1 PU).
 * @param[in]     vel_ref     Target velocity (rad/s).
 * @param[in]     acc_ref     Target acceleration (rad/s^2).
 * @details 
 * x1 = Position Error (\theta_ref - \theta_fb)
 * x2 = Velocity Error (\omega_ref - \omega_fb)
 * SMC Output u = \eta_1 x_1 + \eta_2 x_2 + \rho sgn(s)
 * Total Command = SMC_Output + Acc_Feedforward
 */
GMP_STATIC_INLINE void ctl_step_smc_mech_ctrl(ctl_smc_mech_ctrl_t* ctrl, int32_t target_revs, ctrl_gt target_pu,
                                              ctrl_gt vel_ref, ctrl_gt acc_ref)
{
    if (!ctrl->flag_enable)
    {
        ctrl->cur_output = float2ctrl(0.0f);
        return;
    }

    if (ctl_step_divider(&ctrl->div_mech))
    {
        // 1. Calculate Primary State Variables (x1, x2)
        ctrl_gt pos_err_pu = ctl_calc_position_error(target_revs, target_pu, ctrl->pos_if);

        // Convert to physical radians: x1 = \theta_err
        ctrl_gt x1 = ctl_mul(pos_err_pu, float2ctrl(CTL_PARAM_CONST_2PI));

        // Velocity error: x2 = \omega_err
        ctrl_gt x2 = vel_ref - ctrl->spd_if->speed;

        // 2. Execute Sliding Mode Control Core
        ctrl_gt u_smc = ctl_step_smc(&ctrl->smc_core, x1, x2);

        // 3. Add Acceleration Feedforward
        // Feedforward compensates for the known dynamic term J*\ddot{\theta}_{ref}
        ctrl_gt u_ff = ctl_mul(acc_ref, ctrl->k_ff);

        // 4. Combine and Saturate
        ctrl_gt total_cmd = u_smc + u_ff;
        ctrl->cur_output = ctl_sat(total_cmd, ctrl->cur_limit, -ctrl->cur_limit);
    }
}

/**
 * @brief Gets the calculated current/torque command.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_smc_mech_cmd(const ctl_smc_mech_ctrl_t* ctrl)
{
    return ctrl->cur_output;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SMC_MECH_CTRL_H_
