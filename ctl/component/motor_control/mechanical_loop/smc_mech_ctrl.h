/**
 * @file smc_mech_ctrl.h
 * @brief Implements a Sliding Mode Controller (SMC) for Mechanical Position/Velocity Loop in PU System.
 *
 * @version 1.1
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
 * @defgroup SMC_MECH_CONTROLLER SMC Mechanical Loop Controller (PU System)
 * @brief Direct Position-to-Current robust controller using Sliding Mode Control in PU.
 * @details Uses a switching sliding surface s = \lambda x_1 + x_2 to drive the 
 * mechanical errors to zero. All parameters and gains are pre-scaled to the Per-Unit 
 * domain during initialization, ensuring zero physical unit conversions during the 
 * real-time interrupt execution.
 * * @par Example Usage:
 * @code
 * ctl_smc_mech_init_t smc_init;
 * ctl_smc_mech_ctrl_t smc_ctrl;
 * * // System Configurations
 * smc_init.fs = 10000.0f;
 * smc_init.mech_division = 10;
 * smc_init.inertia = 0.0002f;     // J (kg*m^2)
 * smc_init.torque_const = 0.5f;   // Kt (Nm/A)
 * smc_init.omega_base = 314.159f; // Base speed (rad/s)
 * smc_init.i_base = 20.0f;        // Base current (A)
 * smc_init.cur_limit = 1.0f;      // PU
 * * // SMC Tuning Targets
 * smc_init.target_bw = 30.0f;           // Determines lambda
 * smc_init.dist_reject_torque = 2.0f;   // Max disturbance torque (Nm) to reject
 * ctl_autotuning_smc_mech_ctrl(&smc_init);
 * * // Init & Run
 * ctl_init_smc_mech_ctrl(&smc_ctrl, &smc_init);
 * ctl_attach_smc_mech_ctrl(&smc_ctrl, &encoder_pos, &encoder_spd);
 * ctl_enable_smc_mech_ctrl(&smc_ctrl);
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
    parameter_gt target_bw;          //!< Target sliding surface decay bandwidth (Hz).
    parameter_gt dist_reject_torque; //!< Max disturbance torque to reject (Nm). Determines switching gain rho.

    // --- Auto-Tuned SMC Gains (PU Space) ---
    parameter_gt lambda; //!< PU Sliding surface slope.
    parameter_gt eta11;  //!< Pos Error gain (PU).
    parameter_gt eta12;  //!< Pos Error gain (PU).
    parameter_gt eta21;  //!< Vel Error gain (PU).
    parameter_gt eta22;  //!< Vel Error gain (PU).
    parameter_gt rho;    //!< Switching gain for sgn(s) term (PU).
    parameter_gt k_ff;   //!< Feedforward gain for external acceleration (PU).

} ctl_smc_mech_init_t;

/**
 * @brief Main structure for the SMC Mechanical Controller.
 */
typedef struct _tag_smc_mech_ctrl
{
    // --- Interfaces ---
    rotation_ift* pos_if; //!< Position feedback interface (PU).
    velocity_ift* spd_if; //!< Velocity feedback interface (PU).

    // --- Controller Modules ---
    ctl_smc_t smc_core;     //!< The core Sliding Mode Controller instance.
    ctl_divider_t div_mech; //!< Divider to down-sample execution.

    // --- Real-time Parameters & States ---
    ctrl_gt k_ff;        //!< Real-time Acceleration Feedforward Gain (PU).
    ctrl_gt cur_limit;   //!< Output saturation limit (PU).
    ctrl_gt cur_output;  //!< The final calculated current command (PU).
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
 * @brief Executes one step of the SMC mechanical loop entirely in PU.
 * @param[in,out] ctrl            Pointer to the controller.
 * @param[in]     target_revs     Target position (Full revolutions).
 * @param[in]     target_angle_pu Target position (Fractional angle 0~1 PU).
 * @param[in]     vel_ref_pu      Target velocity (PU).
 * @param[in]     acc_ref_pu      Target acceleration (PU/s).
 */
GMP_STATIC_INLINE void ctl_step_smc_mech_ctrl(ctl_smc_mech_ctrl_t* ctrl, int32_t target_revs, ctrl_gt target_angle_pu,
                                              ctrl_gt vel_ref_pu, ctrl_gt acc_ref_pu)
{
    if (!ctrl->flag_enable)
    {
        ctrl->cur_output = float2ctrl(0.0f);
        return;
    }

    if (ctl_step_divider(&ctrl->div_mech))
    {
        // 防御性断言：SMC 控制极度依赖精确的双状态反馈
        gmp_base_assert(ctrl->pos_if != NULL);
        gmp_base_assert(ctrl->spd_if != NULL);

        // 1. Calculate Primary State Variables (PU directly)
        // x1 = Position Error (Bounded PU via robust calculation)
        ctrl_gt x1 = ctl_calc_position_error(target_revs, target_angle_pu, ctrl->pos_if);

        // x2 = Velocity Error (PU)
        ctrl_gt x2 = vel_ref_pu - ctrl->spd_if->speed;

        // 2. Execute Sliding Mode Control Core (Zero constant multiplication needed!)
        ctrl_gt u_smc = ctl_step_smc(&ctrl->smc_core, x1, x2);

        // 3. Add Acceleration Feedforward (PU)
        ctrl_gt u_ff = ctl_mul(acc_ref_pu, ctrl->k_ff);

        // 4. Combine and Saturate (PU)
        ctrl_gt total_cmd = u_smc + u_ff;
        ctrl->cur_output = ctl_sat(total_cmd, ctrl->cur_limit, -ctrl->cur_limit);
    }
}

/**
 * @brief Gets the calculated current/torque command (PU).
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
