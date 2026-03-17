/**
 * @file basic_mech_ctrl.h
 * @brief Implements an advanced mechanical loop (Velocity/Position) controller in Per-Unit (PU) system.
 *
 * @version 1.1
 * @date 2024-10-26
 *
 */

#ifndef _FILE_MECH_CTRL_H_
#define _FILE_MECH_CTRL_H_

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/motor_control/interface/encoder.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Mechanical Controller (Velocity & Position)                               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MECHANICAL_CONTROLLER Mechanical Loop Controller (PU System)
 * @brief Cascaded P-PI controller for Position and Velocity regulation using Per-Unit values.
 * @details Implements a standard Servo P-PI architecture completely in the PU domain. 
 * The position loop takes PU position error and generates PU velocity reference. 
 * The velocity loop takes PU velocity error and generates PU current/torque reference.
 * * @par Example Usage:
 * @code
 * ctl_mech_ctrl_init_t mech_init;
 * ctl_mech_ctrl_t mech_ctrl;
 * * mech_init.fs_current = 10000.0f;
 * mech_init.mech_division = 10;
 * mech_init.inertia = 0.0002f;         // J (kg*m^2)
 * mech_init.torque_const = 0.5f;       // Kt (Nm/A)
 * mech_init.omega_base = 314.159f;     // Nominal mechanical speed (rad/s)
 * mech_init.i_base = 20.0f;            // Nominal current (A)
 * * ctl_autotuning_mech_ctrl(&mech_init);
 * ctl_init_mech_ctrl(&mech_ctrl, &mech_init);
 * @endcode
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Available operating modes for the mechanical controller.
 */
typedef enum
{
    MECH_MODE_DISABLE = 0,  //!< Controller is disabled, output is 0.
    MECH_MODE_VELOCITY = 1, //!< Velocity mode (PI).
    MECH_MODE_POSITION = 2  //!< Position mode (Cascaded P-PI).
} ctl_mech_mode_e;

/**
 * @brief Initialization parameters for the Mechanical Controller.
 */
typedef struct _tag_mech_ctrl_init
{
    // --- System & Hardware Configurations ---
    parameter_gt fs; //!< Execution frequency of the inner current loop (Hz).
    uint32_t mech_division;  //!< Divider ratio (e.g., 5-10) for the mechanical loop.

    parameter_gt inertia;      //!< Total system inertia J (kg*m^2).
    parameter_gt damping;      //!< Viscous friction coefficient B (Nm/(rad/s)). Optional, often 0.
    parameter_gt torque_const; //!< Motor torque constant Kt (Nm/A).

    // --- Per-Unit Base Values (CRITICAL FOR PU TUNING) ---
    parameter_gt omega_base; //!< Base mechanical speed for per-unit conversion (rad/s).
    parameter_gt i_base;     //!< Base current for per-unit conversion (A).

    // --- Safety Limits (All in PU) ---
    parameter_gt speed_limit;       //!< Absolute maximum speed reference (PU).
    parameter_gt speed_slope_limit; //!< Maximum velocity slew rate (PU/s).
    parameter_gt cur_limit;         //!< Absolute maximum current/torque output reference (PU).

    // --- Tuning Targets ---
    parameter_gt target_vel_bw; //!< Target bandwidth for velocity loop (Hz).
    parameter_gt target_pos_bw; //!< Target bandwidth for position loop (Hz).

    // --- Auto-Tuned PU Gains ---
    parameter_gt vel_kp; //!< Calculated PU Proportional gain for velocity loop.
    parameter_gt vel_ki; //!< Calculated PU Integral gain for velocity loop.
    parameter_gt pos_kp; //!< Calculated PU Proportional gain for position loop (P-only).

} ctl_mech_ctrl_init_t;

/**
 * @brief Main structure for the Mechanical Controller.
 */
typedef struct _tag_mech_ctrl
{
    // --- Interfaces ---
    rotation_ift* pos_if; //!< Standard rotation feedback interface (PU).
    velocity_ift* spd_if; //!< Standard velocity feedback interface (PU).

    // --- State Machine ---
    ctl_mech_mode_e active_mode; //!< Current operating mode.

    // --- Sub-modules ---
    ctl_pid_t vel_ctrl;           //!< Velocity PI/IPD controller.
    ctl_pid_t pos_ctrl;           //!< Position P controller.
    ctl_slope_limiter_t vel_traj; //!< Velocity trajectory/profile generator.
    ctl_divider_t div_mech;       //!< Divider to down-sample from current loop freq.

    // --- Targets & Internal States (All in PU) ---
    int32_t target_revs;     //!< Target position (full revolutions).
    ctrl_gt target_angle;    //!< Target position (fractional angle 0~1 PU).
    ctrl_gt target_velocity; //!< Raw target velocity (PU).

    ctrl_gt speed_limit; //!< Dynamic speed saturation limit (PU).
    ctrl_gt torque_cmd;  //!< The final calculated current/torque command (PU).

} ctl_mech_ctrl_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_autotuning_mech_ctrl(ctl_mech_ctrl_init_t* init);
void ctl_init_mech_ctrl(ctl_mech_ctrl_t* ctrl, const ctl_mech_ctrl_init_t* init);

/**
 * @brief Attaches feedback interfaces to the mechanical controller.
 */
GMP_STATIC_INLINE void ctl_attach_mech_ctrl(ctl_mech_ctrl_t* ctrl, rotation_ift* pos_if, velocity_ift* spd_if)
{
    ctrl->pos_if = pos_if;
    ctrl->spd_if = spd_if;
}

/**
 * @brief Clears the internal states of the mechanical loop.
 */
GMP_STATIC_INLINE void ctl_clear_mech_ctrl(ctl_mech_ctrl_t* ctrl)
{
    ctl_clear_pid(&ctrl->vel_ctrl);
    ctl_clear_pid(&ctrl->pos_ctrl);
    ctl_clear_slope_limiter(&ctrl->vel_traj);
    ctl_clear_divider(&ctrl->div_mech);
    ctrl->torque_cmd = float2ctrl(0.0f);
}

/**
 * @brief Sets the operating mode with Bumpless Transfer.
 * @param[in] current_feedback The ACTUAL Iq current (PU) measured right now to pre-load the integrator.
 */
GMP_STATIC_INLINE void ctl_set_mech_mode(ctl_mech_ctrl_t* ctrl, ctl_mech_mode_e mode, ctrl_gt current_feedback)
{


    if (ctrl->active_mode == mode)
        return;

    if (mode == MECH_MODE_VELOCITY || mode == MECH_MODE_POSITION)
    {
        // Bumpless Transfer Logic
        ctl_set_pid_integrator(&ctrl->vel_ctrl, current_feedback);

        ctrl_gt actual_spd = (ctrl->spd_if != NULL) ? ctrl->spd_if->speed : float2ctrl(0.0f);
        ctl_set_slope_limiter_current(&ctrl->vel_traj, actual_spd);
        ctrl->target_velocity = actual_spd;

        if (mode == MECH_MODE_POSITION && ctrl->pos_if != NULL)
        {
            ctrl->target_revs = ctrl->pos_if->revolutions;
            ctrl->target_angle = ctrl->pos_if->position;
            ctl_clear_pid(&ctrl->pos_ctrl);
        }
    }
    else
    {
        ctl_clear_mech_ctrl(ctrl);
    }

    ctrl->active_mode = mode;
}

/**
 * @brief Sets the target velocity for Velocity Mode.
 */
GMP_STATIC_INLINE void ctl_set_mech_target_velocity(ctl_mech_ctrl_t* ctrl, ctrl_gt spd_pu)
{
    ctrl->target_velocity = ctl_sat(spd_pu, ctrl->speed_limit, -ctrl->speed_limit);
}

/**
 * @brief Sets the target position for Position Mode.
 */
GMP_STATIC_INLINE void ctl_set_mech_target_position(ctl_mech_ctrl_t* ctrl, int32_t revs, ctrl_gt angle_pu)
{
    ctrl->target_revs = revs;
    ctrl->target_angle = angle_pu;
}

/**
 * @brief Executes one step of the mechanical control loop using standard P-PI.
 */
GMP_STATIC_INLINE void ctl_step_mech_ctrl(ctl_mech_ctrl_t* ctrl)
{
    if (ctrl->active_mode == MECH_MODE_DISABLE)
    {
        ctrl->torque_cmd = float2ctrl(0.0f);
        return;
    }

    if (ctl_step_divider(&ctrl->div_mech))
    {
        gmp_base_assert(ctrl->spd_if != NULL);

        ctrl_gt vel_cmd_pu = ctrl->target_velocity;

        if (ctrl->active_mode == MECH_MODE_POSITION)
        {
            gmp_base_assert(ctrl->pos_if != NULL);

            ctrl_gt pos_error_pu = ctl_calc_position_error(ctrl->target_revs, ctrl->target_angle, ctrl->pos_if);
            vel_cmd_pu = ctl_step_pid_par(&ctrl->pos_ctrl, pos_error_pu);
        }

        ctrl_gt smoothed_vel_cmd_pu = ctl_step_slope_limiter(&ctrl->vel_traj, vel_cmd_pu);
        ctrl_gt spd_error_pu = smoothed_vel_cmd_pu - ctrl->spd_if->speed;

        ctrl->torque_cmd = ctl_step_pid_par(&ctrl->vel_ctrl, spd_error_pu);
    }
}

/**
 * @brief Executes one step of the mechanical control loop using PIP (P-IPD) architecture.
 */
GMP_STATIC_INLINE void ctl_step_mech_ctrl_pip(ctl_mech_ctrl_t* ctrl)
{
    if (ctrl->active_mode == MECH_MODE_DISABLE)
    {
        ctrl->torque_cmd = float2ctrl(0.0f);
        return;
    }

    if (ctl_step_divider(&ctrl->div_mech))
    {
        gmp_base_assert(ctrl->spd_if != NULL);

        ctrl_gt vel_cmd_pu = ctrl->target_velocity;

        if (ctrl->active_mode == MECH_MODE_POSITION)
        {
            gmp_base_assert(ctrl->pos_if != NULL);

            ctrl_gt pos_error_pu = ctl_calc_position_error(ctrl->target_revs, ctrl->target_angle, ctrl->pos_if);
            vel_cmd_pu = ctl_step_pid_par(&ctrl->pos_ctrl, pos_error_pu);
        }

        ctrl_gt smoothed_vel_cmd_pu = ctl_step_slope_limiter(&ctrl->vel_traj, vel_cmd_pu);

        // PIP Mode
        ctrl->torque_cmd = ctl_step_ipd_par(&ctrl->vel_ctrl, smoothed_vel_cmd_pu, ctrl->spd_if->speed);
    }
}

/**
 * @brief Gets the calculated current/torque command (in PU).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mech_cmd(const ctl_mech_ctrl_t* ctrl)
{
    return ctrl->torque_cmd;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MECH_CTRL_H_
