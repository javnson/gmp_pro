/**
 * @file mech_ctrl.h
 * @brief Implements an advanced mechanical loop (Velocity/Position) controller.
 *
 * @version 1.0
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
 * @defgroup MECHANICAL_CONTROLLER Mechanical Loop Controller
 * @brief Cascaded P-PI controller for Position and Velocity regulation.
 * @details Implements a standard Servo P-PI architecture. The position loop acts as a 
 * proportional (P) controller generating a velocity reference. The velocity loop is a 
 * PI controller generating a torque/current reference. Features state-machine mode 
 * switching and bumpless transfer (integrator pre-loading).
 * * @par Example Usage:
 * @code
 * // 1. Instantiate
 * ctl_mech_ctrl_init_t mech_init;
 * ctl_mech_ctrl_t mech_ctrl;
 * * // 2. Provide hardware and system parameters
 * mech_init.fs_current = 10000.0f;     // Inner current loop runs at 10kHz
 * mech_init.mech_division = 10;        // Mech loop runs at 1kHz (10000 / 10)
 * mech_init.inertia = 0.0002f;         // System inertia (kg*m^2)
 * mech_init.torque_const = 0.5f;       // Motor torque constant (Nm/A)
 * * mech_init.speed_limit = 300.0f;      // Max speed (rad/s)
 * mech_init.cur_limit = 20.0f;         // Max current (A)
 * mech_init.speed_slope_limit = 10.0f; // Max accel per step
 * * // 3. Auto-tune PI based on target bandwidth
 * mech_init.target_vel_bw = 20.0f;     // Velocity loop BW: 20 Hz
 * mech_init.target_pos_bw = 5.0f;      // Position loop BW: 5 Hz
 * ctl_autotuning_mech_ctrl(&mech_init);
 * * // 4. Initialize
 * ctl_init_mech_ctrl(&mech_ctrl, &mech_init);
 * ctl_attach_mech_ctrl(&mech_ctrl, &encoder_pos, &encoder_spd);
 * * // 5. Switch to Velocity Mode smoothly (Bumpless Transfer)
 * // Assume `real_iq` is the current feedback from the FOC engine
 * ctl_set_mech_mode(&mech_ctrl, MECH_MODE_VELOCITY, real_iq);
 * ctl_set_mech_target_velocity(&mech_ctrl, 100.0f); // Set 100 rad/s
 * * // 6. Real-time loop (Inside Main ISR, running at 10kHz)
 * void Main_ISR(void) {
 * // ... current loop reads ADC ...
 * * // mech loop will internally divide frequency and only run at 1kHz
 * ctl_step_mech_ctrl(&mech_ctrl); 
 * * // Feed this to current distributor
 * target_im = ctl_get_mech_cmd(&mech_ctrl); 
 * }
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
    parameter_gt fs_current; //!< Execution frequency of the inner current loop (Hz).
    uint32_t mech_division;  //!< Divider ratio (e.g., 5-10) for the mechanical loop.

    parameter_gt inertia;      //!< Total system inertia J (kg*m^2).
    parameter_gt damping;      //!< Viscous friction coefficient B (Nm/(rad/s)). Optional, often 0.
    parameter_gt torque_const; //!< Motor torque constant Kt (Nm/A).

    // --- Safety Limits ---
    parameter_gt speed_limit;       //!< Absolute maximum speed reference (rad/s or PU).
    parameter_gt speed_slope_limit; //!< Maximum velocity slew rate (accel/decel profile) per step.
    parameter_gt cur_limit;         //!< Absolute maximum current/torque output reference (A or PU).

    // --- Tuning Targets ---
    parameter_gt target_vel_bw; //!< Target bandwidth for velocity loop (Hz). Typically 10~50Hz.
    parameter_gt target_pos_bw; //!< Target bandwidth for position loop (Hz). Typically Vel_BW / (3~5).

    // --- Auto-Tuned PI Gains ---
    parameter_gt vel_kp; //!< Calculated Proportional gain for velocity loop.
    parameter_gt vel_ki; //!< Calculated Integral gain for velocity loop.
    parameter_gt pos_kp; //!< Calculated Proportional gain for position loop (P-only).

} ctl_mech_ctrl_init_t;

/**
 * @brief Main structure for the Mechanical Controller.
 */
typedef struct _tag_mech_ctrl
{
    // --- Interfaces ---
    rotation_ift* pos_if; //!< Standard rotation feedback interface.
    velocity_ift* spd_if; //!< Standard velocity feedback interface.

    // --- State Machine ---
    ctl_mech_mode_e active_mode; //!< Current operating mode.

    // --- Sub-modules ---
    ctl_pid_t vel_ctrl;           //!< Velocity PI controller.
    ctl_pid_t pos_ctrl;           //!< Position P controller.
    ctl_slope_limiter_t vel_traj; //!< Velocity trajectory/profile generator.
    ctl_divider_t div_mech;       //!< Divider to down-sample from current loop freq.

    // --- Targets & Internal States ---
    int32_t target_revs;     //!< Target position (full revolutions).
    ctrl_gt target_angle;    //!< Target position (fractional angle 0~1).
    ctrl_gt target_velocity; //!< Raw target velocity (before slope limiter).

    ctrl_gt speed_limit; //!< Dynamic speed saturation limit.
    ctrl_gt torque_cmd;  //!< The final calculated current/torque command.

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
 * @param[in,out] ctrl Pointer to the mechanical controller.
 * @param[in] mode The target operating mode.
 * @param[in] current_feedback The ACTUAL Iq current (or torque) measured right now. Used to pre-load the integrator.
 */
GMP_STATIC_INLINE void ctl_set_mech_mode(ctl_mech_ctrl_t* ctrl, ctl_mech_mode_e mode, ctrl_gt current_feedback)
{
    if (ctrl->active_mode == mode)
        return;

    if (mode == MECH_MODE_VELOCITY || mode == MECH_MODE_POSITION)
    {
        // --- Bumpless Transfer Logic ---
        // 1. Pre-load the velocity integrator with the current actual output
        ctl_set_pid_integrator(&ctrl->vel_ctrl, current_feedback);

        // 2. Synchronize the trajectory generator to current actual speed to prevent jumping
        ctrl_gt actual_spd = (ctrl->spd_if != NULL) ? ctrl->spd_if->speed : float2ctrl(0.0f);
        ctl_set_slope_limiter_current(&ctrl->vel_traj, actual_spd);
        ctrl->target_velocity = actual_spd;

        // 3. If entering Position mode, lock target to current actual position
        if (mode == MECH_MODE_POSITION && ctrl->pos_if != NULL)
        {
            ctrl->target_revs = ctrl->pos_if->revolutions;
            ctrl->target_angle = ctrl->pos_if->position;
            ctl_clear_pid(&ctrl->pos_ctrl); // Clear any lingering P-error states
        }
    }
    else // MECH_MODE_DISABLE
    {
        ctl_clear_mech_ctrl(ctrl); // Wipe states when disabling
    }

    ctrl->active_mode = mode;
}

/**
 * @brief Sets the target velocity for Velocity Mode.
 */
GMP_STATIC_INLINE void ctl_set_mech_target_velocity(ctl_mech_ctrl_t* ctrl, ctrl_gt spd)
{
    ctrl->target_velocity = ctl_sat(spd, ctrl->speed_limit, -ctrl->speed_limit);
}

/**
 * @brief Sets the target position for Position Mode.
 */
GMP_STATIC_INLINE void ctl_set_mech_target_position(ctl_mech_ctrl_t* ctrl, int32_t revs, ctrl_gt angle)
{
    ctrl->target_revs = revs;
    ctrl->target_angle = angle; // Assuming 0.0 to 1.0 PU
}

/**
 * @brief Executes one step of the mechanical control loop.
 * @details Checks the divider internally. Must be called at the fast current loop frequency.
 */
GMP_STATIC_INLINE void ctl_step_mech_ctrl(ctl_mech_ctrl_t* ctrl)
{
    if (ctrl->active_mode == MECH_MODE_DISABLE)
    {
        ctrl->torque_cmd = float2ctrl(0.0f);
        return;
    }

    // Only execute if the mechanical division period has triggered
    if (ctl_step_divider(&ctrl->div_mech))
    {
        ctrl_gt vel_cmd = ctrl->target_velocity; // Default to direct velocity target

        // 1. Position Control Loop (Outer Loop)
        if (ctrl->active_mode == MECH_MODE_POSITION)
        {
            // Calculate position error
            ctrl_gt pos_error = ctl_calc_position_error(ctrl->target_revs, ctrl->target_angle, ctrl->pos_if);

            // P-Controller generates raw velocity command
            vel_cmd = ctl_step_pid_par(&ctrl->pos_ctrl, pos_error);
        }

        // 2. Trajectory Profile (Smooth the velocity command)
        ctrl_gt smoothed_vel_cmd = ctl_step_slope_limiter(&ctrl->vel_traj, vel_cmd);

        // 3. Velocity Control Loop (Inner Loop)
        ctrl_gt spd_error = smoothed_vel_cmd - ctrl->spd_if->speed;

        // PI-Controller generates current/torque command
        ctrl->torque_cmd = ctl_step_pid_par(&ctrl->vel_ctrl, spd_error);
    }
}

/**
 * @brief Executes one step of the mechanical control loop using PIP (P-IPD) architecture.
 * @details 
 * - Outer Loop (Position): Uses standard P control (no overshoot by design).
 * - Inner Loop (Velocity): Uses IPD control (Proportional and Derivative terms act 
 * ONLY on feedback, Integral acts on error) to completely eliminate speed 
 * overshoot during step responses or trajectory tracking.
 * @param[in,out] ctrl Pointer to the mechanical controller structure.
 */
GMP_STATIC_INLINE void ctl_step_mech_ctrl_pip(ctl_mech_ctrl_t* ctrl)
{
    if (ctrl->active_mode == MECH_MODE_DISABLE)
    {
        ctrl->cur_output = float2ctrl(0.0f);
        return;
    }

    // Only execute if the mechanical division period has triggered
    if (ctl_step_divider(&ctrl->div_mech))
    {
        ctrl_gt vel_cmd = ctrl->target_velocity; // Default to direct velocity target

        // 1. Position Control Loop (Outer Loop)
        if (ctrl->active_mode == MECH_MODE_POSITION)
        {
            int32_t rev_error = ctrl->target_revs - ctrl->pos_if->revolutions;
            ctrl_gt ang_error = ctrl->target_angle - ctrl->pos_if->position;

            ctrl_gt pos_error = (ctrl_gt)rev_error + ang_error;

            // P-Controller generates raw velocity command
            // Since pos_ctrl is initialized with Ki=0, Kd=0, ctl_step_pid_par acts as a pure P-controller
            vel_cmd = ctl_step_pid_par(&ctrl->pos_ctrl, pos_error);
        }

        // 2. Trajectory Profile (Smooth the velocity command)
        ctrl_gt smoothed_vel_cmd = ctl_step_slope_limiter(&ctrl->vel_traj, vel_cmd);

        // 3. Velocity Control Loop (Inner Loop - IPD Structure)
        // [核心拓展]：调用 IPD 步进函数，拆分 target 和 feedback
        // P和D项仅作用于实际速度，避免给定值突变带来的比例踢击(Proportional Kick)
        ctrl->cur_output = ctl_step_ipd_par(&ctrl->vel_ctrl, smoothed_vel_cmd, ctrl->spd_if->speed);
    }
}

/**
 * @brief Gets the calculated current/torque command.
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
