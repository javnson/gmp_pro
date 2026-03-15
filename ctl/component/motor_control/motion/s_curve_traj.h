/**
 * @file s_curve_trajectory.h
 * @brief Implements a complete, header-only S-Curve trajectory planner.
 *
 */

#ifndef _FILE_S_CURVE_TRAJECTORY_H_
#define _FILE_S_CURVE_TRAJECTORY_H_

#include <math.h> // For fabsf
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* S-Curve Trajectory Planner                                                */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup S_CURVE_PLANNER S-Curve Trajectory Planner
 * @brief Generates smooth S-shaped velocity profiles.
 * @details This module generates a velocity profile with a trapezoidal acceleration
 * shape, resulting in an "S-shaped" velocity curve. This limits the rate
 * of change of acceleration (jerk), leading to smoother motion and reduced
 * mechanical stress. The planner supports dynamic re-targeting and can
 * generate both 7-segment (with a constant acceleration phase) and 5-segment
 * (triangular acceleration) profiles. All functions are defined as static
 * inline for performance.
 *
 *  The S-Curve profile is defined by 7 distinct phases:
 *  1. Constant positive jerk (acceleration increases linearly)
 *  2. Constant maximum acceleration
 *  3. Constant negative jerk (acceleration decreases linearly)
 *  4. Constant velocity (not explicitly managed by this velocity planner)
 *  5. Constant negative jerk (deceleration begins)
 *  6. Constant maximum deceleration
 *  7. Constant positive jerk (deceleration returns to zero)
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Defines the different states of the S-curve generation state machine.
 */
typedef enum
{
    SCURVE_STATE_IDLE = 0,    ///< Planner is inactive, holding current output.
    SCURVE_STATE_ACCEL_RAMP,  ///< Phase 1: Increasing acceleration (constant positive jerk).
    SCURVE_STATE_CONST_ACCEL, ///< Phase 2: Constant maximum acceleration.
    SCURVE_STATE_DECEL_RAMP,  ///< Phase 3 & 5: Ramping acceleration towards a new target.
    SCURVE_STATE_STOP,        ///< Final deceleration phase to zero acceleration.
} s_curve_state_t;

/**
 * @brief Main structure for the S-curve trajectory planner.
 */
typedef struct
{
    // --- Configuration ---
    ctrl_gt max_jerk;  ///< Maximum rate of change of acceleration (Jerk).
    ctrl_gt max_accel; ///< Maximum acceleration.

    // --- State Variables ---
    s_curve_state_t state; ///< Current state of the planner's state machine.
    ctrl_gt current_vel;   ///< The current output velocity of the planner.
    ctrl_gt current_accel; ///< The current internal acceleration.

    // --- Target & Planning Variables ---
    ctrl_gt target_vel;      ///< The desired final velocity.
    ctrl_gt last_target_vel; ///< Target velocity from the previous cycle, to detect changes.
    ctrl_gt accel_target;    ///< The target acceleration for the current phase.
    ctrl_gt accel_dir;       ///< Direction of acceleration (+1.0f or -1.0f).

    // --- Timing & Step Calculation ---
    uint32_t const_accel_steps; ///< Number of steps for the constant acceleration phase.
    uint32_t step_counter;      ///< Counter for the constant acceleration phase.

} ctl_s_curve_planner_t;

//================================================================================
// Function Definitions
//================================================================================

/**
 * @brief Initializes the S-curve planner with given parameters.
 * @param[out] planner Pointer to the S-curve planner structure.
 * @param[in]  max_accel The maximum acceleration desired.
 * @param[in]  max_jerk The maximum jerk (rate of change of acceleration) desired.
 * @param[in]  initial_vel The initial output velocity of the planner.
 */
void ctl_init_s_curve(ctl_s_curve_planner_t* planner, ctrl_gt max_accel, ctrl_gt max_jerk, ctrl_gt initial_vel);

/**
 * @brief Resets the internal states of the planner to idle.
 * @param[out] planner Pointer to the S-curve planner structure.
 */
GMP_STATIC_INLINE void ctl_clear_s_curve(ctl_s_curve_planner_t* planner)
{
    planner->state = SCURVE_STATE_IDLE;
    planner->current_accel = 0.0f;
    // Keep current_vel and target_vel as they are, to allow for smooth restart.
}

/**
 * @brief Sets a new target velocity for the planner.
 * @param[out] planner Pointer to the S-curve planner structure.
 * @param[in]  target  The new target velocity.
 */
GMP_STATIC_INLINE void ctl_set_s_curve_target(ctl_s_curve_planner_t* planner, ctrl_gt target)
{
    planner->target_vel = target;
}

/**
 * @brief Plans a new trajectory when the target changes.
 * @details This internal function is called by ctl_step_s_curve.
 * @param[out] planner Pointer to the S-curve planner structure.
 */
GMP_STATIC_INLINE void _ctl_s_curve_replan(ctl_s_curve_planner_t* planner)
{
    // Determine direction
    ctrl_gt delta_v = planner->target_vel - planner->current_vel;
    planner->accel_dir = (delta_v > 0.0f) ? 1.0f : -1.0f;

    // Time to ramp acceleration to max_accel
    ctrl_gt t_jerk = planner->max_accel / planner->max_jerk;
    // Velocity change during acceleration ramp-up
    ctrl_gt v_jerk = planner->max_jerk * t_jerk * t_jerk;

    if (fabsf(delta_v) > v_jerk)
    {
        // 7-segment profile (trapezoidal acceleration)
        ctrl_gt v_const_accel = fabsf(delta_v) - v_jerk;
        planner->const_accel_steps = (uint32_t)(v_const_accel / planner->max_accel);
    }
    else
    {
        // 5-segment profile (triangular acceleration)
        planner->const_accel_steps = 0;
        t_jerk = sqrtf(fabsf(delta_v) / planner->max_jerk);
    }

    planner->accel_target = planner->max_accel * planner->accel_dir;
    planner->step_counter = 0;
    planner->state = SCURVE_STATE_ACCEL_RAMP;
    planner->last_target_vel = planner->target_vel;
}

/**
 * @brief Generates the next output value in the S-curve trajectory.
 * @details This function must be called periodically (e.g., in a control ISR).
 * It contains the state machine that calculates the jerk, acceleration,
 * and final output velocity for each time step.
 * @param[out] planner Pointer to the S-curve planner structure.
 * @return The calculated velocity for the current time step.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_s_curve(ctl_s_curve_planner_t* planner)
{
    // Check if the target has changed, requiring a new plan
    if (planner->target_vel != planner->last_target_vel)
    {
        _ctl_s_curve_replan(planner);
    }

    ctrl_gt jerk = 0.0f;

    switch (planner->state)
    {
    case SCURVE_STATE_IDLE:
        // Do nothing, hold current velocity
        break;

    case SCURVE_STATE_ACCEL_RAMP:
        jerk = planner->max_jerk * planner->accel_dir;
        if (fabsf(planner->current_accel) >= fabsf(planner->accel_target))
        {
            planner->current_accel = planner->accel_target;
            planner->state = SCURVE_STATE_CONST_ACCEL;
        }
        break;

    case SCURVE_STATE_CONST_ACCEL:
        jerk = 0.0f;
        planner->step_counter++;
        if (planner->step_counter >= planner->const_accel_steps)
        {
            planner->accel_target = 0.0f; // Target zero acceleration
            planner->state = SCURVE_STATE_DECEL_RAMP;
        }
        break;

    case SCURVE_STATE_DECEL_RAMP:
        jerk = -planner->max_jerk * planner->accel_dir;
        // Check if we are approaching the target velocity
        if (fabsf(planner->target_vel - planner->current_vel) < fabsf(planner->current_accel))
        {
            planner->state = SCURVE_STATE_STOP;
        }
        break;

    case SCURVE_STATE_STOP:
        // Ramp acceleration to zero
        if (fabsf(planner->current_accel) < planner->max_jerk)
        {
            planner->current_accel = 0.0f;
            planner->current_vel = planner->target_vel; // Snap to final value
            planner->state = SCURVE_STATE_IDLE;
        }
        else
        {
            jerk = (planner->current_accel > 0.0f) ? -planner->max_jerk : planner->max_jerk;
        }
        break;
    }

    // Integrate jerk to get acceleration, then acceleration to get velocity
    if (planner->state != SCURVE_STATE_IDLE)
    {
        planner->current_accel += jerk;
        planner->current_vel += planner->current_accel;
    }

    return planner->current_vel;
}

/** @} */ // end of S_CURVE_PLANNER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_S_CURVE_TRAJECTORY_H_
