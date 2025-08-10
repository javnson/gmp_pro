/**
 * @file trapezoidal_trajectory.h
 * @brief Implements a complete, header-only Trapezoidal trajectory planner for position control.
 * @details This module generates a position profile based on a trapezoidal velocity
 * profile. It ensures that the motion from a start point to a target point
 * adheres to specified maximum velocity and acceleration limits. The planner
 * continuously calculates the required acceleration based on the distance to
 * the target and the braking distance required from the current velocity.
 *
 * //tex:
 * // The velocity profile is a trapezoid, consisting of three phases:
 * // 1. Constant acceleration: v(t) = a_{max} \cdot t
 * // 2. Constant velocity: v(t) = v_{max}
 * // 3. Constant deceleration: v(t) = v_{max} - a_{max} \cdot t
 *
 */

#ifndef _FILE_TRAPEZOIDAL_TRAJECTORY_H_
#define _FILE_TRAPEZOIDAL_TRAJECTORY_H_

#include <math.h> // For fabsf and signbit
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Trapezoidal Trajectory Planner                                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup TRAPEZOIDAL_PLANNER Trapezoidal Trajectory Planner
 * @brief Generates trapezoidal velocity profiles for position control.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Main structure for the trapezoidal trajectory planner.
 */
typedef struct
{
    // --- Configuration ---
    ctrl_gt max_vel;      ///< Maximum velocity limit.
    ctrl_gt max_accel;    ///< Maximum acceleration limit.
    ctrl_gt pos_deadband; ///< The deadband around the target position to stop motion.

    // --- State Variables ---
    ctrl_gt current_pos; ///< The current output position of the planner.
    ctrl_gt current_vel; ///< The current internal velocity.

    // --- Target ---
    ctrl_gt target_pos; ///< The desired final position.

} ctl_trap_planner_t;

//================================================================================
// Function Definitions
//================================================================================

/**
 * @brief Initializes the trapezoidal planner with given parameters.
 * @param[out] planner Pointer to the trapezoidal planner structure.
 * @param[in]  max_vel The maximum velocity desired.
 * @param[in]  max_accel The maximum acceleration desired.
 * @param[in]  initial_pos The initial output position of the planner.
 */
void ctl_init_trap_planner(ctl_trap_planner_t* planner, ctrl_gt max_vel, ctrl_gt max_accel, ctrl_gt initial_pos);

/**
 * @brief Resets the internal states of the planner.
 * @param[out] planner Pointer to the trapezoidal planner structure.
 */
GMP_STATIC_INLINE void ctl_clear_trap_planner(ctl_trap_planner_t* planner)
{
    planner->current_vel = 0.0f;
    // Keep current_pos and target_pos to allow for smooth restart.
}

/**
 * @brief Sets a new target position for the planner.
 * @param[out] planner Pointer to the trapezoidal planner structure.
 * @param[in]  target  The new target position.
 */
GMP_STATIC_INLINE void ctl_set_trap_target(ctl_trap_planner_t* planner, ctrl_gt target)
{
    planner->target_pos = target;
}

/**
 * @brief Generates the next output position in the trapezoidal trajectory.
 * @details This function must be called periodically (e.g., in a control ISR).
 * It calculates the required acceleration and integrates it to produce the
 * next position setpoint, ensuring velocity and acceleration limits are met.
 * @param[out] planner Pointer to the trapezoidal planner structure.
 * @return The calculated position for the current time step.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_trap_planner(ctl_trap_planner_t* planner)
{
    ctrl_gt dist_to_go = planner->target_pos - planner->current_pos;
    ctrl_gt accel = 0.0f;

    // Check if we are within the target deadband
    if (fabsf(dist_to_go) < planner->pos_deadband)
    {
        planner->current_vel = 0.0f;
        return planner->current_pos;
    }

    // Determine the direction of motion
    ctrl_gt direction = (dist_to_go > 0.0f) ? 1.0f : -1.0f;

    // --- Core Logic: Decide whether to accelerate or decelerate ---

    // 1. Calculate the distance required to stop from the current velocity
    ctrl_gt brake_dist = (planner->current_vel * planner->current_vel) / (2.0f * planner->max_accel);

    // 2. Check if we are within the braking distance
    //    and also moving towards the target (velocity has same sign as dist_to_go)
    int moving_towards_target = !(signbit(planner->current_vel) ^ signbit(dist_to_go));

    if (moving_towards_target && (fabsf(dist_to_go) <= brake_dist))
    {
        // Inside braking zone: must decelerate
        accel = -direction * planner->max_accel;
    }
    else
    {
        // Outside braking zone: accelerate towards target
        accel = direction * planner->max_accel;
    }

    // --- Integration and Saturation ---

    // 3. Integrate acceleration to get new velocity
    planner->current_vel += accel;

    // 4. Saturate velocity at the configured limit
    if (fabsf(planner->current_vel) > planner->max_vel)
    {
        planner->current_vel = direction * planner->max_vel;
    }

    // 5. Integrate velocity to get new position
    planner->current_pos += planner->current_vel;

    return planner->current_pos;
}

/** @} */ // end of TRAPEZOIDAL_PLANNER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_TRAPEZOIDAL_TRAJECTORY_H_
