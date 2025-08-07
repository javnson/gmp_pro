/**
 * @file sinusoidal_trajectory.h
 * @brief Implements a complete, header-only Sinusoidal trajectory planner for position control.
 * @details This module generates a position profile based on a sinusoidal acceleration
 * profile (specifically, a cycloidal profile). This ensures that velocity and
 * acceleration are zero at both the start and end of the motion, providing
 * extremely smooth movement with continuous jerk. The planner generates a
 * trajectory to move from a starting point to a target point over a specified
 * duration.
 *
 * //tex:
 * // The trajectory is based on a cycloidal profile, where the position p(t) is given by:
 * // p(t) = p_{start} + \Delta p \left( \frac{t}{T} - \frac{1}{2\pi} \sin\left(\frac{2\pi t}{T}\right) \right)
 * // This results in a sinusoidal velocity profile:
 * // v(t) = \frac{\Delta p}{T} \left( 1 - \cos\left(\frac{2\pi t}{T}\right) \right)
 * // And a sinusoidal acceleration profile:
 * // a(t) = \frac{2\pi \Delta p}{T^2} \sin\left(\frac{2\pi t}{T}\right)
 *
 */

#ifndef _FILE_SINUSOIDAL_TRAJECTORY_H_
#define _FILE_SINUSOIDAL_TRAJECTORY_H_

#include <math.h> // For sinf
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Sinusoidal Trajectory Planner                                             */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup SINUSOIDAL_PLANNER Sinusoidal Trajectory Planner
 * @brief Generates cycloidal profiles for extremely smooth motion.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

#ifndef GMP_STATIC_INLINE
#define GMP_STATIC_INLINE static inline
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Define the standard control data type
typedef float ctrl_gt;

/**
 * @brief Main structure for the sinusoidal trajectory planner.
 */
typedef struct
{
    // --- State Variables ---
    ctrl_gt current_pos;  ///< The current output position of the planner.
    ctrl_gt current_time; ///< The elapsed time since the start of the current trajectory.
    uint8_t is_active;    ///< Flag indicating if a trajectory is currently being executed.

    // --- Trajectory Parameters ---
    ctrl_gt start_pos;  ///< The position at the beginning of the trajectory.
    ctrl_gt delta_pos;  ///< The total distance to travel (target_pos - start_pos).
    ctrl_gt total_time; ///< The total duration for the trajectory.

} ctl_sin_planner_t;

//================================================================================
// Function Definitions
//================================================================================

/**
 * @brief Initializes the sinusoidal planner.
 * @param[out] planner Pointer to the sinusoidal planner structure.
 * @param[in]  initial_pos The initial output position of the planner.
 */
GMP_STATIC_INLINE void ctl_init_sin_planner(ctl_sin_planner_t* planner, ctrl_gt initial_pos)
{
    planner->current_pos = initial_pos;
    planner->current_time = 0.0f;
    planner->is_active = 0;
    planner->start_pos = initial_pos;
    planner->delta_pos = 0.0f;
    planner->total_time = 0.0f;
}

/**
 * @brief Resets the planner to an idle state at its current position.
 * @param[out] planner Pointer to the sinusoidal planner structure.
 */
GMP_STATIC_INLINE void ctl_clear_sin_planner(ctl_sin_planner_t* planner)
{
    planner->is_active = 0;
    planner->current_time = 0.0f;
    planner->start_pos = planner->current_pos;
    planner->delta_pos = 0.0f;
}

/**
 * @brief Sets a new target position and duration, starting a new trajectory.
 * @param[out] planner Pointer to the sinusoidal planner structure.
 * @param[in]  target  The new target position.
 * @param[in]  time    The total time the trajectory should take. Must be > 0.
 */
GMP_STATIC_INLINE void ctl_set_sin_target(ctl_sin_planner_t* planner, ctrl_gt target, ctrl_gt time)
{
    if (time > 0.0f)
    {
        planner->start_pos = planner->current_pos;
        planner->delta_pos = target - planner->start_pos;
        planner->total_time = time;
        planner->current_time = 0.0f;
        planner->is_active = 1;
    }
}

/**
 * @brief Generates the next output position in the sinusoidal trajectory.
 * @details This function must be called periodically with a fixed time step (dt).
 * It calculates the next position setpoint based on the pre-planned
 * cycloidal trajectory.
 * @param[out] planner Pointer to the sinusoidal planner structure.
 * @param[in]  dt      The time elapsed since the last call (the control period).
 * @return The calculated position for the current time step.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_sin_planner(ctl_sin_planner_t* planner, ctrl_gt dt)
{
    if (!planner->is_active)
    {
        return planner->current_pos; // Not active, hold the last position.
    }

    // Increment the internal time
    planner->current_time += dt;

    if (planner->current_time >= planner->total_time)
    {
        // Trajectory finished, snap to the final position
        planner->current_time = planner->total_time;
        planner->is_active = 0;
        planner->current_pos = planner->start_pos + planner->delta_pos;
    }
    else
    {
        // Calculate position using the cycloidal formula
        ctrl_gt time_ratio = planner->current_time / planner->total_time;
        ctrl_gt angle = 2.0f * M_PI * time_ratio;
        ctrl_gt pos_fraction = time_ratio - (1.0f / (2.0f * M_PI)) * sinf(angle);

        planner->current_pos = planner->start_pos + (planner->delta_pos * pos_fraction);
    }

    return planner->current_pos;
}

/** @} */ // end of SINUSOIDAL_PLANNER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SINUSOIDAL_TRAJECTORY_H_
