/**
 * @file knob_pos_loop.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements a haptic knob position loop with discrete detents.
 * @details This module creates a "knob" effect by defining a series of stable setpoints (detents).
 * When the motor's actual position is near a detent, a proportional controller generates a
 * speed reference to pull the motor towards the center of that detent, simulating the feel
 * of a physical knob.
 * @version 0.2
 * @date 2024-10-02
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_KNOB_POS_LOOP_H_
#define _FILE_KNOB_POS_LOOP_H_

#include <ctl/component/intrinsic/discrete/divider.h>
#include <ctl/component/motor_control/motion/basic_pos_loop_p.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup KNOB_POSITION_LOOP Haptic Knob Position Loop
 * @brief A position controller that simulates discrete detents.
 *
 * This controller calculates a target position corresponding to the nearest "snap" point
 * and uses a P-controller to generate a restoring force (as a speed command) towards it.
 */

/*---------------------------------------------------------------------------*/
/* Haptic Knob Position Loop                                                 */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup KNOB_POSITION_LOOP
 * @{
 */

/**
 * @brief Data structure for the haptic knob position loop controller.
 */
typedef struct _tag_pmsm_knob_pos_loop
{
    //
    // Outputs
    //
    int16_t output_pos; /**< The current detent index that the knob is snapped to. */
    ctrl_gt speed_ref;  /**< The calculated speed reference output. */

    //
    // Parameters
    //
    uint16_t knob_step;  /**< The number of detents (steps) in one full rotation. */
    ctrl_gt kp;          /**< The proportional gain (Kp) for the restoring force. */
    ctrl_gt speed_limit; /**< The maximum and minimum limit for the speed reference output. */
    ctl_divider_t div;   /**< A divider to run the loop at a lower frequency than the main ISR. */

    //
    // State Variables
    //
    int32_t target_pos; /**< The target position (integer part, full rotations). */
    ctrl_gt target_ang; /**< The target angle (fractional part), corresponding to the center of the current detent. */
    int32_t actual_pos; /**< The actual feedback position (integer part). */
    ctrl_gt actual_ang; /**< The actual feedback angle (fractional part). */

} ctl_pmsm_knob_pos_loop;

void ctl_init_knob_pos_loop(ctl_pmsm_knob_pos_loop* knob);
void ctl_setup_knob_pos_loop(ctl_pmsm_knob_pos_loop* knob, ctrl_gt kp, ctrl_gt speed_limit, uint16_t knob_step,
                             uint32_t division);

/**
 * @brief Executes one step of the haptic knob control logic.
 * @details This function calculates the nearest detent position and computes a speed
 * reference to move towards it.
 * @param knob Pointer to the `ctl_pmsm_knob_pos_loop` object.
 */
GMP_STATIC_INLINE void ctl_step_knob_pos_loop(ctl_pmsm_knob_pos_loop* knob)
{
    if (ctl_step_divider(&knob->div))
    {
        // Calculate the angular width of a single detent step.
        ctrl_gt step_width = GMP_CONST_1 / knob->knob_step;
        ctrl_gt half_step_width = step_width / 2;

        // Determine the index of the current detent based on the actual angle.
        knob->output_pos = (int16_t)(knob->actual_ang / step_width);

        // Set the target angle to be the center of the current detent.
        knob->target_ang = knob->output_pos * step_width + half_step_width;
        knob->target_pos = knob->actual_pos;

        // --- P-Controller Logic ---
        // Calculate position error.
        int32_t delta_pos = knob->target_pos - knob->actual_pos;

        // Clamp the multi-turn error to prevent excessive speed when crossing many detents at once.
        delta_pos = (delta_pos > 2) ? 2 : delta_pos;
        delta_pos = (delta_pos < -2) ? -2 : delta_pos;

        // Combine integer and fractional parts into a single error value.
        ctrl_gt position_error = GMP_CONST_2_PI * (ctrl_gt)delta_pos + knob->target_ang - knob->actual_ang;

        // Apply proportional gain.
        knob->speed_ref = ctl_mul(knob->kp, position_error);

        // Saturate the output to the speed limit.
        knob->speed_ref = ctl_sat(knob->speed_ref, knob->speed_limit, -knob->speed_limit);
    }
}

/**
 * @brief Inputs the feedback position from a multi-turn encoder.
 * @param knob Pointer to the `ctl_pmsm_knob_pos_loop` object.
 * @param actual_pos The actual position (number of full rotations).
 * @param actual_ang The actual angle within a rotation (0.0 to 1.0 p.u.).
 */
GMP_STATIC_INLINE void ctl_input_knob_pos(ctl_pmsm_knob_pos_loop* knob, int32_t actual_pos, ctrl_gt actual_ang)
{
    knob->actual_pos = actual_pos;
    knob->actual_ang = actual_ang;
}

/**
 * @brief Inputs the feedback position from a single-turn encoder.
 * @details Reconstructs the multi-turn count by detecting angle wrap-arounds.
 * @param knob Pointer to the `ctl_pmsm_knob_pos_loop` object.
 * @param actual_ang The actual angle from the single-turn encoder (0.0 to 1.0 p.u.).
 */
GMP_STATIC_INLINE void ctl_input_knob_pos_via_only_ang(ctl_pmsm_knob_pos_loop* knob, ctrl_gt actual_ang)
{
    ctrl_gt delta_ang = actual_ang - knob->actual_ang;
    knob->actual_ang = actual_ang;

    // Correct the multi-turn count based on angle wrap-around.
    if (delta_ang < -GMP_CONST_1_OVER_2)
    {
        knob->actual_pos += 1; // Forward wrap
    }
    else if (delta_ang > GMP_CONST_1_OVER_2)
    {
        knob->actual_pos -= 1; // Backward wrap
    }
}

/**
 * @brief Gets the current detent index.
 * @param knob Pointer to the `ctl_pmsm_knob_pos_loop` object.
 * @return The index of the current detent.
 */
GMP_STATIC_INLINE int16_t ctl_get_knob_index(ctl_pmsm_knob_pos_loop* knob)
{
    return knob->output_pos;
}

/**
 * @brief Gets the calculated target speed reference.
 * @param knob Pointer to the `ctl_pmsm_knob_pos_loop` object.
 * @return The speed reference output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_knob_target_speed(ctl_pmsm_knob_pos_loop* knob)
{
    return knob->speed_ref;
}

/**
 * @brief Gets the target angle (the center of the current detent).
 * @param knob Pointer to the `ctl_pmsm_knob_pos_loop` object.
 * @return The target angle in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_knob_target_angle(ctl_pmsm_knob_pos_loop* knob)
{
    return knob->target_ang;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_KNOB_POS_LOOP_H_
