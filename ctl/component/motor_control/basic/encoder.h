/**
 * @file encoder_calibrate.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a set of algorithms to calibrate motor encoders.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This module contains routines for determining the mechanical offset of a
 * position encoder by aligning the motor's rotor with the d-axis.
 */

#ifndef _FILE_ENCODER_CALIBRATE_H_
#define _FILE_ENCODER_CALIBRATE_H_

#include <ctl/component/motor_control/basic/encoder_if.h>
#include <ctl/component/motor_control/current_loop/motor_current_ctrl.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Encoder Calibration                                                       */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ENCODER_CALIBRATE Encoder Calibration
 * @ingroup MC_ENCODER
 * @brief Provides routines for calibrating the offset of position encoders.
 * @{
 */

/**
 * @brief Data structure for the absolute position encoder calibration task.
 */
typedef struct _tag_position_enc_calibrate
{
    // Output
    ctrl_gt offset; /**< @brief The resulting calibrated encoder offset. */

    // Linked Components
    ctl_motor_current_ctrl_t* mc; /**< @brief Pointer to the target motor current controller. */
    ctl_rotation_encif_t*
        encoder; /**< @brief Pointer to the target motor position encoder interface. NOTE: Renamed from pos_enc for consistency. */

    // Parameters
    ctrl_gt current_target; /**< @brief The d-axis current target to apply for rotor alignment. */
    ctrl_gt current_limit;  /**< @brief The current limit to prevent overcurrent during calibration. */
    ctrl_gt
        position_delta_target; /**< @brief The position change threshold (p.u.) to determine if the rotor is stable. */

    // State Machine Variables
    ctrl_gt old_position; /**< @brief Stores the last position to check for convergence. */
    fast_gt
        flag_position_convergence; /**< @brief A bitmask flag to check if the position has been stable for several consecutive samples. */
    fast_gt flag_stage1; /**< @brief The main state flag for the calibration state machine. */
    time_gt switch_time; /**< @brief Timestamp recorded when the rotor is considered stable. */

} position_enc_calibrate_t;

/**
 * @brief Executes one step of the position encoder offset calibration task.
 *
 * This function implements a state machine to find the encoder offset.
 * It works by applying a DC current to the d-axis to lock the rotor in a known
 * position (aligned with the d-axis). It then waits for the rotor to settle
 * and records the measured electrical position as the new offset.
 *
 * @param[in,out] obj Pointer to the position encoder calibration structure.
 * @return `1` upon successful completion, `GMP_EC_OK` (0) while running, or an error code.
 */
ec_gt ctl_task_position_encoder_offset_calibrate(position_enc_calibrate_t* obj)
{
    // Step I: Apply d-axis current to align the rotor.
    ctl_set_motor_current_ctrl_idq_ref(obj->mc, obj->current_target, 0);
    ctl_set_motor_current_ctrl_vdq_ff(obj->mc, 0, 0);

    // TODO: A safety routine should be added here to check if the current
    // exceeds `obj->current_limit` and abort if necessary.

    // Step II: Wait until the encoder position has stabilized.
    if (obj->flag_stage1 == 0)
    {
        ctrl_gt current_pos = ctl_get_encoder_position(obj->encoder);

        // Check if position change is within the target delta.
        if (fabs(current_pos - obj->old_position) < obj->position_delta_target)
        {
            obj->flag_position_convergence = (obj->flag_position_convergence << 1) | 0x00;
        }
        else // Position is still changing.
        {
            obj->flag_position_convergence = (obj->flag_position_convergence << 1) | 0x01;
        }

        obj->old_position = current_pos;

        // If the position has been stable for 4 consecutive checks, move to the next stage.
        if ((obj->flag_position_convergence & 0x0F) == 0)
        {
            obj->switch_time = gmp_base_get_system_tick();
            obj->flag_stage1 = 1;
        }

        return GMP_EC_OK; // Task is still running.
    }
    // Step III: Wait for a final delay and then record the offset.
    else
    {
        // LOGIC NOTE: The original condition `if (gmp_base_get_system_tick() - obj->switch_time)`
        // is likely incorrect. It should compare the elapsed time to a defined timeout.
        // Example: `if (gmp_base_get_system_tick() - obj->switch_time > STABLE_WAIT_TIME_TICKS)`
        if (gmp_base_get_system_tick() - obj->switch_time > 1000) // Assuming a 1-second wait, adjust as needed.
        {
            // LOGIC NOTE: The offset should be the current electrical position that
            // corresponds to d-axis alignment. The original calculation which added
            // the old offset was incorrect.
            ctrl_gt new_offset = ctl_get_encoder_position(obj->encoder);
            obj->offset = ctl_mod_1(new_offset); // Use modulo to keep it in [0, 1) range.

            // Save the calibrated result to the encoder itself.
            ctl_set_encoder_offset(obj->encoder, obj->offset);

            // Task is complete.
            return 1;
        }
        return GMP_EC_OK; // Final wait time has not elapsed.
    }
}

/**
 * @brief Resets the calibration state machine and disarms the current controller.
 * @param[in,out] obj Pointer to the position encoder calibration structure.
 * @return `GMP_EC_OK` on success.
 */
ec_gt ctl_clear_position_encoder_calibrator(position_enc_calibrate_t* obj)
{
    obj->old_position = 0;
    obj->flag_position_convergence = ~0;
    obj->flag_stage1 = 0;

    // Disable current controller output for safety.
    ctl_set_motor_current_ctrl_idq_ref(obj->mc, 0, 0);
    ctl_set_motor_current_ctrl_vdq_ff(obj->mc, 0, 0);

    return GMP_EC_OK;
}

/** @} */ // end of MC_ENCODER_CALIBRATE group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ENCODER_CALIBRATE_H_
