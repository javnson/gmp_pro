/**
 * @file encoder_calibrate.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides an algorithm to calibrate the offset of an absolute position encoder.
 * @details This module implements a state machine to find the electrical zero position
 * of a motor by applying a DC current to the d-axis and aligning the rotor.
 * The measured position at alignment is then saved as the encoder offset.
 * @version 0.2
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_ENCODER_CALIBRATE_H_
#define _FILE_ENCODER_CALIBRATE_H_

#include <ctl/component/motor_control/basic/encoder_if.h>
#include <ctl/component/motor_control/current_loop/motor_current_ctrl.h>
#include <gmp/base/gmp_base_error_code.h>  // Assumed for GMP_EC_* codes
#include <gmp/base/gmp_base_system_tick.h> // Assumed for gmp_base_get_system_tick

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Absolute Position Encoder Calibration                                     */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup EncoderCalibrate Absolute Position Encoder Calibration
 * @brief A module to automatically calibrate the electrical offset of a position encoder.
 *
 * This routine aligns the motor's rotor to a known electrical angle (d-axis, or zero degrees)
 * and records the encoder's reading at that position as the offset.
 * @{
 */

/**
 * @brief State machine status codes for the calibration task.
 */
typedef enum
{
    GMP_EC_RUNNING = 0,  /**< Calibration is in progress. */
    GMP_EC_COMPLETE = 1, /**< Calibration completed successfully. */
    GMP_EC_ERROR = -1    /**< An error occurred (e.g., overcurrent). */
} ec_gt;

/**
 * @brief Data structure for the position encoder calibration task.
 */
typedef struct _tag_position_enc_calibrate
{
    /** @brief Output: The calculated encoder offset after calibration. */
    ctrl_gt offset;

    /** @brief Pointer to the target motor's current controller. */
    ctl_motor_current_ctrl_t* mc;

    /** @brief Pointer to the target motor's position encoder interface. */
    ctl_rotation_encif_t* pos_enc;

    // --- Parameters ---
    /** @brief The d-axis current to apply for rotor alignment (p.u.). */
    ctrl_gt current_target;

    /** @brief The overcurrent protection limit (p.u.). */
    ctrl_gt current_limit;

    /** @brief The threshold for position change to determine if the rotor has stopped (p.u.). */
    ctrl_gt position_delta_target;

    /** @brief The time to wait after the rotor has stabilized before finalizing the offset (in system ticks). */
    time_gt wait_time;

    // --- State Machine Variables ---
    /** @brief Stores the motor position from the previous ISR tick to check for changes. */
    ctrl_gt old_position;

    /** @brief A shift-register used as a debouncing filter to confirm position convergence. */
    fast_gt flag_position_convergence;

    /** @brief The main state flag of the state machine. 0: Aligning rotor, 1: Waiting and finalizing. */
    fast_gt flag_stage1;

    /** @brief Records the system tick when the rotor is considered stable. */
    time_gt switch_time;

} position_enc_calibrate_t;

/**
 * @brief Initializes the position encoder calibrator object.
 * @param[out] obj Pointer to the position_enc_calibrate_t object.
 * @param[in] mc Pointer to the motor current controller.
 * @param[in] pos_enc Pointer to the position encoder interface.
 * @param[in] align_current The d-axis current for alignment (p.u.).
 * @param[in] current_limit The overcurrent limit (p.u.).
 * @param[in] pos_delta The position stability threshold (p.u.).
 * @param[in] wait_time_ticks The final wait time in system ticks.
 */
void ctl_init_position_encoder_calibrator(position_enc_calibrate_t* obj, ctl_motor_current_ctrl_t* mc,
                                          ctl_rotation_encif_t* pos_enc, ctrl_gt align_current, ctrl_gt current_limit,
                                          ctrl_gt pos_delta, time_gt wait_time_ticks);

/**
 * @brief Resets the state machine and clears motor commands.
 * @param[out] obj Pointer to the position_enc_calibrate_t object.
 */
void ctl_clear_position_encoder_calibrator(position_enc_calibrate_t* obj);

/**
 * @brief Executes one step of the position encoder offset calibration state machine.
 * @details This function should be called periodically from the main control ISR. It manages
 * the states of applying current, waiting for stability, and calculating the final offset.
 * @param[in,out] obj Pointer to the position_enc_calibrate_t object.
 * @return ec_gt The current status of the calibration task (RUNNING, COMPLETE, or ERROR).
 */
ec_gt ctl_task_position_encoder_offset_calibrate(position_enc_calibrate_t* obj)
{
    // --- Safety Check ---
    // If an overcurrent condition is detected, abort the calibration.
    if (obj->mc && (obj->mc->sensor.Idq.q > obj->current_limit || obj->mc->sensor.Idq.q < -obj->current_limit))
    {
        ctl_clear_position_encoder_calibrator(obj);
        return GMP_EC_ERROR;
    }

    // --- State 0: Rotor Alignment and Convergence Check ---
    if (obj->flag_stage1 == 0)
    {
        // Step I: Apply d-axis current to align the rotor.
        ctl_set_motor_current_ctrl_idq_ref(obj->mc, obj->current_target, 0);
        ctl_set_motor_current_ctrl_vdq_ff(obj->mc, 0, 0);

        // Step II: Check if the encoder output has stabilized.
        ctrl_gt current_pos = ctl_get_encoder_position(obj->pos_enc);
        ctrl_gt pos_diff = current_pos - obj->old_position;
        if (pos_diff < 0)
            pos_diff = -pos_diff; // Absolute difference

        if (pos_diff < obj->position_delta_target)
        {
            // Position is stable, shift a '0' into the convergence flag.
            obj->flag_position_convergence = (obj->flag_position_convergence << 1) | 0x00;
        }
        else
        {
            // Position is still changing, shift a '1' into the flag.
            obj->flag_position_convergence = (obj->flag_position_convergence << 1) | 0x01;
        }

        obj->old_position = current_pos; // Save current position for the next check.

        // If the last 4 samples were stable (flag is all zeros), move to the next stage.
        if ((obj->flag_position_convergence & 0x0F) == 0)
        {
            obj->switch_time = gmp_base_get_system_tick();
            obj->flag_stage1 = 1; // Transition to Stage 1
        }
        return GMP_EC_RUNNING;
    }
    // --- State 1: Final Wait and Offset Calculation ---
    else
    {
        // Step III: Wait for the specified duration to ensure rotor is fully settled.
        if ((gmp_base_get_system_tick() - obj->switch_time) > obj->wait_time)
        {
            // Step IV: Calculate and save the final offset.
            // The new offset is the sum of any existing offset and the current rotor position.
            ctrl_gt final_pos = ctl_get_encoder_position(obj->pos_enc);
            obj->offset = obj->pos_enc->offset + final_pos;

            // Wrap the offset to the per-unit range [-1.0, 1.0).
            while (obj->offset >= 1.0)
                obj->offset -= 1.0;
            while (obj->offset < 0.0)
                obj->offset += 1.0;

            // Apply the new offset to the encoder instance.
            obj->pos_enc->offset = obj->offset;

            // Stop applying current.
            ctl_clear_position_encoder_calibrator(obj);

            // Task is complete.
            return GMP_EC_COMPLETE;
        }
        // If wait time has not elapsed, continue holding position.
        return GMP_EC_RUNNING;
    }
}

/** @} */ // end of EncoderCalibrate group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ENCODER_CALIBRATE_H_
