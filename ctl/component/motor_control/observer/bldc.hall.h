/**
 * @file bldc_hall_estimator.h
 * @brief Implements a speed and position estimator for BLDC motors using Hall sensors.
 * @details This module processes the discrete signals from 3-phase Hall sensors to
 * provide a continuous, high-resolution electrical angle and a smooth speed
 * estimate. It works by measuring the time between Hall state changes to
 * calculate speed, and then uses this speed to linearly interpolate the angle
 * within the 60-degree electrical sectors defined by the Hall states. This
 * allows for smooth sinusoidal control (FOC) instead of traditional six-step
 * trapezoidal control.
 *
 * @version 1.0
 * @date 2025-08-07
 *
 * //tex:
 * // The core principle is to measure the time \Delta t for the rotor to traverse
 * // a 60-degree electrical sector.
 * // The electrical speed is then calculated as: \omega_e = (\pi/3) / \Delta t
 * // The angle within a sector is interpolated as: \theta(t) = \theta_{start} + \omega_e \cdot t_{elapsed}
 *
 */

#ifndef _FILE_BLDC_HALL_ESTIMATOR_H_
#define _FILE_BLDC_HALL_ESTIMATOR_H_

#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* BLDC Hall Sensor Speed and Position Estimator                             */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup BLDC_HALL_ESTIMATOR BLDC Hall Sensor Estimator
 * @brief A module for calculating smooth speed and position from Hall sensors.
 * @{
 */

//================================================================================
// Type Defines & Data
//================================================================================

/**
 * @brief Lookup table for the starting electrical angle of each Hall state (1-6).
 * @details The angle is in per-unit format (0.0 to 1.0).
 * Index 0 is unused. The table maps Hall state [1, 2, 3, 4, 5, 6] to angles.
 * This corresponds to the standard 60-degree sector boundaries.
 */
extern const ctrl_gt HALL_STATE_TO_ANGLE_PU[7];

/**
 * @brief Main structure for the BLDC Hall sensor estimator.
 */
typedef struct
{
    // --- Outputs ---
    rotation_ift encif; ///< Standard encoder interface providing estimated position.
    velocity_ift spdif; ///< Standard velocity interface providing estimated speed.

    // --- State Variables ---
    uint8_t last_hall_state;              ///< The Hall state from the previous ISR tick.
    uint32_t last_capture_time;           ///< The timer timestamp of the last Hall state change.
    ctrl_gt sector_time_delta;            ///< The measured time to cross the last 60-degree sector (in timer ticks).
    ctrl_gt estimated_speed_rad_per_tick; ///< The estimated electrical speed.
    ctrl_gt coarse_angle_pu;              ///< The coarse angle from the LUT based on the last Hall state.

    // --- Configuration ---
    ctrl_gt pole_pairs;     ///< Number of motor pole pairs.
    ctrl_gt speed_filter_k; ///< Coefficient for the speed low-pass filter.
    ctrl_gt ticks_to_pu_sf; ///< Scale factor to convert speed from rad/tick to per-unit.
    ctrl_gt sector_rad;     ///< The angle of one Hall sector in radians (pi/3).

} ctl_bldc_hall_estimator_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the BLDC Hall estimator module.
 * @param[out] est Pointer to the estimator structure.
 * @param[in]  pole_pairs Number of motor pole pairs.
 * @param[in]  timer_freq_hz The frequency of the timer used for time capture (Hz).
 * @param[in]  speed_base_rpm The base speed for per-unit conversion (RPM).
 * @param[in]  speed_filter_fc_hz Cutoff frequency for the speed low-pass filter (Hz).
 * @param[in]  isr_freq_hz The frequency of the control ISR (Hz).
 */
GMP_STATIC_INLINE void ctl_init_bldc_hall_estimator(ctl_bldc_hall_estimator_t* est, uint16_t pole_pairs,
                                                    parameter_gt timer_freq_hz, parameter_gt speed_base_rpm,
                                                    parameter_gt speed_filter_fc_hz, parameter_gt isr_freq_hz);

/**
 * @brief Executes one step of the Hall sensor estimation.
 * @param[out] est           Pointer to the estimator structure.
 * @param[in]  hall_state    The current Hall sensor state (1-6).
 * @param[in]  capture_time  The current timestamp from a free-running timer.
 */
GMP_STATIC_INLINE void ctl_step_bldc_hall_estimator(ctl_bldc_hall_estimator_t* est, uint8_t hall_state,
                                                    uint32_t capture_time)
{
    // --- Speed Estimation ---
    if (hall_state != est->last_hall_state && hall_state >= 1 && hall_state <= 6)
    {
        // A Hall state change has occurred
        uint32_t time_diff = capture_time - est->last_capture_time;
        est->last_capture_time = capture_time;
        est->last_hall_state = hall_state;

        if (time_diff > 0) // Avoid division by zero
        {
            est->sector_time_delta = (ctrl_gt)time_diff;
        }

        // Update the coarse angle based on the new state
        est->coarse_angle_pu = HALL_STATE_TO_ANGLE_PU[hall_state];
    }

    // Calculate raw speed based on the time to cross the last sector
    ctrl_gt raw_speed_rad_per_tick = 0.0f;
    if (est->sector_time_delta > 0.0f)
    {
        raw_speed_rad_per_tick = est->sector_rad / est->sector_time_delta;
    }

    // Low-pass filter the speed estimate
    est->estimated_speed_rad_per_tick +=
        est->speed_filter_k * (raw_speed_rad_per_tick - est->estimated_speed_rad_per_tick);

    // Update the speed output interface (in per-unit)
    est->spdif.speed = est->estimated_speed_rad_per_tick * est->ticks_to_pu_sf;

    // --- Position Interpolation ---
    // Calculate time elapsed since the last Hall edge
    uint32_t time_since_capture = capture_time - est->last_capture_time;

    // Calculate the interpolated angle offset within the current sector
    ctrl_gt angle_offset_rad = (ctrl_gt)time_since_capture * est->estimated_speed_rad_per_tick;
    ctrl_gt angle_offset_pu = angle_offset_rad / (2.0f * CTL_CTRL_CONST_PI);

    // Add the offset to the coarse angle to get the fine angle
    ctrl_gt fine_elec_angle = est->coarse_angle_pu + angle_offset_pu;

    // Update the position output interface
    est->encif.elec_position = ctrl_mod_1(fine_elec_angle);
    est->encif.position = est->encif.elec_position / est->pole_pairs; // Simplified mechanical angle
}

/** @} */ // end of BLDC_HALL_ESTIMATOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BLDC_HALL_ESTIMATOR_H_
