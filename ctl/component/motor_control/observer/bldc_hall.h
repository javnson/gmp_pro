/**
 * @file bldc_hall_estimator.h
* @brief Implements a Hall Sensor-based Rotor Position Estimator & Interpolator.
 * @details This module calculates rotor speed based on the time elapsed between 
 * Hall sensor edges (60 electrical degrees per sector) and uses this speed to 
 * smoothly interpolate the electrical angle between edges. 
 * It strictly enforces sector boundary clamping to prevent angle drift before 
 * the next physical edge arrives.
 * * * **Core Mathematical Architecture (Per-Unitized):**
 * - **Speed Estimation:** @f$ \omega_{pu} = \frac{1/6}{N_{ticks} \cdot T_s \cdot f_{base}} @f$
 * - **Angle Integration:** @f$ \theta_{pu}[k] = \theta_{pu}[k-1] + \omega_{pu} \cdot (\Omega_{base} T_s / 2\pi) @f$
 * - **Sector Clamping:** @f$ \theta_{pu} \in [\theta_{center} - \frac{1}{12}, \theta_{center} + \frac{1}{12}] @f$
 *
 * @version 1.0
 * @date 2025-08-07
 *
 */

#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>
#include <ctl/component/motor_control/interface/encoder.h>

#ifndef _FILE_PMSM_HALL_OBS_H_
#define _FILE_PMSM_HALL_OBS_H_



#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* PMSM Hall Sensor Position Observer & Interpolator                         */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSM_HALL_OBS PMSM Hall Position Observer
 * @brief Smooth angle interpolator based on discrete Hall sensor sectors.
 * @{
 */

/**
 * @brief Initialization structure for the Hall Observer.
 */
typedef struct _tag_pmsm_hall_obs_init_t
{
    parameter_gt fs;              //!< Controller execution frequency (Hz).
    parameter_gt w_base;          //!< Base Electrical Angular Velocity (rad/s).
    parameter_gt hall_offset_deg; //!< Installation offset of the Hall sensors (Electrical Degrees).
    parameter_gt filter_bw_hz;    //!< Cutoff frequency for the speed estimation low-pass filter (Hz).
    parameter_gt timeout_ms;      //!< Time without Hall edges before speed is forced to 0 (ms).

} ctl_pmsm_hall_obs_init_t;

/**
 * @brief Main state structure for the Hall Observer.
 */
typedef struct _tag_pmsm_hall_obs_t
{
    // --- Standard Outputs ---
    rotation_ift pos_out; //!< Interpolated and clamped electrical position (PU).
    velocity_ift spd_out; //!< Filtered estimated speed (PU).

    // --- Sub-modules ---
    ctl_filter_IIR1_t filter_spd; //!< Low-pass filter for the calculated edge-to-edge speed.

    // --- State Variables ---
    uint8_t prev_hall_state; //!< Previous 3-bit Hall state (1-6).
    uint32_t edge_tick_cnt;  //!< Number of ISR ticks elapsed since the last Hall edge.
    ctrl_gt spd_est_pu;      //!< Filtered estimated speed (PU).
    ctrl_gt theta_interp_pu; //!< Purely integrated angle before clamping (PU).

    // --- Scale Factors & Constants ---
    ctrl_gt sf_speed_calc;  //!< Scale factor: converts 1/tick to PU speed. @f$ \frac{1/6}{T_s \cdot f_{base}} @f$.
    ctrl_gt sf_w_to_angle;  //!< Scale factor: integrates PU speed to PU angle. @f$ f_{base} \cdot T_s @f$.
    ctrl_gt hall_offset_pu; //!< Installation offset mapped to PU.
    uint32_t timeout_ticks; //!< Max ticks allowed before speed is assumed zero.

    // --- Flags ---
    fast_gt flag_enable; //!< Master enable flag.

} ctl_pmsm_hall_obs_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

/**
 * @brief Initializes the Hall Observer and calculates scale factors.
 */
void ctl_init_pmsm_hall_obs(ctl_pmsm_hall_obs_t* obs, const ctl_pmsm_hall_obs_init_t* init);

/**
 * @brief Safely clears internal states and timers.
 */
GMP_STATIC_INLINE void ctl_clear_pmsm_hall_obs(ctl_pmsm_hall_obs_t* obs)
{
    obs->prev_hall_state = 0;
    obs->edge_tick_cnt = 0;
    obs->spd_est_pu = float2ctrl(0.0f);
    obs->theta_interp_pu = float2ctrl(0.0f);

    ctl_clear_filter_iir1(&obs->filter_spd);

    obs->pos_out.elec_position = float2ctrl(0.0f);
    obs->spd_out.velocity = float2ctrl(0.0f);
}

GMP_STATIC_INLINE void ctl_enable_pmsm_hall_obs(ctl_pmsm_hall_obs_t* obs)
{
    obs->flag_enable = 1;
}
GMP_STATIC_INLINE void ctl_disable_pmsm_hall_obs(ctl_pmsm_hall_obs_t* obs)
{
    obs->flag_enable = 0;
}

/**
 * @brief Helper: Reconstructs the 1-6 integer state from raw A, B, C signals.
 * @details Signal C is MSB, A is LSB. 
 * Example: A=1, B=0, C=1 -> State 5 (101).
 * @param a Raw Hall A signal (0 or 1).
 * @param b Raw Hall B signal (0 or 1).
 * @param c Raw Hall C signal (0 or 1).
 * @return uint8_t State from 1 to 6. Returns 0 or 7 on hardware fault.
 */
GMP_STATIC_INLINE uint8_t ctl_get_hall_state(uint8_t a, uint8_t b, uint8_t c)
{
    return (uint8_t)((c << 2) | (b << 1) | a);
}

/**
 * @brief Executes one high-frequency step of the Hall Observer.
 * @details Detects Hall edges to calculate speed, interpolates the angle, 
 * and rigorously clamps the output to the current physical 60-degree sector.
 * @param[in,out] obs        Pointer to the Hall Observer instance.
 * @param[in]     hall_state Current 3-bit Hall state (1-6) generated by `ctl_get_hall_state`.
 */
void ctl_step_pmsm_hall_obs(ctl_pmsm_hall_obs_t* obs, uint8_t hall_state);

/** @} */ // end of BLDC_HALL_ESTIMATOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_HALL_OBS_H_
