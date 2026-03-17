/**
 * @file bldc_zcd_obs.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements a Sensorless Back-EMF Zero-Crossing Detection (ZCD) Observer for BLDC.
 * @details This module performs sensorless commutation for BLDC motors using the 
 * standard 120-degree six-step commutation scheme. It detects the Back-EMF zero-crossing 
 * event on the floating phase, calculates the real-time electrical speed, and rigorously 
 * integrates the speed to achieve a perfect 30-degree electrical phase delay for the 
 * next commutation, maintaining optimal torque output even during harsh acceleration.
 * * * **Core Mathematical Architecture (Per-Unitized):**
 * - **Speed Estimation:** @f$ \omega_{pu} = \frac{1/6}{N_{ticks} \cdot T_s \cdot f_{base}} @f$
 * - **30-Degree Integration:** @f$ \theta_{delay}[k] = \theta_{delay}[k-1] + \omega_{pu} \cdot (\Omega_{base} T_s / 2\pi) @f$
 * - **Commutation Trigger:** Activated when @f$ \theta_{delay} \ge \frac{1}{12} @f$ PU (30 degrees).
 *
 * @version 1.0
 * @date 2024-10-27
 *
 * @copyright Copyright GMP(c) 2024
 */

#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/interface/encoder.h>

#ifndef _FILE_BLDC_ZCD_OBS_H_
#define _FILE_BLDC_ZCD_OBS_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* BLDC Back-EMF Zero-Crossing Detection Observer                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup BLDC_ZCD_OBS BLDC Zero-Crossing Observer
 * @brief Six-step sensorless commutation logic based on BEMF.
 * @{
 */

/**
 * @brief Raw initialization structure for the BLDC ZCD Observer.
 */
typedef struct _tag_bldc_zcd_obs_init_t
{
    parameter_gt fs;               //!< Controller execution frequency (Hz).
    parameter_gt w_base;           //!< Base Electrical Angular Velocity (rad/s) for PU scaling.
    parameter_gt blanking_time_ms; //!< Flyback diode demagnetization blanking time (ms) after commutation.
    parameter_gt debounce_time_ms; //!< Digital filter debounce time (ms) to reject PWM noise.
    parameter_gt timeout_ms;       //!< Max time without ZCD before declaring stall/fault (ms).

} ctl_bldc_zcd_obs_init_t;

/**
 * @brief Main state structure for the BLDC ZCD Observer.
 */
typedef struct _tag_bldc_zcd_obs_t
{
    // --- Standard Outputs ---
    velocity_ift spd_out;      //!< Estimated electrical speed (PU).
    uint8_t next_comm_state;   //!< The next 6-step commutation state (1-6) to be applied.
    fast_gt flag_comm_trigger; //!< Goes HIGH (1) for exactly one tick when commutation should occur.

    // --- State Variables ---
    uint32_t tick_since_last_zcd; //!< Timer counting ISR ticks between consecutive ZCDs (60 degrees).
    uint32_t tick_since_comm;     //!< Timer counting ticks since the last commutation (for blanking).
    uint32_t debounce_cnt;        //!< Noise filter counter for ZCD validation.

    ctrl_gt spd_est_pu;     //!< Estimated speed calculated at the last ZCD (PU).
    ctrl_gt theta_delay_pu; //!< Integral of speed since ZCD, used to trigger the 30-deg delay.

    fast_gt flag_zcd_detected; //!< Goes high once ZCD is found, waiting for 30-deg integration.

    // --- Scale Factors & Thresholds ---
    ctrl_gt sf_speed_calc;   //!< Scale factor: converts 1/tick to PU speed. @f$ \frac{1/6}{T_s \cdot f_{base}} @f$.
    ctrl_gt sf_w_to_angle;   //!< Scale factor: integrates PU speed to PU angle. @f$ \frac{\Omega_{base} T_s}{2\pi} @f$.
    ctrl_gt delay_target_pu; //!< Target delay angle for commutation: @f$ 1/12 @f$ PU (30 degrees).

    uint32_t blanking_ticks; //!< Number of ticks to ignore BEMF after commutation.
    uint32_t debounce_limit; //!< Number of consecutive valid samples required for ZCD.
    uint32_t timeout_ticks;  //!< Ticks allowed before stall is declared.

    // --- Sub-modules ---
    ctl_filter_IIR1_t filter_spd; //!< LPF to smooth the discontinuous speed estimation.

    // --- Flags ---
    fast_gt flag_enable;          //!< Master enable flag.
    fast_gt flag_observer_locked; //!< 1 if motor is spinning and ZCD is regularly detected.

} ctl_bldc_zcd_obs_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

/**
 * @brief Initializes the BLDC ZCD Observer.
 * @param[out] obs  Pointer to the ZCD Observer instance.
 * @param[in]  init Pointer to the initialization structure.
 */
void ctl_init_bldc_zcd_obs(ctl_bldc_zcd_obs_t* obs, const ctl_bldc_zcd_obs_init_t* init);

/**
 * @brief Safely clears internal state machines, timers, and flags.
 */
GMP_STATIC_INLINE void ctl_clear_bldc_zcd_obs(ctl_bldc_zcd_obs_t* obs)
{
    obs->tick_since_last_zcd = 0;
    obs->tick_since_comm = 0;
    obs->debounce_cnt = 0;
    obs->spd_est_pu = float2ctrl(0.0f);
    obs->theta_delay_pu = float2ctrl(0.0f);

    obs->flag_comm_trigger = 0;
    obs->flag_zcd_detected = 0;
    obs->flag_observer_locked = 0;
    obs->spd_out.speed = float2ctrl(0.0f);

    ctl_clear_filter_iir1(&obs->filter_spd);
}

GMP_STATIC_INLINE void ctl_enable_bldc_zcd_obs(ctl_bldc_zcd_obs_t* obs)
{
    obs->flag_enable = 1;
}
GMP_STATIC_INLINE void ctl_disable_bldc_zcd_obs(ctl_bldc_zcd_obs_t* obs)
{
    obs->flag_enable = 0;
}

/**
 * @brief Executes one high-frequency step of the BLDC BEMF Observer.
 * @details Identifies the floating phase, applies flyback blanking, detects the ZCD 
 * with digital debouncing, integrates the angle for a 30-degree delay, and generates 
 * the commutation trigger.
 * @param[in,out] obs        Pointer to the Observer instance.
 * @param[in]     v_u_pu     Measured Phase U voltage (PU).
 * @param[in]     v_v_pu     Measured Phase V voltage (PU).
 * @param[in]     v_w_pu     Measured Phase W voltage (PU).
 * @param[in]     v_bus_pu   Measured DC Bus voltage (PU). Used for Virtual Neutral Point.
 * @param[in]     curr_state Current 6-step commutation state (1-6).
 */
void ctl_step_bldc_zcd_obs(ctl_bldc_zcd_obs_t* obs, ctrl_gt v_u_pu, ctrl_gt v_v_pu, ctrl_gt v_w_pu, ctrl_gt v_bus_pu,
                           uint8_t curr_state);

/** @} */ // end of BLDC_ZCD_OBS group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BLDC_ZCD_OBS_H_
