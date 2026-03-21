/**
 * @file bldc_zcd_obs.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation of the BLDC Back-EMF ZCD Observer.
 *
 * @version 1.0
 * @date 2024-10-27
 *
 * @copyright Copyright GMP(c) 2024
 */

#include <gmp_core.h>

#include <ctl/component/motor_control/observer/bldc_zcd_obs.h>


void ctl_init_bldc_zcd_obs(ctl_bldc_zcd_obs_t* obs, const ctl_bldc_zcd_obs_init_t* init)
{
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt Ts = 1.0f / fs_safe;
    parameter_gt f_base = init->w_base / CTL_PARAM_CONST_2PI;

    // 1. Scale Factors (Avoid division in ISR)
    // Speed PU = (1/6 electrical rev) / (N_ticks * Ts * f_base)
    obs->sf_speed_calc = float2ctrl((1.0f / 6.0f) / (Ts * f_base));

    // Angle integration coefficient: delta_theta = speed_pu * (W_base * Ts / 2PI)
    obs->sf_w_to_angle = float2ctrl(f_base * Ts);

    // Target phase delay from ZCD to Commutation is 30 degrees (1/12 PU)
    obs->delay_target_pu = float2ctrl(1.0f / 12.0f);

    // 2. Timers & Filters
    obs->blanking_ticks = (uint32_t)(init->blanking_time_ms * fs_safe / 1000.0f);
    obs->debounce_limit = (uint32_t)(init->debounce_time_ms * fs_safe / 1000.0f);
    if (obs->debounce_limit < 1)
        obs->debounce_limit = 1;

    obs->timeout_ticks = (uint32_t)(init->timeout_ms * fs_safe / 1000.0f);

    // LPF to smooth the estimated speed (10Hz bandwidth is typical for stable 6-step)
    ctl_init_filter_iir1_lpf(&obs->filter_spd, fs_safe, 10.0f);

    ctl_clear_bldc_zcd_obs(obs);
    ctl_disable_bldc_zcd_obs(obs);
}

void ctl_step_bldc_zcd_obs(ctl_bldc_zcd_obs_t* obs, ctrl_gt v_u_pu, ctrl_gt v_v_pu, ctrl_gt v_w_pu, ctrl_gt v_bus_pu,
                           uint8_t curr_state)
{
    if (!obs->flag_enable)
        return;

    // Clear trigger flag by default (it only pulses high for 1 tick)
    obs->flag_comm_trigger = 0;

    obs->tick_since_last_zcd++;
    obs->tick_since_comm++;

    // Safety Timeout: Motor stalled or decoupled
    if (obs->tick_since_last_zcd > obs->timeout_ticks)
    {
        obs->flag_observer_locked = 0;
        obs->spd_est_pu = float2ctrl(0.0f);
        ctl_clear_filter_iir1(&obs->filter_spd);
    }

    // ========================================================================
    // 1. Commutation 30-Degree Angle Integration Logic
    // ========================================================================
    if (obs->flag_zcd_detected)
    {
        // Integrate speed to calculate delay angle
        obs->theta_delay_pu += ctl_mul(obs->spd_est_pu, obs->sf_w_to_angle);

        // Check if 30-degree electrical delay has been reached
        if (obs->theta_delay_pu >= obs->delay_target_pu)
        {
            // Commutate!
            obs->flag_comm_trigger = 1;
            obs->flag_zcd_detected = 0; // Reset for next sector
            obs->tick_since_comm = 0;
            obs->debounce_cnt = 0;

            // Advance state machine (1 -> 6 -> 1)
            obs->next_comm_state = curr_state + 1;
            if (obs->next_comm_state > 6)
                obs->next_comm_state = 1;
        }
        return; // Skip ZCD detection if we are already waiting to commutate
    }

    // ========================================================================
    // 2. Flyback Diode Blanking Time
    // ========================================================================
    // Immediately after commutation, flyback diodes conduct, pulling the floating
    // phase to V_bus or GND. We must ignore voltage readings during this time.
    if (obs->tick_since_comm < obs->blanking_ticks)
        return;

    // ========================================================================
    // 3. Floating Phase Selection & Zero-Crossing Detection (ZCD)
    // ========================================================================
    // Standard unipolar/bipolar PWM dictates Virtual Neutral Point ~ V_bus / 2
    ctrl_gt v_neutral = ctl_mul(v_bus_pu, CTL_CTRL_CONST_1_OVER_2);
    ctrl_gt v_float = float2ctrl(0.0f);
    fast_gt is_falling_edge = 0;

    // 6-step commutation table analysis
    switch (curr_state)
    {
    case 1: // U->V. Floating W. W goes from V_bus (High) to GND (Low).
        v_float = v_w_pu;
        is_falling_edge = 1;
        break;
    case 2: // U->W. Floating V. V goes from GND to V_bus.
        v_float = v_v_pu;
        is_falling_edge = 0;
        break;
    case 3: // V->W. Floating U. U goes from V_bus to GND.
        v_float = v_u_pu;
        is_falling_edge = 1;
        break;
    case 4: // V->U. Floating W. W goes from GND to V_bus.
        v_float = v_w_pu;
        is_falling_edge = 0;
        break;
    case 5: // W->U. Floating V. V goes from V_bus to GND.
        v_float = v_v_pu;
        is_falling_edge = 1;
        break;
    case 6: // W->V. Floating U. U goes from GND to V_bus.
        v_float = v_u_pu;
        is_falling_edge = 0;
        break;
    default:
        return; // Invalid state
    }

    // ZCD Logic: Compare floating phase to Virtual Neutral
    fast_gt zcd_condition_met = 0;
    if (is_falling_edge)
    {
        zcd_condition_met = (v_float < v_neutral);
    }
    else
    {
        zcd_condition_met = (v_float > v_neutral);
    }

    // ========================================================================
    // 4. Debounce Filter & Speed Estimation
    // ========================================================================
    if (zcd_condition_met)
    {
        obs->debounce_cnt++;
        if (obs->debounce_cnt >= obs->debounce_limit)
        {
            // Valid Zero-Crossing Confirmed!
            obs->flag_zcd_detected = 1;
            obs->flag_observer_locked = 1;
            obs->theta_delay_pu = float2ctrl(0.0f); // Reset integrator for 30-deg delay

            // Calculate Raw Electrical Speed: (1/6 PU) / (N_ticks * Ts * f_base)
            // Prevent division by zero
            ctrl_gt ticks = float2ctrl((float)obs->tick_since_last_zcd);
            if (ticks < float2ctrl(1.0f))
                ticks = float2ctrl(1.0f);

            ctrl_gt raw_spd_pu = ctl_div(obs->sf_speed_calc, ticks);

            // Filter speed to prevent commutation jitter
            obs->spd_est_pu = ctl_step_filter_iir1(&obs->filter_spd, raw_spd_pu);
            obs->spd_out.speed = obs->spd_est_pu;

            // Reset timer for the next 60-degree sector speed calculation
            obs->tick_since_last_zcd = 0;
            obs->debounce_cnt = 0;
        }
    }
    else
    {
        // Reset debounce if the signal bounces back (PWM noise)
        if (obs->debounce_cnt > 0)
            obs->debounce_cnt--;
    }
}
