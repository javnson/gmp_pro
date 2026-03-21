/**
 * @file pmsm_hall_obs.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation of the Hall Sensor Position Observer & Interpolator.
 *
 * @version 1.0
 * @date 2024-10-27
 *
 * @copyright Copyright GMP(c) 2024
 */

#include <gmp_core.h>

#include <ctl/component/motor_control/observer/bldc_hall.h>

// ============================================================================
// Hall Sensor 120-Degree Placement Lookup Tables
// ============================================================================

/**
 * @brief Maps the 3-bit Hall State (index 0-7) to the Center Electrical Angle (PU) of that sector.
 * @details 
 * - State 5 (101): Center 30бу  -> 1/12 PU
 * - State 4 (100): Center 90бу  -> 3/12 PU
 * - State 6 (110): Center 150бу -> 5/12 PU
 * - State 2 (010): Center 210бу -> 7/12 PU
 * - State 3 (011): Center 270бу -> 9/12 PU
 * - State 1 (001): Center 330бу -> 11/12 PU
 * State 0 and 7 are invalid (faults), defaulted to 0.
 */
static const ctrl_gt HALL_CENTER_ANGLE_PU[8] = {
    float2ctrl(0.0f),          // 0: Invalid
    float2ctrl(11.0f / 12.0f), // 1: 330 deg
    float2ctrl(7.0f / 12.0f),  // 2: 210 deg
    float2ctrl(9.0f / 12.0f),  // 3: 270 deg
    float2ctrl(3.0f / 12.0f),  // 4: 90 deg
    float2ctrl(1.0f / 12.0f),  // 5: 30 deg
    float2ctrl(5.0f / 12.0f),  // 6: 150 deg
    float2ctrl(0.0f)           // 7: Invalid
};

/**
 * @brief Fast O(1) transition map to determine rotation direction.
 * @details Array size is 64. Index is created by (prev_state << 3) | curr_state.
 * Returns +1 for forward, -1 for reverse, 0 for invalid/no-change.
 */
static const int8_t HALL_DIR_MAP[64] = {
    // To read: Map[prev * 8 + curr]
    0, 0,  0,  0,  0,  0,  0,  0, // Prev 0 (Invalid)
    0, 0,  0,  -1, 0,  1,  0,  0, // Prev 1 (001): 1->5(+), 1->3(-)
    0, 0,  0,  1,  0,  0,  -1, 0, // Prev 2 (010): 2->3(+), 2->6(-)
    0, 1,  -1, 0,  0,  0,  0,  0, // Prev 3 (011): 3->1(+), 3->2(-)
    0, 0,  0,  0,  0,  -1, 1,  0, // Prev 4 (100): 4->6(+), 4->5(-)
    0, -1, 0,  0,  1,  0,  0,  0, // Prev 5 (101): 5->4(+), 5->1(-)
    0, 0,  1,  0,  -1, 0,  0,  0, // Prev 6 (110): 6->2(+), 6->4(-)
    0, 0,  0,  0,  0,  0,  0,  0  // Prev 7 (Invalid)
};

// ============================================================================
// Core Functions
// ============================================================================

void ctl_init_pmsm_hall_obs(ctl_pmsm_hall_obs_t* obs, const ctl_pmsm_hall_obs_init_t* init)
{
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt f_base = init->w_base / CTL_PARAM_CONST_2PI;
    parameter_gt Ts = 1.0f / fs_safe;

    // 1. Scale Factors Derivation
    // Speed (PU) = (1/6 revolution) / (N_ticks * Ts * f_base)
    // sf_speed_calc = (1/6) / (Ts * f_base)
    parameter_gt sf_spd = (1.0f / 6.0f) / (Ts * f_base);
    obs->sf_speed_calc = float2ctrl(sf_spd);

    // Angle Integration = Speed_PU * (W_base * Ts / 2PI) = Speed_PU * (f_base * Ts)
    obs->sf_w_to_angle = float2ctrl(f_base * Ts);

    // 2. Hardware Alignment Offset
    obs->hall_offset_pu = float2ctrl(init->hall_offset_deg / 360.0f);

    // 3. Timeout and LPF
    obs->timeout_ticks = (uint32_t)(init->timeout_ms * fs_safe / 1000.0f);
    ctl_init_filter_iir1_lpf(&obs->filter_spd, fs_safe, init->filter_bw_hz);

    ctl_clear_pmsm_hall_obs(obs);
    ctl_disable_pmsm_hall_obs(obs);
}

void ctl_step_pmsm_hall_obs(ctl_pmsm_hall_obs_t* obs, uint8_t hall_state)
{
    if (!obs->flag_enable)
        return;

    // Safety check for valid hardware states
    if (hall_state == 0 || hall_state == 7)
        return;

    // ========================================================================
    // 1. Edge Detection & Speed Calculation
    // ========================================================================
    if (hall_state != obs->prev_hall_state)
    {
        // Calculate transition index: (prev << 3) | curr
        uint8_t trans_idx = (obs->prev_hall_state << 3) | hall_state;
        int8_t dir = HALL_DIR_MAP[trans_idx];

        if (dir != 0 && obs->edge_tick_cnt > 0)
        {
            // Calculate raw speed in PU
            ctrl_gt raw_spd_pu = ctl_div(obs->sf_speed_calc, float2ctrl((float)obs->edge_tick_cnt));
            if (dir < 0)
                raw_spd_pu = -raw_spd_pu;

            // Filter the speed to prevent jarring torque ripples on acceleration
            obs->spd_est_pu = ctl_step_filter_iir1(&obs->filter_spd, raw_spd_pu);

            // Sync the interpolator exactly to the sector boundary edge
            // Boundary angle = Center of new sector - dir * (1/12 PU)
            ctrl_gt center_curr = HALL_CENTER_ANGLE_PU[hall_state];
            ctrl_gt margin = float2ctrl(1.0f / 12.0f);

            if (dir > 0)
            {
                obs->theta_interp_pu = center_curr - margin;
            }
            else
            {
                obs->theta_interp_pu = center_curr + margin;
            }
            obs->theta_interp_pu = ctrl_mod_1(obs->theta_interp_pu);
        }
        else if (obs->prev_hall_state == 0)
        {
            // Initial startup sync (first valid read)
            obs->theta_interp_pu = HALL_CENTER_ANGLE_PU[hall_state];
        }

        obs->prev_hall_state = hall_state;
        obs->edge_tick_cnt = 0;
    }
    else
    {
        // ====================================================================
        // 2. Intra-Sector Interpolation & Timeout
        // ====================================================================
        obs->edge_tick_cnt++;

        // Integrate angle based on last known filtered speed
        obs->theta_interp_pu += ctl_mul(obs->spd_est_pu, obs->sf_w_to_angle);
        obs->theta_interp_pu = ctrl_mod_1(obs->theta_interp_pu);

        // Zero-speed timeout protection (Rotor is stalled)
        if (obs->edge_tick_cnt > obs->timeout_ticks)
        {
            obs->spd_est_pu = float2ctrl(0.0f);
            ctl_clear_filter_iir1(&obs->filter_spd); // Reset LPF history
        }
    }

    // ========================================================================
    // 3. Strict Sector Boundary Clamping (Anti-Drift Mechanism)
    // ========================================================================
    // Extract the center angle of the currently active hardware sector
    ctrl_gt center_pu = HALL_CENTER_ANGLE_PU[hall_state];

    // Calculate the interpolation error relative to the center
    ctrl_gt err_from_center = obs->theta_interp_pu - center_pu;

    // Wrap error to [-0.5, 0.5] PU pathologically to handle 0-to-1 wrap-around sectors
    if (err_from_center > float2ctrl(0.5f))
        err_from_center -= float2ctrl(1.0f);
    if (err_from_center < float2ctrl(-0.5f))
        err_from_center += float2ctrl(1.0f);

    // HARD CLAMP: The angle is mathematically forbidden from escaping the +/- 30 deg sector
    // This perfectly addresses the sector limitation requirement.
    ctrl_gt margin = float2ctrl(1.0f / 12.0f); // 30 degrees = 1/12 PU
    err_from_center = ctl_sat(err_from_center, margin, -margin);

    // ========================================================================
    // 4. Update Output Interfaces (Apply Installation Offset)
    // ========================================================================
    obs->pos_out.elec_position = ctrl_mod_1(center_pu + err_from_center + obs->hall_offset_pu);
    obs->spd_out.speed = obs->spd_est_pu;
}
