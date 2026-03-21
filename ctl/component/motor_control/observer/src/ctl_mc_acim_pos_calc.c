/**
 * @file im_pos_calc.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation of the Rotor Flux Position & Slip Estimator (IFOC).
 *
 * @version 2.0
 * @date 2024-10-27
 *
 * @copyright Copyright GMP(c) 2024
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// acm speed calculator (slip observer)

#include <ctl/component/motor_control/observer/acim_pos_calc.h>

void ctl_init_im_pos_calc(ctl_im_pos_calc_t* calc, const ctl_im_pos_calc_init_t* init)
{
    calc->sf_lpf_kr = float2ctrl(init->sf_lpf_kr);
    calc->sf_slip_const = float2ctrl(init->sf_slip_const);
    calc->sf_mech_to_elec = float2ctrl(init->sf_mech_to_elec);
    calc->sf_w_to_angle = float2ctrl(init->sf_w_to_angle);

    // Robust minimum threshold limit to prevent division by zero
    parameter_gt lim = (init->i_md_min_limit_pu > 1e-4f) ? init->i_md_min_limit_pu : 0.01f;
    calc->i_md_min_limit = float2ctrl(lim);

    ctl_clear_im_pos_calc(calc);
    ctl_disable_im_pos_calc(calc);
}

void ctl_init_im_pos_calc_consultant(ctl_im_pos_calc_t* calc, const ctl_consultant_im_t* motor,
                                     const ctl_consultant_pu_im_t* pu, parameter_gt fs)
{
    ctl_im_pos_calc_init_t bare_init;
    parameter_gt Ts = 1.0f / fs;

    // ========================================================================
    // Physical Parameter PU Derivations
    // ========================================================================
    parameter_gt tau_r = motor->tau_r; // Lr / Rr from the IM Consultant

    // 1. Magnetizing Current Filter Constant
    // LPF equation: T_s / tau_r
    bare_init.sf_lpf_kr = Ts / tau_r;

    // 2. Slip Equation Constant
    // Physical: w_slip = i_sq / (tau_r * i_md)
    // In PU: w_slip_pu = i_sq_pu / (tau_r * W_base * i_md_pu)
    bare_init.sf_slip_const = 1.0f / (tau_r * pu->W_base);

    // 3. Mechanical to Electrical Conversion
    // In strict PU architectures, base mechanical speed is W_base / pole_pairs.
    // If the user feeds mechanical speed normalized to this base, the ratio is exactly 1.0.
    bare_init.sf_mech_to_elec = 1.0f;

    // 4. Angle Integration Constant
    // Integration delta: w_sync_pu * (W_base * Ts / 2*PI)
    bare_init.sf_w_to_angle = (pu->W_base * Ts) / CTL_PARAM_CONST_2PI;

    // 5. Protection margin (e.g., 5% of nominal magnetizing current)
    bare_init.i_md_min_limit_pu = 0.05f;

    // Invoke core initialization
    ctl_init_im_pos_calc(calc, &bare_init);
}
