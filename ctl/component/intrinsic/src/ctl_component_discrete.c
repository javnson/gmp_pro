/**
 * @file ctl_common_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */
#include <gmp_core.h>

#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Filter IIR2

#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <math.h> // support for sinf and cosf

void ctl_init_lp_filter(ctl_low_pass_filter_t* lpf, parameter_gt fs, parameter_gt fc)
{
    lpf->out = 0;
    lpf->a = ctl_helper_lp_filter(fs, fc);
}

void ctl_init_filter_iir2(ctl_filter_IIR2_t* obj, ctl_filter_IIR2_setup_t* setup_obj)
{
    // center frequency
    // tex: $$ f_0 = f_c * 2Q$$
    parameter_gt f0 = setup_obj->fc * 2 * setup_obj->q;

    // tex: $$ \theta = 2\pi \frac{f_c}{f_s}$$
    parameter_gt theta = CTL_PARAM_CONST_2PI * f0 / setup_obj->fs;

    parameter_gt sin_theta = sinf(theta);

    parameter_gt cos_theta = cosf(theta);

    // tex: $$\alpha = \frac{\sin(\theta)}{2Q} $$
    parameter_gt alpha = sin_theta / 2 / setup_obj->q;

    // a_0, a_1, a_2
    parameter_gt a0 = (1.0f + alpha);
    obj->a[0] = float2ctrl(-2.0f * cos_theta / a0);
    obj->a[1] = float2ctrl((1.0f - alpha) / a0);

    switch (setup_obj->filter_type)
    {
    case FILTER_IIR2_TYPE_HIGHPASS:
        obj->b[0] = float2ctrl(setup_obj->gain * (1.0f + cos_theta) / (2 * a0));
        obj->b[1] = float2ctrl(-setup_obj->gain * (1.0f + cos_theta) / a0);
        obj->b[2] = float2ctrl(setup_obj->gain * (1.0f + cos_theta) / (2 * a0));
        break;
    case FILTER_IIR2_TYPE_LOWPASS:
        obj->b[0] = float2ctrl(setup_obj->gain * (1.0f - cos_theta) / (2 * a0));
        obj->b[1] = float2ctrl(setup_obj->gain * (1.0f - cos_theta) / a0);
        obj->b[2] = float2ctrl(setup_obj->gain * (1.0f - cos_theta) / (2 * a0));
        break;
    case FILTER_IIR2_TYPE_BANDPASS:
        obj->b[0] = float2ctrl(setup_obj->gain * sin_theta / (2 * a0));
        obj->b[1] = 0;
        obj->b[2] = float2ctrl(setup_obj->gain * sin_theta / (2 * a0));
        break;
    default:
        // do nothing
        break;
    }

    obj->out = 0;
}

////////////////////////////////////////////////////////////////////////////
//// PLL module
//
//#include <ctl/component/intrinsic/discrete/pll.h>
//
//void ctl_init_pll(
//    // PLL Controller Object
//    ctl_pll_t* pll,
//    // PID parameter
//    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
//    // PID output limit
//    ctrl_gt out_min, ctrl_gt out_max,
//    // cutoff frequency
//    parameter_gt fc,
//    // Sample frequency
//    parameter_gt fs)
//{
//    ctl_init_pid(&pll->pid, kp, Ti, Td, fs);
//    ctl_set_pid_limit(&pll->pid, out_max, out_min);
//    ctl_init_lp_filter(&pll->filter, fs, fc);
//}

//////////////////////////////////////////////////////////////////////////
// Signal Generator

#include <ctl/component/intrinsic/discrete/signal_generator.h>

void ctl_init_sine_generator(ctl_sine_generator_t* sg,
                             parameter_gt init_angle, // pu
                             parameter_gt step_angle) // pu
{
    sg->ph_cos = float2ctrl(cos(init_angle));
    sg->ph_sin = float2ctrl(sin(init_angle));

    sg->ph_sin_delta = float2ctrl(sin(step_angle));
    sg->ph_cos_delta = float2ctrl(cos(step_angle));
}

void ctl_init_ramp_generator(ctl_ramp_generator_t* rg, ctrl_gt slope, parameter_gt amp_pos, parameter_gt amp_neg)
{
    rg->current = float2ctrl(0);

    rg->maximum = float2ctrl(amp_pos);
    rg->minimum = float2ctrl(amp_neg);

    rg->slope = slope;
}

void ctl_init_ramp_generator_via_freq(
    // pointer to ramp generator object
    ctl_ramp_generator_t* rg,
    // isr frequency, unit Hz
    parameter_gt isr_freq,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // ramp range
    parameter_gt amp_pos, parameter_gt amp_neg)
{
    rg->current = float2ctrl(0);

    rg->maximum = float2ctrl(amp_pos);
    rg->minimum = float2ctrl(amp_neg);

    float a = isr_freq / target_freq;
    float b = amp_pos - amp_neg;

    // rg->slope = float2ctrl((amp_pos - amp_neg) / (isr_freq / target_freq));
    rg->slope = float2ctrl(b / a);
}

//////////////////////////////////////////////////////////////////////////
// Discrete PID controller

#include <ctl/component/intrinsic/discrete/discrete_pid.h>
#ifdef _USE_DEBUG_DISCRETE_PID
void ctl_init_discrete_pid(
    // pointer to pid object
    discrete_pid_t* pid,
    // gain of the pid controller
    parameter_gt kp,
    // Time constant for integral and differential part, unit Hz
    parameter_gt Ti, parameter_gt Td,
    // sample frequency, unit Hz
    parameter_gt fs)
{
    pid->input = 0;
    pid->input_1 = 0;
    pid->input_2 = 0;
    pid->output = 0;
    pid->output_1 = 0;

    parameter_gt ki = kp / Ti;
    parameter_gt kd = kp * Td;

    parameter_gt b2 = kd * fs;
    parameter_gt b1 = ki / 2.0f / fs - 2.0f * kd * fs;
    parameter_gt b0 = kd * fs + ki / 2.0f / fs;

    pid->kp = float2ctrl(kp);

    pid->b2 = float2ctrl(b2);
    pid->b1 = float2ctrl(b1);
    pid->b0 = float2ctrl(b0);

    pid->output_max = float2ctrl(1.0);
    pid->output_min = float2ctrl(-1.0);
}
#else // _USE_DEBUG_DISCRETE_PID
void ctl_init_discrete_pid(
    // pointer to pid object
    discrete_pid_t* pid,
    // gain of the pid controller
    parameter_gt kp,
    // Time constant for integral and differential part, unit Hz
    parameter_gt Ti, parameter_gt Td,
    // sample frequency, unit Hz
    parameter_gt fs)
{
    pid->input = 0;
    pid->input_1 = 0;
    pid->input_2 = 0;
    pid->output = 0;
    pid->output_1 = 0;

    parameter_gt ki = kp / Ti;
    parameter_gt kd = kp * Td;

    parameter_gt b2 = kd * fs;
    parameter_gt b1 = ki / 2.0f / fs - kp - 2.0f * kd * fs;
    parameter_gt b0 = kp + kd * fs + ki / 2.0f / fs;

    pid->b2 = float2ctrl(b2);
    pid->b1 = float2ctrl(b1);
    pid->b0 = float2ctrl(b0);

    pid->output_max = float2ctrl(1.0);
    pid->output_min = float2ctrl(-1.0);
}

#endif // _USE_DEBUG_DISCRETE_PID

//////////////////////////////////////////////////////////////////////////
// Discrete track pid

#include <ctl/component/intrinsic/discrete/track_discrete_pid.h>

void ctl_init_tracking_pid(
    // pointer to track pid object
    ctl_tracking_discrete_pid_t* tp,
    // pid parameters, unit sec
    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
    // saturation limit
    ctrl_gt sat_max, ctrl_gt sat_min,
    // slope limit, unit: p.u./sec
    parameter_gt slope_max, parameter_gt slope_min,
    // division factor:
    uint32_t division,
    // controller frequency, unit Hz
    parameter_gt fs)
{
    ctl_init_slope_limiter(&tp->traj, slope_max, slope_min, fs);
    ctl_init_divider(&tp->div, division);

    ctl_init_discrete_pid(&tp->pid, kp, Ti, Td, fs);
    ctl_set_discrete_pid_limit(&tp->pid, sat_max, sat_min);
}

//////////////////////////////////////////////////////////////////////////
// Pole-Zero Compensator

#include <ctl/component/intrinsic/discrete/pole_zero.h>

// unit Hz
void ctl_init_2p2z(
    // pointer to a 2p2z compensator
    ctrl_2p2z_t* ctrl,
    // gain of 2P2Z compensator
    parameter_gt gain,
    // two zero frequency, unit Hz
    parameter_gt f_z0, parameter_gt f_z1,
    // one pole frequency, unit Hz
    parameter_gt f_p1,
    // sample frequency
    parameter_gt fs)
{
    parameter_gt z0 = f_z0 * CTL_PARAM_CONST_2PI;
    parameter_gt z1 = f_z1 * CTL_PARAM_CONST_2PI;
    parameter_gt p1 = f_p1 * CTL_PARAM_CONST_2PI;

    // discrete controller parameter
    parameter_gt gain_discrete = gain * (1.0f / 2.0f / fs / (p1 + 2.0f * fs));
    parameter_gt b0 = (z1 + 2.0f * fs) * (z0 + 2.0f * fs);
    parameter_gt b1 = ((z0 + 2.0f * fs) * (z1 - 2.0f * fs) + (z1 + 2.0f * fs) * (z0 - 2.0f * fs));
    parameter_gt b2 = (z1 - 2.0f * fs) * (z0 - 2.0f * fs);
    parameter_gt a1 = -(4.0f * fs / (p1 + 2.0f * fs));
    parameter_gt a2 = (2.0f * fs - p1) / (2.0f * fs + p1);

    ctrl->a1 = float2ctrl(a1);
    ctrl->a2 = float2ctrl(a2);
    ctrl->b0 = float2ctrl(b0);
    ctrl->b1 = float2ctrl(b1);
    ctrl->b2 = float2ctrl(b2);
    ctrl->gain = float2ctrl(gain_discrete);

    ctrl->out_max = float2ctrl(1.0);
    ctrl->out_min = float2ctrl(-1.0);

    ctrl->output_1 = 0;
    ctrl->output_2 = 0;
    ctrl->input_1 = 0;
    ctrl->input_2 = 0;

    ctrl->output = 0;
    ctrl->input = 0;
}

//////////////////////////////////////////////////////////////////////////
// PR / QPR controller

#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

void ctl_init_resonant_controller(resonant_ctrl_t* r, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs)
{
    parameter_gt T = 1.0f / fs;
    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wr_sq_T_sq = wr * wr * T * T;

    // Based on the bilinear transformation of G(s) = kr * (2s) / (s^2 + wr^2)
    // The resulting difference equation is:
    // u(n) = a1*u(n-1) + a2*u(n-2) + b0*e(n) + b2*e(n-2)
    parameter_gt den = wr_sq_T_sq + 4.0f;
    parameter_gt inv_den = 1.0f / den;

    r->b0 = float2ctrl(kr * 2.0f * T * inv_den);
    r->b2 = float2ctrl(-kr * 2.0f * T * inv_den);
    r->a1 = float2ctrl(2.0f * (4.0f - wr_sq_T_sq) * inv_den);
    r->a2 = float2ctrl(-1.0f); // This simplifies from -(4 + T^2*wr^2 - 8)/(4+T^2*wr^2) if no damping

    // In the original code, the denominator of a1 was different.
    // The standard discretization of an ideal resonant controller leads to a2 = -1.
    // The original code's coefficients seem to be from a slightly different form.
    // Let's stick to the standard bilinear transform result.
    // Original code had: r->kr = float2ctrl(2.0f * (4.0f * fs * fs - wr * wr) / (4.0f * fs * fs + wr * wr));
    // and r->krg = float2ctrl(kr * 4.0f * fs / (4.0f * fs * fs + wr * wr));
    // which corresponds to a different difference equation form.
    // The new form is u(n) = a1*u(n-1) + a2*u(n-2) + b0*e(n) + b2*e(n-2)

    ctl_clear_resonant_controller(r);
}

void ctl_init_pr_controller(pr_ctrl_t* pr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                            parameter_gt fs)
{
    pr->kp = kp;
    ctl_init_resonant_controller(&pr->resonant_part, kr, freq_resonant, fs);
}

void ctl_init_qr_controller(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                            parameter_gt fs)
{
    parameter_gt T = 1.0f / fs;
    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wc = CTL_PARAM_CONST_2PI * freq_cut;

    // Based on the bilinear transformation of G(s) = kr * (2*wc*s) / (s^2 + 2*wc*s + wr^2)
    // The resulting difference equation is:
    // u(n) = a1*u(n-1) + a2*u(n-2) + b0*e(n) + b2*e(n-2)
    parameter_gt den = 4.0f + 4.0f * wc * T + wr * wr * T * T;
    parameter_gt inv_den = 1.0f / den;

    qr->b0 = float2ctrl(kr * 4.0f * wc * T * inv_den);
    qr->b2 = float2ctrl(-kr * 4.0f * wc * T * inv_den);
    qr->a1 = float2ctrl((8.0f - 2.0f * wr * wr * T * T) * inv_den);
    qr->a2 = float2ctrl((-4.0f + 4.0f * wc * T - wr * wr * T * T) * inv_den);

    ctl_clear_qr_controller(qr);
}

void ctl_init_qpr_controller(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                             parameter_gt freq_cut, parameter_gt fs)
{
    qpr->kp = kp;
    ctl_init_qr_controller(&qpr->resonant_part, kr, freq_resonant, freq_cut, fs);
}

//void ctl_init_resonant_controller(
//    // handle of PR controller
//    resonant_ctrl_t* r,
//    // gain of resonant frequency
//    parameter_gt kr,
//    // resonant frequency, unit Hz
//    parameter_gt freq_resonant,
//    // controller frequency
//    parameter_gt fs)
//{
//    // resonant frequency, unit rad/s
//    parameter_gt omega_r = CTL_PARAM_CONST_2PI * freq_resonant;
//
//    r->krg = float2ctrl(kr * (4 * fs) / (4 * fs * fs + omega_r * omega_r));
//    r->kr = float2ctrl(2 * (4 * fs * fs - omega_r * omega_r) / (4 * fs * fs + omega_r * omega_r));
//
//    // clear temp variables
//    ctl_clear_resonant_controller(r);
//}
//
//void ctl_init_pr_controller(
//    // handle of PR controller
//    pr_ctrl_t *pr,
//    // Kp
//    parameter_gt kp,
//    // gain of resonant frequency
//    parameter_gt kr,
//    // resonant frequency, unit Hz
//    parameter_gt freq_resonant,
//    // controller frequency
//    parameter_gt fs)
//{
//    // resonant frequency, unit rad/s
//    parameter_gt omega_r = CTL_PARAM_CONST_2PI * freq_resonant;
//
//    pr->kpg = float2ctrl(kp);
//    pr->krg = float2ctrl(kr * (4 * fs) / (4 * fs * fs + omega_r * omega_r));
//    pr->kr = float2ctrl(2 * (4 * fs * fs - omega_r * omega_r) / (4 * fs * fs + omega_r * omega_r));
//
//    // clear temp variables
//    ctl_clear_pr_controller(pr);
//}
//
//void ctl_init_qr_controller(
//    // handle of QR controller
//    qr_ctrl_t* qr,
//    // gain of resonant frequency
//    parameter_gt kr,
//    // resonant frequency, unit Hz
//    parameter_gt freq_resonant,
//    // cut frequency, unit Hz
//    parameter_gt freq_cut,
//    // controller frequency
//    parameter_gt fs)
//{
//    ctl_clear_qr_controller(qr);
//
//    parameter_gt omega_r_sqr = 4.0f * PI * PI * freq_resonant * freq_resonant;
//    parameter_gt omega_c_fs = 4.0f * 2.0f * PI * freq_cut * fs;
//
//    parameter_gt kr_suffix = 4.0f * freq_cut * fs / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//
//    parameter_gt b1 = 2.0f * (4.0f * fs * fs - omega_r_sqr) / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//    parameter_gt b2 = (4.0f * fs * fs - omega_c_fs + omega_r_sqr) / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//
//    // Discrete parameters
//    qr->a0 = float2ctrl(kr_suffix);
//    qr->b1 = float2ctrl(b1);
//    qr->b2 = float2ctrl(b2);
//    qr->kr = float2ctrl(kr);
//}
//
//void ctl_init_qpr_controller(
//    // handle of QPR controller
//    qpr_ctrl_t *qpr,
//    // Kp
//    parameter_gt kp,
//    // gain of resonant frequency
//    parameter_gt kr,
//    // resonant frequency, unit Hz
//    parameter_gt freq_resonant,
//    // cut frequency, unit Hz
//    parameter_gt freq_cut,
//    // controller frequency
//    parameter_gt fs)
//{
//    ctl_clear_qpr_controller(qpr);
//
//    parameter_gt omega_r_sqr = 4.0f * PI * PI * freq_resonant * freq_resonant;
//    parameter_gt omega_c_fs = 4.0f * 2.0f * PI * freq_cut * fs;
//
//    parameter_gt kr_suffix = 4.0f * freq_cut * fs / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//
//    parameter_gt b1 = 2.0f * (4.0f * fs * fs - omega_r_sqr) / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//    parameter_gt b2 = (4.0f * fs * fs - omega_c_fs + omega_r_sqr) / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//
//    // Discrete parameters
//    qpr->a0 = float2ctrl(kr_suffix);
//    qpr->b1 = float2ctrl(b1);
//    qpr->b2 = float2ctrl(b2);
//
//    qpr->kp = float2ctrl(kp);
//    qpr->kr = float2ctrl(kr);
//}

//////////////////////////////////////////////////////////////////////////
//

#include <ctl/component/intrinsic/discrete/discrete_sogi.h>

void ctl_init_discrete_sogi(
    // Handle of discrete SOGI object
    discrete_sogi_t* sogi,
    // damp coefficient, generally is 0.5
    parameter_gt k_damp,
    // center frequency, Hz
    parameter_gt fn,
    // isr frequency, Hz
    parameter_gt fs)
{
    ctl_clear_discrete_sogi(sogi);

    parameter_gt osgx, osgy, temp, wn, delta_t;
    delta_t = 1.0f / fs;
    wn = fn * CTL_PARAM_CONST_2PI;
    // wn = fn * GMP_CONST_2_PI;

    osgx = (2.0f * k_damp * wn * delta_t);
    osgy = (parameter_gt)(wn * delta_t * wn * delta_t);
    temp = (parameter_gt)1.0 / (osgx + osgy + 4.0f);

    sogi->b0 = float2ctrl(osgx * temp);
    sogi->b2 = -sogi->b0;
    sogi->a1 = float2ctrl((2.0f * (4.0f - osgy)) * temp);
    sogi->a2 = float2ctrl((osgx - osgy - 4.0f) * temp);
    sogi->qb0 = float2ctrl((k_damp * osgy) * temp);
    sogi->qb1 = float2ctrl(sogi->qb0 * (2.0f));
    sogi->qb2 = sogi->qb0;
}

//////////////////////////////////////////////////////////////////////////
// Lead Lag controller

#include "ctl/component/intrinsic/discrete/lead_lag.h"

void ctl_init_lead(ctrl_lead_t* obj, parameter_gt K_D, parameter_gt tau_D, parameter_gt fs)
{
    // Sampling period
    parameter_gt T = 1.0f / fs;

    // Denominator term from bilinear transform: (2*tau_D + T)
    parameter_gt den = 2.0f * tau_D + T;
    parameter_gt inv_den;

    // Avoid division by zero
    if (fabsf(den) < 1e-9f)
    {
        inv_den = 0.0f; // Or handle error appropriately
    }
    else
    {
        inv_den = 1.0f / den;
    }

    // Calculate coefficients based on the discretized transfer function
    // H(z) = (b0 + b1*z^-1) / (1 - a1*z^-1)

    // a1 = (2*tau_D - T) / (2*tau_D + T)
    obj->a1 = float2ctrl((2.0f * tau_D - T) * inv_den);

    // b0 = (2*tau_D + 2*K_D + T) / (2*tau_D + T)
    obj->b0 = float2ctrl((2.0f * tau_D + 2.0f * K_D + T) * inv_den);

    // b1 = (T - 2*tau_D - 2*K_D) / (2*tau_D + T)
    obj->b1 = float2ctrl((T - 2.0f * tau_D - 2.0f * K_D) * inv_den);

    // Clear initial states
    ctl_clear_lead(obj);
}

void ctl_init_lag(ctrl_lag_t* obj, parameter_gt tau_L, parameter_gt tau_P, parameter_gt fs)
{
    // Sampling period
    parameter_gt T = 1.0f / fs;

    // Denominator term from bilinear transform: (2*tau_P + T)
    parameter_gt den = 2.0f * tau_P + T;
    parameter_gt inv_den;

    // Avoid division by zero
    if (fabsf(den) < 1e-9f)
    {
        inv_den = 0.0f; // Or handle error appropriately
    }
    else
    {
        inv_den = 1.0f / den;
    }

    // Calculate coefficients based on the discretized transfer function
    // H(z) = (b0 + b1*z^-1) / (1 - a1*z^-1)

    // a1 = (2*tau_P - T) / (2*tau_P + T)
    obj->a1 = float2ctrl((2.0f * tau_P - T) * inv_den);

    // b0 = (2*tau_L + T) / (2*tau_P + T)
    obj->b0 = float2ctrl((2.0f * tau_L + T) * inv_den);

    // b1 = (T - 2*tau_L) / (2*tau_P + T)
    obj->b1 = float2ctrl((T - 2.0f * tau_L) * inv_den);

    // Clear initial states
    ctl_clear_lag(obj);
}

// Note: Implementations for ctl_init_2p2z and other generic pole-zero
// initializers would go here if they were defined.

//////////////////////////////////////////////////////////////////////////
// Z transfer function

#include <ctl/component/intrinsic/discrete/z_function.h>

void ctl_init_z_function(ctl_z_function_t* obj, int32_t num_order, const ctrl_gt* num_coeffs, int32_t den_order,
                         const ctrl_gt* den_coeffs, ctrl_gt* input_buffer, ctrl_gt* output_buffer)
{
    // Assign orders
    obj->num_order = num_order;
    obj->den_order = den_order;

    // Assign pointers to coefficients and state buffers
    obj->num_coeffs = num_coeffs;
    obj->den_coeffs = den_coeffs;
    obj->input_buffer = input_buffer;
    obj->output_buffer = output_buffer;

    // Clear all state buffers to ensure a clean start
    ctl_clear_z_function(obj);
}
