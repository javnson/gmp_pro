/**
 * @file ctl_common_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2025-03-19
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

#include <math.h>

//////////////////////////////////////////////////////////////////////////
// PID regular

#include <ctl/component/intrinsic/continuous/continuous_pid.h>

void ctl_init_pid(
    // continuous pid handle
    ctl_pid_t* hpid,
    // PID parameters
    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
    // controller frequency
    parameter_gt fs)
{
    hpid->kp = float2ctrl(kp);
    hpid->ki = float2ctrl(kp / (fs * Ti));
    hpid->kd = float2ctrl(kp * fs * Td);

    hpid->out_min = float2ctrl(-1.0f);
    hpid->out_max = float2ctrl(1.0f);

    hpid->integral_min = float2ctrl(-0.8f);
    hpid->integral_max = float2ctrl(0.8f);

    hpid->out = 0;
    hpid->dn = 0;
    hpid->sn = 0;
}

// Init a series PID object
void ctl_init_pid_ser(
    // continuous pid handle
    ctl_pid_t* hpid,
    // PID parameters
    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
    // controller frequency
    parameter_gt fs)
{
    hpid->kp = float2ctrl(kp);
    hpid->ki = float2ctrl(1.0f / (fs * Ti));
    hpid->kd = float2ctrl(1.0f * fs * Td);

    hpid->out_min = float2ctrl(-1.0f);
    hpid->out_max = float2ctrl(1.0f);

    hpid->integral_min = float2ctrl(-0.8f);
    hpid->integral_max = float2ctrl(0.8f);

    hpid->out = 0;
    hpid->dn = 0;
    hpid->sn = 0;
}

// init a parallel PID object
void ctl_init_pid_par(
    // continuous pid handle
    ctl_pid_t* hpid,
    // PID parameters
    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
    // controller frequency
    parameter_gt fs)
{
    hpid->kp = float2ctrl(kp);
    hpid->ki = float2ctrl(1.0f / (fs * Ti));
    hpid->kd = float2ctrl(1.0f * fs * Td);

    hpid->out_min = float2ctrl(-1.0f);
    hpid->out_max = float2ctrl(1.0f);

    hpid->integral_min = float2ctrl(-0.8f);
    hpid->integral_max = float2ctrl(0.8f);

    hpid->out = 0;
    hpid->dn = 0;
    hpid->sn = 0;
}

// init a Series PID
void ctl_init_pid_aw_ser(
    // continuous pid handle
    ctl_pid_aw_t* hpid,
    // PID parameters
    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
    // controller frequency
    parameter_gt fs)
{
    hpid->kp = float2ctrl(kp);
    hpid->ki = float2ctrl(kp / (fs * Ti));
    hpid->kd = float2ctrl(kp * fs * Td);

    // set anti-windup parameter based on kp
    if (kp < 0.7f)
        hpid->kc = float2ctrl(1.3f);
    else if (kp > 2.0f)
        hpid->kc = float2ctrl(0.5f);
    else
        hpid->kc = float2ctrl(1 / kp);

    hpid->out_min = float2ctrl(-1.0f);
    hpid->out_max = float2ctrl(1.0f);

    hpid->out = 0;
    hpid->dn = 0;
    hpid->sn = 0;
}

// init a parallel PID
void ctl_init_pid_aw_par(
    // continuous pid handle
    ctl_pid_aw_t* hpid,
    // PID parameters
    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
    // controller frequency
    parameter_gt fs)
{

    hpid->kp = float2ctrl(kp);
    hpid->ki = float2ctrl(kp / (fs * Ti));
    hpid->kd = float2ctrl(kp * fs * Td);

    // set anti-windup parameter based on kp
    if (kp < 0.7f)
        hpid->kc = float2ctrl(1.3f);
    else if (kp > 2.0f)
        hpid->kc = float2ctrl(0.5f);
    else
        hpid->kc = float2ctrl(1 / kp);

    hpid->out_min = float2ctrl(-1.0f);
    hpid->out_max = float2ctrl(1.0f);

    hpid->out = 0;
    hpid->dn = 0;
    hpid->sn = 0;
}

//////////////////////////////////////////////////////////////////////////
// Track_PID.h
//

#include <ctl/component/intrinsic/continuous/track_pid.h>

void ctl_init_tracking_continuous_pid(
    // handle of track pid
    ctl_tracking_continuous_pid_t* tp,
    // pid parameters
    ctrl_gt kp, ctrl_gt ki, ctrl_gt kd,
    // saturation limit
    ctrl_gt sat_max, ctrl_gt sat_min,
    // slope limit
    ctrl_gt slope_max, ctrl_gt slope_min,
    // division factor
    uint32_t division,
    // controller frequency
    parameter_gt fs)
{
    ctl_init_slope_limiter(&tp->traj, slope_max, slope_min, fs);
    ctl_init_divider(&tp->div, division);

    ctl_init_pid(&tp->pid, kp, ki, kd, fs);
    ctl_set_pid_limit(&tp->pid, sat_max, sat_min);
}

//////////////////////////////////////////////////////////////////////////
// SOGI controller

#include <ctl/component/intrinsic/continuous/sogi.h>

void ctl_init_sogi_controller(
    // controller handle
    ctl_sogi_t* sogi,
    // gain of this controller
    parameter_gt gain,
    // resonant frequency
    parameter_gt freq_r,
    // cut frequency
    parameter_gt freq_c,
    // controller frequency
    parameter_gt freq_ctrl)
{
    sogi->k_damp = float2ctrl(2 * freq_c / freq_r);
    sogi->k_r = float2ctrl(2 * PI * freq_r / freq_ctrl);
    sogi->gain = float2ctrl(gain);

    sogi->integrate_reference = 0;
    sogi->d_integrate = 0;
    sogi->q_integrate = 0;
}

void ctl_init_sogi_controller_with_damp(
    // controller handle
    ctl_sogi_t* sogi,
    // gain of this controller
    parameter_gt gain,
    // resonant frequency
    parameter_gt freq_r,
    // cut frequency, generally 1.414 is a great choice
    parameter_gt damp,
    // controller frequency
    parameter_gt freq_ctrl)
{
    sogi->k_damp = float2ctrl(damp);
    sogi->k_r = float2ctrl(2 * PI * freq_r / freq_ctrl);
    sogi->gain = float2ctrl(gain);

    sogi->integrate_reference = 0;
    sogi->d_integrate = 0;
    sogi->q_integrate = 0;
}

//////////////////////////////////////////////////////////////////////////
// S function

#include <ctl/component/intrinsic/continuous/s_function.h>

void ctl_init_s_function(ctl_s_function_t* obj, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2,
                         parameter_gt f_p1, parameter_gt f_p2, parameter_gt fs)
{
    // Convert frequencies (Hz) to angular frequencies (rad/s)
    parameter_gt wz1 = 2.0f * PI * f_z1;
    parameter_gt wz2 = 2.0f * PI * f_z2;
    parameter_gt wp1 = 2.0f * PI * f_p1;
    parameter_gt wp2 = 2.0f * PI * f_p2;

    // From H(s) = K * [(s/wz1+1)(s/wz2+1)] / [(s/wp1+1)(s/wp2+1)],
    // we get the polynomial form H(s) = K * (B2*s^2 + B1*s + B0) / (A2*s^2 + A1*s + A0)
    // Note: This form is normalized so that the constant term is 1.
    parameter_gt B2 = 1.0f / (wz1 * wz2);
    parameter_gt B1 = (wz1 + wz2) / (wz1 * wz2);
    parameter_gt B0 = 1.0f;

    parameter_gt A2 = 1.0f / (wp1 * wp2);
    parameter_gt A1 = (wp1 + wp2) / (wp1 * wp2);
    parameter_gt A0 = 1.0f;

    // Discretize using the Bilinear Transform: s = 2/T * (1-z^-1)/(1+z^-1)
    parameter_gt T = 1.0f / fs;
    parameter_gt T_sq = T * T;
    parameter_gt two_over_T = 2.0f / T;
    parameter_gt four_over_T_sq = 4.0f / T_sq;

    // Common denominator for Z-domain coefficients
    parameter_gt den = A2 * four_over_T_sq + A1 * two_over_T + A0;
    parameter_gt inv_den = 1.0f / den;

    // Calculate Z-domain coefficients for H(z) = (b0+b1*z^-1+b2*z^-2)/(1+a1*z^-1+a2*z^-2)
    obj->b0 = float2ctrl(gain * (B2 * four_over_T_sq + B1 * two_over_T + B0) * inv_den);
    obj->b1 = float2ctrl(gain * (-2.0f * B2 * four_over_T_sq + 2.0f * B0) * inv_den);
    obj->b2 = float2ctrl(gain * (B2 * four_over_T_sq - B1 * two_over_T + B0) * inv_den);

    obj->a1 = float2ctrl((-2.0f * A2 * four_over_T_sq + 2.0f * A0) * inv_den);
    obj->a2 = float2ctrl((A2 * four_over_T_sq - A1 * two_over_T + A0) * inv_den);

    // Clear initial states
    ctl_clear_s_function(obj);
}
