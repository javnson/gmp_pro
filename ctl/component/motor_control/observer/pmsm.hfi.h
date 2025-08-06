/**
 * @file pmsm.hfi.h
 * @author Javnson (javnson@zju.edu.cn); MY_Lin(lammanyee@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2025-05-26
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

// This module is a High frequency injection module
// Ud cos injection

// include pll module
// #include <ctl/component/common/pll.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>
#include <ctl/component/motor_control/consultant/motor_driver_consultant.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>

// #include <arm_math.h>

#ifndef _FILE_PMSM_HFI_H_
#define _FILE_PMSM_HFI_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef struct _tag_ctl_hfi_init_struct
{
    //
    // Motor Model Parameters
    //

    // Motor parameters, unit SI
    parameter_gt Rs;
    parameter_gt Ld;
    parameter_gt Lq;

    uint16_t pole_pairs;

    // Base speed RPM
    parameter_gt speed_base_rpm;

    // Controller ISR frequency
    parameter_gt f_ctrl;

    // per unit base value
    parameter_gt u_base;
    parameter_gt i_base;

    //
    // HFI Controller Parameters
    //

    // HFI ud injection frequency, Hz
    parameter_gt f_hfi;
    // HFI ud injection amplitude, pu
    parameter_gt u_amp_hfi;

    // iq low pass frequency, Hz
    parameter_gt iq_lp_fc;
    // iq low pass damping
    parameter_gt iq_lp_damp;

    // PLL PI controller parameters
    ctrl_gt pid_kp;
    ctrl_gt pid_Ti;
    ctrl_gt pid_Td;
    // PLL PI Controller limit, unit p.u.
    ctrl_gt spd_max_limit;
    ctrl_gt spd_min_limit;

} ctl_hfi_init_t;

typedef struct _tag_ctl_pmsm_hfi_t
{
    //
    // encoder interface, output
    //
    rotation_ift encif;
    velocity_ift spdif;

    //
    // input section
    //
    ctrl_gt iq;

    //
    // output section
    //
    ctrl_gt ud_inj;

    //
    // intermediate variable section
    //
    // iq demodulate
    ctrl_gt iq_demodulate;
    // lowpass output, in proportion to theta error
    ctrl_gt theta_error;
    // PLL output mechanical omega: wr, rad/tick
    ctrl_gt wr;

    //
    // Controller Entity
    //
    // HFI modulation
    ctl_sine_generator_t hfi_sincos_gen;
    ctrl_gt hfi_inj_amp;

    // HFI iq lowpass filter
    ctl_filter_IIR2_t iq_lp_filter;

    // PLL
    // pid controller
    ctl_pid_t pid_pll;
    // speed scale factor
    // scale factor: rad/tick -> p.u.
    ctrl_gt spd_sf;
    // Mechanical Position Estimation
    ctrl_gt theta_r_est;
    // pole pairs
    ctrl_gt pole_pairs;

} pmsm_hfi_t;

void ctl_input_pmsm_hfi(pmsm_hfi_t *hfi, ctrl_gt iq)
{
    hfi->iq = iq;
}

GMP_STATIC_INLINE
ctrl_gt ctl_step_pmsm_hfi(pmsm_hfi_t *hfi)
{
    // HFI modulation
    hfi->ud_inj = ctl_mul(ctl_get_sine_generator_cos(&hfi->hfi_sincos_gen), hfi->hfi_inj_amp);
    hfi->iq_demodulate = ctl_mul(hfi->iq, ctl_get_sine_generator_sin(&hfi->hfi_sincos_gen));
    ctl_step_sine_generator(&hfi->hfi_sincos_gen);

    // iq lowpass filter: iq_modulate->theta error
    hfi->theta_error = ctl_step_filter_iir2(&hfi->iq_lp_filter, hfi->iq_demodulate);

    // PLL:theta_error->speed_rad/tick, theta_r_est
    // 1. PI: theta_error->speed_rad/tick
    hfi->wr = ctl_step_pid_par(&hfi->pid_pll, hfi->theta_error);
    // 2. update mechanical theta, integral
    hfi->theta_r_est += ctl_mul(hfi->wr, CTL_CTRL_CONST_1_OVER_2PI);
    hfi->theta_r_est = ctrl_mod_1(hfi->theta_r_est);
    // 3. update spdif: wr,rad/tick -> spd_sf
    hfi->spdif.speed = ctl_mul(hfi->wr, hfi->spd_sf);
    // 4. update encif: output phase, and phase compensate
    hfi->encif.position = hfi->theta_r_est;
    hfi->encif.elec_position = ctrl_mod_1(hfi->theta_r_est * hfi->pole_pairs);

    return hfi->ud_inj;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_HFI_H_
