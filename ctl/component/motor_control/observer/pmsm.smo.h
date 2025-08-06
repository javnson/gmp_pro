/**
 * @file pmsm.smo.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// This module is a PMSM SMO module

// include pll module
// #include <ctl/component/common/pll.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/consultant/motor_driver_consultant.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>

// #include <arm_math.h>

#ifndef _FILE_PMSM_SMO_H_
#define _FILE_PMSM_SMO_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef struct _tag_ctl_smo_init_struct
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
    // SMO Controller Parameters
    //

    // EMF filter cut frequency, Hz
    parameter_gt fc_e;
    // Speed filter cut frequency, Hz
    parameter_gt fc_omega;
    // PLL PI controller parameters
    ctrl_gt pid_kp;
    ctrl_gt pid_Ti;
    ctrl_gt pid_Td;

    // PLL PI Controller limit, unit p.u.
    ctrl_gt spd_max_limit;
    ctrl_gt spd_min_limit;

    // SMO Gain
    ctrl_gt k_slide;
} ctl_smo_init_t;

typedef struct _tag_ctl_pmsm_smo_t
{
    //
    // encoder interface
    //
    rotation_ift encif;
    velocity_ift spdif;

    //
    // input section
    //
    ctrl_gt u_alpha;
    ctrl_gt u_beta;

    ctrl_gt i_alpha;
    ctrl_gt i_beta;

    //
    // output section
    //
    // output estimated theta, unit rad
    // This result does not include phase compensation of the back EMF filter
    ctrl_gt theta_est;

    //
    // intermediate variable section
    //

    // i estimate
    ctrl_gt i_alpha_est;
    ctrl_gt i_beta_est;

    // EMF estimate
    ctrl_gt e_alpha_est;
    ctrl_gt e_beta_est;

    // slide model output
    ctrl_gt z_alpha;
    ctrl_gt z_beta;

    // angular velocity,
    // unit rad/tick
    ctrl_gt wr;

    // PLL Error Items
    ctrl_gt e_error;

    // phasor for theta
    ctl_vector2_t phasor;

    //
    // Controller Entity
    //

    // SMO Model parameter k1: Ts*1/Ld
    ctrl_gt k1;

    // SMO Model parameter k2: Ts*Rs/Ld
    ctrl_gt k2;

    // SMO Model parameter k3: (Ld-Lq)*Ts*1/Ld
    // coupling term
    ctrl_gt k3;

    // SMO Model parameter k: Slide Model gain
    ctrl_gt k_slide;

    // EMF filter
    ctl_low_pass_filter_t filter_e_alpha;
    ctl_low_pass_filter_t filter_e_beta;

    // Speed Filter
    ctl_low_pass_filter_t filter_spd;

    // PLL pid controller
    ctl_pid_t pid_pll;

    // speed scale factor
    // scale factor: rad/tick -> p.u.
    ctrl_gt spd_sf;

    // theta compensate
    ctrl_gt theta_compensate;
} pmsm_smo_t;

void ctl_init_pmsm_smo(
    // SMO handle
    pmsm_smo_t *smo,
    // SMO Initialize object
    ctl_smo_init_t *init);

GMP_STATIC_INLINE
void ctl_input_pmsm_smo(pmsm_smo_t *smo, ctrl_gt u_alpha, ctrl_gt u_beta, ctrl_gt i_alpha, ctrl_gt i_beta)
{
    smo->i_alpha = i_alpha;
    smo->i_beta = i_beta;
    smo->u_alpha = u_alpha;
    smo->u_beta = u_beta;
}

GMP_STATIC_INLINE
void ctl_clear_pmsm_smo(pmsm_smo_t *smo)
{
    smo->e_alpha_est = 0;
    smo->e_beta_est = 0;

    smo->z_alpha = 0;
    smo->z_beta = 0;

    smo->i_alpha = 0;
    smo->i_beta = 0;
    smo->u_alpha = 0;
    smo->u_beta = 0;

    smo->i_alpha_est = 0;
    smo->i_beta_est = 0;

    smo->theta_est = 0;

    ctl_clear_pid(&smo->pid_pll);

    ctl_clear_lowpass_filter(&smo->filter_e_alpha);
    ctl_clear_lowpass_filter(&smo->filter_e_beta);
    ctl_clear_lowpass_filter(&smo->filter_spd);
}

    GMP_STATIC_INLINE
    ctrl_gt ctl_step_pmsm_smo(pmsm_smo_t *smo)
    {
        // PMSM Model
        ctrl_gt delta_i_alpha = ctl_mul(smo->k1, smo->u_alpha - smo->z_alpha) -
                                ctl_mul(smo->k2, smo->i_alpha_est) -
                                ctl_mul(ctl_mul(smo->wr, smo->k3), smo->i_beta_est);

        ctrl_gt delta_i_beta = ctl_mul(smo->k1, smo->u_beta - smo->z_beta) -
                               ctl_mul(smo->k2, smo->i_beta_est) + ctl_mul(ctl_mul(smo->wr, smo->k3), smo->i_alpha_est);

    // Step i est
    smo->i_alpha_est += delta_i_alpha;
    smo->i_beta_est += delta_i_beta;

    // Slide model
    smo->z_alpha = ctl_mul(ctl_sat(smo->i_alpha_est - smo->i_alpha, float2ctrl(0.1), -float2ctrl(0.1)), smo->k_slide);
    smo->z_beta = ctl_mul(ctl_sat(smo->i_beta_est - smo->i_beta, float2ctrl(0.1), -float2ctrl(0.1)), smo->k_slide);

    // Filter and get e est
    smo->e_alpha_est = ctl_step_lowpass_filter(&smo->filter_e_alpha, smo->z_alpha);
    smo->e_beta_est = ctl_step_lowpass_filter(&smo->filter_e_beta, smo->z_beta);

    // PLL get angle out
    // 0. generate phasor
    ctl_set_phasor_via_angle(smo->theta_est, &smo->phasor);

    // 1. error signal generate
    smo->e_error = -ctl_mul(smo->e_alpha_est, smo->phasor.dat[1]) - ctl_mul(smo->e_beta_est, smo->phasor.dat[0]);

    // 2. PLL speed lock
    ctl_step_pid_par(&smo->pid_pll, smo->e_error);

    // 3. filter speed, unit rad/tick
    smo->wr = ctl_step_lowpass_filter(&smo->filter_spd, smo->pid_pll.out);

    // 4. update theta
    smo->theta_est += ctl_mul(smo->wr, GMP_CONST_1_OVER_2PI);
    smo->theta_est = ctrl_mod_1(smo->theta_est);

    // 5. update speed
    smo->spdif.speed = ctl_mul(smo->wr, smo->spd_sf);

    // 6. output phase, and phase compensate
    smo->encif.elec_position =
        smo->theta_est +
        ctl_mul(ctl_atan2(ctl_mul(smo->spdif.speed, smo->theta_compensate), GMP_CONST_1), GMP_CONST_1_OVER_2PI);
    smo->encif.elec_position = ctrl_mod_1(smo->encif.elec_position);

    return smo->encif.elec_position;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_SMO_H_
