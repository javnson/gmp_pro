/**
 * @file im_ifoc_core.h
 * @brief Implements an Indirect Field-Oriented Control (IFOC) core for AC Induction Motors.
 *
 * @version 1.0
 * @date 2024-10-26
 *
 */

#ifndef _FILE_IM_IFOC_CORE_H_
#define _FILE_IM_IFOC_CORE_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>
#include <ctl/math_block/coordinate/coord_trans.h>
#include <gmp_core.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* IM IFOC Current Controller                                                */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup IM_IFOC_CONTROLLER IM Indirect Field-Oriented Controller
 * @brief The core current regulation and slip generation loop for IM.
 * @details Implements standard IFOC. Integrates slip frequency to generate the 
 * synchronous reference frame angle. Uses transient inductance for PI tuning 
 * and exact IM physical equations for voltage feedforward decoupling.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the IM IFOC Controller.
 */
typedef struct _tag_im_ifoc_init
{
    parameter_gt fs;            //!< Controller execution frequency (Hz).
    parameter_gt v_bus;         //!< DC Bus voltage (V).
    parameter_gt v_phase_limit; //!< Phase voltage limitation (Vrms).
    parameter_gt v_base;        //!< Base voltage for PU conversion (V).
    parameter_gt i_base;        //!< Base current for PU conversion (A).
    parameter_gt spd_base;      //!< Nominal motor speed base (rpm).
    parameter_gt pole_pairs;    //!< Number of pole pairs.

    // --- IM Physical Parameters ---
    parameter_gt mtr_Rs; //!< Stator resistance (Ohm).
    parameter_gt mtr_Rr; //!< Rotor resistance (Ohm).
    parameter_gt mtr_Ls; //!< Stator self-inductance (H).
    parameter_gt mtr_Lr; //!< Rotor self-inductance (H).
    parameter_gt mtr_Lm; //!< Mutual inductance (H).

    // --- Tuning Targets ---
    parameter_gt current_loop_bw; //!< Target current loop bandwidth (Hz).

} im_ifoc_init_t;

/**
 * @brief Main structure for the IM IFOC current controller.
 */
typedef struct _tag_im_ifoc_ctrl
{
    uint32_t isr_tick;

    // --- Interfaces ---
    tri_adc_ift* adc_iuvw;
    adc_ift* adc_udc;
    rotation_ift* pos_if; //!< Rotor mechanical position interface.

    // --- Setpoints ---
    ctl_vector2_t idq_ref; //!< d-q current reference [flux_ref, torque_ref].

    // --- Angles & Phasors (The Core of IFOC) ---
    ctrl_gt slip_angle_pu;     //!< Integrated slip angle (PU).
    ctrl_gt sync_angle_pu;     //!< Final synchronous electrical angle (PU).
    ctl_vector2_t sync_phasor; //!< Phasor for Park/iPark transforms.

    // --- Measurements & States ---
    ctrl_gt udc;
    ctl_vector3_t iuvw;
    ctl_vector3_t iab0;
    ctl_vector3_t idq0;

    ctrl_gt i_mu_pu; //!< Filtered magnetizing current (Rotor flux proxy).

    // --- Controllers & Filters ---
    ctl_filter_IIR1_t filter_iuvw[3];
    ctl_filter_IIR1_t filter_udc;
    ctl_pid_t idq_ctrl[2];

    // --- Pre-calculated Math Constants (PU Space) ---
    ctrl_gt coef_slip_calc;   //!< Rr / (Lr * 2pi * fs). Used for slip integration.
    ctrl_gt coef_imu_lpf;     //!< Filter coefficient for i_mu (dt / tau_r).
    ctrl_gt coef_dec_lsigma;  //!< Decoupling: sigma * Ls * scale_fac.
    ctrl_gt coef_dec_backemf; //!< Decoupling: (Lm^2 / Lr) * scale_fac.

    ctrl_gt max_vs_mag;        //!< Voltage magnitude limit (PU).
    ctrl_gt max_dcbus_voltage; //!< DC bus reference for compensation.

    // --- Output Vectors ---
    ctl_vector2_t vdq_ctrl_out;
    ctl_vector2_t vdq_decouple;
    ctl_vector3_t vdq_out;
    ctl_vector2_t vdq_out_sat;
    ctl_vector3_t vab0;

    // --- Flags ---
    fast_gt flag_enable_current_ctrl;
    fast_gt flag_enable_decouple;
    fast_gt flag_enable_bus_compensation;

} im_ifoc_ctrl_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_autotune_and_init_im_ifoc(im_ifoc_ctrl_t* mc, const im_ifoc_init_t* init);

GMP_STATIC_INLINE void ctl_attach_im_ifoc_port(im_ifoc_ctrl_t* mc, tri_adc_ift* _iuvw, adc_ift* _udc,
                                               rotation_ift* _pos_if)
{
    mc->adc_iuvw = _iuvw;
    mc->adc_udc = _udc;
    mc->pos_if = _pos_if;
}

GMP_STATIC_INLINE void ctl_enable_im_ifoc(im_ifoc_ctrl_t* mc)
{
    mc->flag_enable_current_ctrl = 1;
}
GMP_STATIC_INLINE void ctl_disable_im_ifoc(im_ifoc_ctrl_t* mc)
{
    mc->flag_enable_current_ctrl = 0;
}

GMP_STATIC_INLINE void ctl_set_im_ifoc_ref(im_ifoc_ctrl_t* mc, ctrl_gt id_ref, ctrl_gt iq_ref)
{
    mc->idq_ref.dat[0] = id_ref;
    mc->idq_ref.dat[1] = iq_ref;
}

/**
 * @brief Executes one step of the IM IFOC loop.
 */
GMP_STATIC_INLINE void ctl_step_im_ifoc(im_ifoc_ctrl_t* mc)
{
    mc->isr_tick++;

    // ·ÀÓùÐÔ¶ÏÑÔ
    gmp_base_assert(mc->adc_iuvw != NULL);
    gmp_base_assert(mc->pos_if != NULL);

    // ========================================================================
    // 1. Rotor Flux Estimation (i_mu) & Slip Angle Integration
    // ========================================================================
    // 1st order LPF to emulate rotor flux build-up: i_mu += (dt/tau_r) * (id - i_mu)
    ctrl_gt id_measured = mc->idq0.dat[phase_d];
    mc->i_mu_pu += ctl_mul(mc->coef_imu_lpf, id_measured - mc->i_mu_pu);

    // Safeguard to prevent division by zero during startup
    ctrl_gt safe_imu = (mc->i_mu_pu > float2ctrl(0.01f)) ? mc->i_mu_pu : float2ctrl(0.01f);

    // Slip calculation: d(Theta_slip) = Coef_slip * (Iq_ref / i_mu)
    // Using Iq_ref provides a cleaner slip feedforward than using noisy Iq_measured
    ctrl_gt slip_inc = ctl_mul(mc->coef_slip_calc, ctl_div(mc->idq_ref.dat[phase_q], safe_imu));

    mc->slip_angle_pu += slip_inc;

    // Wrap slip angle to [0.0, 1.0) PU
    while (mc->slip_angle_pu >= float2ctrl(1.0f))
        mc->slip_angle_pu -= float2ctrl(1.0f);
    while (mc->slip_angle_pu < float2ctrl(0.0f))
        mc->slip_angle_pu += float2ctrl(1.0f);

    // Calculate Synchronous Angle: Theta_e = Theta_rotor_elec + Theta_slip
    mc->sync_angle_pu = mc->pos_if->elec_position + mc->slip_angle_pu;
    if (mc->sync_angle_pu >= float2ctrl(1.0f))
        mc->sync_angle_pu -= float2ctrl(1.0f);

    // Generate Phasor for transforms
    ctl_set_phasor_via_angle(mc->sync_angle_pu, &mc->sync_phasor);

    // ========================================================================
    // 2. ADC Filtering & Clarke/Park Transforms
    // ========================================================================
#if MC_CURRENT_SAMPLE_PHASE_MODE == 3
    mc->iuvw.dat[phase_U] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_U], mc->adc_iuvw->value.dat[phase_A]);
    mc->iuvw.dat[phase_V] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_V], mc->adc_iuvw->value.dat[phase_B]);
    mc->iuvw.dat[phase_W] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_W], mc->adc_iuvw->value.dat[phase_C]);
    ctl_ct_clarke(&mc->iuvw, &mc->iab0);
#elif MC_CURRENT_SAMPLE_PHASE_MODE == 2
    mc->iuvw.dat[phase_U] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_U], mc->adc_iuvw->value.dat[phase_A]);
    mc->iuvw.dat[phase_V] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_V], mc->adc_iuvw->value.dat[phase_B]);
    ctl_ct_clarke_2ph((ctl_vector2_t*)&mc->iuvw, (ctl_vector2_t*)&mc->iab0);
    mc->iab0.dat[phase_0] = 0;
#endif

    if (mc->adc_udc)
        mc->udc = ctl_step_filter_iir1(&mc->filter_udc, mc->adc_udc->value);

    // Park Transform
    ctl_ct_park(&mc->iab0, &mc->sync_phasor, &mc->idq0);

    // ========================================================================
    // 3. Current Loop PI Controllers
    // ========================================================================
    if (mc->flag_enable_current_ctrl)
    {
        ctrl_gt err_d = mc->idq_ref.dat[phase_d] - mc->idq0.dat[phase_d];
        ctrl_gt err_q = mc->idq_ref.dat[phase_q] - mc->idq0.dat[phase_q];

        mc->vdq_ctrl_out.dat[phase_d] = ctl_step_pid_ser(&mc->idq_ctrl[phase_d], err_d);
        mc->vdq_ctrl_out.dat[phase_q] = ctl_step_pid_ser(&mc->idq_ctrl[phase_q], err_q);

        // ====================================================================
        // 4. IM Specific Voltage Feedforward Decoupling
        // ====================================================================
        if (mc->flag_enable_decouple)
        {
            // Estimate electrical speed (Derivative of sync_angle).
            // In a real application, you might use filtered (rotor_speed_elec + slip_speed)
            // Here we assume pos_if provides speed in PU (revolutions/s / spd_base)
            ctrl_gt omega_e_pu = mc->pos_if->velocity + slip_inc; // Simplified proxy for demo

            // Vd_dec = -omega_e * (sigma * Ls) * Iq
            mc->vdq_decouple.dat[phase_d] = ctl_mul(-omega_e_pu, ctl_mul(mc->coef_dec_lsigma, mc->idq0.dat[phase_q]));

            // Vq_dec = omega_e * (sigma * Ls) * Id + omega_e * (Lm^2/Lr) * I_mu
            ctrl_gt vq_cross = ctl_mul(mc->coef_dec_lsigma, mc->idq0.dat[phase_d]);
            ctrl_gt vq_emf = ctl_mul(mc->coef_dec_backemf, mc->i_mu_pu);
            mc->vdq_decouple.dat[phase_q] = ctl_mul(omega_e_pu, vq_cross + vq_emf);

            mc->vdq_out.dat[phase_d] = mc->vdq_ctrl_out.dat[phase_d] + mc->vdq_decouple.dat[phase_d];
            mc->vdq_out.dat[phase_q] = mc->vdq_ctrl_out.dat[phase_q] + mc->vdq_decouple.dat[phase_q];
        }
        else
        {
            mc->vdq_out.dat[phase_d] = mc->vdq_ctrl_out.dat[phase_d];
            mc->vdq_out.dat[phase_q] = mc->vdq_ctrl_out.dat[phase_q];
        }

        // ====================================================================
        // 5. Bus Compensation & Saturation (Square or Circular)
        // ====================================================================
        if (mc->flag_enable_bus_compensation && mc->udc > float2ctrl(0.5f))
        {
            ctrl_gt v_scale = mc->max_dcbus_voltage / mc->udc;
            mc->vdq_out.dat[phase_d] = ctl_mul(mc->vdq_out.dat[phase_d], v_scale);
            mc->vdq_out.dat[phase_q] = ctl_mul(mc->vdq_out.dat[phase_q], v_scale);
        }

        mc->vdq_out_sat.dat[phase_d] = ctl_sat(mc->vdq_out.dat[phase_d], mc->max_vs_mag, -mc->max_vs_mag);

        // Dynamic Q-axis limit based on D-axis usage (Circular limit approximation)
        ctrl_gt max_vq = mc->max_vs_mag - (mc->vdq_out_sat.dat[phase_d] > 0 ? mc->vdq_out_sat.dat[phase_d]
                                                                            : -mc->vdq_out_sat.dat[phase_d]);
        mc->vdq_out_sat.dat[phase_q] = ctl_sat(mc->vdq_out.dat[phase_q], max_vq, -max_vq);

        // Anti-Windup
        ctl_pid_clamping_correction_using_real_output(&mc->idq_ctrl[phase_d],
                                                      mc->vdq_out_sat.dat[phase_d] - mc->vdq_decouple.dat[phase_d]);
        ctl_pid_clamping_correction_using_real_output(&mc->idq_ctrl[phase_q],
                                                      mc->vdq_out_sat.dat[phase_q] - mc->vdq_decouple.dat[phase_q]);
    }
    else
    {
        ctl_vector2_clear(&mc->vdq_out_sat);
    }

    // ========================================================================
    // 6. Inverse Park Transform
    // ========================================================================
    mc->vdq_out_sat.dat[phase_0] = 0;
    ctl_ct_ipark((ctl_vector3_t*)&mc->vdq_out_sat, &mc->sync_phasor, &mc->vab0);
    // Note: User will feed mc->vab0 into SVPWM module outside.
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_IM_IFOC_CORE_H_
