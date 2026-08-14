/**
 * @file imfoc_core.h
 * @brief Standalone induction-motor d/q current-loop core.
 *
 * @details This module is the ACIM peer of `mc_foc_core_t`. It owns only the
 * high-frequency current loop: sampling, Clarke/Park transforms, d/q PI,
 * ACIM feedforward, voltage limiting, anti-windup and inverse Park transform.
 * Flux/speed observers and mechanical controllers are deliberately external.
 *
 * @warning An ACIM has two independent position domains. Shaft rotor position
 * belongs to the mechanical loop. This current core must receive rotor-FLUX
 * position and synchronous electrical speed. Passing raw encoder rotor angle
 * to `field_pos_if` omits slip angle and breaks field orientation.
 */

#ifndef _FILE_IM_IFOC_CORE_H_
#define _FILE_IM_IFOC_CORE_H_

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/intrinsic/complex/dq_pi.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/consultant/acim_consultant.h>
#include <ctl/component/motor_control/consultant/pu_consultant.h>
#include <ctl/component/motor_control/interface/motor_universal_interface.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef MC_CURRENT_SAMPLE_PHASE_MODE
#define MC_CURRENT_SAMPLE_PHASE_MODE (3)
#endif

typedef struct _tag_im_ifoc_init
{
    parameter_gt fs;
    parameter_gt v_bus;
    parameter_gt v_phase_limit;
    parameter_gt v_base;
    parameter_gt i_base;
    parameter_gt freq_base;
    parameter_gt spd_base; //!< Mechanical base in krpm; fallback when freq_base is zero.
    parameter_gt pole_pairs;

    parameter_gt mtr_Rs;
    parameter_gt mtr_Rr;
    parameter_gt mtr_Ls;
    parameter_gt mtr_Lr;
    parameter_gt mtr_Lm;
    parameter_gt current_loop_bw; //!< Zero selects delay-based automatic bandwidth.
} im_ifoc_init_t;

typedef struct _tag_im_ifoc_ctrl
{
    uint32_t isr_tick;

    tri_adc_ift* adc_iuvw;
    adc_ift* adc_udc;
    rotation_ift* field_pos_if;       //!< Rotor-flux angle, never raw shaft angle.
    velocity_ift* synchronous_spd_if; //!< Synchronous electrical speed paired with field angle.

    ctrl_gt magnetizing_current_pu; //!< External i_md = psi_r/Lm, normalized by stator-current base.
    ctl_vector2_t vdq_ff;           //!< Optional user voltage feedforward in d/q PU.
    ctl_vector2_t idq_ref;
    ctl_vector2_t vdq_ref;          //!< Open-loop d/q command when current control is disabled.

    ctrl_gt udc;
    ctl_vector3_t iuvw;
    ctl_vector3_t iab0;
    ctl_vector3_t idq0;
    ctl_vector2_t phasor;

    ctl_vector2_t vdq_ctrl_out;
    ctl_vector2_t vdq_decouple;
    ctl_vector3_t vdq_out;
    ctl_vector2_t vdq_out_bus_compensator;
    ctl_vector2_t vdq_out_sat;
    ctl_vector3_t vab0;

    ctl_filter_IIR1_t filter_iuvw[3];
    ctl_filter_IIR1_t filter_udc;
    ctl_dq_pi_t idq_ctrl;

    ctrl_gt sf_dec_lsigma;  //!< sigma*Ls*Wbase*Ibase/Vbase.
    ctrl_gt sf_dec_backemf; //!< (Lm^2/Lr)*Wbase*Ibase/Vbase.
    ctrl_gt max_vs_mag;
    ctrl_gt max_vs_mag_sq;
    ctrl_gt max_vs_rect;
    ctrl_gt max_dcbus_voltage;

    fast_gt flag_enable_current_ctrl;
    fast_gt flag_enable_decouple;
    fast_gt flag_enable_bus_compensation;
    fast_gt flag_enable_vdq_feedforward;
} im_ifoc_ctrl_t;

void ctl_autotune_and_init_im_ifoc(im_ifoc_ctrl_t* mc, const im_ifoc_init_t* init);

void ctl_autotune_and_init_im_ifoc_consultant(im_ifoc_ctrl_t* mc, const ctl_consultant_im_t* motor,
                                              const ctl_consultant_pu_im_t* pu, parameter_gt fs,
                                              parameter_gt v_bus, parameter_gt v_phase_limit,
                                              parameter_gt current_loop_bw);

void ctl_set_im_ifoc_saturation(im_ifoc_ctrl_t* mc, parameter_gt volt_rect_pu, parameter_gt volt_circle_pu);

GMP_STATIC_INLINE void ctl_attach_im_ifoc_port(im_ifoc_ctrl_t* mc, tri_adc_ift* iuvw, adc_ift* udc,
                                               rotation_ift* field_position,
                                               velocity_ift* synchronous_speed)
{
    gmp_ctl_assert(mc && iuvw && udc && field_position);
    mc->adc_iuvw = iuvw;
    mc->adc_udc = udc;
    mc->field_pos_if = field_position;
    mc->synchronous_spd_if = synchronous_speed;
}

GMP_STATIC_INLINE void ctl_clear_im_ifoc(im_ifoc_ctrl_t* mc)
{
    int i;
    for (i = 0; i < 3; ++i) ctl_clear_filter_iir1(&mc->filter_iuvw[i]);
    ctl_clear_filter_iir1(&mc->filter_udc);
    ctl_clear_dq_pi(&mc->idq_ctrl);
    ctl_vector2_clear(&mc->vdq_ff);
    /* User references intentionally survive a dynamic-state reset, matching
     * foc_core. BUILD_LEVEL installs them before ctl_enable_pwm() clears the
     * controller for a bumpless start. */
    ctl_vector3_clear(&mc->iuvw);
    ctl_vector3_clear(&mc->iab0);
    ctl_vector3_clear(&mc->idq0);
    ctl_vector2_clear(&mc->phasor);
    ctl_vector2_clear(&mc->vdq_ctrl_out);
    ctl_vector2_clear(&mc->vdq_decouple);
    ctl_vector3_clear(&mc->vdq_out);
    ctl_vector2_clear(&mc->vdq_out_bus_compensator);
    ctl_vector2_clear(&mc->vdq_out_sat);
    ctl_vector3_clear(&mc->vab0);
    mc->magnetizing_current_pu = float2ctrl(0.0f);
    mc->udc = float2ctrl(0.0f);
    mc->isr_tick = 0;
}

GMP_STATIC_INLINE void ctl_enable_im_ifoc(im_ifoc_ctrl_t* mc) { mc->flag_enable_current_ctrl = 1; }
GMP_STATIC_INLINE void ctl_disable_im_ifoc(im_ifoc_ctrl_t* mc) { mc->flag_enable_current_ctrl = 0; }
GMP_STATIC_INLINE void ctl_enable_im_ifoc_decouple(im_ifoc_ctrl_t* mc) { mc->flag_enable_decouple = 1; }
GMP_STATIC_INLINE void ctl_disable_im_ifoc_decouple(im_ifoc_ctrl_t* mc) { mc->flag_enable_decouple = 0; }
GMP_STATIC_INLINE void ctl_enable_im_ifoc_bus_compensation(im_ifoc_ctrl_t* mc) { mc->flag_enable_bus_compensation = 1; }
GMP_STATIC_INLINE void ctl_disable_im_ifoc_bus_compensation(im_ifoc_ctrl_t* mc) { mc->flag_enable_bus_compensation = 0; }
GMP_STATIC_INLINE void ctl_enable_im_ifoc_vdq_ff(im_ifoc_ctrl_t* mc) { mc->flag_enable_vdq_feedforward = 1; }
GMP_STATIC_INLINE void ctl_disable_im_ifoc_vdq_ff(im_ifoc_ctrl_t* mc) { mc->flag_enable_vdq_feedforward = 0; }

GMP_STATIC_INLINE void ctl_set_im_ifoc_ref(im_ifoc_ctrl_t* mc, ctrl_gt id_ref, ctrl_gt iq_ref)
{
    mc->idq_ref.dat[phase_d] = id_ref;
    mc->idq_ref.dat[phase_q] = iq_ref;
}

GMP_STATIC_INLINE void ctl_set_im_ifoc_vdq_ref(im_ifoc_ctrl_t* mc, ctrl_gt vd, ctrl_gt vq)
{
    mc->vdq_ref.dat[phase_d] = vd;
    mc->vdq_ref.dat[phase_q] = vq;
}

GMP_STATIC_INLINE void ctl_set_im_ifoc_magnetizing_current(im_ifoc_ctrl_t* mc, ctrl_gt i_md_pu)
{
    mc->magnetizing_current_pu = i_md_pu;
}

GMP_STATIC_INLINE void ctl_step_im_ifoc(im_ifoc_ctrl_t* mc)
{
    ctl_vector2_t total_ff;
    ctl_vector2_t limit_max;
    ctl_vector2_t limit_min;
    ctrl_gt v_scale = float2ctrl(1.0f);

    gmp_ctl_assert(mc->adc_iuvw && mc->adc_udc && mc->field_pos_if);
    ++mc->isr_tick;
    ctl_set_phasor_via_angle(mc->field_pos_if->elec_position, &mc->phasor);

#if MC_CURRENT_SAMPLE_PHASE_MODE == 3
    mc->iuvw.dat[phase_U] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_U], mc->adc_iuvw->value.dat[phase_A]);
    mc->iuvw.dat[phase_V] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_V], mc->adc_iuvw->value.dat[phase_B]);
    mc->iuvw.dat[phase_W] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_W], mc->adc_iuvw->value.dat[phase_C]);
    ctl_ct_clarke(&mc->iuvw, &mc->iab0);
#else
    mc->iuvw.dat[phase_U] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_U], mc->adc_iuvw->value.dat[phase_A]);
    mc->iuvw.dat[phase_V] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_V], mc->adc_iuvw->value.dat[phase_B]);
    ctl_ct_clarke_2ph((ctl_vector2_t*)&mc->iuvw, (ctl_vector2_t*)&mc->iab0);
    mc->iab0.dat[phase_0] = float2ctrl(0.0f);
#endif
    mc->udc = ctl_step_filter_iir1(&mc->filter_udc, mc->adc_udc->value);
    ctl_ct_park(&mc->iab0, &mc->phasor, &mc->idq0);

    ctl_vector2_clear(&mc->vdq_decouple);
    if (mc->flag_enable_decouple)
    {
        gmp_ctl_assert(mc->synchronous_spd_if);
        mc->vdq_decouple.dat[phase_d] =
            -ctl_mul(mc->synchronous_spd_if->speed,
                     ctl_mul(mc->sf_dec_lsigma, mc->idq0.dat[phase_q]));
        mc->vdq_decouple.dat[phase_q] =
            ctl_mul(mc->synchronous_spd_if->speed,
                    ctl_mul(mc->sf_dec_lsigma, mc->idq0.dat[phase_d]) +
                        ctl_mul(mc->sf_dec_backemf, mc->magnetizing_current_pu));
    }
    ctl_vector2_copy(&total_ff, &mc->vdq_decouple);
    if (mc->flag_enable_vdq_feedforward)
        ctl_vector2_add(&total_ff, &total_ff, &mc->vdq_ff);

    if (mc->flag_enable_current_ctrl)
    {
        ctl_step_dq_pi(&mc->idq_ctrl, &mc->idq_ref, (ctl_vector2_t*)&mc->idq0, &total_ff, &mc->vdq_ref);
        ctl_vector2_copy(&mc->vdq_ctrl_out, &mc->idq_ctrl.ctrl_out);
    }
    else
    {
        ctl_vector2_add((ctl_vector2_t*)&mc->vdq_out, &mc->vdq_ref, &total_ff);
        ctl_vector2_sat_circle_sq((ctl_vector2_t*)&mc->vdq_out, (ctl_vector2_t*)&mc->vdq_out,
                                  mc->max_vs_mag_sq);
        ctl_vector2_copy(&mc->vdq_ref, (ctl_vector2_t*)&mc->vdq_out);
    }

    ctl_vector2_copy((ctl_vector2_t*)&mc->vdq_out, &mc->vdq_ref);
    if (mc->flag_enable_bus_compensation)
    {
        v_scale = (mc->udc > float2ctrl(0.5f)) ? ctl_div(mc->max_dcbus_voltage, mc->udc)
                                               : mc->max_dcbus_voltage;
    }
    mc->vdq_out_bus_compensator.dat[phase_d] = ctl_mul(mc->vdq_out.dat[phase_d], v_scale);
    mc->vdq_out_bus_compensator.dat[phase_q] = ctl_mul(mc->vdq_out.dat[phase_q], v_scale);

    ctl_vector2_copy(&mc->vdq_out_sat, &mc->vdq_out_bus_compensator);
    ctl_vector2_sat_circle_sq(&mc->vdq_out_sat, &mc->vdq_out_sat, mc->max_vs_mag_sq);
    limit_max.dat[0] = limit_max.dat[1] = mc->max_vs_rect;
    limit_min.dat[0] = limit_min.dat[1] = -mc->max_vs_rect;
    ctl_vector2_sat_rect(&mc->vdq_out_sat, &mc->vdq_out_sat, &limit_max, &limit_min);
    mc->vdq_out.dat[phase_d] = mc->vdq_out_sat.dat[phase_d];
    mc->vdq_out.dat[phase_q] = mc->vdq_out_sat.dat[phase_q];
    mc->vdq_out.dat[phase_0] = float2ctrl(0.0f);
    ctl_ct_ipark(&mc->vdq_out, &mc->phasor, &mc->vab0);
}

#ifdef __cplusplus
}
#endif

#endif
