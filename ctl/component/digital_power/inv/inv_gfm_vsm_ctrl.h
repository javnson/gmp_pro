/**
 * @file inv_gfm_vsm_ctrl.h
 * @brief Virtual synchronous machine outer loop for grid-forming inverters.
 */

#ifndef _FILE_DP_INV_GFM_VSM_CTRL_H_
#define _FILE_DP_INV_GFM_VSM_CTRL_H_

#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _tag_inv_gfm_vsm_ctrl
{
    ctl_vector2_t* vdq;
    ctl_vector2_t* idq;
    ctl_vector2_t pq_inst;
    ctl_vector2_t pq_filt;
    ctl_vector2_t pq_set;
    ctl_vector2_t vdq_ref;
    ctl_filter_IIR1_t power_lpf[2];

    ctrl_gt frequency_nominal_hz;
    ctrl_gt frequency_ref_hz;
    ctrl_gt voltage_nominal;
    ctrl_gt inertia_s;
    ctrl_gt damping_pu_per_hz;
    ctrl_gt q_droop_v_per_pu;
    ctrl_gt inv_fs;
    ctrl_gt frequency_delta_limit_hz;
    ctrl_gt voltage_min;
    ctrl_gt voltage_max;
    fast_gt flag_enable;
} inv_gfm_vsm_ctrl_t;

typedef struct _tag_inv_gfm_vsm_init
{
    parameter_gt fs;
    parameter_gt frequency_nominal_hz;
    parameter_gt voltage_nominal;
    parameter_gt power_lpf_hz;
    parameter_gt inertia_s;
    parameter_gt damping_pu_per_hz;
    parameter_gt q_droop_v_per_pu;
    parameter_gt frequency_delta_limit_hz;
    parameter_gt voltage_min;
    parameter_gt voltage_max;
} inv_gfm_vsm_init_t;

void ctl_init_inv_gfm_vsm(inv_gfm_vsm_ctrl_t* vsm, const inv_gfm_vsm_init_t* init);

GMP_STATIC_INLINE void ctl_attach_inv_gfm_vsm(inv_gfm_vsm_ctrl_t* vsm,
                                               ctl_vector2_t* vdq, ctl_vector2_t* idq)
{
    gmp_base_assert(vsm);
    vsm->vdq = vdq;
    vsm->idq = idq;
}

GMP_STATIC_INLINE void ctl_set_inv_gfm_vsm_power_reference(inv_gfm_vsm_ctrl_t* vsm,
                                                            ctrl_gt p, ctrl_gt q)
{
    vsm->pq_set.dat[0] = p;
    vsm->pq_set.dat[1] = q;
}

GMP_STATIC_INLINE void ctl_clear_inv_gfm_vsm(inv_gfm_vsm_ctrl_t* vsm)
{
    ctl_vector2_clear(&vsm->pq_inst);
    ctl_vector2_clear(&vsm->pq_filt);
    ctl_clear_filter_iir1(&vsm->power_lpf[0]);
    ctl_clear_filter_iir1(&vsm->power_lpf[1]);
    vsm->frequency_ref_hz = vsm->frequency_nominal_hz;
    vsm->vdq_ref.dat[phase_d] = vsm->voltage_nominal;
    vsm->vdq_ref.dat[phase_q] = 0;
}

GMP_STATIC_INLINE void ctl_enable_inv_gfm_vsm(inv_gfm_vsm_ctrl_t* vsm)
{
    vsm->flag_enable = 1;
}

GMP_STATIC_INLINE void ctl_disable_inv_gfm_vsm(inv_gfm_vsm_ctrl_t* vsm)
{
    vsm->flag_enable = 0;
}

GMP_STATIC_INLINE void ctl_step_inv_gfm_vsm(inv_gfm_vsm_ctrl_t* vsm)
{
    ctrl_gt frequency_error;
    ctrl_gt acceleration_hz_s;
    ctrl_gt q_error;

    gmp_base_assert(vsm);
    gmp_base_assert(vsm->vdq);
    gmp_base_assert(vsm->idq);

    vsm->pq_inst.dat[0] =
        ctl_mul(vsm->vdq->dat[phase_d], vsm->idq->dat[phase_d]) +
        ctl_mul(vsm->vdq->dat[phase_q], vsm->idq->dat[phase_q]);
    vsm->pq_inst.dat[1] =
        ctl_mul(vsm->vdq->dat[phase_q], vsm->idq->dat[phase_d]) -
        ctl_mul(vsm->vdq->dat[phase_d], vsm->idq->dat[phase_q]);
    vsm->pq_filt.dat[0] =
        ctl_step_filter_iir1(&vsm->power_lpf[0], vsm->pq_inst.dat[0]);
    vsm->pq_filt.dat[1] =
        ctl_step_filter_iir1(&vsm->power_lpf[1], vsm->pq_inst.dat[1]);

    if (!vsm->flag_enable)
    {
        vsm->frequency_ref_hz = vsm->frequency_nominal_hz;
        vsm->vdq_ref.dat[phase_d] = vsm->voltage_nominal;
        vsm->vdq_ref.dat[phase_q] = 0;
        return;
    }

    /* Normalized swing equation. inertia_s is the time needed for a
     * one-PU power mismatch to change frequency by one hertz. */
    frequency_error = vsm->frequency_ref_hz - vsm->frequency_nominal_hz;
    acceleration_hz_s =
        ctl_div(vsm->pq_set.dat[0] - vsm->pq_filt.dat[0] -
                    ctl_mul(vsm->damping_pu_per_hz, frequency_error),
                vsm->inertia_s);
    vsm->frequency_ref_hz += ctl_mul(acceleration_hz_s, vsm->inv_fs);
    vsm->frequency_ref_hz =
        ctl_sat(vsm->frequency_ref_hz,
                vsm->frequency_nominal_hz + vsm->frequency_delta_limit_hz,
                vsm->frequency_nominal_hz - vsm->frequency_delta_limit_hz);

    q_error = vsm->pq_filt.dat[1] - vsm->pq_set.dat[1];
    vsm->vdq_ref.dat[phase_d] =
        ctl_sat(vsm->voltage_nominal - ctl_mul(vsm->q_droop_v_per_pu, q_error),
                vsm->voltage_max, vsm->voltage_min);
    vsm->vdq_ref.dat[phase_q] = 0;
}

#ifdef __cplusplus
}
#endif

#endif
