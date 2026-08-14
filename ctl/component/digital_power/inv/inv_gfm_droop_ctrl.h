/**
 * @file inv_gfm_droop_ctrl.h
 * @brief Replaceable P-f/Q-V droop reference generator for grid-forming inverters.
 */

#ifndef _FILE_DP_INV_GFM_DROOP_CTRL_H_
#define _FILE_DP_INV_GFM_DROOP_CTRL_H_

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Droop outer-loop state.
 * @details This module only converts measured power into frequency and voltage
 * references. It deliberately owns neither the voltage PI nor the angle source,
 * so it can be replaced by VSM, virtual-oscillator, inertia-synchronization, or
 * other grid-forming algorithms without changing the inner loops.
 */
typedef struct _tag_inv_gfm_droop_ctrl
{
    ctl_vector2_t* vdq; //!< Attached terminal/capacitor voltage in the active frame.
    ctl_vector2_t* idq; //!< Attached output current in the active frame.

    ctl_vector2_t pq_inst; //!< Instantaneous {P,Q} in per-unit.
    ctl_vector2_t pq_filt; //!< Filtered {P,Q} in per-unit.
    ctl_vector2_t pq_set;  //!< Requested {P,Q} in per-unit.
    ctl_vector2_t vdq_ref; //!< Generated voltage-loop reference.

    ctl_filter_IIR1_t power_lpf[2];

    ctrl_gt frequency_nominal_hz;
    ctrl_gt voltage_nominal;
    ctrl_gt frequency_ref_hz;
    ctrl_gt droop_p_hz_per_pu;
    ctrl_gt droop_q_v_per_pu;
    ctrl_gt frequency_delta_limit_hz;
    ctrl_gt voltage_min;
    ctrl_gt voltage_max;

    fast_gt flag_enable;
} inv_gfm_droop_ctrl_t;

typedef struct _tag_inv_gfm_droop_init
{
    parameter_gt fs;
    parameter_gt frequency_nominal_hz;
    parameter_gt voltage_nominal;
    parameter_gt power_lpf_hz;
    parameter_gt droop_p_hz_per_pu;
    parameter_gt droop_q_v_per_pu;
    parameter_gt frequency_delta_limit_hz;
    parameter_gt voltage_min;
    parameter_gt voltage_max;
} inv_gfm_droop_init_t;

void ctl_init_inv_gfm_droop(inv_gfm_droop_ctrl_t* droop, const inv_gfm_droop_init_t* init);

GMP_STATIC_INLINE void ctl_clear_inv_gfm_droop(inv_gfm_droop_ctrl_t* droop)
{
    ctl_vector2_clear(&droop->pq_inst);
    ctl_vector2_clear(&droop->pq_filt);
    ctl_clear_filter_iir1(&droop->power_lpf[0]);
    ctl_clear_filter_iir1(&droop->power_lpf[1]);
    droop->frequency_ref_hz = droop->frequency_nominal_hz;
    droop->vdq_ref.dat[phase_d] = droop->voltage_nominal;
    droop->vdq_ref.dat[phase_q] = 0;
}

GMP_STATIC_INLINE void ctl_attach_inv_gfm_droop(inv_gfm_droop_ctrl_t* droop,
                                                 ctl_vector2_t* vdq, ctl_vector2_t* idq)
{
    gmp_ctl_assert(droop);
    droop->vdq = vdq;
    droop->idq = idq;
}

GMP_STATIC_INLINE void ctl_set_inv_gfm_droop_power_reference(inv_gfm_droop_ctrl_t* droop,
                                                             ctrl_gt p, ctrl_gt q)
{
    droop->pq_set.dat[0] = p;
    droop->pq_set.dat[1] = q;
}

GMP_STATIC_INLINE void ctl_enable_inv_gfm_droop(inv_gfm_droop_ctrl_t* droop)
{
    droop->flag_enable = 1;
}

GMP_STATIC_INLINE void ctl_disable_inv_gfm_droop(inv_gfm_droop_ctrl_t* droop)
{
    droop->flag_enable = 0;
}

GMP_STATIC_INLINE void ctl_step_inv_gfm_droop(inv_gfm_droop_ctrl_t* droop)
{
    ctrl_gt p_error;
    ctrl_gt q_error;

    gmp_ctl_assert(droop);
    gmp_ctl_assert(droop->vdq);
    gmp_ctl_assert(droop->idq);

    droop->pq_inst.dat[0] =
        ctl_mul(droop->vdq->dat[phase_d], droop->idq->dat[phase_d]) +
        ctl_mul(droop->vdq->dat[phase_q], droop->idq->dat[phase_q]);
    droop->pq_inst.dat[1] =
        ctl_mul(droop->vdq->dat[phase_q], droop->idq->dat[phase_d]) -
        ctl_mul(droop->vdq->dat[phase_d], droop->idq->dat[phase_q]);

    droop->pq_filt.dat[0] =
        ctl_step_filter_iir1(&droop->power_lpf[0], droop->pq_inst.dat[0]);
    droop->pq_filt.dat[1] =
        ctl_step_filter_iir1(&droop->power_lpf[1], droop->pq_inst.dat[1]);

    if (!droop->flag_enable)
    {
        droop->frequency_ref_hz = droop->frequency_nominal_hz;
        droop->vdq_ref.dat[phase_d] = droop->voltage_nominal;
        droop->vdq_ref.dat[phase_q] = 0;
        return;
    }

    p_error = droop->pq_filt.dat[0] - droop->pq_set.dat[0];
    q_error = droop->pq_filt.dat[1] - droop->pq_set.dat[1];

    droop->frequency_ref_hz =
        ctl_sat(droop->frequency_nominal_hz - ctl_mul(droop->droop_p_hz_per_pu, p_error),
                droop->frequency_nominal_hz + droop->frequency_delta_limit_hz,
                droop->frequency_nominal_hz - droop->frequency_delta_limit_hz);
    droop->vdq_ref.dat[phase_d] =
        ctl_sat(droop->voltage_nominal - ctl_mul(droop->droop_q_v_per_pu, q_error),
                droop->voltage_max, droop->voltage_min);
    droop->vdq_ref.dat[phase_q] = 0;
}

#ifdef __cplusplus
}
#endif

#endif
