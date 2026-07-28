/**
 * @file gfl_pq_droop_ctrl.h
 * @brief Frequency-active-power and voltage-reactive-power droop reference generator.
 */

#ifndef _FILE_DP_INV_GFL_PQ_DROOP_CTRL_H_
#define _FILE_DP_INV_GFL_PQ_DROOP_CTRL_H_

#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _tag_gfl_pq_droop_ctrl
{
    ctrl_gt* frequency_hz;
    ctl_vector2_t* vdq;
    ctl_vector2_t pq_base;
    ctl_vector2_t pq_ref;
    ctl_filter_IIR1_t frequency_lpf;
    ctl_filter_IIR1_t voltage_lpf;
    ctrl_gt frequency_nominal_hz;
    ctrl_gt voltage_nominal;
    ctrl_gt p_gain_pu_per_hz;
    ctrl_gt q_gain_pu_per_v_pu;
    ctrl_gt p_min;
    ctrl_gt p_max;
    ctrl_gt q_min;
    ctrl_gt q_max;
    ctrl_gt frequency_filt_hz;
    ctrl_gt voltage_filt;
    fast_gt flag_enable;
} gfl_pq_droop_ctrl_t;

typedef struct _tag_gfl_pq_droop_init
{
    parameter_gt fs;
    parameter_gt lpf_hz;
    parameter_gt frequency_nominal_hz;
    parameter_gt voltage_nominal;
    parameter_gt p_gain_pu_per_hz;
    parameter_gt q_gain_pu_per_v_pu;
    parameter_gt p_min;
    parameter_gt p_max;
    parameter_gt q_min;
    parameter_gt q_max;
} gfl_pq_droop_init_t;

void ctl_init_gfl_pq_droop(gfl_pq_droop_ctrl_t* droop,
                           const gfl_pq_droop_init_t* init);

GMP_STATIC_INLINE void ctl_attach_gfl_pq_droop(
    gfl_pq_droop_ctrl_t* droop, ctrl_gt* frequency_hz, ctl_vector2_t* vdq)
{
    gmp_base_assert(droop);
    droop->frequency_hz = frequency_hz;
    droop->vdq = vdq;
}

GMP_STATIC_INLINE void ctl_set_gfl_pq_droop_base(
    gfl_pq_droop_ctrl_t* droop, ctrl_gt p, ctrl_gt q)
{
    droop->pq_base.dat[0] = p;
    droop->pq_base.dat[1] = q;
}

GMP_STATIC_INLINE void ctl_clear_gfl_pq_droop(gfl_pq_droop_ctrl_t* droop)
{
    ctl_clear_filter_iir1(&droop->frequency_lpf);
    ctl_clear_filter_iir1(&droop->voltage_lpf);
    droop->frequency_filt_hz = droop->frequency_nominal_hz;
    droop->voltage_filt = droop->voltage_nominal;
    ctl_vector2_copy(&droop->pq_base, &droop->pq_ref);
}

GMP_STATIC_INLINE void ctl_enable_gfl_pq_droop(gfl_pq_droop_ctrl_t* droop)
{
    droop->flag_enable = 1;
}

GMP_STATIC_INLINE void ctl_disable_gfl_pq_droop(gfl_pq_droop_ctrl_t* droop)
{
    droop->flag_enable = 0;
}

GMP_STATIC_INLINE void ctl_step_gfl_pq_droop(gfl_pq_droop_ctrl_t* droop)
{
    ctrl_gt voltage;

    gmp_base_assert(droop);
    gmp_base_assert(droop->frequency_hz);
    gmp_base_assert(droop->vdq);

    voltage = ctl_vector2_mag(droop->vdq);
    droop->frequency_filt_hz =
        ctl_step_filter_iir1(&droop->frequency_lpf, *droop->frequency_hz);
    droop->voltage_filt =
        ctl_step_filter_iir1(&droop->voltage_lpf, voltage);

    if (!droop->flag_enable)
    {
        ctl_vector2_copy(&droop->pq_base, &droop->pq_ref);
        return;
    }

    droop->pq_ref.dat[0] =
        ctl_sat(droop->pq_base.dat[0] +
                    ctl_mul(droop->p_gain_pu_per_hz,
                            droop->frequency_nominal_hz -
                                droop->frequency_filt_hz),
                droop->p_max, droop->p_min);
    droop->pq_ref.dat[1] =
        ctl_sat(droop->pq_base.dat[1] +
                    ctl_mul(droop->q_gain_pu_per_v_pu,
                            droop->voltage_nominal - droop->voltage_filt),
                droop->q_max, droop->q_min);
}

#ifdef __cplusplus
}
#endif

#endif
