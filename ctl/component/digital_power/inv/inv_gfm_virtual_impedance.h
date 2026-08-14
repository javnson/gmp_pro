/**
 * @file inv_gfm_virtual_impedance.h
 * @brief Synchronous-frame virtual impedance voltage-reference conditioner.
 */

#ifndef _FILE_DP_INV_GFM_VIRTUAL_IMPEDANCE_H_
#define _FILE_DP_INV_GFM_VIRTUAL_IMPEDANCE_H_

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _tag_inv_gfm_virtual_impedance
{
    ctl_vector2_t* idq;
    ctl_vector2_t vdq_base;
    ctl_vector2_t vdq_ref;
    ctrl_gt resistance_pu;
    ctrl_gt reactance_pu;
    ctrl_gt voltage_limit;
    fast_gt flag_enable;
} inv_gfm_virtual_impedance_t;

typedef struct _tag_inv_gfm_virtual_impedance_init
{
    parameter_gt resistance_pu;
    parameter_gt reactance_pu;
    parameter_gt voltage_limit;
} inv_gfm_virtual_impedance_init_t;

void ctl_init_inv_gfm_virtual_impedance(
    inv_gfm_virtual_impedance_t* impedance,
    const inv_gfm_virtual_impedance_init_t* init);

GMP_STATIC_INLINE void ctl_attach_inv_gfm_virtual_impedance(
    inv_gfm_virtual_impedance_t* impedance, ctl_vector2_t* idq)
{
    gmp_ctl_assert(impedance);
    impedance->idq = idq;
}

GMP_STATIC_INLINE void ctl_set_inv_gfm_virtual_impedance_base(
    inv_gfm_virtual_impedance_t* impedance, ctrl_gt vd, ctrl_gt vq)
{
    impedance->vdq_base.dat[phase_d] = vd;
    impedance->vdq_base.dat[phase_q] = vq;
}

GMP_STATIC_INLINE void ctl_clear_inv_gfm_virtual_impedance(
    inv_gfm_virtual_impedance_t* impedance)
{
    ctl_vector2_copy(&impedance->vdq_base, &impedance->vdq_ref);
}

GMP_STATIC_INLINE void ctl_enable_inv_gfm_virtual_impedance(
    inv_gfm_virtual_impedance_t* impedance)
{
    impedance->flag_enable = 1;
}

GMP_STATIC_INLINE void ctl_disable_inv_gfm_virtual_impedance(
    inv_gfm_virtual_impedance_t* impedance)
{
    impedance->flag_enable = 0;
}

GMP_STATIC_INLINE void ctl_step_inv_gfm_virtual_impedance(
    inv_gfm_virtual_impedance_t* impedance)
{
    ctrl_gt magnitude_sq;
    ctrl_gt scale;

    gmp_ctl_assert(impedance);
    gmp_ctl_assert(impedance->idq);

    if (impedance->flag_enable)
    {
        /* V* = Vbase - (R + jX) I in the active synchronous frame. */
        impedance->vdq_ref.dat[phase_d] =
            impedance->vdq_base.dat[phase_d] -
            ctl_mul(impedance->resistance_pu, impedance->idq->dat[phase_d]) +
            ctl_mul(impedance->reactance_pu, impedance->idq->dat[phase_q]);
        impedance->vdq_ref.dat[phase_q] =
            impedance->vdq_base.dat[phase_q] -
            ctl_mul(impedance->resistance_pu, impedance->idq->dat[phase_q]) -
            ctl_mul(impedance->reactance_pu, impedance->idq->dat[phase_d]);
    }
    else
    {
        ctl_vector2_copy(&impedance->vdq_base, &impedance->vdq_ref);
    }

    magnitude_sq = ctl_vector2_mag_sq(&impedance->vdq_ref);
    if ((impedance->voltage_limit > 0) &&
        (magnitude_sq > ctl_mul(impedance->voltage_limit,
                                impedance->voltage_limit)))
    {
        scale = ctl_div(impedance->voltage_limit, ctl_sqrt(magnitude_sq));
        impedance->vdq_ref.dat[phase_d] =
            ctl_mul(impedance->vdq_ref.dat[phase_d], scale);
        impedance->vdq_ref.dat[phase_q] =
            ctl_mul(impedance->vdq_ref.dat[phase_q], scale);
    }
}

#ifdef __cplusplus
}
#endif

#endif
