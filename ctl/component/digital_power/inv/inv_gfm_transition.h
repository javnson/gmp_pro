/**
 * @file inv_gfm_transition.h
 * @brief Generic bumpless PLL-to-grid-forming angle-source transition.
 */

#ifndef _FILE_DP_INV_GFM_TRANSITION_H_
#define _FILE_DP_INV_GFM_TRANSITION_H_

#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/math_block/coordinate/coordinate.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum _tag_inv_gfm_transition_mode
{
    INV_GFM_TRANSITION_TRACK_PLL = 0,
    INV_GFM_TRANSITION_RAMP = 1,
    INV_GFM_TRANSITION_FORMING = 2
} inv_gfm_transition_mode_t;

typedef struct _tag_inv_gfm_transition
{
    ctrl_gt* pll_angle;          //!< Attached PLL angle in per-unit revolutions.
    ctl_vector2_t* pll_phasor;   //!< Attached PLL phasor {sin,cos}.
    ctl_vector2_t phasor_gfm;    //!< Free-running grid-forming phasor.
    ctl_vector2_t phasor_out;    //!< Selected/blended phasor for the inner loops.
    ctrl_gt angle_gfm;           //!< Free-running grid-forming angle in [0,1).
    ctrl_gt frequency_ref_hz;    //!< Active grid-forming frequency command.
    ctrl_gt inv_fs;              //!< Inverse execution frequency.
    ctrl_gt blend;               //!< PLL-to-GFM blend in [0,1].
    ctrl_gt blend_step;          //!< Blend increment per execution.
    inv_gfm_transition_mode_t mode;
} inv_gfm_transition_t;

typedef struct _tag_inv_gfm_transition_init
{
    parameter_gt fs;
    parameter_gt transfer_time_s;
    parameter_gt frequency_nominal_hz;
} inv_gfm_transition_init_t;

void ctl_init_inv_gfm_transition(inv_gfm_transition_t* transition,
                                 const inv_gfm_transition_init_t* init);

GMP_STATIC_INLINE void ctl_attach_inv_gfm_transition(inv_gfm_transition_t* transition,
                                                      ctrl_gt* pll_angle,
                                                      ctl_vector2_t* pll_phasor)
{
    gmp_base_assert(transition);
    transition->pll_angle = pll_angle;
    transition->pll_phasor = pll_phasor;
}

GMP_STATIC_INLINE void ctl_track_pll_inv_gfm_transition(inv_gfm_transition_t* transition)
{
    transition->mode = INV_GFM_TRANSITION_TRACK_PLL;
    transition->blend = 0;
}

GMP_STATIC_INLINE void ctl_request_forming_inv_gfm_transition(inv_gfm_transition_t* transition)
{
    gmp_base_assert(transition);
    gmp_base_assert(transition->pll_angle);
    transition->angle_gfm = *transition->pll_angle;
    transition->blend = 0;
    transition->mode = INV_GFM_TRANSITION_RAMP;
}

GMP_STATIC_INLINE fast_gt ctl_is_forming_inv_gfm_transition(
    const inv_gfm_transition_t* transition)
{
    return transition->mode == INV_GFM_TRANSITION_FORMING;
}

GMP_STATIC_INLINE void ctl_blend_inv_gfm_transition(
    const inv_gfm_transition_t* transition, const ctl_vector2_t* tracking_command,
    const ctl_vector2_t* forming_command, ctl_vector2_t* output)
{
    gmp_base_assert(transition);
    gmp_base_assert(tracking_command);
    gmp_base_assert(forming_command);
    gmp_base_assert(output);
    output->dat[0] =
        ctl_mul(float2ctrl(1.0f) - transition->blend, tracking_command->dat[0]) +
        ctl_mul(transition->blend, forming_command->dat[0]);
    output->dat[1] =
        ctl_mul(float2ctrl(1.0f) - transition->blend, tracking_command->dat[1]) +
        ctl_mul(transition->blend, forming_command->dat[1]);
}

GMP_STATIC_INLINE void ctl_step_inv_gfm_transition(inv_gfm_transition_t* transition,
                                                    ctrl_gt frequency_ref_hz)
{
    ctl_vector2_t blended;

    gmp_base_assert(transition);
    gmp_base_assert(transition->pll_angle);
    gmp_base_assert(transition->pll_phasor);
    transition->frequency_ref_hz = frequency_ref_hz;

    if (transition->mode == INV_GFM_TRANSITION_TRACK_PLL)
    {
        transition->angle_gfm = *transition->pll_angle;
        ctl_vector2_copy(&transition->phasor_gfm, transition->pll_phasor);
        ctl_vector2_copy(&transition->phasor_out, transition->pll_phasor);
        transition->blend = 0;
        return;
    }

    transition->angle_gfm += ctl_mul(frequency_ref_hz, transition->inv_fs);
    if (transition->angle_gfm >= float2ctrl(1.0f))
        transition->angle_gfm -= float2ctrl(1.0f);
    else if (transition->angle_gfm < 0)
        transition->angle_gfm += float2ctrl(1.0f);
    ctl_set_phasor_via_angle(transition->angle_gfm, &transition->phasor_gfm);

    if (transition->mode == INV_GFM_TRANSITION_RAMP)
    {
        transition->blend =
            ctl_sat(transition->blend + transition->blend_step, float2ctrl(1.0f), 0);
        blended.dat[0] =
            ctl_mul(float2ctrl(1.0f) - transition->blend, transition->pll_phasor->dat[0]) +
            ctl_mul(transition->blend, transition->phasor_gfm.dat[0]);
        blended.dat[1] =
            ctl_mul(float2ctrl(1.0f) - transition->blend, transition->pll_phasor->dat[1]) +
            ctl_mul(transition->blend, transition->phasor_gfm.dat[1]);
        ctl_vector2_normalize(&transition->phasor_out, &blended);

        if (transition->blend >= float2ctrl(1.0f))
        {
            transition->mode = INV_GFM_TRANSITION_FORMING;
            ctl_vector2_copy(&transition->phasor_out, &transition->phasor_gfm);
        }
    }
    else
    {
        ctl_vector2_copy(&transition->phasor_out, &transition->phasor_gfm);
    }
}

#ifdef __cplusplus
}
#endif

#endif
