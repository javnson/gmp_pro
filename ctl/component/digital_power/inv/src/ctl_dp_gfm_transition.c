#include <gmp_core.h>

#include <ctl/component/digital_power/inv/inv_gfm_transition.h>

void ctl_init_inv_gfm_transition(inv_gfm_transition_t* transition,
                                 const inv_gfm_transition_init_t* init)
{
    gmp_base_assert(transition);
    gmp_base_assert(init);
    gmp_base_assert(init->fs > 0.0f);
    gmp_base_assert(init->transfer_time_s > 0.0f);

    transition->pll_angle = NULL;
    transition->pll_phasor = NULL;
    transition->angle_gfm = 0;
    transition->frequency_ref_hz = float2ctrl(init->frequency_nominal_hz);
    transition->inv_fs = float2ctrl(1.0f / init->fs);
    transition->blend = 0;
    transition->blend_step = float2ctrl(1.0f / (init->fs * init->transfer_time_s));
    transition->mode = INV_GFM_TRANSITION_TRACK_PLL;
    ctl_set_phasor_via_angle(0, &transition->phasor_gfm);
    ctl_vector2_copy(&transition->phasor_out, &transition->phasor_gfm);
}
