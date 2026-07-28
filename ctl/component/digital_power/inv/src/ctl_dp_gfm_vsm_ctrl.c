#include <gmp_core.h>

#include <ctl/component/digital_power/inv/inv_gfm_vsm_ctrl.h>

void ctl_init_inv_gfm_vsm(inv_gfm_vsm_ctrl_t* vsm, const inv_gfm_vsm_init_t* init)
{
    gmp_base_assert(vsm);
    gmp_base_assert(init);
    gmp_base_assert(init->fs > 0.0f);
    gmp_base_assert(init->frequency_nominal_hz > 0.0f);
    gmp_base_assert(init->power_lpf_hz > 0.0f);
    gmp_base_assert(init->inertia_s > 0.0f);
    gmp_base_assert(init->damping_pu_per_hz >= 0.0f);
    gmp_base_assert(init->frequency_delta_limit_hz >= 0.0f);
    gmp_base_assert(init->voltage_max >= init->voltage_min);

    vsm->vdq = NULL;
    vsm->idq = NULL;
    vsm->frequency_nominal_hz = float2ctrl(init->frequency_nominal_hz);
    vsm->frequency_ref_hz = vsm->frequency_nominal_hz;
    vsm->voltage_nominal = float2ctrl(init->voltage_nominal);
    vsm->inertia_s = float2ctrl(init->inertia_s);
    vsm->damping_pu_per_hz = float2ctrl(init->damping_pu_per_hz);
    vsm->q_droop_v_per_pu = float2ctrl(init->q_droop_v_per_pu);
    vsm->inv_fs = float2ctrl(1.0f / init->fs);
    vsm->frequency_delta_limit_hz = float2ctrl(init->frequency_delta_limit_hz);
    vsm->voltage_min = float2ctrl(init->voltage_min);
    vsm->voltage_max = float2ctrl(init->voltage_max);
    ctl_init_filter_iir1_lpf(&vsm->power_lpf[0], init->fs, init->power_lpf_hz);
    ctl_init_filter_iir1_lpf(&vsm->power_lpf[1], init->fs, init->power_lpf_hz);
    ctl_vector2_clear(&vsm->pq_set);
    vsm->flag_enable = 0;
    ctl_clear_inv_gfm_vsm(vsm);
}
