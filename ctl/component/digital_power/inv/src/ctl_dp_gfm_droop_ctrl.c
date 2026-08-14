#include <ctl/math_block/gmp_math.h>

#include <ctl/component/digital_power/inv/inv_gfm_droop_ctrl.h>

void ctl_init_inv_gfm_droop(inv_gfm_droop_ctrl_t* droop, const inv_gfm_droop_init_t* init)
{
    gmp_ctl_assert(droop);
    gmp_ctl_assert(init);
    gmp_ctl_assert(init->fs > 0.0f);
    gmp_ctl_assert(init->frequency_nominal_hz > 0.0f);
    gmp_ctl_assert(init->power_lpf_hz > 0.0f);
    gmp_ctl_assert(init->frequency_delta_limit_hz >= 0.0f);
    gmp_ctl_assert(init->voltage_max >= init->voltage_min);

    droop->vdq = NULL;
    droop->idq = NULL;
    droop->frequency_nominal_hz = float2ctrl(init->frequency_nominal_hz);
    droop->voltage_nominal = float2ctrl(init->voltage_nominal);
    droop->droop_p_hz_per_pu = float2ctrl(init->droop_p_hz_per_pu);
    droop->droop_q_v_per_pu = float2ctrl(init->droop_q_v_per_pu);
    droop->frequency_delta_limit_hz = float2ctrl(init->frequency_delta_limit_hz);
    droop->voltage_min = float2ctrl(init->voltage_min);
    droop->voltage_max = float2ctrl(init->voltage_max);
    ctl_init_filter_iir1_lpf(&droop->power_lpf[0], init->fs, init->power_lpf_hz);
    ctl_init_filter_iir1_lpf(&droop->power_lpf[1], init->fs, init->power_lpf_hz);
    ctl_vector2_clear(&droop->pq_set);
    droop->flag_enable = 0;
    ctl_clear_inv_gfm_droop(droop);
}
