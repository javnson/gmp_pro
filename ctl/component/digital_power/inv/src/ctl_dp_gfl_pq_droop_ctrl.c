#include <ctl/math_block/gmp_math.h>

#include <ctl/component/digital_power/inv/gfl_pq_droop_ctrl.h>

void ctl_init_gfl_pq_droop(gfl_pq_droop_ctrl_t* droop,
                           const gfl_pq_droop_init_t* init)
{
    gmp_ctl_assert(droop);
    gmp_ctl_assert(init);
    gmp_ctl_assert(init->fs > 0.0f);
    gmp_ctl_assert(init->lpf_hz > 0.0f);
    gmp_ctl_assert(init->frequency_nominal_hz > 0.0f);
    gmp_ctl_assert(init->p_max >= init->p_min);
    gmp_ctl_assert(init->q_max >= init->q_min);

    droop->frequency_hz = NULL;
    droop->vdq = NULL;
    droop->frequency_nominal_hz = float2ctrl(init->frequency_nominal_hz);
    droop->voltage_nominal = float2ctrl(init->voltage_nominal);
    droop->p_gain_pu_per_hz = float2ctrl(init->p_gain_pu_per_hz);
    droop->q_gain_pu_per_v_pu = float2ctrl(init->q_gain_pu_per_v_pu);
    droop->p_min = float2ctrl(init->p_min);
    droop->p_max = float2ctrl(init->p_max);
    droop->q_min = float2ctrl(init->q_min);
    droop->q_max = float2ctrl(init->q_max);
    ctl_init_filter_iir1_lpf(&droop->frequency_lpf, init->fs, init->lpf_hz);
    ctl_init_filter_iir1_lpf(&droop->voltage_lpf, init->fs, init->lpf_hz);
    ctl_vector2_clear(&droop->pq_base);
    ctl_vector2_clear(&droop->pq_ref);
    droop->flag_enable = 0;
    ctl_clear_gfl_pq_droop(droop);
}
