/**
 * @file ctl_dp_sms_pq.c
 * @brief Implements single-phase instantaneous active/reactive power measurement.
 */

#include <ctl/math_block/gmp_math.h>

#include <ctl/component/digital_power/sinv/sms_pq.h>

void ctl_init_sms_pq(ctl_sms_pq_t* pq, parameter_gt grid_freq, parameter_gt fs, parameter_gt lpf_fc)
{
    /* A Butterworth damping ratio gives the current SOGI a balanced transient response. */
    ctl_init_discrete_sogi(&pq->sogi_i, CTL_PARAM_CONST_1_OVER_SQRT2, grid_freq, fs);

    /* The SOGI removes the double-line-frequency ripple, so these LPFs may use a relatively high cutoff. */
    ctl_init_biquad_lpf(&pq->lpf_p, fs, lpf_fc, CTL_PARAM_CONST_1_OVER_SQRT2);
    ctl_init_biquad_lpf(&pq->lpf_q, fs, lpf_fc, CTL_PARAM_CONST_1_OVER_SQRT2);

    ctl_clear_sms_pq(pq);
}

