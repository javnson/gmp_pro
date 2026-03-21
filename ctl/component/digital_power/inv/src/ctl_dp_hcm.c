
#include <gmp_core.h>

#include <ctl/component/digital_power/inv/inv_hcm.h>


//////////////////////////////////////////////////////////////////////////
// Three Phase Converter negative sequence controller
//////////////////////////////////////////////////////////////////////////

void ctl_init_dq_hcm(inv_dq_hcm_t* hcm, const inv_dq_hcm_init_t* init)
{
    gmp_base_assert(hcm);
    gmp_base_assert(init);

    // update controller parameters
    ctl_update_dq_hcm_freq(hcm, init);

    // 4. Reset & Defaults
    ctl_clear_dq_hcm(hcm);
    hcm->flag_enable_6th = 0;
    hcm->flag_enable_12th = 0;
}

void ctl_update_dq_hcm_freq(inv_dq_hcm_t* hcm, const inv_dq_hcm_init_t* init)
{
    // 1. Calculate Resonant Frequencies
    parameter_gt f_6th = init->freq_base * 6.0f;
    parameter_gt f_12th = init->freq_base * 12.0f;

    // 2. Init QR Controllers (Using Pre-warping!)
    // D-Axis
    ctl_init_qr_controller_prewarped(&hcm->qr_d_6th, init->kr_6th, f_6th, init->bw_6th, init->fs);
    ctl_init_qr_controller_prewarped(&hcm->qr_d_12th, init->kr_12th, f_12th, init->bw_12th, init->fs);

    // Q-Axis (Same parameters)
    ctl_init_qr_controller_prewarped(&hcm->qr_q_6th, init->kr_6th, f_6th, init->bw_6th, init->fs);
    ctl_init_qr_controller_prewarped(&hcm->qr_q_12th, init->kr_12th, f_12th, init->bw_12th, init->fs);

    // 3. Init Saturation
    ctl_init_saturation(&hcm->sat_d, -init->out_limit, init->out_limit);
    ctl_init_saturation(&hcm->sat_q, -init->out_limit, init->out_limit);
}
