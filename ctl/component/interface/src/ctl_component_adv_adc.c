
#include <gmp_core.h>

#include <ctl/component/interface/adv_nonlinear_adc_channel.h>

void ctl_init_adv_adc_channel(ctl_adv_adc_channel_t* adv_adc, fast_gt resolution, fast_gt iqn,
                              const ctl_lut1d_pair_t* table, uint32_t size)
{
    adv_adc->resolution = resolution;
    adv_adc->iqn = iqn;
    adv_adc->control_port.value = float2ctrl(0.0f);

    // Initialize the embedded LUT with user provided dataset
    ctl_init_paired_lut1d(&adv_adc->lut, table, size);
}
