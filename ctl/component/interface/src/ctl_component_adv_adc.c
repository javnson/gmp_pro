
#include <ctl/math_block/gmp_math.h>

#include <ctl/component/interface/adv_adc_channel.h>

void ctl_init_adv_adc_channel(ctl_adv_adc_channel_t* adv_adc, fast_gt resolution, fast_gt iqn,
                              const ctl_lut1d_pair_t* table, uint32_t size)
{
    adv_adc->resolution = resolution;
    adv_adc->iqn = iqn;
    adv_adc->control_port.value = CTL_CTRL_CONST_ZERO;

    // Initialize the embedded LUT with user provided dataset
    ctl_init_paired_lut1d(&adv_adc->lut, table, size);
}
