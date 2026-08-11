/** @file xplt.peripheral.cpp @brief Simulated ADC channel setup. */

#include <gmp_core.hpp>
#include "user_main.h"
#include <xplt.peripheral.h>

#ifdef __cplusplus
extern "C"
{
#endif

tri_ptr_adc_channel_t uuvw;
adc_gt uuvw_src[3];
tri_ptr_adc_channel_t iuvw;
adc_gt iuvw_src[3];
ptr_adc_channel_t udc;
adc_gt udc_src;
ptr_adc_channel_t idc;
adc_gt idc_src;

void setup_peripheral(void)
{
    ctl_init_tri_ptr_adc_channel(
        &uuvw, uuvw_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_BIAS), 12, 24);

    ctl_init_tri_ptr_adc_channel(
        &iuvw, iuvw_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_BIAS), 12, 24);

    ctl_init_ptr_adc_channel(
        &udc, &udc_src, ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_BIAS), 12, 24);

    ctl_init_ptr_adc_channel(
        &idc, &idc_src, ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_CURRENT_BIAS), 12, 24);
}

void send_monitor_data(void)
{
}

void flush_dl_rx_buffer(void)
{
}

void flush_dl_tx_buffer(void)
{
}

#ifdef __cplusplus
}
#endif

