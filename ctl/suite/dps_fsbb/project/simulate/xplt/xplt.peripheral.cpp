/** @file xplt.peripheral.cpp @brief Simulated ADC setup for the FSBB SIL target. */
#include <gmp_core.hpp>
#include "user_main.h"
#include <xplt.peripheral.h>

extern "C" {
extern gpio_halt user_led;
fast_gt g_fsbb_sim_enable_pending = 0;

void setup_peripheral(void)
{
    user_led = nullptr;
    ctl_init_adc_channel(&adc_v_in,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_VIN_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_VIN_BIAS), CTRL_ADC_RESOLUTION, 24);
    ctl_init_adc_channel(&adc_v_out,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_VOUT_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_VOUT_BIAS), CTRL_ADC_RESOLUTION, 24);
    ctl_init_adc_channel(&adc_i_L,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_IL_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_IL_BIAS), CTRL_ADC_RESOLUTION, 24);
    ctl_init_adc_channel(&adc_i_load,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_IOUT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_IOUT_BIAS), CTRL_ADC_RESOLUTION, 24);
}

void flush_dl_rx_buffer(void) { /* ASIO owns the SIL receive path. */ }
void send_monitor_data(void) { /* Populated by ctl_output_callback(). */ }
}
