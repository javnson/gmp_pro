/** @file xplt.peripheral.cpp @brief Simulated SINV ADC and host peripheral setup. */
#include <gmp_core.hpp>
#include "user_main.h"
#include <xplt.peripheral.h>

extern "C" {
extern gpio_halt user_led;
adc_channel_t adc_v_grid;
adc_channel_t adc_i_ac;
adc_channel_t adc_v_bus;
fast_gt g_sinv_sim_enable_pending = 0;

void setup_peripheral(void)
{
    user_led = nullptr;
    ctl_init_adc_channel(&adc_v_grid,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_AC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_AC_VOLTAGE_BIAS), CTRL_ADC_RESOLUTION, 24);
    ctl_init_adc_channel(&adc_i_ac,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_AC_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_AC_CURRENT_BIAS), CTRL_ADC_RESOLUTION, 24);
    ctl_init_adc_channel(&adc_v_bus,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_BIAS), CTRL_ADC_RESOLUTION, 24);
}

void flush_dl_rx_buffer(void) { }
void send_monitor_data(void) { }
}
