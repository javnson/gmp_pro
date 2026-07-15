#include <gmp_core.hpp>
#include "user_main.h"
#include <xplt.peripheral.h>
extern "C" {
extern gpio_halt user_led;
fast_gt g_clllc_sim_enable_pending = 0;
static void init_voltage(adc_channel_t* adc)
{
    ctl_init_adc_channel(adc,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
            CLLLC_VOLTAGE_SENSITIVITY_V_PER_V, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CLLLC_VOLTAGE_BIAS_V),
        CTRL_ADC_BITS, 24);
}
static void init_current(adc_channel_t* adc)
{
    ctl_init_adc_channel(adc,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
            CLLLC_CURRENT_SENSITIVITY_V_PER_A, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CLLLC_CURRENT_BIAS_V),
        CTRL_ADC_BITS, 24);
}
void setup_peripheral(void)
{
    user_led = nullptr;
    init_voltage(&adc_v_primary); init_voltage(&adc_v_secondary);
    init_current(&adc_i_primary); init_current(&adc_i_secondary);
    init_current(&adc_i_resonant);
}
void flush_dl_rx_buffer(void) { }
void flush_dl_tx_buffer(void) { }
void send_monitor_data(void) { }
}
