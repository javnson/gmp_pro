#include <gmp_core.h>
#include <xplt.peripheral.h>

namespace
{
void initialize_adc(adc_channel_t& channel, parameter_gt sensitivity,
                    parameter_gt bias, parameter_gt base)
{
    ctl_init_adc_channel(&channel,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, sensitivity, base),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, bias),
        CTRL_ADC_RESOLUTION, 24);
}
} // namespace

extern "C"
{
adc_gt fsbb_adc_result[FSBB_ADC_COUNT];
pwm_gt fsbb_buck_compare;
pwm_gt fsbb_boost_compare;
fast_gt fsbb_enable_pending;

void setup_peripheral(void)
{
    initialize_adc(adc_v_in, CTRL_FSBB_VIN_SENSITIVITY, CTRL_FSBB_VIN_BIAS, CTRL_VOLTAGE_BASE);
    initialize_adc(adc_v_out, CTRL_FSBB_VOUT_SENSITIVITY, CTRL_FSBB_VOUT_BIAS, CTRL_VOLTAGE_BASE);
    initialize_adc(adc_i_L, CTRL_FSBB_IL_SENSITIVITY, CTRL_FSBB_IL_BIAS, CTRL_CURRENT_BASE);
    initialize_adc(adc_i_load, CTRL_FSBB_IOUT_SENSITIVITY, CTRL_FSBB_IOUT_BIAS, CTRL_CURRENT_BASE);
}

void fsbb_adc_interrupt(void)
{
    csp_cctl_notify_controller_interrupt();
    gmp_base_ctl_step();
}

void send_monitor_data(void) {}
void flush_dl_rx_buffer(void) {}

ec_gt gmp_hal_gpio_write(gpio_halt, fast_gt) { return GMP_EC_OK; }
} // extern "C"
