/** @file xplt.ctl_interface.h @brief CLLLC SIL ADC/advanced-PWM binding. */
#ifndef DPS_CLLLC_SIM_CTL_INTERFACE_H
#define DPS_CLLLC_SIM_CTL_INTERFACE_H
#include <xplt.peripheral.h>

enum { CLLLC_ADC_VPRI, CLLLC_ADC_IPRI, CLLLC_ADC_VSEC,
       CLLLC_ADC_ISEC, CLLLC_ADC_IRESONANT };
extern fast_gt g_clllc_sim_enable_pending;

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    ctl_step_adc_channel(&adc_v_primary, simulink_rx_buffer.adc_result[CLLLC_ADC_VPRI]);
    ctl_step_adc_channel(&adc_i_primary, simulink_rx_buffer.adc_result[CLLLC_ADC_IPRI]);
    ctl_step_adc_channel(&adc_v_secondary, simulink_rx_buffer.adc_result[CLLLC_ADC_VSEC]);
    ctl_step_adc_channel(&adc_i_secondary, simulink_rx_buffer.adc_result[CLLLC_ADC_ISEC]);
    ctl_step_adc_channel(&adc_i_resonant, simulink_rx_buffer.adc_result[CLLLC_ADC_IRESONANT]);
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    int i;
    for (i = 0; i < 4; ++i)
    {
        simulink_tx_buffer.pwm_cmp[i] = clllc_mod.leg[i].duty;
        simulink_tx_buffer.pwm_cmp[i + 4] = clllc_mod.leg[i].phase;
    }
    simulink_tx_buffer.monitor[0] = ctrl2float(clllc_mod.leg[0].raw.period);
    simulink_tx_buffer.monitor[1] = ctrl2float(clllc_mod.leg[0].raw.deadband);
    simulink_tx_buffer.monitor[2] = ctrl2float(adc_v_primary.control_port.value) * CTRL_VOLTAGE_BASE;
    simulink_tx_buffer.monitor[3] = ctrl2float(adc_i_primary.control_port.value) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[4] = ctrl2float(adc_v_secondary.control_port.value) * CTRL_VOLTAGE_BASE;
    simulink_tx_buffer.monitor[5] = ctrl2float(adc_i_resonant.control_port.value) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[6] = ctrl2float(g_modulation_command);
    if (g_clllc_sim_enable_pending)
    {
        csp_sl_enable_output();
        g_clllc_sim_enable_pending = 0;
    }
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    clear_all_controllers();
    g_clllc_sim_enable_pending = 1;
}
GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    g_clllc_sim_enable_pending = 0;
    csp_sl_disable_output();
}
#endif
