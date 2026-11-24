#ifndef DPS_FSBB_CCTL_CTL_INTERFACE_H
#define DPS_FSBB_CCTL_CTL_INTERFACE_H

#include <ctl_main.h>
#include <csp.general.h>
#include <xplt.peripheral.h>

GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    fsbb_enable_pending = 0;
    csp_sl_disable_output();
    g_fsbb_output_enabled = 0;
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    clear_all_controllers();
    fsbb_enable_pending = 1;
}

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    ctl_step_adc_channel(&adc_v_in, fsbb_adc_result[FSBB_ADC_VIN]);
    ctl_step_adc_channel(&adc_v_out, fsbb_adc_result[FSBB_ADC_VOUT]);
    ctl_step_adc_channel(&adc_i_L, fsbb_adc_result[FSBB_ADC_IL]);
    ctl_step_adc_channel(&adc_i_load, fsbb_adc_result[FSBB_ADC_IOUT]);
    if (!g_fsbb_output_enabled)
        return;

    if (adc_v_in.control_port.value < real2ctrl(FSBB_INPUT_VOLTAGE_MIN / CTRL_VOLTAGE_BASE))
        g_fsbb_faults |= FSBB_FAULT_VIN_UNDERVOLTAGE;
    if (adc_v_in.control_port.value > real2ctrl(FSBB_INPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE))
        g_fsbb_faults |= FSBB_FAULT_VIN_OVERVOLTAGE;
    if (adc_v_out.control_port.value > real2ctrl(FSBB_OUTPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE))
        g_fsbb_faults |= FSBB_FAULT_VOUT_OVERVOLTAGE;
    if (adc_i_L.control_port.value > real2ctrl(FSBB_PROTECT_IL_MAX / CTRL_CURRENT_BASE))
        g_fsbb_faults |= FSBB_FAULT_IL_POSITIVE_OVERCURRENT;
    if (adc_i_L.control_port.value < real2ctrl(FSBB_PROTECT_IL_MIN / CTRL_CURRENT_BASE))
        g_fsbb_faults |= FSBB_FAULT_IL_NEGATIVE_OVERCURRENT;
    if (adc_i_load.control_port.value > real2ctrl(FSBB_OUTPUT_CURRENT_LIM / CTRL_CURRENT_BASE))
        g_fsbb_faults |= FSBB_FAULT_IOUT_OVERCURRENT;
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    if (g_fsbb_faults != FSBB_FAULT_NONE)
    {
        ctl_fast_disable_output();
        return;
    }

    fsbb_buck_compare = ctl_get_fsbb_buck_cmp(&fsbb_mod);
    fsbb_boost_compare = ctl_get_fsbb_boost_cmp(&fsbb_mod);
    if (fsbb_enable_pending)
    {
        csp_sl_enable_output();
        fsbb_enable_pending = 0;
    }
}

#endif
