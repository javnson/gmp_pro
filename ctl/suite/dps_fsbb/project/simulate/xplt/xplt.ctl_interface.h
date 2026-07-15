/** @file xplt.ctl_interface.h @brief Raw ADC/PWM mapping for FSBB SIL. */
#ifndef _FILE_FSBB_SIM_XPLT_CTL_INTERFACE_H_
#define _FILE_FSBB_SIM_XPLT_CTL_INTERFACE_H_
#include <xplt.peripheral.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef enum _tag_dcdc_adc_index_items {
    DCDC_ADC_ID_VIN = 0,
    DCDC_ADC_ID_VOUT = 1,
    DCDC_ADC_ID_IL = 2,
    DCDC_ADC_ID_IOUT = 3,
    DCDC_ADC_ID_IIN = 4,
    DCDC_ADC_SENSOR_NUMBER = 5
} dcdc_adc_index_items;

extern fast_gt g_fsbb_sim_enable_pending;

GMP_STATIC_INLINE uint16_t ctl_fsbb_sim_active_faults(void)
{
    uint16_t faults = FSBB_FAULT_NONE;
    if (adc_v_in.control_port.value < float2ctrl(FSBB_INPUT_VOLTAGE_MIN / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VIN_UNDERVOLTAGE;
    if (adc_v_in.control_port.value > float2ctrl(FSBB_INPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VIN_OVERVOLTAGE;
    if (adc_v_out.control_port.value > float2ctrl(FSBB_OUTPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VOUT_OVERVOLTAGE;
    if (adc_i_L.control_port.value > float2ctrl(FSBB_PROTECT_IL_MAX / CTRL_CURRENT_BASE))
        faults |= FSBB_FAULT_IL_POSITIVE_OVERCURRENT;
    if (adc_i_L.control_port.value < float2ctrl(FSBB_PROTECT_IL_MIN / CTRL_CURRENT_BASE))
        faults |= FSBB_FAULT_IL_NEGATIVE_OVERCURRENT;
    if (adc_i_load.control_port.value > float2ctrl(FSBB_OUTPUT_CURRENT_LIM / CTRL_CURRENT_BASE))
        faults |= FSBB_FAULT_IOUT_OVERCURRENT;
    return faults;
}

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    /* Sensor blocks already output quantized ADC codes. */
    ctl_step_adc_channel(&adc_v_in, simulink_rx_buffer.adc_result[DCDC_ADC_ID_VIN]);
    ctl_step_adc_channel(&adc_v_out, simulink_rx_buffer.adc_result[DCDC_ADC_ID_VOUT]);
    ctl_step_adc_channel(&adc_i_L, simulink_rx_buffer.adc_result[DCDC_ADC_ID_IL]);
    ctl_step_adc_channel(&adc_i_load, simulink_rx_buffer.adc_result[DCDC_ADC_ID_IOUT]);
    if (g_fsbb_output_enabled)
        g_fsbb_faults |= ctl_fsbb_sim_active_faults();
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    simulink_tx_buffer.pwm_cmp[0] = ctl_get_fsbb_buck_cmp(&fsbb_mod);
    /* CH2 is the Boost low-side Q4 duty, while the Simulink phase input
       directly defines the upper Q3 gate duty. Send its complement. */
    simulink_tx_buffer.pwm_cmp[1] = CTRL_PWM_CMP_MAX - ctl_get_fsbb_boost_cmp(&fsbb_mod);
    simulink_tx_buffer.monitor[0] = ctrl2float(adc_v_in.control_port.value) * CTRL_VOLTAGE_BASE;
    simulink_tx_buffer.monitor[1] = ctrl2float(adc_v_out.control_port.value) * CTRL_VOLTAGE_BASE;
    simulink_tx_buffer.monitor[2] = ctrl2float(adc_i_L.control_port.value) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[3] = ctrl2float(adc_i_load.control_port.value) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[4] = ctrl2float(dcdc_core.v_out_formal) * CTRL_VOLTAGE_BASE;
    simulink_tx_buffer.monitor[5] = ctrl2float(v_req) * CTRL_VOLTAGE_BASE;
    simulink_tx_buffer.monitor[6] = (double)cia402_sm.current_state;
    simulink_tx_buffer.monitor[7] = (double)cia402_sm.current_cmd;
    if (g_fsbb_sim_enable_pending)
    {
        csp_sl_enable_output();
        g_fsbb_sim_enable_pending = 0;
    }
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    clear_all_controllers();
    /* Commit Enable in ctl_output_callback(), after the freshly calculated
       PWM compare values have been copied into the same UDP frame. */
    g_fsbb_sim_enable_pending = 1;
    g_fsbb_output_enabled = 1;
}
GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    g_fsbb_sim_enable_pending = 0;
    csp_sl_disable_output();
    g_fsbb_output_enabled = 0;
}
#ifdef __cplusplus
}
#endif
#endif
