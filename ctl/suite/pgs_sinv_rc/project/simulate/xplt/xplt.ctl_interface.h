/** @file xplt.ctl_interface.h @brief Raw ADC/PWM mapping for the SINV SIL target. */
#ifndef _FILE_SINV_SIM_XPLT_CTL_INTERFACE_H_
#define _FILE_SINV_SIM_XPLT_CTL_INTERFACE_H_
#include <xplt.peripheral.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum _tag_sinv_adc_index_items {
    /* Plant ADC bus: IL, IDC, VDC, IC, VC, IG. */
    SINV_ADC_ID_IAC = 0,
    SINV_ADC_ID_VBUS = 2,
    SINV_ADC_ID_VAC = 4,
    SINV_ADC_SENSOR_NUMBER = 6
} sinv_adc_index_items;
extern fast_gt g_sinv_sim_enable_pending;

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    ctl_step_adc_channel(&adc_v_bus, simulink_rx_buffer.adc_result[SINV_ADC_ID_VBUS]);
    ctl_step_adc_channel(&adc_v_grid, simulink_rx_buffer.adc_result[SINV_ADC_ID_VAC]);
    ctl_step_adc_channel(&adc_i_ac, simulink_rx_buffer.adc_result[SINV_ADC_ID_IAC]);
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    simulink_tx_buffer.pwm_cmp[0] = ctl_get_single_phase_modulation_L_phase(&hpwm);
    simulink_tx_buffer.pwm_cmp[1] = ctl_get_single_phase_modulation_N_phase(&hpwm);
    simulink_tx_buffer.monitor[0] = ctrl2float(adc_v_grid.control_port.value) * CTRL_VOLTAGE_BASE;
    simulink_tx_buffer.monitor[1] = ctrl2float(adc_i_ac.control_port.value) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[2] = ctrl2float(adc_v_bus.control_port.value) * CTRL_VOLTAGE_BASE;
    simulink_tx_buffer.monitor[3] = ctrl2float(ref_gen.i_ref_inst) * CTRL_CURRENT_BASE;
    simulink_tx_buffer.monitor[4] = ctrl2float(rc_core.v_out_ref);
    simulink_tx_buffer.monitor[5] = ctrl2float(pll.frequency) * CTRL_GRID_FREQUENCY;
    simulink_tx_buffer.monitor[6] = ctrl2float(pq_meter.active_power_p);
    simulink_tx_buffer.monitor[7] = ctrl2float(pq_meter.reactive_power_q);
    simulink_tx_buffer.monitor[8] = ctrl2float(rc_core.current_error);
    simulink_tx_buffer.monitor[9] = ctrl2float(rc_core.u_qpr);
    simulink_tx_buffer.monitor[10] = ctrl2float(rc_core.u_fdrc);
    simulink_tx_buffer.monitor[11] = (double)rc_core.flag_enable_fdrc;
    simulink_tx_buffer.monitor[12] = (double)cia402_sm.current_state;
    simulink_tx_buffer.monitor[13] = (double)cia402_sm.current_cmd;
    simulink_tx_buffer.monitor[14] = (double)protection.active_errors;
    simulink_tx_buffer.monitor[15] = ctrl2float(protection.node_ctrl_diverge.fault_record_val);
    if (g_sinv_sim_enable_pending) {
        csp_sl_enable_output();
        g_sinv_sim_enable_pending = 0;
    }
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    clear_all_controllers();
    g_sinv_sim_enable_pending = 1;
}

GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    g_sinv_sim_enable_pending = 0;
    csp_sl_disable_output();
}
#ifdef __cplusplus
}
#endif
#endif
