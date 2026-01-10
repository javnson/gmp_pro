//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of controller objects in this file.
//
// User should implement the Main ISR of the controller tasks.
//
// User should ensure that all the controller codes here is platform-independent.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include <ctl/component/motor_control/basic/std_sil_motor_interface.h>

#include <xplt.peripheral.h>

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////
// device related functions
// Controller interface
//
//

typedef enum _tag_sinv_adc_index_items
{
    INV_ADC_ID_IDC = 0,
    INV_ADC_ID_VDC = 1,
    INV_ADC_ID_UAB = 2,
    INV_ADC_ID_UBC = 3,
    INV_ADC_ID_IA = 4,
    INV_ADC_ID_IB = 5,
    INV_ADC_ID_IC = 6,
    INV_ADC_SENSOR_NUMBER = 7

} inv_adc_index_items;

//extern ptr_adc_channel_t inv_adc[INV_ADC_SENSOR_NUMBER];

//extern pwm_channel_t inv_pwm_out[3];

extern inv_ctrl_t inv_ctrl;

// Input Callback
GMP_STATIC_INLINE
void ctl_input_callback(void)
{
    // copy source ADC data
    vabc_src[phase_A] = simulink_rx_buffer.adc_result[INV_ADC_ID_UAB];
    vabc_src[phase_B] = simulink_rx_buffer.adc_result[INV_ADC_ID_UBC];
    vabc_src[phase_C] = 0;
                        
    iabc_src[phase_A] = simulink_rx_buffer.adc_result[INV_ADC_ID_IA];
    iabc_src[phase_B] = simulink_rx_buffer.adc_result[INV_ADC_ID_IB];
    iabc_src[phase_C] = simulink_rx_buffer.adc_result[INV_ADC_ID_IC];
                        
    uuvw_src[phase_U] = 0;
    uuvw_src[phase_V] = 0;
    uuvw_src[phase_W] = 0;
                        
    iuvw_src[phase_U] = 0;
    iuvw_src[phase_V] = 0;
    iuvw_src[phase_W] = 0;

    udc_src = simulink_rx_buffer.adc_result[INV_ADC_ID_IDC];
    idc_src = simulink_rx_buffer.adc_result[INV_ADC_ID_VDC];

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&vabc);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

// Output Callback
GMP_STATIC_INLINE
void ctl_output_callback(void)
{
    // invoke PWM p.u. routine
    ctl_calc_pwm_tri_channel(&pwm_out);

    //
    // PWM channel
    //
    simulink_tx_buffer.pwm_cmp[0] = pwm_out.value[phase_U];
    simulink_tx_buffer.pwm_cmp[1] = pwm_out.value[phase_V];
    simulink_tx_buffer.pwm_cmp[2] = pwm_out.value[phase_W];

    //
    // monitor
    //

#if BUILD_LEVEL == 1

    // Scope 1
    //simulink_tx_buffer.monitor[0] = inv_adc[INV_ADC_ID_VDC].control_port.value;
    //simulink_tx_buffer.monitor[1] = inv_adc[INV_ADC_ID_IDC].control_port.value;

    // Scope 2
    //simulink_tx_buffer.monitor[2] = inv_adc[INV_ADC_ID_UAB].control_port.value;
    //simulink_tx_buffer.monitor[3] = inv_adc[INV_ADC_ID_UBC].control_port.value;

    // Scope 3
    //simulink_tx_buffer.monitor[4] = inv_adc[INV_ADC_ID_IA].control_port.value;
    //simulink_tx_buffer.monitor[5] = inv_adc[INV_ADC_ID_IB].control_port.value;

    // Scope 4
    simulink_tx_buffer.monitor[6] = inv_ctrl.pwm_out->value.dat[phase_A];
    simulink_tx_buffer.monitor[7] = inv_ctrl.pwm_out->value.dat[phase_B];

    // Scope 5
    simulink_tx_buffer.monitor[8] = inv_ctrl.pll.phasor.dat[phasor_sin];
    simulink_tx_buffer.monitor[9] = inv_ctrl.pll.phasor.dat[phasor_cos];

    // Scope 6
    simulink_tx_buffer.monitor[10] = inv_ctrl.idq.dat[phase_d];
    simulink_tx_buffer.monitor[11] = inv_ctrl.idq.dat[phase_q];

    // Scope 7
    simulink_tx_buffer.monitor[12] = inv_ctrl.vdq.dat[phase_d];
    simulink_tx_buffer.monitor[13] = inv_ctrl.vdq.dat[phase_q];

#elif BUILD_LEVEL == 2 || BUILD_LEVEL == 3 || BUILD_LEVEL == 4 || BUILD_LEVEL == 5

    // Scope 1 d current control
    simulink_tx_buffer.monitor[0] = inv_ctrl.idq_set.dat[phase_d];
    simulink_tx_buffer.monitor[1] = inv_ctrl.idq.dat[phase_d];

    // Scope 2 q current control
    simulink_tx_buffer.monitor[2] = inv_ctrl.idq_set.dat[phase_q];
    simulink_tx_buffer.monitor[3] = inv_ctrl.idq.dat[phase_q];

    // Scope 3 output voltage dq
    simulink_tx_buffer.monitor[4] = inv_ctrl.vdq.dat[phase_d];
    simulink_tx_buffer.monitor[5] = inv_ctrl.vdq.dat[phase_q];

    // Scope 4 output modulation
    simulink_tx_buffer.monitor[6] = inv_ctrl.vab_out.dat[phase_d];
    simulink_tx_buffer.monitor[7] = inv_ctrl.vab_out.dat[phase_q];

    // Scope 5 PLL
    simulink_tx_buffer.monitor[8] = inv_ctrl.pll.phasor.dat[phasor_sin];
    simulink_tx_buffer.monitor[9] = inv_ctrl.pll.phasor.dat[phasor_cos];

    // Scope 6
    simulink_tx_buffer.monitor[10] = inv_ctrl.iab0.dat[phase_alpha];
    simulink_tx_buffer.monitor[11] = inv_ctrl.iab0.dat[phase_beta];

    // Scope 7
    simulink_tx_buffer.monitor[12] = inv_ctrl.iabc.dat[phase_A];
    simulink_tx_buffer.monitor[13] = inv_ctrl.iabc.dat[phase_B];

#elif BUILD_LEVEL == 6 || BUILD_LEVEL == 7

    // Scope 1 d current control
    simulink_tx_buffer.monitor[0] = inv_ctrl.idq_set.dat[phase_d];
    simulink_tx_buffer.monitor[1] = inv_ctrl.idq.dat[phase_d];

    // Scope 2 q current control
    simulink_tx_buffer.monitor[2] = inv_ctrl.idq_set.dat[phase_q];
    simulink_tx_buffer.monitor[3] = inv_ctrl.idq.dat[phase_q];

    // Scope 3 output voltage dq
    simulink_tx_buffer.monitor[4] = inv_ctrl.vdq.dat[phase_d];
    simulink_tx_buffer.monitor[5] = inv_ctrl.vdq.dat[phase_q];

    // Scope 4 voltage loop
    simulink_tx_buffer.monitor[6] = inv_ctrl.vdq_set.dat[phase_d];
    simulink_tx_buffer.monitor[7] = inv_ctrl.vdq.dat[phase_d];

    // Scope 5 PLL
    simulink_tx_buffer.monitor[8] = inv_ctrl.pll.phasor.dat[phasor_sin];
    simulink_tx_buffer.monitor[9] = inv_ctrl.pll.phasor.dat[phasor_cos];

    // Scope 6
    simulink_tx_buffer.monitor[10] = inv_ctrl.iab0.dat[phase_alpha];
    simulink_tx_buffer.monitor[11] = inv_ctrl.iab0.dat[phase_beta];

    // Scope 7
    simulink_tx_buffer.monitor[12] = inv_ctrl.iabc.dat[phase_A];
    simulink_tx_buffer.monitor[13] = inv_ctrl.iabc.dat[phase_B];

#endif // BUILD LEVEL
}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE
void ctl_enable_output()
{
    ctl_enable_three_phase_inverter(&inv_ctrl);

    csp_sl_enable_output();

    flag_system_running = 1;
}

// Disable Output
GMP_STATIC_INLINE
void ctl_disable_output()
{
    flag_system_running = 0;
    csp_sl_disable_output();
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
