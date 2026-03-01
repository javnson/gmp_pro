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

//=================================================================================================
// Board peripheral mapping

typedef enum _tag_adc_index_items
{
    INV_ADC_ID_UDC = 0,

    INV_ADC_ID_UA = 1,
    INV_ADC_ID_UB = 2,
    INV_ADC_ID_UC = 3,

    INV_ADC_ID_IA = 4,
    INV_ADC_ID_IB = 5,
    INV_ADC_ID_IC = 6,
    INV_ADC_SENSOR_NUMBER = 7

} inv_adc_index_items;

typedef enum _tag_digital_index_items
{
    MTR1_ENCODER_OUTPUT = 0,
    MTR1_ENCODER_TURNS = 1,

    DIGITAL_INDEX_NUMBER = 2
} digital_index_items;

    //=================================================================================================
// Controller interface

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    // copy source ADC data
    uuvw_src[phase_U] = simulink_rx_buffer.adc_result[INV_ADC_ID_UA];
    uuvw_src[phase_V] = simulink_rx_buffer.adc_result[INV_ADC_ID_UB];
    uuvw_src[phase_W] = simulink_rx_buffer.adc_result[INV_ADC_ID_UC];

    iuvw_src[phase_U] = simulink_rx_buffer.adc_result[INV_ADC_ID_IA];
    iuvw_src[phase_V] = simulink_rx_buffer.adc_result[INV_ADC_ID_IB];
    iuvw_src[phase_W] = simulink_rx_buffer.adc_result[INV_ADC_ID_IC];

    udc_src = simulink_rx_buffer.adc_result[INV_ADC_ID_UDC];

    // Step auto turn pos encoder
    ctl_step_autoturn_pos_encoder(&pos_enc, simulink_rx_buffer.digital[MTR1_ENCODER_OUTPUT]);

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    //
    // PWM channel
    //
    simulink_tx_buffer.pwm_cmp[0] = spwm.pwm_out[phase_U];
    simulink_tx_buffer.pwm_cmp[1] = spwm.pwm_out[phase_V];
    simulink_tx_buffer.pwm_cmp[2] = spwm.pwm_out[phase_W];

    //
    // monitor
    //

    //// Scope 1
    //simulink_tx_buffer.monitor[0] = inv_ctrl.iabc.dat[phase_A];
    //simulink_tx_buffer.monitor[1] = inv_ctrl.iabc.dat[phase_B];

    //// Scope 2
    //simulink_tx_buffer.monitor[2] = inv_ctrl.idq.dat[phase_d];
    //simulink_tx_buffer.monitor[3] = inv_ctrl.idq.dat[phase_q];

    //// Scope 3
    //simulink_tx_buffer.monitor[4] = inv_ctrl.vab0.dat[phase_alpha];
    //simulink_tx_buffer.monitor[5] = inv_ctrl.vab0.dat[phase_beta];

    //// Scope 4
    //simulink_tx_buffer.monitor[6] = ctl_get_gfl_pll_error(&inv_ctrl);
    //simulink_tx_buffer.monitor[7] = inv_ctrl.angle;

    //// Scope 5
    //simulink_tx_buffer.monitor[8] = inv_ctrl.pll.phasor.dat[phasor_sin];
    //simulink_tx_buffer.monitor[9] = inv_ctrl.pll.phasor.dat[phasor_cos];

    //// Scope 6
    //simulink_tx_buffer.monitor[10] = inv_ctrl.idq.dat[phase_d];
    //simulink_tx_buffer.monitor[11] = inv_ctrl.idq.dat[phase_q];

    //// Scope 7
    //simulink_tx_buffer.monitor[12] = inv_ctrl.vdq.dat[phase_d];
    //simulink_tx_buffer.monitor[13] = inv_ctrl.vdq.dat[phase_q];
}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_fast_enable_output()
{
    //ctl_enable_gfl_inv(&inv_ctrl);

    csp_sl_enable_output();

    flag_system_running = 1;
}

// Disable Output
GMP_STATIC_INLINE void ctl_fast_disable_output()
{
    flag_system_running = 0;
    csp_sl_disable_output();
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
