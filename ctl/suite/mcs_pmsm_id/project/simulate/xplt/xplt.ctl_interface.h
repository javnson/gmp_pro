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

#include <ctl/component/motor_control/interface/std_sil_motor_interface.h>

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

    // Offline-identification result and progress contract. Keep the first six
    // channels stable because run_pmsm_id_sil.m consumes them automatically.
    simulink_tx_buffer.monitor[0] = (double)pmsm_oid.sm;
    simulink_tx_buffer.monitor[1] = (double)pmsm_oid.pmsm_param.Rs;
    simulink_tx_buffer.monitor[2] = (double)pmsm_oid.pmsm_param.Ld;
    simulink_tx_buffer.monitor[3] = (double)pmsm_oid.pmsm_param.Lq;
    simulink_tx_buffer.monitor[4] = (double)pmsm_oid.pmsm_param.flux_linkage;
    simulink_tx_buffer.monitor[5] = (double)pmsm_oid.V_comp_volts;

    // Sensored encoder/mechanical identification progress and results.
    simulink_tx_buffer.monitor[6] = (double)pmsm_oid.sub_encoder.sm;
    simulink_tx_buffer.monitor[7] = (double)pmsm_oid.sub_encoder.fault;
    simulink_tx_buffer.monitor[8] = (double)pmsm_oid.sub_mech.sm;
    simulink_tx_buffer.monitor[9] = (double)pmsm_oid.sub_encoder.identified_pole_pairs;
    simulink_tx_buffer.monitor[10] = (double)pmsm_oid.sub_encoder.encoder_offset_pu;
    simulink_tx_buffer.monitor[11] = (double)pmsm_oid.pmsm_mech_param.J_total;
    simulink_tx_buffer.monitor[12] = (double)pmsm_oid.pmsm_mech_param.B_viscous;
    simulink_tx_buffer.monitor[13] = (double)pmsm_oid.sub_mech.load_torque_Nm;
    simulink_tx_buffer.monitor[14] = (double)mtr_ctrl.spd_if->speed;
    simulink_tx_buffer.monitor[15] = (double)flag_system_running;
}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_fast_enable_output()
{
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
