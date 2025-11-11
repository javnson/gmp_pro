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
    SINV_ADC_ID_IL = 0,
    SINV_ADC_ID_IDC = 1,
    SINV_ADC_ID_VDC = 2,
    SINV_ADC_ID_IC = 3,
    SINV_ADC_ID_VG = 4,
    SINV_ADC_ID_IG = 5,
    SINV_ADC_ID_BUCK_IL = 6,
    SINV_ADC_ID_BUCK_VIN = 7,
    SINV_ADC_ID_BUCK_VOUT = 8,
    SINV_ADC_SENSOR_NUMBER = 9

} sinv_adc_index_items;

extern ptr_adc_channel_t sinv_adc[SINV_ADC_SENSOR_NUMBER];

extern pwm_channel_t sinv_pwm_out[3];

extern sinv_ctrl_t sinv_ctrl;

extern buck_ctrl_t buck_ctrl;

// Input Callback
GMP_STATIC_INLINE
void ctl_input_callback(void)
{
    //// invoke ADC p.u. routine
    for (size_gt i = 0; i < SINV_ADC_SENSOR_NUMBER; ++i)
        ctl_step_ptr_adc_channel(&sinv_adc[i]);
}

// Output Callback
GMP_STATIC_INLINE
void ctl_output_callback(void)
{
    //
    // PWM channel
    //
    simulink_tx_buffer.pwm_cmp[0] = ctl_step_pwm_channel(&sinv_pwm_out[0], sinv_ctrl.sinv_pwm_pu[0]);
    simulink_tx_buffer.pwm_cmp[1] = ctl_step_pwm_channel(&sinv_pwm_out[1], sinv_ctrl.sinv_pwm_pu[1]);
    simulink_tx_buffer.pwm_cmp[2] = ctl_step_pwm_channel(&sinv_pwm_out[2], buck_ctrl.pwm_out_pu);

    //
    // monitor
    //

#if BUILD_LEVEL == 1

    // Scope 1
    simulink_tx_buffer.monitor[0] = sinv_adc[SINV_ADC_ID_IL].control_port.value;
    simulink_tx_buffer.monitor[1] = sinv_adc[SINV_ADC_ID_IC].control_port.value;

    // Scope 2
    simulink_tx_buffer.monitor[2] = sinv_adc[SINV_ADC_ID_IDC].control_port.value;
    simulink_tx_buffer.monitor[3] = sinv_adc[SINV_ADC_ID_VDC].control_port.value;

    // Scope 3
    simulink_tx_buffer.monitor[4] = sinv_adc[SINV_ADC_ID_VG].control_port.value;
    simulink_tx_buffer.monitor[5] = sinv_adc[SINV_ADC_ID_IG].control_port.value;

    // Scope 4
    simulink_tx_buffer.monitor[6] = sinv_ctrl.sinv_pwm_pu[PHASE_L];
    simulink_tx_buffer.monitor[7] = sinv_ctrl.sinv_pwm_pu[PHASE_N];

    // Scope 5
    simulink_tx_buffer.monitor[8] = sinv_ctrl.rg.current;
    simulink_tx_buffer.monitor[9] = sinv_ctrl.spll.theta;

#elif BUILD_LEVEL == 2 || BUILD_LEVEL == 3

    // inverter current control

    // Scope 1: Current set / fbk
    simulink_tx_buffer.monitor[0] = sinv_ctrl.adc_igrid->value;
    simulink_tx_buffer.monitor[1] = sinv_ctrl.ig_ref;

    // Scope 2
    simulink_tx_buffer.monitor[2] = sinv_adc[SINV_ADC_ID_IDC].control_port.value;
    simulink_tx_buffer.monitor[3] = sinv_adc[SINV_ADC_ID_VDC].control_port.value;

    // Scope 3
    simulink_tx_buffer.monitor[4] = sinv_adc[SINV_ADC_ID_VG].control_port.value;
    simulink_tx_buffer.monitor[5] = sinv_adc[SINV_ADC_ID_IG].control_port.value;

#elif BUILD_LEVEL == 4 || BUILD_LEVEL == 5 || BUILD_LEVEL == 10

    // Scope 1: Current set / fbk
    simulink_tx_buffer.monitor[0] = sinv_ctrl.adc_igrid->value;
    simulink_tx_buffer.monitor[1] = sinv_ctrl.ig_ref;

    // Scope 2: SPLL
    simulink_tx_buffer.monitor[2] = sinv_ctrl.spll.phasor.dat[phasor_sin];
    simulink_tx_buffer.monitor[3] = sinv_ctrl.adc_ugrid->value;

    // Scope 3
    simulink_tx_buffer.monitor[4] = sinv_adc[SINV_ADC_ID_IDC].control_port.value;
    simulink_tx_buffer.monitor[5] = sinv_adc[SINV_ADC_ID_VDC].control_port.value;

    // Scope 4 power factor
    simulink_tx_buffer.monitor[6] = sinv_adc[SINV_ADC_ID_VG].control_port.value;
    simulink_tx_buffer.monitor[7] = sinv_adc[SINV_ADC_ID_IG].control_port.value;

#elif BUILD_LEVEL == 6 || BUILD_LEVEL == 7

     // Scope 1: Current set / fbk
    simulink_tx_buffer.monitor[0] = sinv_ctrl.adc_igrid->value;
    simulink_tx_buffer.monitor[1] = sinv_ctrl.ig_ref;

    // Scope 2: SPLL
    simulink_tx_buffer.monitor[2] = sinv_ctrl.spll.phasor.dat[phasor_sin];
    simulink_tx_buffer.monitor[3] = sinv_ctrl.adc_ugrid->value;

    // Scope 3
    simulink_tx_buffer.monitor[4] = sinv_adc[SINV_ADC_ID_IDC].control_port.value;
    simulink_tx_buffer.monitor[5] = sinv_adc[SINV_ADC_ID_VDC].control_port.value;

    // Scope 4 power factor
    simulink_tx_buffer.monitor[6] = sinv_adc[SINV_ADC_ID_VG].control_port.value;
    simulink_tx_buffer.monitor[7] = sinv_adc[SINV_ADC_ID_IG].control_port.value;

    // Scope 5 voltage set / fbk
    simulink_tx_buffer.monitor[8] = sinv_ctrl.v_set;
    simulink_tx_buffer.monitor[9] = sinv_ctrl.adc_udc->value;

    #elif BUILD_LEVEL == 8 || BUILD_LEVEL == 9

    // Scope 1: Current set / fbk
    simulink_tx_buffer.monitor[0] = sinv_ctrl.adc_igrid->value;
    simulink_tx_buffer.monitor[1] = sinv_ctrl.ig_ref;

    // Scope 2: SPLL
    simulink_tx_buffer.monitor[2] = sinv_ctrl.spll.phasor.dat[phasor_sin];
    simulink_tx_buffer.monitor[3] = sinv_ctrl.adc_ugrid->value;

    // Scope 3
    simulink_tx_buffer.monitor[4] = sinv_adc[SINV_ADC_ID_IDC].control_port.value;
    simulink_tx_buffer.monitor[5] = sinv_adc[SINV_ADC_ID_VDC].control_port.value;

    // Scope 4 power factor
    simulink_tx_buffer.monitor[6] = sinv_adc[SINV_ADC_ID_VG].control_port.value;
    simulink_tx_buffer.monitor[7] = sinv_adc[SINV_ADC_ID_IG].control_port.value;

    // Scope 5 voltage set / fbk
    simulink_tx_buffer.monitor[8] = sinv_ctrl.v_set;
    simulink_tx_buffer.monitor[9] = sinv_ctrl.vg_rms;

    // Zero Cross detect
    simulink_tx_buffer.monitor[10] = sinv_ctrl.ac_current_measure.curr_sign;



#endif // BUILD LEVEL



    // Buck controller
    simulink_tx_buffer.monitor[10] = sinv_adc[SINV_ADC_ID_BUCK_IL].control_port.value;
    simulink_tx_buffer.monitor[11] = sinv_adc[SINV_ADC_ID_BUCK_VIN].control_port.value;
    simulink_tx_buffer.monitor[12] = sinv_adc[SINV_ADC_ID_BUCK_VOUT].control_port.value;

}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE
void ctl_enable_output()
{
    ctl_enable_sinv_ctrl(&sinv_ctrl);
    // enable buck controller
    ctl_enable_buck_ctrl(&buck_ctrl);
    //
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
