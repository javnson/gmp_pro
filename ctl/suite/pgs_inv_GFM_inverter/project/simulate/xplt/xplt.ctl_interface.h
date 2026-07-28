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

//=================================================================================================
// Controller interface

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
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

    idc_src = simulink_rx_buffer.adc_result[INV_ADC_ID_IDC];
    udc_src = simulink_rx_buffer.adc_result[INV_ADC_ID_VDC];

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&vabc);
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
#if defined USING_3D_SVPWM
    simulink_tx_buffer.pwm_cmp[0] = pwm_3d_out[phase_A];
    simulink_tx_buffer.pwm_cmp[1] = pwm_3d_out[phase_B];
    simulink_tx_buffer.pwm_cmp[2] = pwm_3d_out[phase_C];
    simulink_tx_buffer.pwm_cmp[3] = pwm_3d_out[phase_N];
#else
    simulink_tx_buffer.pwm_cmp[0] = spwm.pwm_out[phase_U];
    simulink_tx_buffer.pwm_cmp[1] = spwm.pwm_out[phase_V];
    simulink_tx_buffer.pwm_cmp[2] = spwm.pwm_out[phase_W];
    simulink_tx_buffer.pwm_cmp[3] = CTRL_PWM_CMP_MAX / 2;
#endif

    //
    // monitor
    //

    // Scope 1
    simulink_tx_buffer.monitor[0] = inv_ctrl.iabc.dat[phase_A];
    simulink_tx_buffer.monitor[1] = inv_ctrl.iabc.dat[phase_B];

    // Scope 2
    simulink_tx_buffer.monitor[2] = inv_ctrl.iab0.dat[phase_0];
    simulink_tx_buffer.monitor[3] = neg_current_ctrl.idqn.dat[phase_d];

    // Scope 3
    simulink_tx_buffer.monitor[4] = inv_ctrl.vab0.dat[phase_alpha];
    simulink_tx_buffer.monitor[5] = inv_ctrl.vab0.dat[phase_beta];

    // Scope 4
    simulink_tx_buffer.monitor[6] = ctl_get_gfl_pll_error(&inv_ctrl);
#if BUILD_LEVEL == 5
    simulink_tx_buffer.monitor[7] = gfm_transition.angle_gfm;
#else
    simulink_tx_buffer.monitor[7] = inv_ctrl.angle;
#endif

    // Scope 5
#if BUILD_LEVEL == 5
    simulink_tx_buffer.monitor[8] = gfm_droop_ctrl.vdq_ref.dat[phase_d];
    simulink_tx_buffer.monitor[9] = gfm_droop_ctrl.frequency_ref_hz;
#else
#ifdef USING_DSOGI_PLL
    simulink_tx_buffer.monitor[8] = inv_ctrl.pll.srf_pll.phasor.dat[phasor_sin];
    simulink_tx_buffer.monitor[9] = inv_ctrl.pll.srf_pll.phasor.dat[phasor_cos];
#else
    simulink_tx_buffer.monitor[8] = inv_ctrl.pll.phasor.dat[phasor_sin];
    simulink_tx_buffer.monitor[9] = inv_ctrl.pll.phasor.dat[phasor_cos];
#endif // USING_DSOGI_PLL
#endif

    // Scope 6
    simulink_tx_buffer.monitor[10] = inv_ctrl.idq.dat[phase_d];
    simulink_tx_buffer.monitor[11] = inv_ctrl.idq.dat[phase_q];

    // Scope 7
    simulink_tx_buffer.monitor[12] = inv_ctrl.vdq.dat[phase_d];
    simulink_tx_buffer.monitor[13] = inv_ctrl.vdq.dat[phase_q];
#if (BUILD_LEVEL == 3) || (BUILD_LEVEL == 5)
#if BUILD_LEVEL == 5
    simulink_tx_buffer.monitor[14] = inv_ctrl.idq_set.dat[phase_d];
#else
    simulink_tx_buffer.monitor[14] = gfl_voltage_ctrl.idq_out.dat[phase_d];
#endif
#if BUILD_LEVEL == 5
    simulink_tx_buffer.monitor[15] = gfm_transition.blend;
#else
    simulink_tx_buffer.monitor[15] = gfl_zero_ctrl.v0_out;
#endif
#else
    simulink_tx_buffer.monitor[14] = inv_ctrl.idq_set.dat[phase_d];
    simulink_tx_buffer.monitor[15] = inv_ctrl.idq_set.dat[phase_q];
#endif
}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_fast_enable_output()
{
    ctl_enable_gfl_inv(&inv_ctrl);

    csp_sl_enable_output();

    // The host simulation CSP owns the physical-output state.
}

// Disable Output
GMP_STATIC_INLINE void ctl_fast_disable_output()
{
    csp_sl_disable_output();
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
