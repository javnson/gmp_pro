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


//! Trip Zones all interrupt
//!
#define HAL_TZ_INTERRUPT_ALL     ( EPWM_TZ_INTERRUPT_DCBEVT2 \
                                 + EPWM_TZ_INTERRUPT_DCBEVT1 \
                                 + EPWM_TZ_INTERRUPT_DCAEVT2 \
                                 + EPWM_TZ_INTERRUPT_DCAEVT1 \
                                 + EPWM_TZ_INTERRUPT_OST \
                                 + EPWM_TZ_INTERRUPT_CBC )

//////////////////////////////////////////////////////////////////////////
// device related functions
// Controller interface
//

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&uabc);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);

    // invoke position encoder routine.
//    ctl_step_autoturn_pos_encoder(&pos_enc, simulink_rx_buffer.encoder);

    // Get panel input here.
#if (BUILD_LEVEL == 1)

    //ctl_set_pmsm_ctrl_vdq_ff(&pmsm_ctrl, float2ctrl(csp_sl_get_panel_input(0)), float2ctrl(csp_sl_get_panel_input(1)));

#endif // BUILD_LEVEL
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    ctl_calc_pwm_tri_channel(&pwm_out);

    EPWM_setCounterCompareValue(IRIS_EPWM1_BASE ,EPWM_COUNTER_COMPARE_A, pwm_out.value[phase_A]);
    EPWM_setCounterCompareValue(IRIS_EPWM2_BASE ,EPWM_COUNTER_COMPARE_A, pwm_out.value[phase_B]);
    EPWM_setCounterCompareValue(IRIS_EPWM3_BASE ,EPWM_COUNTER_COMPARE_A, pwm_out.value[phase_C]);

//    // PWM output
//    simulink_tx_buffer.tabc[phase_A] = pwm_out.value[phase_A];
//    simulink_tx_buffer.tabc[phase_B] = pwm_out.value[phase_B];
//    simulink_tx_buffer.tabc[phase_C] = pwm_out.value[phase_C];

    // Monitor Port, 8 channels
#if BUILD_LEVEL == 1

    // output DAC
    DAC_setShadowValue(IRIS_DACA_BASE, 1000);
    DAC_setShadowValue(IRIS_DACB_BASE, 1000);


//    // angle set
//    simulink_tx_buffer.monitor_port[0] = rg.rg.current;
//    simulink_tx_buffer.monitor_port[1] = rg.current_freq;
//
//    // current feedback
//    simulink_tx_buffer.monitor_port[2] = pmsm_ctrl.idq0.dat[phase_d];
//    simulink_tx_buffer.monitor_port[3] = pmsm_ctrl.idq0.dat[phase_q];
//
//    // voltage feedback
//    simulink_tx_buffer.monitor_port[4] = pmsm_ctrl.mtr_interface.iabc->value.dat[phase_A];
//    simulink_tx_buffer.monitor_port[5] = pmsm_ctrl.mtr_interface.iabc->value.dat[phase_B];
//
//    // encoder feedback
//    simulink_tx_buffer.monitor_port[6] = pmsm_ctrl.mtr_interface.position->elec_position;
//    simulink_tx_buffer.monitor_port[7] = pmsm_ctrl.mtr_interface.velocity->speed;

#endif // BUILD_LEVEL
}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_enable_output()
{
    //ctl_enable_pmsm_ctrl_output(&pmsm_ctrl);

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(IRIS_EPWM1_BASE, HAL_TZ_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(IRIS_EPWM2_BASE, HAL_TZ_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(IRIS_EPWM3_BASE, HAL_TZ_INTERRUPT_ALL);

}

// Disable Output
GMP_STATIC_INLINE void ctl_disable_output()
{
    // Disables the PWM device
    EPWM_forceTripZoneEvent(IRIS_EPWM1_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(IRIS_EPWM2_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(IRIS_EPWM3_BASE, EPWM_TZ_FORCE_EVENT_OST);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
