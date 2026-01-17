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

//#include <ctl/component/motor_control/basic/std_sil_motor_interface.h>

#include <xplt.peripheral.h>

//#include <ext/encoder/as5048/as5048a.h>

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

// raw data
extern adc_gt uabc_raw[3];
extern adc_gt iabc_raw[3];
extern adc_gt udc_raw;
extern adc_gt idc_raw;
extern pmsm_controller_t pmsm_ctrl;

// Functions without controller nano framework.
#ifndef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

// Input Callback
GMP_STATIC_INLINE
void ctl_input_callback(void)
{
    // update system tick
    gmp_step_system_tick();

    // copy ADC data to raw buffer
    // NOTICE use Result base not adc base.
    udc_raw = ADC_readResult(MOTOR_VBUS_RESULT_BASE, MOTOR_VBUS);

//    uabc_raw[phase_U] = ADC_readResult(MOTOR_VU_RESULT_BASE, MOTOR_VU);
//    uabc_raw[phase_V] = ADC_readResult(MOTOR_VV_RESULT_BASE, MOTOR_VV);
//    uabc_raw[phase_W] = ADC_readResult(MOTOR_VW_RESULT_BASE, MOTOR_VW);

    iabc_raw[phase_U] = ADC_readResult(MOTOR_IU_RESULT_BASE, MOTOR_IU);
    iabc_raw[phase_V] = ADC_readResult(MOTOR_IV_RESULT_BASE, MOTOR_IV);
    iabc_raw[phase_W] = ADC_readResult(MOTOR_IW_RESULT_BASE, MOTOR_IW);

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&uabc);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);

#ifdef PMSM_CTRL_USING_QEP_ENCODER
    // Step auto turn pos encoder
    ctl_step_autoturn_pos_encoder(&pos_enc, EQEP_getPosition(EQEP_Encoder1_BASE));
#else // PMSM_CTRL_USING_QEP_ENCODER
    // invoke position encoder routine.
    ctl_step_as5048a_pos_encoder(&pos_enc);
#endif // PMSM_CTRL_USING_QEP_ENCODER


}

// Output Callback
GMP_STATIC_INLINE
void ctl_output_callback(void)
{
    ctl_calc_pwm_tri_channel(&pwm_out);

    DAC_setShadowValue(DAC_C_BASE, pmsm_ctrl.iab0.dat[phase_A] * 2048 + 2048);
//    DAC_setShadowValue(DAC_B_BASE, pmsm_ctrl.vab0_set.dat[phase_A] * 2048 + 2048);

    //        EPWM_setCounterCompareValue(PHASE_U_PWM_BASE, EPWM_COUNTER_COMPARE_A,
    //                            (uint16_t)((INV_PWM_HALF_TBPRD * pwm1.Vabc_pu[0]) +
    //                                        INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[phase_U]);
    EPWM_setCounterCompareValue(PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[phase_V]);
    EPWM_setCounterCompareValue(PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[phase_W]);
}

void ctl_disable_output(void);
void ctl_enable_output(void);

#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

// Functions with controller nano framework

#ifdef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

// Controller Nano input stage routine
GMP_STATIC_INLINE
void ctl_fmif_input_stage_routine(ctl_object_nano_t *pctl_obj)
{
    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&uabc);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);

    // invoke position encoder routine.
    ctl_step_autoturn_pos_encoder(&pos_enc, simulink_rx_buffer.encoder);
}

// Controller Nano output stage routine
GMP_STATIC_INLINE
void ctl_fmif_output_stage_routine(ctl_object_nano_t *pctl_obj)
{
    ctl_calc_pwm_tri_channel(&pwm_out);

    simulink_tx_buffer.tabc[phase_A] = pwm_out.value[phase_A];
    simulink_tx_buffer.tabc[phase_B] = pwm_out.value[phase_B];
    simulink_tx_buffer.tabc[phase_C] = pwm_out.value[phase_C];

    // simulink_tx_buffer.monitor_port[0] = pmsm_ctrl.idq0.dat[phase_d];
    simulink_tx_buffer.monitor_port[0] = pmsm_ctrl.idq_set.dat[phase_q];
    simulink_tx_buffer.monitor_port[1] = pmsm_ctrl.idq0.dat[phase_q];

    simulink_tx_buffer.monitor_port[2] = pmsm_ctrl.vdq_set.dat[phase_d];
    // simulink_tx_buffer.monitor_port[3] = pmsm_ctrl.vdq_set.dat[phase_q];

    // simulink_tx_buffer.monitor_port[3] = pmsm_ctrl.mtr_interface.position->elec_position;
    simulink_tx_buffer.monitor_port[3] = pmsm_ctrl.mtr_interface.velocity->speed;
}

// Controller Request stage
GMP_STATIC_INLINE
void ctl_fmif_request_stage_routine(ctl_object_nano_t *pctl_obj)
{
}

// Enable Output
GMP_STATIC_INLINE
void ctl_fmif_output_enable(ctl_object_nano_t *pctl_obj)
{
    csp_sl_enable_output();
}

// Disable Output
GMP_STATIC_INLINE
void ctl_fmif_output_disable(ctl_object_nano_t *pctl_obj)
{
    csp_sl_disable_output();
}

#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
