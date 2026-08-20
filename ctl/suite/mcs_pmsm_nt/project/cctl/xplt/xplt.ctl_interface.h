/** @file xplt.ctl_interface.h @brief ADC/eQEP/ePWM binding for direct simulation. */
#ifndef MCS_PMSM_NT_CCTL_XPLT_CTL_INTERFACE_H
#define MCS_PMSM_NT_CCTL_XPLT_CTL_INTERFACE_H

#include <xplt.peripheral.h>

#ifdef __cplusplus
extern "C"
{
#endif

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    uuvw_src[phase_U] = cctl_adc_result[CCTL_ADC_UA];
    uuvw_src[phase_V] = cctl_adc_result[CCTL_ADC_UB];
    uuvw_src[phase_W] = cctl_adc_result[CCTL_ADC_UC];
    iuvw_src[phase_U] = cctl_adc_result[CCTL_ADC_IA];
    iuvw_src[phase_V] = cctl_adc_result[CCTL_ADC_IB];
    iuvw_src[phase_W] = cctl_adc_result[CCTL_ADC_IC];
    udc_src = cctl_adc_result[CCTL_ADC_UDC];
    idc_src = 0;

    ctl_step_autoturn_pos_encoder(&pos_enc, cctl_encoder_position);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    cctl_pwm_compare[phase_U] = spwm.pwm_out[phase_U];
    cctl_pwm_compare[phase_V] = spwm.pwm_out[phase_V];
    cctl_pwm_compare[phase_W] = spwm.pwm_out[phase_W];
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    clear_all_controllers();
    cctl_pwm_output_enabled = 1;
}

GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    cctl_pwm_output_enabled = 0;
    cctl_pwm_compare[phase_U] = 0;
    cctl_pwm_compare[phase_V] = 0;
    cctl_pwm_compare[phase_W] = 0;
}

#ifdef __cplusplus
}
#endif

#endif // MCS_PMSM_NT_CCTL_XPLT_CTL_INTERFACE_H
