#ifndef GMP_MCS_PMSM_NT_F29H85X_XPLT_CTL_INTERFACE_H
#define GMP_MCS_PMSM_NT_F29H85X_XPLT_CTL_INTERFACE_H

#include <xplt.peripheral.h>

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    uuvw_src[phase_U] = ADC_readResult(INV_UU_RESULT_BASE, INV_UU);
    uuvw_src[phase_V] = ADC_readResult(INV_UV_RESULT_BASE, INV_UV);
    uuvw_src[phase_W] = ADC_readResult(INV_UW_RESULT_BASE, INV_UW);

    iuvw_src[phase_U] = ADC_readResult(INV_IU_RESULT_BASE, INV_IU);
    iuvw_src[phase_V] = ADC_readResult(INV_IV_RESULT_BASE, INV_IV);
    iuvw_src[phase_W] = ADC_readResult(INV_IW_RESULT_BASE, INV_IW);
    udc_src = ADC_readResult(INV_VBUS_RESULT_BASE, INV_VBUS);

    ctl_step_autoturn_pos_encoder(&pos_enc, EQEP_getPosition(EQEP_Encoder_BASE));
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

GMP_STATIC_INLINE uint16_t c29x_pwm_compare(ctrl_gt compare)
{
    if (compare <= 0.0f)
        return 0U;
    if (compare >= (ctrl_gt)CTRL_PWM_CMP_MAX)
        return (uint16_t)CTRL_PWM_CMP_MAX;
    return (uint16_t)compare;
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    uint16_t cmp_u = c29x_pwm_compare(spwm.pwm_out[phase_U]);
    uint16_t cmp_v = c29x_pwm_compare(spwm.pwm_out[phase_V]);
    uint16_t cmp_w = c29x_pwm_compare(spwm.pwm_out[phase_W]);

    EPWM_setCounterCompareValue(PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, cmp_u);
    EPWM_setCounterCompareValue(PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, cmp_v);
    EPWM_setCounterCompareValue(PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, cmp_w);
}

GMP_STATIC_INLINE void ctl_force_physical_output_safe(void)
{
    EPWM_forceTripZoneEvent(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);
    GPIO_writePin(PWM_ENABLE_PORT, PWM_DISABLE_LEVEL);
    GPIO_writePin(CONTROLLER_LED, 1U);
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    clear_all_controllers();
    EPWM_clearTripZoneFlag(PHASE_U_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(PHASE_V_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(PHASE_W_BASE, EPWM_TZ_FLAG_OST);
    GPIO_writePin(PWM_ENABLE_PORT, PWM_ENABLE_ACTIVE_LEVEL);
    GPIO_writePin(CONTROLLER_LED, 0U);
}

GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    ctl_force_physical_output_safe();
}

#endif /* GMP_MCS_PMSM_NT_F29H85X_XPLT_CTL_INTERFACE_H */
