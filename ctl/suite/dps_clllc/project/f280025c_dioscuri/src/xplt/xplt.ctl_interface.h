/** @file xplt.ctl_interface.h @brief F280025C CLLLC hardware binding. */
#ifndef DPS_CLLLC_XPLT_CTL_INTERFACE_H
#define DPS_CLLLC_XPLT_CTL_INTERFACE_H
#include <xplt.peripheral.h>

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    ctl_step_adc_channel(&adc_v_primary,
        ADC_readResult(CLLLC_PRIMARY_V_ADC_BASE, CLLLC_PRIMARY_V_ADC_SOC));
    ctl_step_adc_channel(&adc_i_primary,
        ADC_readResult(CLLLC_PRIMARY_I_ADC_BASE, CLLLC_PRIMARY_I_ADC_SOC));
    ctl_step_adc_channel(&adc_v_secondary,
        ADC_readResult(CLLLC_SECONDARY_V_ADC_BASE, CLLLC_SECONDARY_V_ADC_SOC));
    ctl_step_adc_channel(&adc_i_secondary,
        ADC_readResult(CLLLC_SECONDARY_I_ADC_BASE, CLLLC_SECONDARY_I_ADC_SOC));
}

GMP_STATIC_INLINE void clllc_write_leg(uint32_t base, const adv_pwm_channel_t* leg)
{
    EPWM_setTimeBasePeriod(base, leg->period);
    EPWM_setPhaseShift(base, leg->phase);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, leg->duty);
    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(base, leg->deadband);
    EPWM_setFallingEdgeDelayCount(base, leg->deadband);
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    clllc_write_leg(CLLLC_PRIMARY_LEG_A_BASE, &clllc_mod.leg[0]);
    clllc_write_leg(CLLLC_PRIMARY_LEG_B_BASE, &clllc_mod.leg[1]);
    clllc_write_leg(CLLLC_SECONDARY_LEG_A_BASE, &clllc_mod.leg[2]);
    clllc_write_leg(CLLLC_SECONDARY_LEG_B_BASE, &clllc_mod.leg[3]);
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    EPWM_clearTripZoneFlag(EPWM1_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(EPWM2_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(EPWM3_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(EPWM4_BASE, EPWM_TZ_FLAG_OST);
}

GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    EPWM_forceTripZoneEvent(EPWM1_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWM2_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWM3_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWM4_BASE, EPWM_TZ_FORCE_EVENT_OST);
}

enum { CLLLC_PIL_VPRI, CLLLC_PIL_IPRI, CLLLC_PIL_VSEC,
       CLLLC_PIL_ISEC, CLLLC_PIL_IRESONANT, CLLLC_PIL_ADC_COUNT };

GMP_STATIC_INLINE void ctl_input_callback_pil(const gmp_sim_rx_buf_t* rx)
{
    ctl_step_adc_channel(&adc_v_primary, rx->adc_result[CLLLC_PIL_VPRI]);
    ctl_step_adc_channel(&adc_i_primary, rx->adc_result[CLLLC_PIL_IPRI]);
    ctl_step_adc_channel(&adc_v_secondary, rx->adc_result[CLLLC_PIL_VSEC]);
    ctl_step_adc_channel(&adc_i_secondary, rx->adc_result[CLLLC_PIL_ISEC]);
    ctl_step_adc_channel(&adc_i_resonant, rx->adc_result[CLLLC_PIL_IRESONANT]);
}

GMP_STATIC_INLINE void ctl_output_callback_pil(gmp_sim_tx_buf_t* tx)
{
    int i;
    for (i = 0; i < 4; ++i)
    {
        tx->pwm_cmp[i] = clllc_mod.leg[i].duty;
        tx->pwm_cmp[i + 4] = clllc_mod.leg[i].phase;
    }
    tx->monitor[0] = adc_v_secondary.control_port.value;
    tx->monitor[1] = adc_i_resonant.control_port.value;
    tx->monitor[2] = g_modulation_command;
    tx->monitor[3] = ctrl2float(clllc_mod.leg[0].raw.period);
}
#endif
