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

GMP_STATIC_INLINE pwm_gt clllc_master_phase(void)
{
#if CLLLC_SYNC_MASTER_PWM_BASE == CLLLC_PRIMARY_LEG_A_BASE
    return clllc_mod.leg[0].phase;
#elif CLLLC_SYNC_MASTER_PWM_BASE == CLLLC_PRIMARY_LEG_B_BASE
    return clllc_mod.leg[1].phase;
#elif CLLLC_SYNC_MASTER_PWM_BASE == CLLLC_SECONDARY_LEG_A_BASE
    return clllc_mod.leg[2].phase;
#else
    return clllc_mod.leg[3].phase;
#endif
}

GMP_STATIC_INLINE void clllc_write_leg(uint32_t base,
                                       const adv_pwm_channel_t* leg,
                                       pwm_gt master_phase)
{
    pwm_gt relative_phase;
    pwm_gt phase_load;

    /* TBPHS is ignored by the synchronization master.  Express all requested
       phases relative to that master's logical phase so arbitrary SDPE leg
       ordering still produces the intended 180-degree bridge relationship. */
    if (leg->phase >= master_phase)
        relative_phase = leg->phase - master_phase;
    else
        relative_phase = leg->period - (master_phase - leg->phase);

    /* In up-count mode a sync loads TBCTR=TBPHS and the next ZERO appears
       after TBPRD-TBPHS ticks.  Convert the desired output delay into the
       register representation so positive/negative DAB commands retain their
       physical direction. */
    phase_load = (relative_phase == 0U) ? 0U :
                 (leg->period - relative_phase);

    EPWM_setTimeBasePeriod(base, leg->period);
    EPWM_setPhaseShift(base,
        (base == CLLLC_SYNC_MASTER_PWM_BASE) ? 0U : phase_load);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, leg->duty);
    if (base == CLLLC_ADC_TRIGGER_PWM_BASE)
        EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_C,
                                    (3U * leg->period) / 4U);
    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(base, leg->deadband);
    EPWM_setFallingEdgeDelayCount(base, leg->deadband);
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    pwm_gt master_phase = clllc_master_phase();
    clllc_write_leg(CLLLC_PRIMARY_LEG_A_BASE, &clllc_mod.leg[0], master_phase);
    clllc_write_leg(CLLLC_PRIMARY_LEG_B_BASE, &clllc_mod.leg[1], master_phase);
    clllc_write_leg(CLLLC_SECONDARY_LEG_A_BASE, &clllc_mod.leg[2], master_phase);
    clllc_write_leg(CLLLC_SECONDARY_LEG_B_BASE, &clllc_mod.leg[3], master_phase);
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    EPWM_clearTripZoneFlag(CLLLC_PRIMARY_LEG_A_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(CLLLC_PRIMARY_LEG_B_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(CLLLC_SECONDARY_LEG_A_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(CLLLC_SECONDARY_LEG_B_BASE, EPWM_TZ_FLAG_OST);
    GPIO_writePin(CLLLC_GATE_ENABLE_GPIO, CLLLC_GATE_ENABLE_ACTIVE_LEVEL);
}

GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    GPIO_writePin(CLLLC_GATE_ENABLE_GPIO, CLLLC_GATE_DISABLE_LEVEL);
    EPWM_forceTripZoneEvent(DIOSCURI_PWM_PAIR1_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(DIOSCURI_PWM_PAIR2_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(DIOSCURI_PWM_PAIR3_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(DIOSCURI_PWM_PAIR4_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(DIOSCURI_PWM_PAIR5_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(DIOSCURI_PWM_PAIR6_BASE, EPWM_TZ_FORCE_EVENT_OST);
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
