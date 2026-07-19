/** @file xplt.peripheral.c @brief Dioscuri F280025C peripheral runtime. */
#include <gmp_core.h>
#include "user_main.h"
#include "ctl_main.h"
#include <xplt.peripheral.h>

#if (CLLLC_PRIMARY_LEG_A_BASE == CLLLC_PRIMARY_LEG_B_BASE) || \
    (CLLLC_PRIMARY_LEG_A_BASE == CLLLC_SECONDARY_LEG_A_BASE) || \
    (CLLLC_PRIMARY_LEG_A_BASE == CLLLC_SECONDARY_LEG_B_BASE) || \
    (CLLLC_PRIMARY_LEG_B_BASE == CLLLC_SECONDARY_LEG_A_BASE) || \
    (CLLLC_PRIMARY_LEG_B_BASE == CLLLC_SECONDARY_LEG_B_BASE) || \
    (CLLLC_SECONDARY_LEG_A_BASE == CLLLC_SECONDARY_LEG_B_BASE)
#error "The four CLLLC bridge legs must use four distinct Dioscuri PWM pairs."
#endif
#if (CLLLC_SYNC_MASTER_PWM_BASE != CLLLC_PRIMARY_LEG_A_BASE) && \
    (CLLLC_SYNC_MASTER_PWM_BASE != CLLLC_PRIMARY_LEG_B_BASE) && \
    (CLLLC_SYNC_MASTER_PWM_BASE != CLLLC_SECONDARY_LEG_A_BASE) && \
    (CLLLC_SYNC_MASTER_PWM_BASE != CLLLC_SECONDARY_LEG_B_BASE)
#error "CLLLC_SYNC_MASTER_PWM_BASE must be one of the four selected bridge legs."
#endif

extern gpio_halt user_led;
extern gmp_datalink_t dl;
interrupt void CLLLCTickISR(void);

/* Keep the SDPE-selectable PWM trigger base and the ADC SOC trigger source
 * inseparable.  SysConfig uses EPWM1 for the default board arrangement; these
 * mappings preserve correct sampling when a user reorders the PWM pairs. */
#if CLLLC_ADC_TRIGGER_PWM_BASE == EPWM1_BASE
#define CLLLC_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCA
#elif CLLLC_ADC_TRIGGER_PWM_BASE == EPWM2_BASE
#define CLLLC_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM2_SOCA
#elif CLLLC_ADC_TRIGGER_PWM_BASE == EPWM3_BASE
#define CLLLC_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM3_SOCA
#elif CLLLC_ADC_TRIGGER_PWM_BASE == EPWM4_BASE
#define CLLLC_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM4_SOCA
#elif CLLLC_ADC_TRIGGER_PWM_BASE == EPWM5_BASE
#define CLLLC_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM5_SOCA
#elif CLLLC_ADC_TRIGGER_PWM_BASE == EPWM7_BASE
#define CLLLC_ADC_TRIGGER_SOURCE ADC_TRIGGER_EPWM7_SOCA
#else
#error "CLLLC_ADC_TRIGGER_PWM_BASE must be one of the six Dioscuri PWM pairs."
#endif

static void configure_clllc_leg(uint32_t base, bool phase_enable)
{
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
    EPWM_setTimeBasePeriod(base, CLLLC_NOMINAL_PERIOD_TICKS);
    EPWM_setPeriodLoadMode(base, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_setTimeBaseCounter(base, 0U);
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A,
                                CLLLC_NOMINAL_PERIOD_TICKS / 2U);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED,
                                  EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED,
                                  EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDelayCount(base,
        (uint16_t)(CLLLC_DEADBAND_S * CLLLC_TIMER_CLOCK_HZ));
    EPWM_setFallingEdgeDelayCount(base,
        (uint16_t)(CLLLC_DEADBAND_S * CLLLC_TIMER_CLOCK_HZ));
    if (phase_enable)
    {
        EPWM_setSyncInPulseSource(base, CLLLC_SYNC_IN_SOURCE);
        EPWM_enablePhaseShiftLoad(base);
        EPWM_setCountModeAfterSync(base, EPWM_COUNT_MODE_UP_AFTER_SYNC);
        EPWM_disableSyncOutPulseSource(base, EPWM_SYNC_OUT_PULSE_ON_ALL);
    }
    else
    {
        EPWM_disablePhaseShiftLoad(base);
        EPWM_disableSyncOutPulseSource(base, EPWM_SYNC_OUT_PULSE_ON_ALL);
        EPWM_enableSyncOutPulseSource(base, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);
    }
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_OST);
}

void setup_peripheral(void)
{
    /* SN74LVC8T245 OE# is active low.  Hold all six PWM pairs disconnected
       until the state machine explicitly enables the four selected legs. */
    GPIO_writePin(CLLLC_GATE_ENABLE_GPIO, CLLLC_GATE_DISABLE_LEVEL);
    debug_uart = CLLLC_UART_BASE;
    user_led = CLLLC_STATUS_LED_GPIO;

    ctl_init_adc_channel(&adc_v_primary,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
            CLLLC_VOLTAGE_SENSITIVITY_V_PER_V, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CLLLC_VOLTAGE_BIAS_V),
        CTRL_ADC_BITS, 24);
    ctl_init_adc_channel(&adc_v_secondary,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
            CLLLC_VOLTAGE_SENSITIVITY_V_PER_V, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CLLLC_VOLTAGE_BIAS_V),
        CTRL_ADC_BITS, 24);
    ctl_init_adc_channel(&adc_i_primary,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
            CLLLC_CURRENT_SENSITIVITY_V_PER_A, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CLLLC_CURRENT_BIAS_V),
        CTRL_ADC_BITS, 24);
    ctl_init_adc_channel(&adc_i_secondary,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
            CLLLC_CURRENT_SENSITIVITY_V_PER_A, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CLLLC_CURRENT_BIAS_V),
        CTRL_ADC_BITS, 24);
    ctl_init_adc_channel(&adc_i_resonant,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
            CLLLC_CURRENT_SENSITIVITY_V_PER_A, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CLLLC_CURRENT_BIAS_V),
        CTRL_ADC_BITS, 24);

    /* Initialize all six pairs before OE# can be asserted.  The two spare
       FSBB pairs remain held by one-shot trip while the selected four pairs
       can later be released by ctl_fast_enable_output(). */
    configure_clllc_leg(DIOSCURI_PWM_PAIR1_BASE,
        DIOSCURI_PWM_PAIR1_BASE != CLLLC_SYNC_MASTER_PWM_BASE);
    configure_clllc_leg(DIOSCURI_PWM_PAIR2_BASE,
        DIOSCURI_PWM_PAIR2_BASE != CLLLC_SYNC_MASTER_PWM_BASE);
    configure_clllc_leg(DIOSCURI_PWM_PAIR3_BASE,
        DIOSCURI_PWM_PAIR3_BASE != CLLLC_SYNC_MASTER_PWM_BASE);
    configure_clllc_leg(DIOSCURI_PWM_PAIR4_BASE,
        DIOSCURI_PWM_PAIR4_BASE != CLLLC_SYNC_MASTER_PWM_BASE);
    configure_clllc_leg(DIOSCURI_PWM_PAIR5_BASE,
        DIOSCURI_PWM_PAIR5_BASE != CLLLC_SYNC_MASTER_PWM_BASE);
    configure_clllc_leg(DIOSCURI_PWM_PAIR6_BASE,
        DIOSCURI_PWM_PAIR6_BASE != CLLLC_SYNC_MASTER_PWM_BASE);

    /* Rebind every ADC SOC to the selected trigger PWM.  ADCA INT1 is raised
       by EOC2, after all three ADCA conversions; the parallel ADCC SOC0 has
       also completed by then, so ctl_input_callback() sees one coherent set. */
    ADC_setupSOC(ADCA_BASE, PRIMARY_CURRENT, CLLLC_ADC_TRIGGER_SOURCE,
                 PRIMARY_CURRENT_CHANNEL, 8U);
    ADC_setupSOC(ADCA_BASE, SECONDARY_CURRENT, CLLLC_ADC_TRIGGER_SOURCE,
                 SECONDARY_CURRENT_CHANNEL, 8U);
    ADC_setupSOC(ADCA_BASE, SECONDARY_VOLTAGE, CLLLC_ADC_TRIGGER_SOURCE,
                 SECONDARY_VOLTAGE_CHANNEL, 8U);
    ADC_setupSOC(ADCC_BASE, PRIMARY_VOLTAGE, CLLLC_ADC_TRIGGER_SOURCE,
                 PRIMARY_VOLTAGE_CHANNEL, 8U);
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, SECONDARY_VOLTAGE);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
    EPWM_setCounterCompareShadowLoadMode(CLLLC_ADC_TRIGGER_PWM_BASE,
                                         EPWM_COUNTER_COMPARE_C,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(CLLLC_ADC_TRIGGER_PWM_BASE,
                                EPWM_COUNTER_COMPARE_C,
                                (3U * CLLLC_NOMINAL_PERIOD_TICKS) / 4U);
    EPWM_enableADCTrigger(CLLLC_ADC_TRIGGER_PWM_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(CLLLC_ADC_TRIGGER_PWM_BASE, EPWM_SOC_A,
                             EPWM_SOC_TBCTR_U_CMPC);
    EPWM_setADCTriggerEventPrescale(CLLLC_ADC_TRIGGER_PWM_BASE, EPWM_SOC_A,
                                    CLLLC_PWM_CYCLES_PER_CONTROL);
    EPWM_clearADCTriggerFlag(CLLLC_ADC_TRIGGER_PWM_BASE, EPWM_SOC_A);
    EPWM_forceSyncPulse(CLLLC_SYNC_MASTER_PWM_BASE);

    /* The scheduler clock is independent of the variable switching period. */
    CPUTimer_stopTimer(CLLLC_SCHEDULER_TIMER_BASE);
    CPUTimer_setPreScaler(CLLLC_SCHEDULER_TIMER_BASE, 0U);
    CPUTimer_setPeriod(CLLLC_SCHEDULER_TIMER_BASE,
                       (uint32_t)(CLLLC_TIMER_CLOCK_HZ / CTRL_SYSTEM_TICK_HZ) - 1U);
    CPUTimer_reloadTimerCounter(CLLLC_SCHEDULER_TIMER_BASE);
    Interrupt_register(INT_TIMER0, &CLLLCTickISR);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_enableInterrupt(CLLLC_SCHEDULER_TIMER_BASE);
    CPUTimer_startTimer(CLLLC_SCHEDULER_TIMER_BASE);

    gmp_base_print(TEXT_STRING("Dioscuri CLLLC controller online.\r\n"));
}

interrupt void MainISR(void)
{
    gmp_base_ctl_step();
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    if (ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    }
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

interrupt void CLLLCTickISR(void)
{
    gmp_step_system_tick();
    CPUTimer_clearOverflowFlag(CLLLC_SCHEDULER_TIMER_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

void reset_controller(void) { ctl_fast_disable_output(); }

void flush_dl_tx_buffer(void)
{
    gmp_hal_uart_write(CLLLC_UART_BASE, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl),
                       gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10);
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
        gmp_hal_uart_write(CLLLC_UART_BASE, gmp_dev_dl_get_tx_hw_pld_ptr(&dl),
                           gmp_dev_dl_get_tx_hw_pld_size(&dl), 10);
}

void flush_dl_rx_buffer(void)
{
    uint16_t level = SCI_getRxFIFOStatus(CLLLC_UART_BASE);
    data_gt buffer[16];
    if (level > 16U) level = 16U;
    if (level > 0U)
    {
        SCI_readCharArray(CLLLC_UART_BASE, (uint16_t*)buffer, level);
        gmp_dev_dl_push_str(&dl, buffer, level);
    }
}

interrupt void INT_UART_USB_RX_ISR(void)
{
    flush_dl_rx_buffer();
    if (SCI_getRxStatus(CLLLC_UART_BASE) & SCI_RXSTATUS_OVERRUN)
        SCI_clearOverflowStatus(CLLLC_UART_BASE);
    SCI_clearInterruptStatus(CLLLC_UART_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INT_UART_USB_RX_INTERRUPT_ACK_GROUP);
}

/* CAN is retained as an optional board service.  No control command is acted
   upon until the schematic/pinmux mismatch is resolved. */
interrupt void INT_COMM_CAN_0_ISR(void)
{
    CAN_clearGlobalInterruptStatus(COMM_CAN_BASE, CAN_GLOBAL_INT_CANINT0);
    Interrupt_clearACKGroup(INT_COMM_CAN_0_INTERRUPT_ACK_GROUP);
}

interrupt void INT_COMM_CAN_1_ISR(void)
{
    CAN_clearGlobalInterruptStatus(COMM_CAN_BASE, CAN_GLOBAL_INT_CANINT1);
    Interrupt_clearACKGroup(INT_COMM_CAN_1_INTERRUPT_ACK_GROUP);
}

void send_monitor_data(void) { }
