/** @file xplt.peripheral.c @brief Dioscuri F280025C peripheral runtime. */
#include <gmp_core.h>
#include "user_main.h"
#include "ctl_main.h"
#include <xplt.peripheral.h>

extern gpio_halt user_led;
extern gmp_datalink_t dl;
interrupt void CLLLCTickISR(void);

static void configure_clllc_leg(uint32_t base, bool phase_enable)
{
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
    EPWM_setTimeBasePeriod(base, CLLLC_NOMINAL_PERIOD_TICKS);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A,
                                CLLLC_NOMINAL_PERIOD_TICKS / 2U);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    if (phase_enable)
        EPWM_enablePhaseShiftLoad(base);
    else
        EPWM_disablePhaseShiftLoad(base);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_OST);
}

void setup_peripheral(void)
{
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

    configure_clllc_leg(EPWM1_BASE, false);
    configure_clllc_leg(EPWM2_BASE, true);
    configure_clllc_leg(EPWM3_BASE, true);
    configure_clllc_leg(EPWM4_BASE, true);
    EPWM_enableADCTrigger(CLLLC_ADC_TRIGGER_PWM_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(CLLLC_ADC_TRIGGER_PWM_BASE, EPWM_SOC_A,
                             EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(CLLLC_ADC_TRIGGER_PWM_BASE, EPWM_SOC_A,
                                    CLLLC_PWM_CYCLES_PER_CONTROL);

    /* The scheduler clock is independent of the variable switching period. */
    CPUTimer_stopTimer(CLLLC_TICK_TIMER_BASE);
    CPUTimer_setPreScaler(CLLLC_TICK_TIMER_BASE, 0U);
    CPUTimer_setPeriod(CLLLC_TICK_TIMER_BASE,
                       (uint32_t)(CLLLC_TIMER_CLOCK_HZ / CTRL_SYSTEM_TICK_HZ) - 1U);
    CPUTimer_reloadTimerCounter(CLLLC_TICK_TIMER_BASE);
    Interrupt_register(INT_TIMER0, &CLLLCTickISR);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_enableInterrupt(CLLLC_TICK_TIMER_BASE);
    CPUTimer_startTimer(CLLLC_TICK_TIMER_BASE);

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
    CPUTimer_clearOverflowFlag(CLLLC_TICK_TIMER_BASE);
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
