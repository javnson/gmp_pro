/** @file xplt.peripheral.c @brief FSBB hardware, CAN and XDS110 bindings. */
#include <gmp_core.h>
#include <core/dev/datalink/datalink.h>
#include <xplt.ctl_interface.h>

extern gpio_halt user_led;
extern gmp_datalink_t dl;
volatile uint16_t fsbb_enable_delay;

static void fsbb_setup_pwm(uint32_t base)
{
    EPWM_setTimeBasePeriod(base, CTRL_PWM_CMP_MAX);
    EPWM_setTimeBaseCounter(base, 0);
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, CTRL_PWM_CMP_MAX / 2U);
    // High around counter zero, so CMPA is proportional to duty.
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDelayCount(base, CTRL_PWM_DEADBAND_CMP);
    EPWM_setFallingEdgeDelayCount(base, CTRL_PWM_DEADBAND_CMP);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);
}

void setup_peripheral(void)
{
    ctl_fast_disable_output();
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    fsbb_setup_pwm(FSBB_BUCK_PWM_BASE);
    fsbb_setup_pwm(FSBB_BOOST_PWM_BASE);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    user_led = SYSTEM_LED;
    SCI_setBaud(LAUNCHXL_UART_USB_BASE, DEVICE_LSPCLK_FREQ, GMP_DL_UART_BAUDRATE);
    SCI_enableInterrupt(LAUNCHXL_UART_USB_BASE, SCI_INT_RXFF | SCI_INT_RXERR);
    // Leave debug_uart unbound to keep raw print messages out of the Data Link stream.

    ctl_init_adc_channel(&adc_v_in,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_VIN_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_VIN_BIAS), 12, 24);
    ctl_init_adc_channel(&adc_v_out,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_VOUT_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_VOUT_BIAS), 12, 24);
    ctl_init_adc_channel(&adc_i_L,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_IL_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_IL_BIAS), 12, 24);
    ctl_init_adc_channel(&adc_i_load,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_IOUT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_FSBB_IOUT_BIAS), 12, 24);
}

interrupt void MainISR(void)
{
#if !defined ENABLE_GMP_DL_PIL_SIM
    gmp_base_ctl_step();
#endif
    gmp_step_system_tick();

    ADC_clearInterruptStatus(FSBB_IL_ADC_BASE, ADC_INT_NUMBER1);
    if (ADC_getInterruptOverflowStatus(FSBB_IL_ADC_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(FSBB_IL_ADC_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(FSBB_IL_ADC_BASE, ADC_INT_NUMBER1);
    }
    Interrupt_clearACKGroup(INT_FSBB_IL_ADC_1_INTERRUPT_ACK_GROUP);
}

#define CAN_SCALE_FACTOR 10000.0f

typedef union
{
    int32_t i32;
    uint16_t u16[2];
} can_data_t;

interrupt void INT_LAUNCHXL_CAN_0_ISR(void)
{
    const uint32_t status = CAN_getInterruptCause(LAUNCHXL_CAN_BASE);
    uint16_t data[8] = {0};

    if (status == 1U || status == 2U)
    {
        const bool received = CAN_readMessage(LAUNCHXL_CAN_BASE, status, data);
        CAN_clearInterruptStatus(LAUNCHXL_CAN_BASE, status);
#if !defined ENABLE_GMP_DL_PIL_SIM
        if (received && status == 1U)
        {
            if (data[0] == 0U)
            {
                ctl_disable_pwm();
                cia402_send_cmd(&cia402_sm, CIA402_CMD_DISABLE_VOLTAGE);
            }
            else if (data[0] == 1U && g_fsbb_faults == FSBB_FAULT_NONE)
            {
                cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);
            }
            else if (data[0] == 2U && cia402_sm.current_state == CIA402_SM_FAULT &&
                     fsbb_active_faults() == FSBB_FAULT_NONE)
            {
                ctl_disable_pwm();
                EPWM_clearOneShotTripZoneFlag(FSBB_BUCK_PWM_BASE, EPWM_TZ_OST_FLAG_OST1);
                EPWM_clearOneShotTripZoneFlag(FSBB_BOOST_PWM_BASE, EPWM_TZ_OST_FLAG_OST1);
                g_fsbb_faults = FSBB_FAULT_NONE;
                cia402_send_cmd(&cia402_sm, CIA402_CMD_FAULT_RESET);
            }
        }
        else if (received && status == 2U)
        {
            can_data_t voltage;
            can_data_t current;
            voltage.u16[0] = data[0] | (data[1] << 8);
            voltage.u16[1] = data[2] | (data[3] << 8);
            current.u16[0] = data[4] | (data[5] << 8);
            current.u16[1] = data[6] | (data[7] << 8);
            g_v_out_ref_user = real2ctrl(voltage.i32 / CAN_SCALE_FACTOR);
            g_i_limit_user = real2ctrl(current.i32 / CAN_SCALE_FACTOR);
        }
#else
        GMP_UNUSED_VAR(received);
#endif
    }

    CAN_clearGlobalInterruptStatus(LAUNCHXL_CAN_BASE, CAN_GLOBAL_INT_CANINT0);
    Interrupt_clearACKGroup(INT_LAUNCHXL_CAN_0_INTERRUPT_ACK_GROUP);
}

interrupt void INT_LAUNCHXL_CAN_1_ISR(void)
{
    CAN_clearGlobalInterruptStatus(LAUNCHXL_CAN_BASE, CAN_GLOBAL_INT_CANINT1);
    Interrupt_clearACKGroup(INT_LAUNCHXL_CAN_1_INTERRUPT_ACK_GROUP);
}

#if !defined ENABLE_GMP_DL_PIL_SIM
static void send_monitor_pair(uint16_t mailbox, int32_t first, int32_t second)
{
    const uint32_t a = (uint32_t)first;
    const uint32_t b = (uint32_t)second;
    const uint16_t data[8] = {
        a & 0xFFU, (a >> 8) & 0xFFU, (a >> 16) & 0xFFU, (a >> 24) & 0xFFU,
        b & 0xFFU, (b >> 8) & 0xFFU, (b >> 16) & 0xFFU, (b >> 24) & 0xFFU
    };
    CAN_sendMessage(LAUNCHXL_CAN_BASE, mailbox, 8, data);
}
#endif

void send_monitor_data(void)
{
#if !defined ENABLE_GMP_DL_PIL_SIM
    send_monitor_pair(4, (int32_t)(adc_v_in.control_port.value * CAN_SCALE_FACTOR),
                     (int32_t)(adc_v_out.control_port.value * CAN_SCALE_FACTOR));
    send_monitor_pair(5, (int32_t)(adc_i_L.control_port.value * CAN_SCALE_FACTOR), 0);
    send_monitor_pair(6, (int32_t)(dcdc_core.v_target * CAN_SCALE_FACTOR),
                     (int32_t)(dcdc_core.v_out_formal * CAN_SCALE_FACTOR));
    send_monitor_pair(7, cia402_sm.state_word.all, g_fsbb_faults);
#endif
}

// XDS110 backchannel Data Link transport.
/** @brief Bounded time allowed for each framed UART segment. */
#define DL_UART_TX_TIMEOUT_MS 50U

void flush_dl_tx_buffer(void)
{
    // Send head
    gmp_hal_uart_write(LAUNCHXL_UART_USB_BASE, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl),
                       gmp_dev_dl_get_tx_hw_hdr_size(&dl), DL_UART_TX_TIMEOUT_MS);

    // Send data body, if necessary
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
    {
        gmp_hal_uart_write(LAUNCHXL_UART_USB_BASE, gmp_dev_dl_get_tx_hw_pld_ptr(&dl),
                           gmp_dev_dl_get_tx_hw_pld_size(&dl), DL_UART_TX_TIMEOUT_MS);
    }
}

/**
 * @brief Drain every currently available SCI receive unit without blocking.
 * @details The caller must prevent the background task and the receive ISR
 *          from entering this function concurrently.
 */
static void drain_dl_rx_fifo_nonblocking(void)
{
    while (SCI_getRxFIFOStatus(LAUNCHXL_UART_USB_BASE) != SCI_FIFO_RX0)
    {
        gmp_dev_dl_push_byte(&dl, (byte_gt)SCI_readCharNonBlocking(LAUNCHXL_UART_USB_BASE));
    }
}

/**
 * @brief Restore SCI reception after a framing, parity, break, or overrun error.
 * @details Error data is discarded before the receiver and FIFO are reset.
 *          Interrupt enables are restored explicitly because malformed baud
 *          traffic may assert several receiver error sources at once.
 */
static void recover_dl_rx_transport(void)
{
    SCI_disableInterrupt(LAUNCHXL_UART_USB_BASE, SCI_INT_RXFF | SCI_INT_RXERR);
    while (SCI_getRxFIFOStatus(LAUNCHXL_UART_USB_BASE) != SCI_FIFO_RX0)
    {
        (void)SCI_readCharNonBlocking(LAUNCHXL_UART_USB_BASE);
    }
    (void)SCI_readCharNonBlocking(LAUNCHXL_UART_USB_BASE);
    LAUNCHXL_UART_USB_init();
    SCI_setBaud(LAUNCHXL_UART_USB_BASE, DEVICE_LSPCLK_FREQ, GMP_DL_UART_BAUDRATE);
    SCI_enableInterrupt(LAUNCHXL_UART_USB_BASE, SCI_INT_RXFF | SCI_INT_RXERR);
    gmp_dev_dl_request_rx_reset(&dl);
}

/** @brief Poll the SCI receive FIFO from the Data Link background task. */
void flush_dl_rx_buffer(void)
{
    /* Keep the RX ISR from consuming a FIFO depth observed by this task. */
    gmp_base_enter_critical();
    if ((SCI_getRxStatus(LAUNCHXL_UART_USB_BASE) & SCI_RXSTATUS_ERROR) != 0U)
    {
        recover_dl_rx_transport();
    }
    else
    {
        drain_dl_rx_fifo_nonblocking();
    }
    gmp_base_leave_critical();
}

interrupt void INT_LAUNCHXL_UART_USB_RX_ISR(void)
{
    uint32_t rx_status = SCI_getRxStatus(LAUNCHXL_UART_USB_BASE);

    if ((rx_status & SCI_RXSTATUS_ERROR) != 0U)
    {
        recover_dl_rx_transport();
    }
    else
    {
        drain_dl_rx_fifo_nonblocking();
        SCI_clearInterruptStatus(LAUNCHXL_UART_USB_BASE, SCI_INT_RXFF);
    }

    Interrupt_clearACKGroup(INT_LAUNCHXL_UART_USB_RX_INTERRUPT_ACK_GROUP);
}
