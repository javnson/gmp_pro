/**
 * @file xplt.peripheral.c
 * @brief LAUNCHXL-F280049C binding for SCIA, CPU Timer 0, and the user LED.
 */

#include <gmp_core.h>

#include "user_main.h"
#include <xplt.peripheral.h>

#ifndef XPLT_DL_BAUD_RATE
#define XPLT_DL_BAUD_RATE 115200UL
#endif

#define XPLT_DL_RX_FIFO_LEVEL SCI_FIFO_RX8
#define XPLT_DL_LOCAL_RX_SIZE 16U

#if XPLT_DL_BAUD_RATE > 500000UL
#define XPLT_DL_LSPCLK_HZ (DEVICE_SYSCLK_FREQ / 2UL)
#else
#define XPLT_DL_LSPCLK_HZ DEVICE_LSPCLK_FREQ
#endif

static gmp_datalink_t* bound_datalink;

/** @brief SCIA receive FIFO interrupt handler. */
__interrupt void xplt_scia_rx_isr(void);

/** @brief CPU Timer 0 sampling interrupt handler. */
__interrupt void xplt_cpu_timer0_isr(void);

/**
 * @brief Drain all currently buffered SCIA characters into the Data Link FIFO.
 * @details Each C28x character occupies one native 16-bit data unit while only
 * the low eight bits are part of the wire protocol.
 */
static void xplt_dl_drain_rx_fifo(void)
{
    data_gt local_buffer[XPLT_DL_LOCAL_RX_SIZE];
    size_gt count;
    size_gt index;

    if (bound_datalink == NULL)
    {
        while (SCI_getRxFIFOStatus(SCIA_BASE) != SCI_FIFO_RX0)
            (void)SCI_readCharNonBlocking(SCIA_BASE);
        return;
    }

    do
    {
        count = (size_gt)SCI_getRxFIFOStatus(SCIA_BASE);
        if (count > XPLT_DL_LOCAL_RX_SIZE)
            count = XPLT_DL_LOCAL_RX_SIZE;
        for (index = 0U; index < count; ++index)
            local_buffer[index] = (data_gt)(SCI_readCharNonBlocking(SCIA_BASE) & 0x00FFU);
        if (count > 0U)
            gmp_dev_dl_push_str(bound_datalink, local_buffer, count);
    } while (SCI_getRxFIFOStatus(SCIA_BASE) != SCI_FIFO_RX0);
}

/** @brief Configure SCIA for the LaunchPad XDS110 Application/User UART. */
static void xplt_configure_scia(void)
{
#if XPLT_DL_BAUD_RATE > 500000UL
    /* A 50 MHz LSPCLK minimizes the F280049C divider error at 921600 baud. */
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_2);
#endif

    GPIO_setControllerCore(DEVICE_GPIO_PIN_SCIRXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    GPIO_setControllerCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);

    SCI_performSoftwareReset(SCIA_BASE);
    SCI_setConfig(SCIA_BASE, XPLT_DL_LSPCLK_HZ, XPLT_DL_BAUD_RATE,
                  SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE);
#if XPLT_DL_BAUD_RATE == 921600UL
    /* Force BRR=6: 50 MHz / (8 * (6 + 1)) = 892857 baud. */
    HWREGH(SCIA_BASE + SCI_O_HBAUD) = 0U;
    HWREGH(SCIA_BASE + SCI_O_LBAUD) = 6U;
#endif
    SCI_resetChannels(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, XPLT_DL_RX_FIFO_LEVEL);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_RXERR);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXFF | SCI_INT_RXERR);
    SCI_enableModule(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);
}

/** @brief Configure CPU Timer 0 as the 1 kHz waveform time base. */
static void xplt_configure_sample_timer(void)
{
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    CPUTimer_setPeriod(CPUTIMER0_BASE, (DEVICE_SYSCLK_FREQ / 1000UL) - 1UL);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
}

void setup_peripheral(void)
{
    debug_uart = NULL;
    bound_datalink = NULL;

    GPIO_setPinConfig(DEVICE_GPIO_CFG_LED1);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);
    GPIO_writePin(DEVICE_GPIO_PIN_LED1, 1U);

    xplt_configure_scia();
    xplt_configure_sample_timer();
    Interrupt_register(INT_SCIA_RX, &xplt_scia_rx_isr);
    Interrupt_register(INT_TIMER0, &xplt_cpu_timer0_isr);
    Interrupt_enable(INT_SCIA_RX);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

void xplt_dl_bind(gmp_datalink_t* datalink)
{
    bound_datalink = datalink;
}

void xplt_dl_poll_rx(void)
{
    Interrupt_disable(INT_SCIA_RX);
    xplt_dl_drain_rx_fifo();
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    Interrupt_enable(INT_SCIA_RX);
}

void xplt_dl_start_tx(gmp_datalink_t* datalink)
{
    SCI_writeCharArray(SCIA_BASE,
                       (const uint16_t*)gmp_dev_dl_get_tx_hw_hdr_ptr(datalink),
                       (uint16_t)gmp_dev_dl_get_tx_hw_hdr_size(datalink));
    if (gmp_dev_dl_get_tx_hw_pld_size(datalink) > 0U)
    {
        SCI_writeCharArray(SCIA_BASE,
                           (const uint16_t*)gmp_dev_dl_get_tx_hw_pld_ptr(datalink),
                           (uint16_t)gmp_dev_dl_get_tx_hw_pld_size(datalink));
    }
    while (SCI_isTransmitterBusy(SCIA_BASE))
    {
    }
    gmp_dev_dl_tx_state_done(datalink);
}

void xplt_start_sample_timer(void)
{
    Interrupt_enable(INT_TIMER0);
    CPUTimer_startTimer(CPUTIMER0_BASE);
}

void xplt_toggle_user_led(void)
{
    GPIO_togglePin(DEVICE_GPIO_PIN_LED1);
}

/** @brief Drain the receive FIFO whenever at least eight characters are ready. */
__interrupt void xplt_scia_rx_isr(void)
{
    xplt_dl_drain_rx_fifo();
    if ((SCI_getRxStatus(SCIA_BASE) & SCI_RXSTATUS_OVERRUN) != 0U)
    {
        SCI_clearOverflowStatus(SCIA_BASE);
        SCI_resetRxFIFO(SCIA_BASE);
    }
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF | SCI_INT_RXERR);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

/** @brief Generate one waveform sample and advance the GMP millisecond tick. */
__interrupt void xplt_cpu_timer0_isr(void)
{
    user_dsa_timer_step();
    gmp_step_system_tick();
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
