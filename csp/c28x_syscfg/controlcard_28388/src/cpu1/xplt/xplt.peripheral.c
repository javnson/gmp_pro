/** @file xplt.peripheral.c CPU1 board, timer and SCI binding. */

#include <gmp_core.h>
#include "device.h"
#include "driverlib.h"
#include "board.h"
#include "user_main.h"
#include <xplt.peripheral.h>

#pragma DATA_SECTION(cpu1_to_cpu2_command, "GMP_MSGRAM_CPU1_TO_CPU2")
volatile gmp_wave_command_t cpu1_to_cpu2_command;
#pragma DATA_SECTION(cpu2_to_cpu1_snapshot, "GMP_MSGRAM_CPU2_TO_CPU1")
volatile gmp_wave_snapshot_t cpu2_to_cpu1_snapshot;

void cpu1_board_initialize_and_handoff(void);

static void xplt_cpu1_timer_init(void);

void xplt_cpu1_dl_receive(gmp_datalink_t *datalink)
{
    byte_gt buffer[16];
    size_gt count;
    size_gt index;
    uint16_t status = SCI_getRxStatus(SCIA_BASE);
    if ((status & (SCI_RXSTATUS_OVERRUN | SCI_RXSTATUS_FRAMING |
                   SCI_RXSTATUS_PARITY | SCI_RXSTATUS_BREAK)) != 0U)
    {
        SCI_performSoftwareReset(SCIA_BASE);
        SCI_resetRxFIFO(SCIA_BASE);
        SCI_performSoftwareReset(SCIA_BASE);
        gmp_dev_dl_request_rx_reset(datalink);
        return;
    }
    count = (size_gt)SCI_getRxFIFOStatus(SCIA_BASE);
    if (count > sizeof(buffer) / sizeof(buffer[0]))
        count = sizeof(buffer) / sizeof(buffer[0]);
    for (index = 0U; index < count; ++index)
        buffer[index] = (byte_gt)(SCI_readCharNonBlocking(SCIA_BASE) & 0xFFU);
    if (count != 0U)
        gmp_dev_dl_push_str(datalink, buffer, count);
}

fast_gt xplt_cpu1_dl_send(gmp_datalink_t *datalink)
{
    SCI_writeCharArray(SCIA_BASE,
        (const uint16_t *)gmp_dev_dl_get_tx_hw_hdr_ptr(datalink),
        (uint16_t)gmp_dev_dl_get_tx_hw_hdr_size(datalink));
    if (gmp_dev_dl_get_tx_hw_pld_size(datalink) != 0U)
        SCI_writeCharArray(SCIA_BASE,
            (const uint16_t *)gmp_dev_dl_get_tx_hw_pld_ptr(datalink),
            (uint16_t)gmp_dev_dl_get_tx_hw_pld_size(datalink));
    while (SCI_isTransmitterBusy(SCIA_BASE))
    {
    }
    return 1;
}

void xplt_cpu1_toggle_status_led(void)
{
    GPIO_togglePin(LED1);
}

__interrupt static void xplt_cpu1_timer_isr(void)
{
    user_cpu1_control_step();
    gmp_step_system_tick();
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

static void xplt_cpu1_sci_init(void)
{
    SCI_performSoftwareReset(SCIA_BASE);
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, GMP_F28388D_SCI_BAUDRATE,
                  SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE);
    SCI_resetChannels(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX1);
    SCI_enableModule(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);
}

static void xplt_cpu1_timer_init(void)
{
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    CPUTimer_setPeriod(CPUTIMER0_BASE, (DEVICE_SYSCLK_FREQ / 1000UL) - 1UL);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_register(INT_TIMER0, &xplt_cpu1_timer_isr);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_startTimer(CPUTIMER0_BASE);
}

void setup_peripheral(void)
{
    cpu1_board_initialize_and_handoff();
    xplt_cpu1_sci_init();
    xplt_cpu1_timer_init();
}
