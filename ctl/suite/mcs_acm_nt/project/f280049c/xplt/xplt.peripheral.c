//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all definitions of peripheral objects in this file.
//
// User should implement the peripheral objects initialization in setup_peripheral function.
//
// This file is platform-related.
//

// GMP basic core header
#include <gmp_core.h>
#include <ctl/component/dsa/dsa_dl_scope.h>

#include "user_main.h"
#include <xplt.peripheral.h>


//=================================================================================================
// definitions of peripheral

// inverter side voltage feedback
tri_ptr_adc_channel_t uuvw;
adc_gt uuvw_src[3];

// inverter side current feedback
tri_ptr_adc_channel_t iuvw;
adc_gt iuvw_src[3];

// DC bus current & voltage feedback
ptr_adc_channel_t udc;
adc_gt udc_src;
ptr_adc_channel_t idc;
adc_gt idc_src;

// GPIO port
extern gpio_halt user_led;

#if defined ENABLE_GMP_DL_PIL_SIM
/** @brief Virtual enable state sent to the Simulink plant without energizing hardware. */
volatile fast_gt pil_output_enabled = 0;
#endif

//=================================================================================================
// peripheral setup function

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    // Setup Debug Uart
    debug_uart = LAUNCHXL_UART_USB_BASE;
    SCI_setBaud(LAUNCHXL_UART_USB_BASE, DEVICE_LSPCLK_FREQ, GMP_DL_UART_BAUDRATE);
    SCI_enableInterrupt(LAUNCHXL_UART_USB_BASE, SCI_INT_RXFF | SCI_INT_RXERR);

    // Test print function
    gmp_base_print(TEXT_STRING("Hello World!\r\n"));
    asm(" RPT #255 || NOP");

    user_led = SYSTEM_LED;

    // inverter side ADC
    ctl_init_tri_ptr_adc_channel(
        &uuvw, uuvw_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        &iuvw, iuvw_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_ptr_adc_channel(
        &udc, &udc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_ptr_adc_channel(
        &idc, &idc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    /* Controller-port ownership belongs to ctl_init().  In particular, an
     * ACIM current core must receive rotor-flux field angle/synchronous speed,
     * never the raw rotor encoder pair attached by the old PMSM template. */

}

//=================================================================================================
// ADC Interrupt ISR and controller related function

// ADC interrupt
interrupt void MainISR(void)
{
    GPIO_WritePin(MONITOR_IO, 0);

    /**
     * Keep physical control entirely outside an SDPE-enabled PIL build.
     * The PIL Data Link STEP request is the sole owner of ctl_dispatch().
     */
#if !defined ENABLE_GMP_DL_PIL_SIM
    gmp_base_ctl_step();
    user_step_dl_scope();
#endif

    //
    // Call GMP Timer
    //
    gmp_step_system_tick();

    //
    // Blink LED
    //
//    if (gmp_base_get_system_tick() % 1000 < 500)
//        GPIO_WritePin(SYSTEM_LED, 0);
//    else
//        GPIO_WritePin(SYSTEM_LED, 1);

    GPIO_WritePin(MONITOR_IO, 1);

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(LAUNCHXL_ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if (true == ADC_getInterruptOverflowStatus(LAUNCHXL_ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(LAUNCHXL_ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(LAUNCHXL_ADCA_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_LAUNCHXL_ADCA_1_INTERRUPT_ACK_GROUP);
}

//=================================================================================================
// communication functions and interrupt functions here

// 10000 -> 1.0
#define CAN_SCALE_FACTOR 10000

// 32 bit union
typedef union {
    int32_t i32;
    uint16_t u16[2];
} can_data_t;

// CAN interrupt
interrupt void INT_LAUNCHXL_CAN_0_ISR(void)
{
    uint32_t status = CAN_getInterruptCause(LAUNCHXL_CAN_BASE);

    uint16_t rx_data[4];
    can_data_t recv_content[2];

    if (status == 1)
    {
        CAN_readMessage(LAUNCHXL_CAN_BASE, 1, rx_data);
        CAN_clearInterruptStatus(CANA_BASE, 1);

        // Control Flag, Enable System
        if (rx_data[0] == 1)
        {
            cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);
        }
        if (rx_data[0] == 0)
        {
            cia402_send_cmd(&cia402_sm, CIA402_CMD_DISABLE_VOLTAGE);
        }
    }
    else if (status == 2)
    {
        CAN_readMessage(LAUNCHXL_CAN_BASE, 2, (uint16_t*)recv_content);
        CAN_clearInterruptStatus(CANA_BASE, 2);

        //        // set target value
        //#if BUILD_LEVEL == 1
        //        // For level 1 Set target voltage
        //        ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, real2ctrl((float)recv_content[0].i32 / CAN_SCALE_FACTOR),
        //                                         real2ctrl((float)recv_content[1].i32 / CAN_SCALE_FACTOR));
        //
        //#endif // BUILD_LEVEL
    }

    //
    // Clear the interrupt flag
    //
    CAN_clearGlobalInterruptStatus(LAUNCHXL_CAN_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_LAUNCHXL_CAN_0_INTERRUPT_ACK_GROUP);
}

interrupt void INT_LAUNCHXL_CAN_1_ISR(void)
{
    // Nothing here

    //
    // Clear the interrupt flag
    //
    CAN_clearGlobalInterruptStatus(LAUNCHXL_CAN_BASE, CAN_GLOBAL_INT_CANINT1);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_LAUNCHXL_CAN_1_INTERRUPT_ACK_GROUP);
}

void send_monitor_data(void)
{
    can_data_t tran_content[2];

    // 0x201: Monitor Motor Current
    tran_content[0].i32 = (int32_t)(mtr_ctrl.idq0.dat[phase_d] * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(mtr_ctrl.idq0.dat[phase_q] * CAN_SCALE_FACTOR);
    CAN_sendMessage(LAUNCHXL_CAN_BASE, 4, 8, (uint16_t*)tran_content);

    //0x202: Monitor inverter voltage
    tran_content[0].i32 = (int32_t)(mtr_ctrl.vdq_out.dat[phase_d] * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(mtr_ctrl.vdq_out.dat[phase_q] * CAN_SCALE_FACTOR);
    CAN_sendMessage(LAUNCHXL_CAN_BASE, 5, 8, (uint16_t*)tran_content);

    // 0x203: Monitor Velocity following
//    tran_content[0].i32 = (int32_t)(motion_ctrl.spd_if->speed * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(motion_ctrl.target_velocity * CAN_SCALE_FACTOR);
    CAN_sendMessage(LAUNCHXL_CAN_BASE, 6, 8, (uint16_t*)tran_content);

    // 0x204: TODO Monitor elec-position following
//    tran_content[0].i32 = (int32_t)(motion_ctrl.pos_if->position * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(motion_ctrl.target_angle * CAN_SCALE_FACTOR);
    CAN_sendMessage(LAUNCHXL_CAN_BASE, 7, 8, (uint16_t*)tran_content);

    // 0x205: Monitor DC Voltage / ISR tick
    tran_content[0].i32 = (int32_t)(mtr_ctrl.udc * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(mtr_ctrl.isr_tick);
    CAN_sendMessage(LAUNCHXL_CAN_BASE, 8, 8, (uint16_t*)tran_content);

    // 0x206: ia,ib
    tran_content[0].i32 = (int32_t)(mtr_ctrl.iuvw.dat[phase_U] * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(mtr_ctrl.iuvw.dat[phase_V] * CAN_SCALE_FACTOR);
    CAN_sendMessage(LAUNCHXL_CAN_BASE, 9, 8, (uint16_t*)tran_content);

    // 0x207: ualpha, ubeta
    tran_content[0].i32 = (int32_t)(mtr_ctrl.vab0.dat[phase_alpha] * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(mtr_ctrl.vab0.dat[phase_beta] * CAN_SCALE_FACTOR);
    CAN_sendMessage(LAUNCHXL_CAN_BASE, 10, 8, (uint16_t*)tran_content);
}

//=================================================================================================
// Debug interface

extern gmp_datalink_t dl;

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

////
