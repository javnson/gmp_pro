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

//=================================================================================================
// peripheral setup function

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{

    // Setup Debug Uart
    debug_uart = IRIS_UART_USB_BASE;

    // Test print function
    gmp_base_print(TEXT_STRING("Hello World!\r\n"));
    asm(" RPT #255 || NOP");

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
}

//=================================================================================================
// ADC Interrupt ISR and controller related function

// ADC interrupt
interrupt void MainISR(void)
{
    GPIO_WritePin(MONITOR_IO, 0);
    //
    // call GMP ISR  Controller operation callback function
    //
    gmp_base_ctl_step();

    //
    // Call GMP Timer
    //
    gmp_step_system_tick();

    //
    // Blink LED
    //
    if (gmp_base_get_system_tick() % 1000 < 500)
        GPIO_WritePin(SYSTEM_LED, 0);
    else
        GPIO_WritePin(SYSTEM_LED, 1);

    GPIO_WritePin(MONITOR_IO, 1);

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if (true == ADC_getInterruptOverflowStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_ADCA_1_INTERRUPT_ACK_GROUP);
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
interrupt void INT_IRIS_CAN_0_ISR(void)
{
    uint32_t status = CAN_getInterruptCause(IRIS_CAN_BASE);

    uint16_t rx_data[4];
    can_data_t recv_content[2];

    if (status == 1)
    {
        CAN_readMessage(IRIS_CAN_BASE, 1, rx_data);
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
        CAN_readMessage(IRIS_CAN_BASE, 2, (uint16_t*)recv_content);
        CAN_clearInterruptStatus(CANA_BASE, 2);

        //        // set target value
        //#if BUILD_LEVEL == 1
        //        // For level 1 Set target voltage
        //        ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, float2ctrl((float)recv_content[0].i32 / CAN_SCALE_FACTOR),
        //                                         float2ctrl((float)recv_content[1].i32 / CAN_SCALE_FACTOR));
        //
        //#endif // BUILD_LEVEL
    }

    //
    // Clear the interrupt flag
    //
    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_CAN_0_INTERRUPT_ACK_GROUP);
}

interrupt void INT_IRIS_CAN_1_ISR(void)
{
    // Nothing here

    //
    // Clear the interrupt flag
    //
    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT1);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_CAN_1_INTERRUPT_ACK_GROUP);
}

void send_monitor_data(void)
{
    uint16_t rx_raw[4];
    can_data_t tran_content[2];

    // 0x201: Monitor Motor Current
    tran_content[0].i32 = (int32_t)(mtr_ctrl.idq0.dat[phase_d] * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(mtr_ctrl.idq0.dat[phase_q] * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 4, 8, (uint16_t*)tran_content);

    //0x202: Monitor inverter voltage
    tran_content[0].i32 = (int32_t)(mtr_ctrl.vdq0.dat[phase_d] * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(mtr_ctrl.vdq0.dat[phase_q] * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 5, 8, (uint16_t*)tran_content);

    // 0x203: Monitor Velocity following
    tran_content[0].i32 = (int32_t)(motion_ctrl.spd_if->speed * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(motion_ctrl.target_velocity] * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 6, 8, (uint16_t*)tran_content);

    // 0x204: TODO Monitor elec-position following
    tran_content[0].i32 = (int32_t)(motion_ctrl.pos_if->position * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(motion_ctrl.target_angle * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 7, 8, (uint16_t*)tran_content);

    // 0x205: Monitor DC Voltage / ISR tick
    tran_content[0].i32 = (int32_t)(mtr_ctrl.udc * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(mtr_ctrl.isr_tick);
    CAN_sendMessage(IRIS_CAN_BASE, 8, 8, (uint16_t*)tran_content);

    // 0x206: ia,ib
    tran_content[0].i32 = (int32_t)(mtr_ctrl.iuvw.dat[phase_U] * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(mtr_ctrl.iuvw.dat[phase_V] * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 9, 8, (uint16_t*)tran_content);

    // 0x207: ualpha, ubeta
    tran_content[0].i32 = (int32_t)(mtr_ctrl.vab0.dat[phase_alpha] * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(inv_ctrl.vab0.dat[phase_beta] * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 10, 8, (uint16_t*)tran_content);
}

// a local small cache size, capable of covering the depth of the hardware FIFO (typically 16 bytes)
#define ISR_LOCAL_BUF_SIZE 16

void at_device_flush_rx_buffer()
{
    uint16_t fifoLevel;
    uint16_t rxBuf[ISR_LOCAL_BUF_SIZE];

    // Read all FIFO content
    while ((fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE)) > 0)
    {
        // Get data
        SCI_readCharArray(IRIS_UART_USB_BASE, rxBuf, fifoLevel);

        // send to AT device
        at_device_rx_isr(&at_dev, (char*)rxBuf, fifoLevel);
    }
}

interrupt void INT_IRIS_UART_USB_RX_ISR(void)
{
    uint32_t rxStatus;

    // clear receive FIFO
    at_device_flush_rx_buffer();

    // Fault reaction
    rxStatus = SCI_getRxStatus(IRIS_UART_USB_BASE);

    if (rxStatus & SCI_RXSTATUS_OVERRUN)
    {
        // The Overrun flag indicates that the RX FIFO is full and data was lost.
        // C2000 DriverLib typically clears this by writing to the RXFFOVRCLR bit.
        // If there is no direct API, use HWREG or check if resetRxFIFO clears the Overrun.

        // Suggestion: Only perform a Reset if an actual Overflow occurs.
        // Ordinary errors do not require a FIFO reset.
        SCI_clearOverflowStatus(IRIS_UART_USB_BASE);

        // Resetting the FIFO is an option to ensure a clean state after an overflow.
        // SCI_resetRxFIFO(IRIS_UART_USB_BASE);
    }

    if (rxStatus & SCI_RXSTATUS_ERROR)
    {
        // Handle Frame Error / Parity Error (e.g., receiving 0xFF or Break signal).
        // Reading the data register usually automatically clears these error flags.
        // We only need to ensure the SCI status flags are cleared.
        // It is NOT necessary to reset the FIFO for these errors, doing so would lose valid data.

        // Therefore, DO NOT call SCI_resetRxFIFO() here !!!
    }

    //
    // Clear the interrupt flag
    //
    SCI_clearInterruptStatus(IRIS_UART_USB_BASE, SCI_INT_RXFF | SCI_INT_RXERR);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_UART_USB_RX_INTERRUPT_ACK_GROUP);
}
