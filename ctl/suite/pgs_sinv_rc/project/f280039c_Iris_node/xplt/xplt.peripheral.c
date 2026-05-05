//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all definitions of peripheral objects in this file.
// User should implement the peripheral objects initialization in setup_peripheral function.
// This file is platform-related.
//

// GMP basic core header
#include <gmp_core.h>

#include "ctl_main.h" // Includes SINV modules, ADC structures, and ctrl_settings.h
#include "user_main.h"
#include <ctl/component/dsa/dsa_trigger.h>
#include <xplt.peripheral.h>

//=================================================================================================
// Definitions of Peripheral

// Grid Voltage Feedback
adc_channel_t adc_v_grid;

// AC Current Feedback
adc_channel_t adc_i_ac;

// DC Bus Voltage Feedback
adc_channel_t adc_v_bus;

// dlog DSA objects
basic_trigger_t trigger;

// dlog variables
ctrl_gt dlog_mem1[DLOG_MEM_LENGTH];
ctrl_gt dlog_mem2[DLOG_MEM_LENGTH];

// GPIO port
extern gpio_halt user_led;

//=================================================================================================
// Peripheral Setup Function

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    // Setup Debug UART
    debug_uart = IRIS_UART_USB_BASE;

    // Test print function
    gmp_base_print(TEXT_STRING("Hello SINV!\r\n"));
    asm(" RPT #255 || NOP");

    user_led = SYSTEM_LED;

    // ---------------------------------------------------------
    // 1. Initialize AC Grid Voltage ADC Channel
    // ---------------------------------------------------------
    ctl_init_adc_channel(&adc_v_grid,
                         // ADC gain
                         ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_AC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
                         // ADC bias
                         ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_AC_VOLTAGE_BIAS),
                         // ADC resolution, IQN
                         12, 24);

    // ---------------------------------------------------------
    // 2. Initialize AC Current ADC Channel
    // ---------------------------------------------------------
    ctl_init_adc_channel(&adc_i_ac,
                         // ADC gain
                         ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_AC_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
                         // ADC bias
                         ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_AC_CURRENT_BIAS),
                         // ADC resolution, IQN
                         12, 24);

    // ---------------------------------------------------------
    // 3. Initialize DC Bus Voltage ADC Channel
    // ---------------------------------------------------------
    ctl_init_adc_channel(&adc_v_bus,
                         // ADC gain
                         ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
                         // ADC bias
                         ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_BIAS),
                         // ADC resolution, IQN
                         12, 24);

    // ---------------------------------------------------------
    // 4. Attach ADC Ports to the RC Core (Zero-Copy Binding)
    // ---------------------------------------------------------
    // This allows the step function inside ctl_dispatch to directly
    // fetch the ADC values from the control_port without parameter passing.
    ctl_attach_sinv_rc(&rc_core, &adc_v_bus.control_port, &adc_v_grid.control_port, &adc_i_ac.control_port);

    // Initialize data logger module
    dsa_init_basic_trigger(&trigger, DLOG_MEM_LENGTH);
}

//=================================================================================================
// ADC Interrupt ISR and controller related function

// ADC interrupt
interrupt void MainISR(void)
{
    // Call GMP ISR Controller operation callback function (invokes ctl_dispatch)
    gmp_base_ctl_step();

    // Call GMP Timer
    gmp_step_system_tick();

    // Blink LED (Heartbeat)
    if (gmp_base_get_system_tick() % 1000 < 500)
        GPIO_WritePin(SYSTEM_LED, 0);
    else
        GPIO_WritePin(SYSTEM_LED, 1);

    // Clear the interrupt flag
    ADC_clearInterruptStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);

    // Check if overflow has occurred
    if (true == ADC_getInterruptOverflowStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);
    }

    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INT_IRIS_ADCA_1_INTERRUPT_ACK_GROUP);
}

void reset_controller(void)
{
    int i = 0;

    GPIO_WritePin(PWM_RESET_PORT, 0);

    for (i = 0; i < 10000; ++i)
        ;

    GPIO_WritePin(PWM_RESET_PORT, 1);
}

//=================================================================================================
// Communication functions and interrupt functions

// 10000 -> 1.0 (Fixed point communication scale)
#define CAN_SCALE_FACTOR 10000

// 32-bit union for C2000 (uint16_t takes 1 word, 32-bit takes 2 words)
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

        // Control Flag, Enable/Disable System
        if (rx_data[0] == 1)
        {
            cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);
        }
        else if (rx_data[0] == 0)
        {
            cia402_send_cmd(&cia402_sm, CIA402_CMD_DISABLE_VOLTAGE);
        }
    }
    else if (status == 2)
    {
        CAN_readMessage(IRIS_CAN_BASE, 2, (uint16_t*)recv_content);
        CAN_clearInterruptStatus(CANA_BASE, 2);

        // Set target User Power References
        // Used in BUILD_LEVEL >= 2 (Grid-tied PQ control)
        g_p_ref_user = float2ctrl((float)recv_content[0].i32 / CAN_SCALE_FACTOR);
        g_q_ref_user = float2ctrl((float)recv_content[1].i32 / CAN_SCALE_FACTOR);
    }

    // Clear the interrupt flag
    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT0);

    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INT_IRIS_CAN_0_INTERRUPT_ACK_GROUP);
}

interrupt void INT_IRIS_CAN_1_ISR(void)
{
    // Nothing here

    // Clear the interrupt flag
    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT1);

    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INT_IRIS_CAN_1_INTERRUPT_ACK_GROUP);
}

void send_monitor_data(void)
{
    can_data_t tran_content[2];

    // 0x201: Monitor Grid Voltage Magnitude & PLL Frequency
    tran_content[0].i32 = (int32_t)(pll.v_mag * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(pll.frequency * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 4, 8, (uint16_t*)tran_content);

    // 0x202: Monitor Average Active(P) and Reactive(Q) Power
    tran_content[0].i32 = (int32_t)(pq_meter.p_avg * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(pq_meter.q_avg * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 5, 8, (uint16_t*)tran_content);

    // 0x203: Monitor RMS Grid Voltage & RMS Inverter Current
    tran_content[0].i32 = (int32_t)(pq_meter.v_rms * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(pq_meter.i_rms * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 6, 8, (uint16_t*)tran_content);

    // 0x204: Monitor Target AC Current (I_ref) & Modulator Duty Cycle Ref
    tran_content[0].i32 = (int32_t)(ref_gen.i_ref_inst * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(rc_core.v_out_ref * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 7, 8, (uint16_t*)tran_content);

    // 0x205: Monitor DC Bus Voltage
    tran_content[0].i32 = (int32_t)(adc_v_bus.control_port.value * CAN_SCALE_FACTOR);
    tran_content[1].i32 = 0; // Reserved
    CAN_sendMessage(IRIS_CAN_BASE, 8, 8, (uint16_t*)tran_content);

    // 0x206: Monitor PLL Phase Angle (Theta) & Real-time Voltage Feedforward
    tran_content[0].i32 = (int32_t)(pll.theta * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(rc_core.u_ff * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 9, 8, (uint16_t*)tran_content);

    // 0x207: Monitor CiA 402 State Machine Status Word & Error Flags
    tran_content[0].i32 = (int32_t)(cia402_sm.state_word.all);
    tran_content[1].i32 = (int32_t)(protection.active_errors);
    CAN_sendMessage(IRIS_CAN_BASE, 10, 8, (uint16_t*)tran_content);
}

#if BOARD_SELECTION == GMP_IRIS

interrupt void INT_IRIS_UART_RS232_RX_ISR(void)
{
    // Nothing here

    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INT_IRIS_UART_RS232_RX_INTERRUPT_ACK_GROUP);
}

#endif // BOARD_SELECTION == GMP_IRIS

//=================================================================================================
// Debug interface

// A local small cache size, capable of covering the depth of the hardware FIFO (typically 16 bytes)
#define ISR_LOCAL_BUF_SIZE 16

extern gmp_datalink_t dl;

void flush_dl_tx_buffer()
{
    // Send head
    gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl), gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10);

    // Send data body, if necessary
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
    {
        gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_pld_ptr(&dl), gmp_dev_dl_get_tx_hw_pld_size(&dl),
                           10);
    }
}

void flush_dl_rx_buffer()
{
    uint16_t fifoLevel;
    data_gt rxBuf[ISR_LOCAL_BUF_SIZE];

    // Read all FIFO messages
    fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE);

    if (fifoLevel > 0)
    {
        SCI_readCharArray(IRIS_UART_USB_BASE, (uint16_t*)rxBuf, fifoLevel);

        // Lock-free ring queue pushed into the protocol stack (very fast, O(1))
        gmp_dev_dl_push_str(&dl, rxBuf, fifoLevel);
    }
}

interrupt void INT_IRIS_UART_USB_RX_ISR(void)
{
    flush_dl_rx_buffer();

    // Deal with overrun
    if (SCI_getRxStatus(IRIS_UART_USB_BASE) & SCI_RXSTATUS_OVERRUN)
    {
        SCI_clearOverflowStatus(IRIS_UART_USB_BASE);
    }

    // Clear interrupt flags
    SCI_clearInterruptStatus(IRIS_UART_USB_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INT_IRIS_UART_USB_RX_INTERRUPT_ACK_GROUP);
}
