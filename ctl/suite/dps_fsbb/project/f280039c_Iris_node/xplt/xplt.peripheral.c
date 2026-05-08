//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all definitions of peripheral objects in this file.
// User should implement the peripheral objects initialization in setup_peripheral function.
// This file is platform-related.
//

// GMP basic core header
#include <gmp_core.h>

#include "user_main.h"
#include "ctrl_main.h" // Includes FSBB modules, ADC structures, and ctrl_settings.h
#include <xplt.peripheral.h>
#include <ctl/component/dsa/dsa_trigger.h>

//=================================================================================================
// Definitions of Peripheral

// dlog DSA objects
basic_trigger_t trigger;
#define DLOG_MEM_LENGTH 100

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
    gmp_base_print(TEXT_STRING("Hello FSBB World!\r\n"));
    asm(" RPT #255 || NOP");

    user_led = SYSTEM_LED;

    // ---------------------------------------------------------
    // 1. Initialize Input Voltage ADC Channel (V_in)
    // ---------------------------------------------------------
    ctl_init_adc_channel(
        &adc_v_in,
        // ADC gain calculation
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_VIN_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        // ADC bias calculation
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_VIN_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    // ---------------------------------------------------------
    // 2. Initialize Output Voltage ADC Channel (V_out)
    // ---------------------------------------------------------
    ctl_init_adc_channel(
        &adc_v_out,
        // ADC gain calculation
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_VOUT_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        // ADC bias calculation
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_VOUT_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    // ---------------------------------------------------------
    // 3. Initialize Inductor Current ADC Channel (I_L)
    // ---------------------------------------------------------
    ctl_init_adc_channel(
        &adc_i_L,
        // ADC gain calculation
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INDUCTOR_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        // ADC bias calculation
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INDUCTOR_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    // ---------------------------------------------------------
    // 4. Initialize Load Current ADC Channel (I_load) - For Feedforward
    // ---------------------------------------------------------
    ctl_init_adc_channel(
        &adc_i_load,
        // ADC gain calculation
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_LOAD_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        // ADC bias calculation
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_LOAD_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    // Initialize data logger module
    dsa_init_basic_trigger(&trigger, DLOG_MEM_LENGTH);
}

//=================================================================================================
// ADC Interrupt ISR and controller related function

// Main High-Frequency Interrupt (Triggered by ePWM/ADC at 20kHz)
interrupt void MainISR(void)
{
    // NOTE: The hardware ADC reading and step functions are encapsulated
    // in `ctl_input_callback()` inside `xplt.ctl_interface.h`.
    // The GMP base controller handles the pipeline automatically.

    // Call GMP ISR Controller operation callback function (invokes ctl_dispatch)
    gmp_base_ctl_step();

    // Call GMP System Timer
    gmp_step_system_tick();

    // Blink LED (Heartbeat at 1Hz)
    if (gmp_base_get_system_tick() % 1000 < 500)
        GPIO_WritePin(SYSTEM_LED, 0);
    else
        GPIO_WritePin(SYSTEM_LED, 1);

    // Clear the interrupt flag
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    // Check if overflow has occurred
    if (true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    }

    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INT_ADCA_1_INTERRUPT_ACK_GROUP);
}

void reset_controller(void)
{
    int i = 0;

    GPIO_WritePin(PWM_RESET_PORT, 0);
    for(i = 0; i < 10000; ++i);
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

// CAN interrupt 0 (Receiver)
interrupt void INT_IRIS_CAN_0_ISR(void)
{
    uint32_t status = CAN_getInterruptCause(IRIS_CAN_BASE);

    uint16_t rx_data[4];
    can_data_t recv_content[2];

    // Mailbox 1: System Enable/Disable Control
    if (status == 1)
    {
        CAN_readMessage(IRIS_CAN_BASE, 1, rx_data);
        CAN_clearInterruptStatus(CANA_BASE, 1);

        if (rx_data[0] == 1)
        {
            cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);
        }
        else if (rx_data[0] == 0)
        {
            cia402_send_cmd(&cia402_sm, CIA402_CMD_DISABLE_VOLTAGE);
        }
    }
    // Mailbox 2: Reference Commands (V_out Setpoint & I_limit)
    else if (status == 2)
    {
        CAN_readMessage(IRIS_CAN_BASE, 2, (uint16_t*)recv_content);
        CAN_clearInterruptStatus(CANA_BASE, 2);

        // Map received CAN payload to user setpoints (Converted to PU)
        g_v_out_ref_user = float2ctrl((float)recv_content[0].i32 / CAN_SCALE_FACTOR);
        g_i_limit_user   = float2ctrl((float)recv_content[1].i32 / CAN_SCALE_FACTOR);
    }

    // Clear the global interrupt flag
    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT0);
    Interrupt_clearACKGroup(INT_IRIS_CAN_0_INTERRUPT_ACK_GROUP);
}

interrupt void INT_IRIS_CAN_1_ISR(void)
{
    // Clear the global interrupt flag
    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT1);
    Interrupt_clearACKGroup(INT_IRIS_CAN_1_INTERRUPT_ACK_GROUP);
}

// Telemetry Transmit Function (Executed in 5ms low-frequency task)
void send_monitor_data(void)
{
    can_data_t tran_content[2];

    // 0x201: Monitor Input & Output Voltage (PU)
    tran_content[0].i32 = (int32_t)(adc_v_in.control_port.value * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(adc_v_out.control_port.value * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 4, 8, (uint16_t*)tran_content);

    // 0x202: Monitor Inductor Current & Load Current (PU)
    tran_content[0].i32 = (int32_t)(adc_i_L.control_port.value * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(adc_i_load.control_port.value * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 5, 8, (uint16_t*)tran_content);

    // 0x203: Monitor Voltage Reference (Ramped) & Final Equivalent PWM Voltage (PU)
    tran_content[0].i32 = (int32_t)(dcdc_core.v_out_ref * CAN_SCALE_FACTOR);
    tran_content[1].i32 = (int32_t)(dcdc_core.v_pwm_req * CAN_SCALE_FACTOR);
    CAN_sendMessage(IRIS_CAN_BASE, 6, 8, (uint16_t*)tran_content);

    // 0x204: Monitor State Machine Status Word & Active Errors
    tran_content[0].i32 = (int32_t)(cia402_sm.state_word.all);
    tran_content[1].i32 = (int32_t)(protection.active_errors);
    CAN_sendMessage(IRIS_CAN_BASE, 7, 8, (uint16_t*)tran_content);
}

#if BOARD_SELECTION == GMP_IRIS

interrupt void INT_IRIS_UART_RS232_RX_ISR(void)
{
    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INT_IRIS_UART_RS232_RX_INTERRUPT_ACK_GROUP);
}

#endif // BOARD_SELECTION == GMP_IRIS

//=================================================================================================
// Debug interface (Datalink Protocol)

// A local small cache size, capable of covering the depth of the hardware FIFO
#define ISR_LOCAL_BUF_SIZE 16

extern gmp_datalink_t dl;

void flush_dl_tx_buffer()
{
    // Send head
    gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl), gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10);

    // Send data body, if necessary
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
    {
        gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_pld_ptr(&dl), gmp_dev_dl_get_tx_hw_pld_size(&dl), 10);
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

        // Lock-free ring queue pushed into the protocol stack (O(1))
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
