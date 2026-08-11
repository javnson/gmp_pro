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
#include <gmp_core.hpp>

// user main header
#include "user_main.h"
#include <xplt.peripheral.h>

#include <ctrl_rt_trace.h>

// console
#include <conio.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

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

// Offline-identification waveform capture buffer.
ctrl_gt dsa_buffer[DSA_BUFFER_SIZE];

// Trace RT objects
typedef enum _tag_trace_rt_nodes
{
    TRT_TEST = 0,
    TRT_NODE_NUMBER
} trace_rt_nodes;

trace_rt_node_t* trt_node[TRT_NODE_NUMBER];

//=================================================================================================
// peripheral setup function

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    //
    // input channel
    //

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

    //
    // attach
    //
#if BUILD_LEVEL <= 2
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &rg.enc, &spd_enc.encif);
#else  // BUILD_LEVEL
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &pos_enc.encif, &spd_enc.encif);
#endif // BUILD_LEVEL

    //
    // Trace RT ports
    //
    trt_node[TRT_TEST] = trace_rt_register_node(&trace_rt_context, "pwm_out_A", TRT_TYPE_DOUBLE);
}

//=================================================================================================
// communication functions and interrupt functions here

// Use a local buffer to keep Windows console input non-blocking.
#define ISR_LOCAL_BUF_SIZE 1024

// Using Windows console to simulate UART
void at_device_flush_rx_buffer()
{
    uint16_t fifoLevel = 0;
    uint16_t rxBuf[ISR_LOCAL_BUF_SIZE];

    // Drain all currently available console characters.
    while (_kbhit())
    {
        // _getch() reads one character without waiting for Enter.
        int ch = _getch();

        // Discard the second byte of an extended key sequence.
        // Only ordinary ASCII input is forwarded.
        if (ch == 0 || ch == 0xE0)
        {
            _getch();
            continue;
        }

        // Echo the character because console input does not echo automatically.
        putchar(ch);

        // Store the received character.
        rxBuf[fifoLevel++] = (uint16_t)ch;
    }

    // Forward the accumulated characters to the AT device.
    if (fifoLevel > 0)
    {
        at_device_rx_isr(&at_dev, (char*)rxBuf, fifoLevel);
    }
}

// Execute RT monitor
void send_monitor_data(void)
{
    //gmp_trace_rt_log_double(trt_node[TRT_TEST], inv_ctrl.isr_tick, inv_ctrl.vab0.dat[phase_A]);
}

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus
