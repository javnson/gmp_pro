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

// user main header
#include "user_main.h"

#include <xplt.peripheral.h>


// console 
#include <conio.h>

//////////////////////////////////////////////////////////////////////////
// definitions of peripheral
//

// inverter side voltage feedback
tri_ptr_adc_channel_t uuvw;
adc_gt uuvw_src[3];

// inverter side current feedback
tri_ptr_adc_channel_t iuvw;
adc_gt iuvw_src[3];

// grid side voltage feedback
tri_ptr_adc_channel_t vabc;
adc_gt vabc_src[3];

// grid side current feedback
tri_ptr_adc_channel_t iabc;
adc_gt iabc_src[3];

// DC bus current & voltage feedback
ptr_adc_channel_t udc;
adc_gt udc_src;
ptr_adc_channel_t idc;
adc_gt idc_src;

// PWM output channel
pwm_tri_channel_t pwm_out;

//ptr_adc_channel_t inv_adc[INV_ADC_SENSOR_NUMBER];
//pwm_tri_channel_t pwm_out;
//
//pwm_channel_t inv_pwm_out[3];

//////////////////////////////////////////////////////////////////////////
// peripheral setup function
//

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

    // grid side ADC
    ctl_init_tri_ptr_adc_channel(
        &vabc, vabc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        &iabc, iabc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_CURRENT_BIAS),
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

    // output channel
    ctl_init_pwm_tri_channel(&pwm_out, 0, CTRL_PWM_CMP_MAX);

    //
    // attach
    //
    ctl_attach_three_phase_inv(
        // inv controller
        &inv_ctrl,
        // output PWM wave
        &pwm_out.raw,
        // udc, idc
        &udc.control_port, &idc.control_port,
        // grid side iabc, vabc
        &iabc.control_port, &vabc.control_port,
        // inverter siede iuvw, uuvw
        &iuvw.control_port, &uuvw.control_port);
}

// ---------------- 核心移植代码 ----------------

// 为了防止卡住，在Windows平台上的buffer留大一些
#define ISR_LOCAL_BUF_SIZE 1024

// Using Windows console to simulate UART
void at_device_flush_rx_buffer()
{
    uint16_t fifoLevel = 0;
    uint16_t rxBuf[ISR_LOCAL_BUF_SIZE];

    // 使用while一次性读取FIFO中的所有内容
    while (_kbhit())
    {
        //_getch() 读取字符但不回显，也不等待回车
        int ch = _getch();

        // 处理特殊键 (例如方向键会产生两个码: 0/0xE0 和 键码)
        // 这里我们简单处理，只接收普通 ASCII
        if (ch == 0 || ch == 0xE0)
        {
            _getch(); // 读走无效部分
            continue;
        }

        // 【重要】Windows控制台输入不自动回显，手动回显以便用户看到自己打的字
        putchar(ch);

        // 读取数据
        rxBuf[fifoLevel++] = (uint16_t)ch;
    }

    // 推送给设备
    if (fifoLevel > 0)
    {
        at_device_rx_isr(&at_dev, (char*)rxBuf, fifoLevel);
    }
}

// do nothing here
void send_monitor_data(void)
{

}
