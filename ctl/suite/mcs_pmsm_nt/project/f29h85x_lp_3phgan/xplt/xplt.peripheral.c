/** Hardware binding for LAUNCHXL-F29H85X + BOOSTXL-3PHGANINV. */

#include <gmp_core.h>
#include <ctl/component/dsa/dsa_dl_scope.h>
#include "user_main.h"
#include <xplt.peripheral.h>

tri_ptr_adc_channel_t uuvw;
adc_gt uuvw_src[3];
tri_ptr_adc_channel_t iuvw;
adc_gt iuvw_src[3];
ptr_adc_channel_t udc;
adc_gt udc_src;
ptr_adc_channel_t idc;
adc_gt idc_src;

extern gpio_halt user_led;
extern gmp_datalink_t dl;

void setup_peripheral(void)
{
    debug_uart = DEBUG_UART_BASE;
    user_led = SYSTEM_LED;
    gmp_c29x_set_tick_divider((uint32_t)DSP_C2000_DSP_TIME_DIV);

    ctl_force_physical_output_safe();

    ctl_init_tri_ptr_adc_channel(
        &uuvw, uuvw_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
                              CTRL_INVERTER_VOLTAGE_SENSITIVITY,
                              CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF,
                                     CTRL_INVERTER_VOLTAGE_BIAS),
        12, 24);
    ctl_init_tri_ptr_adc_channel(
        &iuvw, iuvw_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
                              CTRL_INVERTER_CURRENT_SENSITIVITY,
                              CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF,
                                     CTRL_INVERTER_CURRENT_BIAS),
        12, 24);
    ctl_init_ptr_adc_channel(
        &udc, &udc_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
                              CTRL_DC_VOLTAGE_SENSITIVITY,
                              CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF,
                                     CTRL_DC_VOLTAGE_BIAS),
        12, 24);
    ctl_init_ptr_adc_channel(
        &idc, &idc_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
                              CTRL_DC_CURRENT_SENSITIVITY,
                              CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF,
                                     CTRL_DC_CURRENT_BIAS),
        12, 24);

#if BUILD_LEVEL <= 2
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port,
                             &rg.enc, &spd_enc.encif);
#else
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port,
                             &pos_enc.encif, &spd_enc.encif);
#endif
}

void motor1CtrlISR(void)
{
    gmp_base_ctl_step();
    user_step_dl_scope();
    gmp_step_system_tick();

    ADC_clearInterruptStatus(CONTROL_ISR_ADC_BASE, ADC_INT_NUMBER1);
    if (ADC_getInterruptOverflowStatus(CONTROL_ISR_ADC_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(CONTROL_ISR_ADC_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(CONTROL_ISR_ADC_BASE, ADC_INT_NUMBER1);
    }
}

void flush_dl_tx_buffer(void)
{
    (void)gmp_hal_uart_write(DEBUG_UART_BASE,
                             gmp_dev_dl_get_tx_hw_hdr_ptr(&dl),
                             gmp_dev_dl_get_tx_hw_hdr_size(&dl), 50U);
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) != 0U)
    {
        (void)gmp_hal_uart_write(DEBUG_UART_BASE,
                                 gmp_dev_dl_get_tx_hw_pld_ptr(&dl),
                                 gmp_dev_dl_get_tx_hw_pld_size(&dl), 50U);
    }
}

void flush_dl_rx_buffer(void)
{
    data_gt byte;
    size_gt count;
    while (gmp_hal_uart_get_rx_available(DEBUG_UART_BASE) != 0U)
    {
        if (gmp_hal_uart_read(DEBUG_UART_BASE, &byte, 1U, 0U, &count) != GMP_EC_OK)
            break;
        gmp_dev_dl_push_byte(&dl, byte);
    }
}

void send_monitor_data(void)
{
    /* Data Link Scope and Tunable provide monitoring on this baseline target. */
}
