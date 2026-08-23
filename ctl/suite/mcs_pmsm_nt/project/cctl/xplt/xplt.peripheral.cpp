/**
 * @file xplt.peripheral.cpp
 * @brief Controller-visible register and channel storage for MCU simulation.
 */

#include <gmp_core.h>
#include <csp.general.h>
#include <xplt.peripheral.h>

extern "C"
{

tri_ptr_adc_channel_t uuvw;
adc_gt uuvw_src[3];
tri_ptr_adc_channel_t iuvw;
adc_gt iuvw_src[3];
ptr_adc_channel_t udc;
adc_gt udc_src;
ptr_adc_channel_t idc;
adc_gt idc_src;

adc_gt cctl_adc_result[CCTL_ADC_COUNT];
uint32_t cctl_encoder_position;
pwm_gt cctl_pwm_compare[3];
extern gmp_datalink_t dl;

/** @copydoc setup_peripheral */
void setup_peripheral(void)
{
    ctl_init_tri_ptr_adc_channel(&uuvw,
        uuvw_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_BIAS),
        CCTL_SIM_ADC_RESOLUTION_BITS,
        24);

    ctl_init_tri_ptr_adc_channel(&iuvw,
        iuvw_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_BIAS),
        CCTL_SIM_ADC_RESOLUTION_BITS,
        24);

    ctl_init_ptr_adc_channel(&udc,
        &udc_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_BIAS),
        CCTL_SIM_ADC_RESOLUTION_BITS,
        24);

    ctl_init_ptr_adc_channel(&idc, &idc_src, real2ctrl(1.0f), real2ctrl(0.0f), CCTL_SIM_ADC_RESOLUTION_BITS, 24);

    for (unsigned index = 0; index < CCTL_ADC_COUNT; ++index)
        cctl_adc_result[index] = 0;
    cctl_encoder_position = 0;
    cctl_pwm_compare[0] = cctl_pwm_compare[1] = cctl_pwm_compare[2] = 0;
}

/**
 * @brief Simulated ADC ISR entry for the hosted MCU.
 *
 * The peripheral model calls this only after ADC result and encoder registers
 * have been latched. Controller calculation therefore remains interrupt-owned,
 * exactly as on the target MCU, and is never run from the plant main loop.
 */
void cctl_adc_interrupt(void)
{
    csp_cctl_notify_controller_interrupt();
    gmp_base_ctl_step();
}

/** @brief Hosted monitor transport placeholder. */
void send_monitor_data(void)
{
}

/** @brief Drain the Viewer-managed virtual UART into the standard Data Link. */
void flush_dl_rx_buffer(void)
{
    byte_gt buffer[64];
    size_gt count;
    do
    {
        count = csp_cctl_datalink_read(buffer, (size_gt)sizeof(buffer));
        if (count != 0U)
            gmp_dev_dl_push_str(&dl, buffer, count);
    } while (count == (size_gt)sizeof(buffer));
}

/** @brief Publish a fully framed standard Data Link response to the Viewer. */
void flush_dl_tx_buffer(void)
{
    const byte_gt* header = gmp_dev_dl_get_tx_hw_hdr_ptr(&dl);
    const size_gt header_size = gmp_dev_dl_get_tx_hw_hdr_size(&dl);
    const byte_gt* payload = gmp_dev_dl_get_tx_hw_pld_ptr(&dl);
    const size_gt payload_size = gmp_dev_dl_get_tx_hw_pld_size(&dl);
    (void)csp_cctl_datalink_write(header, header_size);
    if (payload_size != 0U)
        (void)csp_cctl_datalink_write(payload, payload_size);
}

/** @brief Hosted GPIO direction placeholder. */
ec_gt gmp_hal_gpio_set_dir(gpio_halt, gpio_dir_et)
{
    return GMP_EC_OK;
}

/** @brief Hosted GPIO output placeholder. */
ec_gt gmp_hal_gpio_write(gpio_halt, fast_gt)
{
    return GMP_EC_OK;
}

/** @brief Hosted GPIO input placeholder. */
fast_gt gmp_hal_gpio_read(gpio_halt)
{
    return 0;
}

} // extern "C"
