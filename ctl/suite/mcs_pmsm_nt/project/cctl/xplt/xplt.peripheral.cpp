/**
 * @file xplt.peripheral.cpp
 * @brief Controller-visible register and channel storage for MCU simulation.
 */

#include <xplt.peripheral.h>
#include <gmp_core.h>

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
uint64_t cctl_controller_tick;

/** @copydoc setup_peripheral */
void setup_peripheral(void)
{
    ctl_init_tri_ptr_adc_channel(
        &uuvw, uuvw_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
                              CTRL_INVERTER_VOLTAGE_SENSITIVITY,
                              CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF,
                                    CTRL_INVERTER_VOLTAGE_BIAS),
        CCTL_SIM_ADC_RESOLUTION_BITS, 24);
    ctl_init_tri_ptr_adc_channel(
        &iuvw, iuvw_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
                              CTRL_INVERTER_CURRENT_SENSITIVITY,
                              CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF,
                                    CTRL_INVERTER_CURRENT_BIAS),
        CCTL_SIM_ADC_RESOLUTION_BITS, 24);
    ctl_init_ptr_adc_channel(
        &udc, &udc_src,
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF,
                              CTRL_DC_VOLTAGE_SENSITIVITY,
                              CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF,
                                    CTRL_DC_VOLTAGE_BIAS),
        CCTL_SIM_ADC_RESOLUTION_BITS, 24);
    ctl_init_ptr_adc_channel(&idc, &idc_src, real2ctrl(1.0f), real2ctrl(0.0f),
                             CCTL_SIM_ADC_RESOLUTION_BITS, 24);

    for (unsigned index = 0; index < CCTL_ADC_COUNT; ++index)
        cctl_adc_result[index] = 0;
    cctl_encoder_position = 0;
    cctl_pwm_compare[0] = cctl_pwm_compare[1] = cctl_pwm_compare[2] = 0;
    cctl_controller_tick = 0;
}

/** @copydoc cctl_advance_controller_tick */
void cctl_advance_controller_tick(void)
{
    ++cctl_controller_tick;
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
    gmp_base_ctl_step();
    cctl_advance_controller_tick();
}

/** @copydoc gmp_base_get_system_tick */
time_gt gmp_base_get_system_tick(void)
{
    const uint64_t ticks_per_ms = (uint64_t)(CONTROLLER_FREQUENCY / 1000.0f);
    return (time_gt)(cctl_controller_tick / (ticks_per_ms == 0U ? 1U : ticks_per_ms));
}

/** @brief Hosted monitor transport placeholder. */
void send_monitor_data(void)
{
}

/** @brief Hosted data-link receive placeholder. */
void flush_dl_rx_buffer(void)
{
}

/** @brief Discard a debug-print buffer in this headless simulation. */
ec_gt gmp_hal_uart_send(GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE,
                        gmp_print_buffer_t *)
{
    return 0;
}

} // extern "C"
