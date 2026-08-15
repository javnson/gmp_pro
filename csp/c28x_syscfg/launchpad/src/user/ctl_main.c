/**
 * @file ctl_main.c
 * @brief Portable ADC-to-output controller used by every LaunchPad target.
 */

#include <gmp_core.h>

volatile uint16_t launchpad_adc_raw;
volatile uint16_t launchpad_pwm_compare;
volatile ctrl_gt launchpad_adc_pu;
volatile ctrl_gt launchpad_output_pu;

void ctl_init(void)
{
    launchpad_adc_raw = 0U;
    launchpad_pwm_compare = 0U;
    launchpad_adc_pu = 0;
    launchpad_output_pu = 0;
}

void ctl_mainloop(void)
{
    /* Reference control law: mirror the selected ADC channel to PWM duty.
     * On DAC-capable devices the platform callback mirrors the same raw
     * sample to the selected external DAC channel. */
    launchpad_output_pu = launchpad_adc_pu;
}
