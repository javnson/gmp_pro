/**
 * @file ctl_main.c
 * @brief Portable ADC-to-output controller used by every LaunchPad target.
 */

#include <gmp_core.h>

volatile uint16_t launchpad_adc_raw;
volatile ctrl_gt launchpad_adc_pu;
volatile ctrl_gt launchpad_output_pu;

void ctl_init(void)
{
    launchpad_adc_raw = 0U;
    launchpad_adc_pu = 0;
    launchpad_output_pu = 0;
}

void ctl_mainloop(void)
{
}
