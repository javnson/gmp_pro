/**
 * @file ctl_main.c
 * @brief Observable state and safe compare defaults for Nucleo-144 bring-up.
 */

#include <gmp_core.h>

volatile uint32_t gmp_nucleo_adc_raw[6];
volatile uint32_t gmp_nucleo_pwm_compare[3];
volatile int32_t gmp_nucleo_qep_count;

void ctl_init(void)
{
    uint32_t safe_compare = (uint32_t)GMP_NUCLEO_PWM_PERIOD / 2U;
    size_gt index;

    for (index = 0U; index < 6U; ++index)
        gmp_nucleo_adc_raw[index] = 0U;
    for (index = 0U; index < 3U; ++index)
        gmp_nucleo_pwm_compare[index] = safe_compare;
    gmp_nucleo_qep_count = 0;
}

void ctl_mainloop(void)
{
}


