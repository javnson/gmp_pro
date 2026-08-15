/**
 * @file xplt.ctl_interface.h
 * @brief Cross-board ADC input and PWM/DAC output callbacks.
 */

#ifndef GMP_LAUNCHPAD_CTL_INTERFACE_H
#define GMP_LAUNCHPAD_CTL_INTERFACE_H

#include <launchpad_board.h>

/** @brief Acquire platform inputs before a control step. */
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    launchpad_adc_raw = ADC_readResult(LAUNCHPAD_CONTROL_ADC_RESULT,
                                       LAUNCHPAD_CONTROL_ADC_SOC);
    launchpad_adc_pu = real2ctrl((float)launchpad_adc_raw / 4095.0F);
}

/** @brief Apply platform outputs after a control step. */
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    ctrl_gt duty = launchpad_output_pu;
    uint16_t period;
    uint16_t compare;

    if (duty < 0)
        duty = 0;
    else if (duty > (ctrl_gt)1.0F)
        duty = (ctrl_gt)1.0F;

    period = EPWM_getTimeBasePeriod(LAUNCHPAD_CONTROL_PWM_BASE);
    compare = (uint16_t)((float)duty * (float)period);
    EPWM_setCounterCompareValue(LAUNCHPAD_CONTROL_PWM_BASE,
                                EPWM_COUNTER_COMPARE_A, compare);

#if LAUNCHPAD_HAS_EXTERNAL_DAC
    DAC_setShadowValue(LAUNCHPAD_CONTROL_DAC_BASE, launchpad_adc_raw);
#endif
}

#endif // GMP_LAUNCHPAD_CTL_INTERFACE_H
