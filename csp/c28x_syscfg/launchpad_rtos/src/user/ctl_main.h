/**
 * @file ctl_main.h
 * @brief CTL declarations required by the standard GMP entry sequence.
 */

#ifndef GMP_LAUNCHPAD_CTL_MAIN_H
#define GMP_LAUNCHPAD_CTL_MAIN_H

/** Last raw ADC conversion and normalized control values. */
extern volatile uint16_t launchpad_adc_raw;
extern volatile uint16_t launchpad_pwm_compare;
extern volatile ctrl_gt launchpad_adc_pu;
extern volatile ctrl_gt launchpad_output_pu;

/** @brief Initialize controller-layer services. */
void ctl_init(void);

/** @brief Run controller-layer background work. */
void ctl_mainloop(void);

/** @brief Sample the Data Link Scope at the undivided control rate. */
void user_dl_control_step(void);

/** @brief Dispatch one synchronous controller step. */
GMP_STATIC_INLINE void ctl_dispatch(void)
{
    launchpad_output_pu = launchpad_adc_pu;
    user_dl_control_step();
}

#endif // GMP_LAUNCHPAD_CTL_MAIN_H
