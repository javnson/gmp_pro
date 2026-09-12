/**
 * @file ctl_main.h
 * @brief Safe bring-up controller for the shared Nucleo-32 platform.
 */

#ifndef GMP_STM32_NUCLEO_32_CTL_MAIN_H
#define GMP_STM32_NUCLEO_32_CTL_MAIN_H

extern volatile uint32_t gmp_nucleo_adc_raw[6];
extern volatile uint32_t gmp_nucleo_pwm_compare[3];
extern volatile int32_t gmp_nucleo_qep_count;

void ctl_init(void);
void ctl_mainloop(void);
void user_dl_control_step(void);

GMP_STATIC_INLINE void ctl_dispatch(void)
{
    /* Bring-up firmware only exposes observations; it never enables power. */
    user_dl_control_step();
}

#endif // GMP_STM32_NUCLEO_32_CTL_MAIN_H
