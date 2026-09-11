/** @file xplt.ctl_interface.h Fast control-path bindings for Nucleo-144. */

#ifndef GMP_STM32_NUCLEO_144_CTL_INTERFACE_H
#define GMP_STM32_NUCLEO_144_CTL_INTERFACE_H

void xplt_ctl_input(void);
void xplt_ctl_output(void);

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    xplt_ctl_input();
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    xplt_ctl_output();
}

#endif // GMP_STM32_NUCLEO_144_CTL_INTERFACE_H


