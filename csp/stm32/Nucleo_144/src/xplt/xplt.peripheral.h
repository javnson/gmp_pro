/** @file xplt.peripheral.h Shared STM32 Nucleo-144 peripheral services. */

#ifndef GMP_STM32_NUCLEO_144_XPLT_PERIPHERAL_H
#define GMP_STM32_NUCLEO_144_XPLT_PERIPHERAL_H

#include <core/dev/datalink/datalink.h>

/* Control ISR, PWM, UART, LED, Ethernet echo and Ethernet DL diagnostics. */
extern volatile uint32_t gmp_nucleo_platform_diag[18];

void xplt_uart_dl_bind(gmp_datalink_t* datalink);
void xplt_uart_dl_start_tx(gmp_datalink_t* datalink);
void xplt_toggle_status_led(void);
void xplt_ctl_input(void);
void xplt_ctl_output(void);
void xplt_pwm_enable(void);
void xplt_pwm_disable(void);

#if GMP_NUCLEO_HAS_DAC
void xplt_dac_write(uint32_t value);
#endif

#endif // GMP_STM32_NUCLEO_144_XPLT_PERIPHERAL_H
