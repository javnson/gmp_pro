/**
 * @file xplt.config.h
 * @brief GMP configuration for the portable C2000 LaunchPad target.
 */

#ifndef GMP_LAUNCHPAD_XPLT_CONFIG_H
#define GMP_LAUNCHPAD_XPLT_CONFIG_H

#include <launchpad_board.h>

#define SPECIFY_ENABLE_GMP_CTL
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#define SPECIFY_DISABLE_GMP_LOGO
#define SPECIFY_BASE_PRINT_NOT_IMPL

/** @brief Divide the board control ISR into the one-millisecond GMP tick. */
#define DSP_C2000_DSP_TIME_DIV (GMP_LAUNCHPAD_PWM_FREQUENCY_HZ / 1000UL)

/**
 * @brief Select the XDS110 application UART speed.
 * @details Both 115200 and 921600 baud are hardware-tested. At 921600 the
 * target uses a 50 MHz LSPCLK and BRR=6, producing 892857 baud on the wire.
 */
#define XPLT_DL_BAUD_RATE GMP_DL_UART_BAUDRATE

#endif // GMP_LAUNCHPAD_XPLT_CONFIG_H
