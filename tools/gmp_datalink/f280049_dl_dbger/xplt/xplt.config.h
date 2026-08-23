/**
 * @file xplt.config.h
 * @brief GMP configuration for the LAUNCHXL-F280049C Data Link target.
 */

#ifndef GMP_F280049_DL_DBGER_XPLT_CONFIG_H
#define GMP_F280049_DL_DBGER_XPLT_CONFIG_H

#define SPECIFY_ENABLE_GMP_CTL
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#define SPECIFY_DISABLE_GMP_LOGO
#define SPECIFY_BASE_PRINT_NOT_IMPL

/** @brief Generate one millisecond system tick from each 1 kHz timer ISR. */
#define DSP_C2000_DSP_TIME_DIV 1U

/**
 * @brief Select the XDS110 application UART speed.
 * @details Both 115200 and 921600 baud are hardware-tested. At 921600 the
 * target uses a 50 MHz LSPCLK and BRR=6, producing 892857 baud on the wire.
 */
#define XPLT_DL_BAUD_RATE 921600UL

#endif // GMP_F280049_DL_DBGER_XPLT_CONFIG_H
