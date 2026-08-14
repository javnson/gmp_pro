/**
 * @file xplt.peripheral.h
 * @brief STM32 platform transport services for the Data Link user module.
 */

#ifndef GMP_STM32_DL_DBGER_XPLT_PERIPHERAL_H
#define GMP_STM32_DL_DBGER_XPLT_PERIPHERAL_H

#include <core/dev/datalink/datalink.h>

/** @brief Bind the user-owned Data Link context to the UART DMA transport. */
void xplt_dl_bind(gmp_datalink_t* datalink);

/** @brief Start DMA transmission of the response currently queued by Data Link. */
void xplt_dl_start_tx(gmp_datalink_t* datalink);

/** @brief Start the 1 kHz TIM3 sample interrupt. */
void xplt_start_sample_timer(void);

/** @brief Toggle the NUCLEO user LED through the STM32 GPIO CSP. */
void xplt_toggle_user_led(void);

#endif // GMP_STM32_DL_DBGER_XPLT_PERIPHERAL_H
