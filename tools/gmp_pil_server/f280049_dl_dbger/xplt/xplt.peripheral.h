/**
 * @file xplt.peripheral.h
 * @brief F280049C platform services for Data Link, sampling, and heartbeat I/O.
 */

#ifndef GMP_F280049_DL_DBGER_XPLT_PERIPHERAL_H
#define GMP_F280049_DL_DBGER_XPLT_PERIPHERAL_H

#include <core/dev/datalink.h>

/** @brief Bind the user-owned Data Link context to the SCIA transport. */
void xplt_dl_bind(gmp_datalink_t* datalink);

/** @brief Drain any residual SCIA RX FIFO data into the Data Link FIFO. */
void xplt_dl_poll_rx(void);

/** @brief Transmit the response currently queued by Data Link. */
void xplt_dl_start_tx(gmp_datalink_t* datalink);

/** @brief Start the 1 kHz CPU Timer 0 sample interrupt. */
void xplt_start_sample_timer(void);

/** @brief Toggle the LaunchPad user LED. */
void xplt_toggle_user_led(void);

#endif // GMP_F280049_DL_DBGER_XPLT_PERIPHERAL_H
