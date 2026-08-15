/**
 * @file xplt.peripheral.h
 * @brief Portable LaunchPad services for Data Link and BOOSTXL control I/O.
 */

#ifndef GMP_F280049_DL_DBGER_XPLT_PERIPHERAL_H
#define GMP_F280049_DL_DBGER_XPLT_PERIPHERAL_H

#include <core/dev/datalink/datalink.h>

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

/** @brief Drive BOOSTXL position 13 gate-enable output when available. */
void xplt_set_gate_enable(uint16_t enable);

/** @brief Drive BOOSTXL position 33 relay output when available. */
void xplt_set_relay(uint16_t closed);

/** @brief Read BOOSTXL position 34 over-temperature input when available. */
uint16_t xplt_get_over_temperature(void);

/** @brief Service the board CAN example without blocking the 2 ms task. */
void xplt_can_service(void);

#endif // GMP_F280049_DL_DBGER_XPLT_PERIPHERAL_H
