/**
 * @file xplt.peripheral.h
 * @brief Pico-specific peripheral declarations for the GMP blink example.
 */

#ifndef GMP_PICO_BLINK_PERIPHERAL_H
#define GMP_PICO_BLINK_PERIPHERAL_H

#include <gmp_core.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Encoded GMP handle for the board status LED. */
extern gpio_halt user_led;

/**
 * @brief Start the periodic timer that invokes MainISR.
 * @return GMP_EC_OK on success, otherwise GMP_EC_GENERAL_ERROR.
 */
ec_gt xplt_start_main_interrupt(void);

/**
 * @brief Execute one GMP control interrupt cycle.
 */
void MainISR(void);

#ifdef __cplusplus
}
#endif

#endif /* GMP_PICO_BLINK_PERIPHERAL_H */
