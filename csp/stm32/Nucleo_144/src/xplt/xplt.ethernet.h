/** @file xplt.ethernet.h Mandatory Nucleo-144 Ethernet service. */

#ifndef GMP_STM32_NUCLEO_144_XPLT_ETHERNET_H
#define GMP_STM32_NUCLEO_144_XPLT_ETHERNET_H

#include <core/dev/datalink/datalink.h>

void xplt_eth_init(void);
void xplt_eth_poll(void);
void xplt_eth_dl_bind(gmp_datalink_t* datalink);
void xplt_eth_dl_start_tx(gmp_datalink_t* datalink);

#endif /* GMP_STM32_NUCLEO_144_XPLT_ETHERNET_H */
