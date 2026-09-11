/**
 * @file xplt.ethernet.c
 * @brief Bare-metal LwIP polling and the Nucleo-144 UDP acceptance endpoint.
 */

#include <gmp_core.h>

#include "lwip.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include <xplt.peripheral.h>

extern struct netif gnetif;

static struct udp_pcb* echo_pcb;

static void xplt_eth_udp_receive(void* argument, struct udp_pcb* pcb,
                                 struct pbuf* packet,
                                 const ip_addr_t* remote_address,
                                 u16_t remote_port)
{
    err_t result;

    GMP_UNUSED_VAR(argument);
    if (packet == NULL)
        return;

    gmp_nucleo_platform_diag[8]++;
    gmp_nucleo_platform_diag[10] += packet->tot_len;
    result = udp_sendto(pcb, packet, remote_address, remote_port);
    if (result == ERR_OK)
        gmp_nucleo_platform_diag[9]++;
    else
        gmp_nucleo_platform_diag[11]++;
    pbuf_free(packet);
}

void xplt_eth_init(void)
{
    echo_pcb = udp_new();
    if (echo_pcb == NULL ||
        udp_bind(echo_pcb, IP_ADDR_ANY,
                 (u16_t)GMP_NUCLEO_ETH_UDP_ECHO_PORT) != ERR_OK)
    {
        gmp_nucleo_platform_diag[11]++;
        if (echo_pcb != NULL)
        {
            udp_remove(echo_pcb);
            echo_pcb = NULL;
        }
        return;
    }
    udp_recv(echo_pcb, xplt_eth_udp_receive, NULL);
}

void xplt_eth_poll(void)
{
    MX_LWIP_Process();
    gmp_nucleo_platform_diag[7] =
        (netif_is_up(&gnetif) && netif_is_link_up(&gnetif)) ? 1U : 0U;
}
