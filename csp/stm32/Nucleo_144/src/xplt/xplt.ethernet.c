/**
 * @file xplt.ethernet.c
 * @brief Bare-metal LwIP services and selectable TCP/UDP GMP Data Link.
 */

#include <gmp_core.h>

#include "lwip.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include <xplt.ethernet.h>
#include <xplt.peripheral.h>

extern struct netif gnetif;

static struct udp_pcb* echo_pcb;
static gmp_datalink_t* ethernet_datalink;

#if GMP_NUCLEO_ETH_DL_TRANSPORT == GMP_NUCLEO_ETH_DL_UDP
static struct udp_pcb* dl_udp_pcb;
static ip_addr_t dl_udp_peer_address;
static u16_t dl_udp_peer_port;
static fast_gt dl_udp_peer_valid;
#else
static struct tcp_pcb* dl_tcp_listener;
static struct tcp_pcb* dl_tcp_client;
#endif

static void xplt_eth_note_error(void)
{
    gmp_nucleo_platform_diag[11]++;
    gmp_nucleo_platform_diag[17]++;
}

static void xplt_eth_push_datalink_pbuf(const struct pbuf* packet)
{
    const struct pbuf* segment;

    if (ethernet_datalink == NULL)
        return;
    gmp_nucleo_platform_diag[12]++;
    gmp_nucleo_platform_diag[14] += packet->tot_len;
    for (segment = packet; segment != NULL; segment = segment->next)
        gmp_dev_dl_push_str(ethernet_datalink,
                            (const uint8_t*)segment->payload,
                            (size_gt)segment->len);
}

static void xplt_eth_udp_echo_receive(void* argument, struct udp_pcb* pcb,
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

#if GMP_NUCLEO_ETH_DL_TRANSPORT == GMP_NUCLEO_ETH_DL_UDP
static void xplt_eth_udp_dl_receive(void* argument, struct udp_pcb* pcb,
                                    struct pbuf* packet,
                                    const ip_addr_t* remote_address,
                                    u16_t remote_port)
{
    GMP_UNUSED_VAR(argument);
    GMP_UNUSED_VAR(pcb);
    if (packet == NULL)
        return;

    ip_addr_copy(dl_udp_peer_address, *remote_address);
    dl_udp_peer_port = remote_port;
    dl_udp_peer_valid = 1;
    xplt_eth_push_datalink_pbuf(packet);
    pbuf_free(packet);
}

static err_t xplt_eth_udp_dl_send(gmp_datalink_t* datalink)
{
    const uint8_t* header = gmp_dev_dl_get_tx_hw_hdr_ptr(datalink);
    const uint8_t* payload = gmp_dev_dl_get_tx_hw_pld_ptr(datalink);
    u16_t header_size = (u16_t)gmp_dev_dl_get_tx_hw_hdr_size(datalink);
    u16_t payload_size = (u16_t)gmp_dev_dl_get_tx_hw_pld_size(datalink);
    struct pbuf* packet;
    err_t result;

    if (dl_udp_pcb == NULL || !dl_udp_peer_valid)
        return ERR_CONN;
    packet = pbuf_alloc(PBUF_TRANSPORT, (u16_t)(header_size + payload_size),
                        PBUF_RAM);
    if (packet == NULL)
        return ERR_MEM;
    result = pbuf_take_at(packet, header, header_size, 0U);
    if (result == ERR_OK && payload_size > 0U)
        result = pbuf_take_at(packet, payload, payload_size, header_size);
    if (result == ERR_OK)
        result = udp_sendto(dl_udp_pcb, packet, &dl_udp_peer_address,
                            dl_udp_peer_port);
    pbuf_free(packet);
    return result;
}
#else
static void xplt_eth_tcp_error(void* argument, err_t error)
{
    GMP_UNUSED_VAR(argument);
    GMP_UNUSED_VAR(error);
    dl_tcp_client = NULL;
    xplt_eth_note_error();
}

static err_t xplt_eth_tcp_close_client(struct tcp_pcb* pcb)
{
    err_t result;

    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_err(pcb, NULL);
    if (dl_tcp_client == pcb)
        dl_tcp_client = NULL;
    result = tcp_close(pcb);
    if (result != ERR_OK)
    {
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    return ERR_OK;
}

static err_t xplt_eth_tcp_receive(void* argument, struct tcp_pcb* pcb,
                                  struct pbuf* packet, err_t error)
{
    GMP_UNUSED_VAR(argument);
    if (error != ERR_OK)
    {
        if (packet != NULL)
            pbuf_free(packet);
        xplt_eth_note_error();
        return error;
    }
    if (packet == NULL)
        return xplt_eth_tcp_close_client(pcb);

    tcp_recved(pcb, packet->tot_len);
    xplt_eth_push_datalink_pbuf(packet);
    pbuf_free(packet);
    return ERR_OK;
}

static err_t xplt_eth_tcp_accept(void* argument, struct tcp_pcb* pcb,
                                 err_t error)
{
    GMP_UNUSED_VAR(argument);
    if (error != ERR_OK)
        return error;
    if (dl_tcp_client != NULL)
    {
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    dl_tcp_client = pcb;
    gmp_nucleo_platform_diag[16]++;
    tcp_nagle_disable(pcb);
    tcp_recv(pcb, xplt_eth_tcp_receive);
    tcp_err(pcb, xplt_eth_tcp_error);
    return ERR_OK;
}

static err_t xplt_eth_tcp_dl_send(gmp_datalink_t* datalink)
{
    const uint8_t* header = gmp_dev_dl_get_tx_hw_hdr_ptr(datalink);
    const uint8_t* payload = gmp_dev_dl_get_tx_hw_pld_ptr(datalink);
    u16_t header_size = (u16_t)gmp_dev_dl_get_tx_hw_hdr_size(datalink);
    u16_t payload_size = (u16_t)gmp_dev_dl_get_tx_hw_pld_size(datalink);
    err_t result;

    if (dl_tcp_client == NULL)
        return ERR_CONN;
    if (tcp_sndbuf(dl_tcp_client) < (u16_t)(header_size + payload_size))
        return ERR_MEM;
    result = tcp_write(dl_tcp_client, header, header_size,
                       payload_size > 0U ? TCP_WRITE_FLAG_MORE : 0U);
    if (result == ERR_OK && payload_size > 0U)
        result = tcp_write(dl_tcp_client, payload, payload_size, 0U);
    if (result == ERR_OK)
        result = tcp_output(dl_tcp_client);
    return result;
}
#endif

void xplt_eth_dl_bind(gmp_datalink_t* datalink)
{
    ethernet_datalink = datalink;
}

void xplt_eth_dl_start_tx(gmp_datalink_t* datalink)
{
    err_t result;
    uint32_t frame_size;

#if GMP_NUCLEO_ETH_DL_TRANSPORT == GMP_NUCLEO_ETH_DL_UDP
    result = xplt_eth_udp_dl_send(datalink);
#else
    result = xplt_eth_tcp_dl_send(datalink);
#endif
    frame_size = (uint32_t)gmp_dev_dl_get_tx_hw_hdr_size(datalink) +
                 (uint32_t)gmp_dev_dl_get_tx_hw_pld_size(datalink);
    if (result == ERR_OK)
    {
        gmp_nucleo_platform_diag[13]++;
        gmp_nucleo_platform_diag[15] += frame_size;
    }
    else
        xplt_eth_note_error();
    gmp_dev_dl_tx_state_done(datalink);
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
    }
    else
        udp_recv(echo_pcb, xplt_eth_udp_echo_receive, NULL);

#if GMP_NUCLEO_ETH_DL_TRANSPORT == GMP_NUCLEO_ETH_DL_UDP
    dl_udp_peer_valid = 0;
    dl_udp_pcb = udp_new();
    if (dl_udp_pcb == NULL ||
        udp_bind(dl_udp_pcb, IP_ADDR_ANY,
                 (u16_t)GMP_NUCLEO_ETH_DL_UDP_PORT) != ERR_OK)
    {
        xplt_eth_note_error();
        if (dl_udp_pcb != NULL)
        {
            udp_remove(dl_udp_pcb);
            dl_udp_pcb = NULL;
        }
    }
    else
        udp_recv(dl_udp_pcb, xplt_eth_udp_dl_receive, NULL);
#else
    {
        struct tcp_pcb* server = tcp_new_ip_type(IPADDR_TYPE_V4);
        err_t result = server == NULL ? ERR_MEM :
            tcp_bind(server, IP_ADDR_ANY, (u16_t)GMP_NUCLEO_ETH_DL_TCP_PORT);
        if (result == ERR_OK)
            dl_tcp_listener = tcp_listen_with_backlog(server, 1U);
        if (result != ERR_OK || dl_tcp_listener == NULL)
        {
            xplt_eth_note_error();
            if (server != NULL)
                (void)tcp_close(server);
        }
        else
            tcp_accept(dl_tcp_listener, xplt_eth_tcp_accept);
    }
#endif
}

void xplt_eth_poll(void)
{
    MX_LWIP_Process();
    gmp_nucleo_platform_diag[7] =
        (netif_is_up(&gnetif) && netif_is_link_up(&gnetif)) ? 1U : 0U;
}
