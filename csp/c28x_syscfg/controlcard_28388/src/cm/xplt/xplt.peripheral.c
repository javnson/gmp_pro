/** @file xplt.peripheral.c CM Ethernet/lwIP and communication binding. */

#include <gmp_core.h>
#include "driverlib_cm.h"
#include "cm.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "utils/lwiplib.h"
#include <xplt.peripheral.h>

#define XPLT_CM_RX_RING_SIZE (2048U)
#define XPLT_CM_RX_RING_MASK (XPLT_CM_RX_RING_SIZE - 1U)
#define XPLT_CM_TX_FRAME_CAPACITY (544U)

#if (XPLT_CM_RX_RING_SIZE & XPLT_CM_RX_RING_MASK) != 0
#error "CM receive ring size must be a power of two"
#endif

#pragma DATA_SECTION(cm_to_cpu2_command, "GMP_MSGRAM_CM_TO_CPU2")
volatile gmp_wave_command_t cm_to_cpu2_command;
#pragma DATA_SECTION(cpu2_to_cm_snapshot, "GMP_MSGRAM_CPU2_TO_CM")
volatile gmp_wave_snapshot_t cpu2_to_cm_snapshot;

extern void CM_init(void);
extern void Ethernet_init(const unsigned char *mac);
extern uint32_t systickPeriodValue;
extern void SysTickIntHandler(void);

static uint8_t cm_rx_ring[XPLT_CM_RX_RING_SIZE];
static volatile uint16_t cm_rx_head;
static volatile uint16_t cm_rx_tail;
static volatile uint32_t cm_rx_reset_pending;
static uint8_t cm_tx_frame[XPLT_CM_TX_FRAME_CAPACITY];

volatile uint32_t cm_rx_frames;
volatile uint32_t cm_tx_frames;
volatile int32_t cm_last_network_error;
volatile uint32_t cm_err_mem_count;
volatile uint32_t cm_err_mem_stage;
volatile uint32_t cm_last_tx_frame_size;
volatile uint32_t cm_last_tcp_sndbuf;
volatile uint32_t cm_last_tcp_queuelen;
volatile uint32_t cm_rx_ring_backpressure_count;
volatile uint32_t cm_rx_ring_reset_count;
volatile uint32_t cm_tcp_accept_count;
volatile uint32_t cm_tcp_close_count;
volatile uint32_t cm_tcp_abort_count;

#if GMP_F28388D_CM_DL_TRANSPORT == GMP_F28388D_CM_DL_UDP
static struct udp_pcb *cm_udp;
static ip_addr_t cm_udp_peer;
static u16_t cm_udp_peer_port;
static uint32_t cm_udp_peer_valid;
#else
static struct tcp_pcb *cm_tcp_listener;
static struct tcp_pcb *cm_tcp_client;
#endif

static fast_gt cm_rx_enqueue(const struct pbuf *packet)
{
    const struct pbuf *segment;
    uint16_t head = cm_rx_head;
    uint16_t tail = cm_rx_tail;
    uint16_t available = (uint16_t)((tail - head - 1U) & XPLT_CM_RX_RING_MASK);
    if (packet == NULL || packet->tot_len > available)
        return 0;
    for (segment = packet; segment != NULL; segment = segment->next)
    {
        const uint8_t *source = (const uint8_t *)segment->payload;
        u16_t index;
        for (index = 0U; index < segment->len; ++index)
        {
            cm_rx_ring[head] = source[index];
            head = (uint16_t)((head + 1U) & XPLT_CM_RX_RING_MASK);
        }
    }
    /* Publish only after the complete TCP segment/UDP datagram is copied. */
    cm_rx_head = head;
    cm_rx_frames++;
    return 1;
}

static u16_t cm_pack_tx_frame(gmp_datalink_t *datalink)
{
    const byte_gt *header = gmp_dev_dl_get_tx_hw_hdr_ptr(datalink);
    const byte_gt *payload = gmp_dev_dl_get_tx_hw_pld_ptr(datalink);
    u16_t header_size = (u16_t)gmp_dev_dl_get_tx_hw_hdr_size(datalink);
    u16_t payload_size = (u16_t)gmp_dev_dl_get_tx_hw_pld_size(datalink);
    u16_t index;
    if ((u16_t)(header_size + payload_size) > XPLT_CM_TX_FRAME_CAPACITY)
        return 0U;
    for (index = 0U; index < header_size; ++index)
        cm_tx_frame[index] = (uint8_t)(header[index] & 0xFFU);
    for (index = 0U; index < payload_size; ++index)
        cm_tx_frame[header_size + index] = (uint8_t)(payload[index] & 0xFFU);
    return (u16_t)(header_size + payload_size);
}

#if GMP_F28388D_CM_DL_TRANSPORT == GMP_F28388D_CM_DL_UDP
static void cm_udp_receive(void *arg, struct udp_pcb *pcb, struct pbuf *packet,
                           const ip_addr_t *address, u16_t port)
{
    GMP_UNUSED_VAR(arg);
    GMP_UNUSED_VAR(pcb);
    if (packet == NULL)
        return;
    ip_addr_copy(cm_udp_peer, *address);
    cm_udp_peer_port = port;
    cm_udp_peer_valid = 1UL;
    if (!cm_rx_enqueue(packet))
        cm_rx_ring_backpressure_count++;
    pbuf_free(packet);
}
#else
static void cm_tcp_request_rx_reset(void)
{
    cm_rx_reset_pending = 1UL;
}

static void cm_tcp_error(void *arg, err_t error)
{
    if ((struct tcp_pcb *)arg == cm_tcp_client)
        cm_tcp_client = NULL;
    cm_last_network_error = (int32_t)error;
    cm_tcp_abort_count++;
    cm_tcp_request_rx_reset();
}

static err_t cm_tcp_close_client(struct tcp_pcb *pcb)
{
    err_t result;
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_err(pcb, NULL);
    if (cm_tcp_client == pcb)
        cm_tcp_client = NULL;
    cm_tcp_request_rx_reset();
    result = tcp_close(pcb);
    if (result != ERR_OK)
    {
        tcp_abort(pcb);
        cm_tcp_abort_count++;
        return ERR_ABRT;
    }
    cm_tcp_close_count++;
    return ERR_OK;
}

static err_t cm_tcp_receive(void *arg, struct tcp_pcb *pcb,
                            struct pbuf *packet, err_t error)
{
    GMP_UNUSED_VAR(arg);
    if (error != ERR_OK)
    {
        if (packet != NULL)
            pbuf_free(packet);
        cm_last_network_error = (int32_t)error;
        return error;
    }
    if (packet == NULL)
        return cm_tcp_close_client(pcb);
    if (!cm_rx_enqueue(packet))
    {
        /* Do not acknowledge or free refused data. lwIP retains it and invokes
         * this callback again after the main loop drains the SPSC ring. */
        cm_rx_ring_backpressure_count++;
        return ERR_MEM;
    }
    tcp_recved(pcb, packet->tot_len);
    pbuf_free(packet);
    return ERR_OK;
}

static err_t cm_tcp_accept(void *arg, struct tcp_pcb *pcb, err_t error)
{
    struct tcp_pcb *previous;
    GMP_UNUSED_VAR(arg);
    if (error != ERR_OK)
        return error;
    if (cm_tcp_client != NULL)
    {
        /* The host can reconnect before the FIN from its previous socket has
         * reached this NO_SYS stack.  Prefer the newest accepted connection
         * and explicitly reclaim the stale PCB instead of rejecting the new
         * client until the old close timeout expires. */
        previous = cm_tcp_client;
        cm_tcp_client = NULL;
        tcp_arg(previous, NULL);
        tcp_recv(previous, NULL);
        tcp_err(previous, NULL);
        tcp_abort(previous);
        cm_tcp_abort_count++;
    }
    cm_tcp_client = pcb;
    cm_tcp_accept_count++;
    cm_tcp_request_rx_reset();
    tcp_arg(pcb, pcb);
    tcp_nagle_disable(pcb);
    tcp_recv(pcb, cm_tcp_receive);
    tcp_err(pcb, cm_tcp_error);
    return ERR_OK;
}
#endif

static void cm_network_server_init(void)
{
#if GMP_F28388D_CM_DL_TRANSPORT == GMP_F28388D_CM_DL_UDP
    cm_udp = udp_new();
    if (cm_udp != NULL &&
        udp_bind(cm_udp, IP_ADDR_ANY, GMP_F28388D_CM_UDP_PORT) == ERR_OK)
        udp_recv(cm_udp, cm_udp_receive, NULL);
    else
        cm_last_network_error = ERR_MEM;
#else
    struct tcp_pcb *server = tcp_new_ip_type(IPADDR_TYPE_V4);
    err_t result = (server == NULL) ? ERR_MEM :
        tcp_bind(server, IP_ADDR_ANY, GMP_F28388D_CM_TCP_PORT);
    if (result != ERR_OK)
    {
        if (server != NULL)
            tcp_abort(server);
        cm_last_network_error = (int32_t)result;
        return;
    }
    cm_tcp_listener = tcp_listen_with_backlog(server, 1U);
    if (cm_tcp_listener == NULL)
    {
        tcp_abort(server);
        cm_last_network_error = ERR_MEM;
        return;
    }
    tcp_accept(cm_tcp_listener, cm_tcp_accept);
#endif
}

void xplt_cm_dl_receive(gmp_datalink_t *datalink)
{
    byte_gt units[64];
    size_gt count = 0U;
    uint16_t tail;
    if (cm_rx_reset_pending != 0UL)
    {
        sys_prot_t protection;
        SYS_ARCH_PROTECT(protection);
        cm_rx_tail = cm_rx_head;
        cm_rx_reset_pending = 0UL;
        SYS_ARCH_UNPROTECT(protection);
        cm_rx_ring_reset_count++;
        gmp_dev_dl_request_rx_reset(datalink);
    }
    tail = cm_rx_tail;
    while (tail != cm_rx_head && count < sizeof(units) / sizeof(units[0]))
    {
        units[count++] = (byte_gt)cm_rx_ring[tail];
        tail = (uint16_t)((tail + 1U) & XPLT_CM_RX_RING_MASK);
    }
    cm_rx_tail = tail;
    if (count != 0U)
        gmp_dev_dl_push_str(datalink, units, count);
}

xplt_cm_dl_send_result_t xplt_cm_dl_send(gmp_datalink_t *datalink)
{
    sys_prot_t protection;
    u16_t frame_size = cm_pack_tx_frame(datalink);
    err_t result = ERR_CONN;
    cm_last_tx_frame_size = frame_size;
    if (frame_size == 0U)
        return XPLT_CM_DL_SEND_ERROR;
    SYS_ARCH_PROTECT(protection);
#if GMP_F28388D_CM_DL_TRANSPORT == GMP_F28388D_CM_DL_UDP
    if (cm_udp != NULL && cm_udp_peer_valid != 0UL)
    {
        struct pbuf *packet = pbuf_alloc(PBUF_TRANSPORT, frame_size, PBUF_RAM);
        if (packet == NULL)
            result = ERR_MEM;
        else
        {
            result = pbuf_take(packet, cm_tx_frame, frame_size);
            if (result == ERR_OK)
                result = udp_sendto(cm_udp, packet, &cm_udp_peer, cm_udp_peer_port);
            pbuf_free(packet);
        }
    }
#else
    if (cm_tcp_client != NULL)
    {
        cm_last_tcp_sndbuf = tcp_sndbuf(cm_tcp_client);
        cm_last_tcp_queuelen = cm_tcp_client->snd_queuelen;
        if (cm_last_tcp_sndbuf < frame_size)
        {
            cm_err_mem_stage = 1UL;
            result = ERR_MEM;
        }
        else
        {
            result = tcp_write(cm_tcp_client, cm_tx_frame, frame_size,
                               TCP_WRITE_FLAG_COPY);
            if (result == ERR_MEM)
                cm_err_mem_stage = 2UL;
            if (result == ERR_OK)
                (void)tcp_output(cm_tcp_client);
        }
    }
#endif
    SYS_ARCH_UNPROTECT(protection);
    cm_last_network_error = (int32_t)result;
    if (result == ERR_OK)
    {
        cm_tx_frames++;
        return XPLT_CM_DL_SEND_OK;
    }
    if (result == ERR_MEM)
    {
        cm_err_mem_count++;
        return XPLT_CM_DL_SEND_RETRY;
    }
    if (result == ERR_CONN)
        return XPLT_CM_DL_SEND_DISCONNECTED;
    return XPLT_CM_DL_SEND_ERROR;
}

uint32_t xplt_cm_ethercat_memory_ready(void)
{
    return (uint32_t)ESCSS_getMemoryInitDoneStatusNonBlocking(ESC_SS_BASE);
}

void gmp_c28x_syscfg_cm_device_init(void)
{
    unsigned char mac[6] = {0xA8U, 0x63U, 0xF2U, 0x00U, 0x28U, 0x88U};
    cm_rx_head = 0U;
    cm_rx_tail = 0U;
    cm_rx_reset_pending = 0UL;
    CM_init();
    IPC_sync(IPC_CM_L_CPU1_R, IPC_FLAG30);
    IPC_sync(IPC_CM_L_CPU1_R, IPC_FLAG31);
    Ethernet_init(mac);
    lwIPInit(0U, mac, GMP_F28388D_CM_IPV4, 0xFFFFFF00UL, 0U,
             IPADDR_USE_STATIC);
    systickPeriodValue = GMP_F28388D_CM_CLOCK_HZ / 1000UL;
    SYSTICK_setPeriod(systickPeriodValue);
    SYSTICK_registerInterruptHandler(SysTickIntHandler);
    SYSTICK_enableInterrupt();
    SYSTICK_enableCounter();
}

void gmp_c28x_syscfg_cm_device_loop(void)
{
    sys_prot_t protection;
    SYS_ARCH_PROTECT(protection);
    sys_check_timeouts();
    SYS_ARCH_UNPROTECT(protection);
}

void gmp_c28x_syscfg_cm_post_start(void)
{
}

void setup_peripheral(void)
{
    sys_prot_t protection;
    SYS_ARCH_PROTECT(protection);
    cm_network_server_init();
    SYS_ARCH_UNPROTECT(protection);
}
