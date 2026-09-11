/** @file cm_app.c System-u16 GMP Data Link over the CM Ethernet octet stream. */

#include <gmp_core.h>
#include <core/dev/datalink/mem_presp.h>
#include <core/dev/datalink/tunable.h>
#include <core/pm/function_scheduler/function_scheduler.h>
#include <ctl/component/dsa/dsa_dl_scope.h>

#include <string.h>
#include "driverlib_cm.h"
#include "cm.h"
#include "lwip/init.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "tricore_shared.h"
#include "utils/lwiplib.h"

#if GMP_PORT_DATA_SIZE_PER_BYTES != 2
#error "F28388D CM must inherit the C28x-family u16 Data Link backend"
#endif

#define CM_DL_TUNABLE_CMD (0x30U)
#define CM_DL_MEMORY_CMD  (0x50U)
#define CM_DL_SCOPE_CMD   (0x60U)
#define CM_SCOPE_DEPTH    (400UL)

#pragma DATA_SECTION(cm_to_cpu2_command, "MSGRAM_CM_TO_CPU2")
volatile gmp_wave_command_t cm_to_cpu2_command;
#pragma DATA_SECTION(cpu2_to_cm_snapshot, "MSGRAM_CPU2_TO_CM")
volatile gmp_wave_snapshot_t cpu2_to_cm_snapshot;

extern void CM_init(void);
extern void Ethernet_init(const unsigned char *mac);
extern uint32_t systickPeriodValue;
extern void SysTickIntHandler(void);

static gmp_datalink_t cm_datalink;
static gmp_param_tunable_t cm_tunable;
static gmp_mem_persp_t cm_memory;
static ctl_dsa_dl_scope_t cm_scope;
static ctrl_gt cm_scope_storage[
    CTL_DSA_DL_SCOPE_STORAGE_ELEMENTS(2U, CM_SCOPE_DEPTH)];
static byte_gt cm_scratch[128];
static gmp_scheduler_t cm_scheduler;

float cm_frequency_hz = GMP_TRICORE_DEFAULT_FREQ_HZ;
float cm_gain = 1.0F;
float cm_offset = 0.0F;
volatile uint32_t cm_dl_errors;
volatile uint32_t cm_rx_frames;
volatile uint32_t cm_tx_frames;
volatile uint32_t cm_scheduler_heartbeats;
volatile uint32_t cm_ethercat_ready;

#if GMP_F28388D_CM_DL_TRANSPORT == GMP_F28388D_CM_DL_UDP
static struct udp_pcb *cm_udp;
static ip_addr_t cm_udp_peer;
static u16_t cm_udp_peer_port;
static uint32_t cm_udp_peer_valid;
#else
static struct tcp_pcb *cm_tcp_listener;
static struct tcp_pcb *cm_tcp_client;
#endif

static const gmp_param_item_t cm_tunable_dictionary[] = {
    {&cm_frequency_hz, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, "CPU2 sine frequency (Hz)"},
    {&cm_gain, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, "CPU2 sine gain"},
    {&cm_offset, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, "CPU2 sine offset"}
};

static const gmp_mem_region_t cm_memory_regions[] = {
    {cm_scratch, sizeof(cm_scratch), GMP_MEM_PERM_RW, "CM scratch"},
    {(void *)&cpu2_to_cm_snapshot, sizeof(cpu2_to_cm_snapshot),
     GMP_MEM_PERM_RO, "CPU2 waveform snapshot"}
};

static void cm_publish_command(void)
{
    uint32_t sequence = cm_to_cpu2_command.sequence_end + 2UL;
    cm_to_cpu2_command.sequence_begin = sequence | 1UL;
    cm_to_cpu2_command.magic = GMP_TRICORE_MAGIC;
    cm_to_cpu2_command.frequency_hz = cm_frequency_hz;
    cm_to_cpu2_command.gain = cm_gain;
    cm_to_cpu2_command.offset = cm_offset;
    cm_to_cpu2_command.sequence_end = sequence;
    cm_to_cpu2_command.sequence_begin = sequence;
}

static void cm_push_pbuf(const struct pbuf *packet)
{
    const struct pbuf *segment;
    for (segment = packet; segment != NULL; segment = segment->next)
    {
        const uint8_t *source = (const uint8_t *)segment->payload;
        u16_t remaining = segment->len;
        while (remaining != 0U)
        {
            byte_gt units[64];
            u16_t count = remaining > 64U ? 64U : remaining;
            u16_t index;
            for (index = 0U; index < count; ++index)
                units[index] = (byte_gt)source[index];
            gmp_dev_dl_push_str(&cm_datalink, units, (size_gt)count);
            source += count;
            remaining -= count;
        }
    }
    cm_rx_frames++;
}

static u16_t cm_pack_tx_frame(uint8_t *frame, u16_t capacity)
{
    const byte_gt *header = gmp_dev_dl_get_tx_hw_hdr_ptr(&cm_datalink);
    const byte_gt *payload = gmp_dev_dl_get_tx_hw_pld_ptr(&cm_datalink);
    u16_t header_size = (u16_t)gmp_dev_dl_get_tx_hw_hdr_size(&cm_datalink);
    u16_t payload_size = (u16_t)gmp_dev_dl_get_tx_hw_pld_size(&cm_datalink);
    u16_t index;
    if ((u16_t)(header_size + payload_size) > capacity)
        return 0U;
    for (index = 0U; index < header_size; ++index)
        frame[index] = (uint8_t)(header[index] & 0xFFU);
    for (index = 0U; index < payload_size; ++index)
        frame[header_size + index] = (uint8_t)(payload[index] & 0xFFU);
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
    cm_push_pbuf(packet);
    pbuf_free(packet);
}

static err_t cm_network_send(void)
{
    uint8_t frame[544];
    u16_t frame_size = cm_pack_tx_frame(frame, (u16_t)sizeof(frame));
    struct pbuf *packet;
    err_t result;
    if ((frame_size == 0U) || (cm_udp == NULL) || (cm_udp_peer_valid == 0UL))
        return ERR_CONN;
    packet = pbuf_alloc(PBUF_TRANSPORT, frame_size, PBUF_RAM);
    if (packet == NULL)
        return ERR_MEM;
    result = pbuf_take(packet, frame, frame_size);
    if (result == ERR_OK)
        result = udp_sendto(cm_udp, packet, &cm_udp_peer, cm_udp_peer_port);
    pbuf_free(packet);
    return result;
}
#else
static void cm_tcp_error(void *arg, err_t error)
{
    GMP_UNUSED_VAR(arg);
    GMP_UNUSED_VAR(error);
    cm_tcp_client = NULL;
    cm_dl_errors++;
}

static err_t cm_tcp_receive(void *arg, struct tcp_pcb *pcb,
                            struct pbuf *packet, err_t error)
{
    GMP_UNUSED_VAR(arg);
    if (error != ERR_OK)
    {
        if (packet != NULL) pbuf_free(packet);
        return error;
    }
    if (packet == NULL)
    {
        cm_tcp_client = NULL;
        tcp_recv(pcb, NULL);
        return tcp_close(pcb);
    }
    tcp_recved(pcb, packet->tot_len);
    cm_push_pbuf(packet);
    pbuf_free(packet);
    return ERR_OK;
}

static err_t cm_tcp_accept(void *arg, struct tcp_pcb *pcb, err_t error)
{
    GMP_UNUSED_VAR(arg);
    if (error != ERR_OK) return error;
    if (cm_tcp_client != NULL)
    {
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    cm_tcp_client = pcb;
    tcp_nagle_disable(pcb);
    tcp_recv(pcb, cm_tcp_receive);
    tcp_err(pcb, cm_tcp_error);
    return ERR_OK;
}

static err_t cm_network_send(void)
{
    uint8_t frame[544];
    u16_t frame_size = cm_pack_tx_frame(frame, (u16_t)sizeof(frame));
    err_t result;
    if ((frame_size == 0U) || (cm_tcp_client == NULL)) return ERR_CONN;
    if (tcp_sndbuf(cm_tcp_client) < frame_size) return ERR_MEM;
    result = tcp_write(cm_tcp_client, frame, frame_size, 0U);
    if (result == ERR_OK) result = tcp_output(cm_tcp_client);
    return result;
}
#endif

static void cm_network_server_init(void)
{
#if GMP_F28388D_CM_DL_TRANSPORT == GMP_F28388D_CM_DL_UDP
    cm_udp = udp_new();
    if ((cm_udp == NULL) ||
        (udp_bind(cm_udp, IP_ADDR_ANY, GMP_F28388D_CM_UDP_PORT) != ERR_OK))
        cm_dl_errors++;
    else
        udp_recv(cm_udp, cm_udp_receive, NULL);
#else
    struct tcp_pcb *server = tcp_new_ip_type(IPADDR_TYPE_V4);
    if ((server == NULL) ||
        (tcp_bind(server, IP_ADDR_ANY, GMP_F28388D_CM_TCP_PORT) != ERR_OK))
    {
        cm_dl_errors++;
        return;
    }
    cm_tcp_listener = tcp_listen_with_backlog(server, 1U);
    if (cm_tcp_listener == NULL)
        cm_dl_errors++;
    else
        tcp_accept(cm_tcp_listener, cm_tcp_accept);
#endif
}

static gmp_task_status_t cm_dl_task(gmp_task_t *task)
{
    gmp_dl_event_t event;
    GMP_UNUSED_VAR(task);
    event = gmp_dev_dl_loop_cb(&cm_datalink);
    if (event == GMP_DL_EVENT_TX_RDY)
    {
        if (cm_network_send() == ERR_OK) cm_tx_frames++;
        else cm_dl_errors++;
        gmp_dev_dl_tx_state_done(&cm_datalink);
    }
    else if (event == GMP_DL_EVENT_RX_OK)
        (void)gmp_dev_dl_dispatch_rx(&cm_datalink);
    cm_publish_command();
    return GMP_TASK_DONE;
}

static gmp_task_status_t cm_scope_task(gmp_task_t *task)
{
    GMP_UNUSED_VAR(task);
    if (gmp_wave_snapshot_valid(&cpu2_to_cm_snapshot))
        ctl_step_dsa_dl_scope_2ch(&cm_scope,
            real2ctrl(cpu2_to_cm_snapshot.scaled_sine),
            real2ctrl(cpu2_to_cm_snapshot.scaled_cosine));
    return GMP_TASK_DONE;
}

static gmp_task_status_t cm_heartbeat_task(gmp_task_t *task)
{
    GMP_UNUSED_VAR(task);
    cm_scheduler_heartbeats++;
    return GMP_TASK_DONE;
}

static gmp_task_status_t cm_communication_health_task(gmp_task_t *task)
{
    GMP_UNUSED_VAR(task);
    cm_ethercat_ready =
        (uint32_t)ESCSS_getMemoryInitDoneStatusNonBlocking(ESC_SS_BASE);
    return GMP_TASK_DONE;
}

static gmp_task_t cm_tasks[] = {
    {"ethernet-dl-u16", cm_dl_task, 1U, 0U, 1, NULL},
    {"scope", cm_scope_task, 1U, 0U, 1, NULL},
    {"communication-health", cm_communication_health_task, 100U, 0U, 1, NULL},
    {"heartbeat", cm_heartbeat_task, 500U, 0U, 1, NULL}
};

static void cm_init_ethercat_owner(void)
{
    // CPU1 has already assigned the shared block and its pins to CM.  CM owns
    // the EtherCAT reset, memory and (later) SSC protocol service lifecycle.
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAT);
    ESCSS_configureEEPROMSize(ESC_SS_CONFIG_BASE, ESCSS_LESS_THAN_16K);
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_ECAT);
    ESCSS_initMemory(ESC_SS_BASE);
    cm_ethercat_ready = (uint32_t)ESCSS_getMemoryInitDoneStatusBlocking(
        ESC_SS_BASE, 0x300UL);
}

void gmp_c28x_syscfg_cm_device_init(void)
{
    unsigned char mac[6] = {0xA8U, 0x63U, 0xF2U, 0x00U, 0x28U, 0x88U};
    CM_init();
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_USB);
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_USB);
    USBDevMode(USB0_BASE);
    cm_init_ethercat_owner();
    Ethernet_init(mac);
    lwIPInit(0U, mac, 0xC0A88902UL, 0xFFFFFF00UL, 0U, IPADDR_USE_STATIC);
    systickPeriodValue = 125000UL;
    SYSTICK_setPeriod(systickPeriodValue);
    SYSTICK_registerInterruptHandler(SysTickIntHandler);
    SYSTICK_enableInterrupt();
    SYSTICK_enableCounter();
}

void gmp_c28x_syscfg_cm_device_loop(void)
{
    sys_check_timeouts();
}

void gmp_c28x_syscfg_cm_post_start(void)
{
}

void setup_peripheral(void)
{
}

void init(void)
{
    size_gt index;
    for (index = 0U; index < sizeof(cm_scratch) / sizeof(cm_scratch[0]); ++index)
        cm_scratch[index] = (byte_gt)index;
    gmp_dev_dl_init(&cm_datalink);
    gmp_param_tunable_init(&cm_tunable, &cm_datalink, CM_DL_TUNABLE_CMD,
        cm_tunable_dictionary,
        (fast16_gt)(sizeof(cm_tunable_dictionary) / sizeof(cm_tunable_dictionary[0])));
    (void)gmp_dev_dl_append_facility(&cm_datalink, &cm_tunable.facility);
    gmp_mem_persp_init(&cm_memory, &cm_datalink, CM_DL_MEMORY_CMD,
        cm_memory_regions,
        (fast16_gt)(sizeof(cm_memory_regions) / sizeof(cm_memory_regions[0])));
    (void)gmp_dev_dl_append_facility(&cm_datalink, &cm_memory.facility);
    if (!ctl_init_dsa_dl_scope_workspace(&cm_scope, &cm_datalink,
        CM_DL_SCOPE_CMD, "CPU2 sine/cosine", cm_scope_storage,
        (uint32_t)(sizeof(cm_scope_storage) / sizeof(cm_scope_storage[0])),
        2U, 1000UL) ||
        !gmp_dev_dl_append_facility(&cm_datalink,
            ctl_dsa_dl_scope_facility(&cm_scope)))
        cm_dl_errors++;
    cm_publish_command();
    cm_network_server_init();
    gmp_scheduler_init(&cm_scheduler);
    for (index = 0U; index < sizeof(cm_tasks) / sizeof(cm_tasks[0]); ++index)
        (void)gmp_scheduler_add_task(&cm_scheduler, &cm_tasks[index]);
}

void mainloop(void)
{
    gmp_scheduler_dispatch(&cm_scheduler);
}
