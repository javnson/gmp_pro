/** @file user_main.c Platform-independent CM Data Link application. */

#include <gmp_core.h>
#include <core/dev/datalink/mem_presp.h>
#include <core/dev/datalink/tunable.h>
#include <core/pm/function_scheduler/function_scheduler.h>
#include <ctl/component/dsa/dsa_dl_scope.h>

#include "tricore_shared.h"
#include "user_main.h"
#include <xplt.peripheral.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES != 2
#error "F28388D CM must inherit the C28x-family u16 Data Link backend"
#endif

#define CM_DL_TUNABLE_CMD (0x30U)
#define CM_DL_MEMORY_CMD  (0x50U)
#define CM_DL_SCOPE_CMD   (0x60U)
#define CM_SCOPE_DEPTH    (400UL)

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
volatile uint32_t cm_tx_retry_pending;
volatile uint32_t cm_scheduler_heartbeats;
volatile uint32_t cm_ethercat_ready;

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
    static float published_frequency;
    static float published_gain;
    static float published_offset;
    static fast_gt initialized;
    uint32_t sequence;
    if (initialized && published_frequency == cm_frequency_hz &&
        published_gain == cm_gain && published_offset == cm_offset)
        return;
    sequence = cm_to_cpu2_command.sequence_end + 2UL;
    cm_to_cpu2_command.sequence_begin = sequence | 1UL;
    cm_to_cpu2_command.magic = GMP_TRICORE_MAGIC;
    cm_to_cpu2_command.frequency_hz = cm_frequency_hz;
    cm_to_cpu2_command.gain = cm_gain;
    cm_to_cpu2_command.offset = cm_offset;
    cm_to_cpu2_command.sequence_end = sequence;
    cm_to_cpu2_command.sequence_begin = sequence;
    published_frequency = cm_frequency_hz;
    published_gain = cm_gain;
    published_offset = cm_offset;
    initialized = 1;
}

static void cm_try_send_pending_frame(void)
{
    xplt_cm_dl_send_result_t result = xplt_cm_dl_send(&cm_datalink);
    if (result == XPLT_CM_DL_SEND_OK)
    {
        cm_tx_retry_pending = 0UL;
        gmp_dev_dl_tx_state_done(&cm_datalink);
    }
    else if (result == XPLT_CM_DL_SEND_RETRY)
    {
        cm_tx_retry_pending = 1UL;
    }
    else
    {
        cm_dl_errors++;
        cm_tx_retry_pending = 0UL;
        gmp_dev_dl_tx_state_done(&cm_datalink);
    }
}

static gmp_task_status_t cm_dl_task(gmp_task_t *task)
{
    gmp_dl_event_t event;
    GMP_UNUSED_VAR(task);
    xplt_cm_dl_receive(&cm_datalink);
    if (cm_tx_retry_pending != 0UL)
    {
        cm_try_send_pending_frame();
        return GMP_TASK_DONE;
    }
    event = gmp_dev_dl_loop_cb(&cm_datalink);
    if (event == GMP_DL_EVENT_TX_RDY)
        cm_try_send_pending_frame();
    else if (event == GMP_DL_EVENT_RX_OK)
        (void)gmp_dev_dl_dispatch_rx(&cm_datalink);
    cm_publish_command();
    return GMP_TASK_DONE;
}

static gmp_task_status_t cm_scope_task(gmp_task_t *task)
{
    gmp_wave_snapshot_t snapshot;
    GMP_UNUSED_VAR(task);
    if (gmp_wave_snapshot_read(&cpu2_to_cm_snapshot, &snapshot))
        ctl_step_dsa_dl_scope_2ch(&cm_scope,
            real2ctrl(snapshot.scaled_sine), real2ctrl(snapshot.scaled_cosine));
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
    cm_ethercat_ready = xplt_cm_ethercat_memory_ready();
    return GMP_TASK_DONE;
}

static gmp_task_t cm_tasks[] = {
    {"ethernet-dl-u16", cm_dl_task, 1U, 0U, 1, NULL},
    {"scope", cm_scope_task, 1U, 0U, 1, NULL},
    {"communication-health", cm_communication_health_task, 100U, 0U, 1, NULL},
    {"heartbeat", cm_heartbeat_task, 500U, 0U, 1, NULL}
};

void init(void)
{
    size_gt index;
    for (index = 0U; index < sizeof(cm_scratch) / sizeof(cm_scratch[0]); ++index)
        cm_scratch[index] = (byte_gt)index;
    gmp_dev_dl_init(&cm_datalink);
    gmp_param_tunable_init(&cm_tunable, &cm_datalink, CM_DL_TUNABLE_CMD,
        cm_tunable_dictionary,
        (fast16_gt)(sizeof(cm_tunable_dictionary) / sizeof(cm_tunable_dictionary[0])));
    if (!gmp_dev_dl_append_facility(&cm_datalink, &cm_tunable.facility))
        cm_dl_errors++;
    gmp_mem_persp_init(&cm_memory, &cm_datalink, CM_DL_MEMORY_CMD,
        cm_memory_regions,
        (fast16_gt)(sizeof(cm_memory_regions) / sizeof(cm_memory_regions[0])));
    if (!gmp_dev_dl_append_facility(&cm_datalink, &cm_memory.facility))
        cm_dl_errors++;
    if (!ctl_init_dsa_dl_scope_workspace(&cm_scope, &cm_datalink,
        CM_DL_SCOPE_CMD, "CPU2 sine/cosine", cm_scope_storage,
        (uint32_t)(sizeof(cm_scope_storage) / sizeof(cm_scope_storage[0])),
        2U, 1000UL) ||
        !gmp_dev_dl_append_facility(&cm_datalink,
            ctl_dsa_dl_scope_facility(&cm_scope)))
        cm_dl_errors++;
    cm_publish_command();
    gmp_scheduler_init(&cm_scheduler);
    for (index = 0U; index < sizeof(cm_tasks) / sizeof(cm_tasks[0]); ++index)
        (void)gmp_scheduler_add_task(&cm_scheduler, &cm_tasks[index]);
}

void mainloop(void)
{
    gmp_scheduler_dispatch(&cm_scheduler);
}
