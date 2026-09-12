/** @file user_main.c CPU1 application, scheduler and serial Data Link service. */

#include <gmp_core.h>
#include <core/dev/datalink/mem_presp.h>
#include <core/dev/datalink/tunable.h>
#include <core/pm/function_scheduler/function_scheduler.h>
#include <ctl/component/dsa/dsa_dl_scope.h>

#include "tricore_shared.h"
#include "user_main.h"
#include <xplt.peripheral.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES != 2
#error "F28388D CPU1 must use the native C28x u16 Data Link backend"
#endif

#define CPU1_DL_TUNABLE_CMD  (0x30U)
#define CPU1_DL_MEMORY_CMD   (0x50U)
#define CPU1_DL_SCOPE_CMD    (0x60U)
#define CPU1_SCOPE_DEPTH     (400UL)
#define CPU1_SCOPE_CHANNELS  (2U)

static gmp_datalink_t cpu1_datalink;
static gmp_param_tunable_t cpu1_tunable;
static gmp_mem_persp_t cpu1_memory;
static ctl_dsa_dl_scope_t cpu1_scope;
static ctrl_gt cpu1_scope_storage[
    CTL_DSA_DL_SCOPE_STORAGE_ELEMENTS(CPU1_SCOPE_CHANNELS, CPU1_SCOPE_DEPTH)];
static byte_gt cpu1_scratch[64];
static gmp_scheduler_t cpu1_scheduler;

float cpu2_frequency_hz = GMP_TRICORE_DEFAULT_FREQ_HZ;
float cpu2_gain = 1.0F;
float cpu2_offset = 0.0F;
volatile uint32_t cpu1_dl_errors;
volatile uint32_t cpu1_timer_ticks;
volatile uint32_t cpu1_snapshot_updates;

static const gmp_param_item_t cpu1_tunable_dictionary[] = {
    {&cpu2_frequency_hz, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, "CPU2 sine frequency (Hz)"},
    {&cpu2_gain, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, "CPU2 sine gain"},
    {&cpu2_offset, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, "CPU2 sine offset"}
};

static const gmp_mem_region_t cpu1_memory_regions[] = {
    {cpu1_scratch, sizeof(cpu1_scratch) * GMP_PORT_DATA_SIZE_PER_BYTES,
     GMP_MEM_PERM_RW, "CPU1 scratch"},
    {(void *)&cpu2_to_cpu1_snapshot,
     sizeof(cpu2_to_cpu1_snapshot) * GMP_PORT_DATA_SIZE_PER_BYTES,
     GMP_MEM_PERM_RO, "CPU2 waveform snapshot"}
};

static void cpu1_publish_command(void)
{
    static float published_frequency;
    static float published_gain;
    static float published_offset;
    static fast_gt initialized;
    uint32_t sequence;
    if (initialized && published_frequency == cpu2_frequency_hz &&
        published_gain == cpu2_gain && published_offset == cpu2_offset)
        return;
    sequence = cpu1_to_cpu2_command.sequence_end + 2UL;
    cpu1_to_cpu2_command.sequence_begin = sequence | 1UL;
    cpu1_to_cpu2_command.magic = GMP_TRICORE_MAGIC;
    cpu1_to_cpu2_command.frequency_hz = cpu2_frequency_hz;
    cpu1_to_cpu2_command.gain = cpu2_gain;
    cpu1_to_cpu2_command.offset = cpu2_offset;
    cpu1_to_cpu2_command.sequence_end = sequence;
    cpu1_to_cpu2_command.sequence_begin = sequence;
    published_frequency = cpu2_frequency_hz;
    published_gain = cpu2_gain;
    published_offset = cpu2_offset;
    initialized = 1;
}

static gmp_task_status_t cpu1_dl_task(gmp_task_t *task)
{
    gmp_dl_event_t event;
    GMP_UNUSED_VAR(task);
    xplt_cpu1_dl_receive(&cpu1_datalink);
    event = gmp_dev_dl_loop_cb(&cpu1_datalink);
    if (event == GMP_DL_EVENT_TX_RDY)
    {
        if (xplt_cpu1_dl_send(&cpu1_datalink))
            gmp_dev_dl_tx_state_done(&cpu1_datalink);
        else
            cpu1_dl_errors++;
    }
    else if (event == GMP_DL_EVENT_RX_OK)
    {
        (void)gmp_dev_dl_dispatch_rx(&cpu1_datalink);
    }
    cpu1_publish_command();
    return GMP_TASK_DONE;
}

static gmp_task_status_t cpu1_led_task(gmp_task_t *task)
{
    GMP_UNUSED_VAR(task);
    xplt_cpu1_toggle_status_led();
    return GMP_TASK_DONE;
}

static gmp_task_t cpu1_tasks[] = {
    {"datalink-u16", cpu1_dl_task, 1U, 0U, 1, NULL},
    {"heartbeat", cpu1_led_task, 500U, 0U, 1, NULL}
};

void user_cpu1_control_step(void)
{
    gmp_wave_snapshot_t snapshot;
    cpu1_timer_ticks++;
    if (gmp_wave_snapshot_read(&cpu2_to_cpu1_snapshot, &snapshot))
    {
        ctl_step_dsa_dl_scope_2ch(&cpu1_scope,
            real2ctrl(snapshot.scaled_sine), real2ctrl(snapshot.scaled_cosine));
        cpu1_snapshot_updates++;
    }
}

void init(void)
{
    size_gt index;
    size_gt task_index;
    for (index = 0U; index < sizeof(cpu1_scratch) / sizeof(cpu1_scratch[0]); ++index)
        cpu1_scratch[index] = (byte_gt)index;

    gmp_dev_dl_init(&cpu1_datalink);
    gmp_param_tunable_init(&cpu1_tunable, &cpu1_datalink,
        CPU1_DL_TUNABLE_CMD, cpu1_tunable_dictionary,
        (fast16_gt)(sizeof(cpu1_tunable_dictionary) / sizeof(cpu1_tunable_dictionary[0])));
    if (!gmp_dev_dl_append_facility(&cpu1_datalink, &cpu1_tunable.facility))
        cpu1_dl_errors++;
    gmp_mem_persp_init(&cpu1_memory, &cpu1_datalink, CPU1_DL_MEMORY_CMD,
        cpu1_memory_regions,
        (fast16_gt)(sizeof(cpu1_memory_regions) / sizeof(cpu1_memory_regions[0])));
    if (!gmp_dev_dl_append_facility(&cpu1_datalink, &cpu1_memory.facility))
        cpu1_dl_errors++;
    if (!ctl_init_dsa_dl_scope_workspace(&cpu1_scope, &cpu1_datalink,
        CPU1_DL_SCOPE_CMD, "CPU2 sine/cosine", cpu1_scope_storage,
        (uint32_t)(sizeof(cpu1_scope_storage) / sizeof(cpu1_scope_storage[0])),
        CPU1_SCOPE_CHANNELS, 1000UL) ||
        !gmp_dev_dl_append_facility(&cpu1_datalink,
            ctl_dsa_dl_scope_facility(&cpu1_scope)))
        cpu1_dl_errors++;

    cpu1_publish_command();
    gmp_scheduler_init(&cpu1_scheduler);
    for (task_index = 0U;
         task_index < sizeof(cpu1_tasks) / sizeof(cpu1_tasks[0]);
         ++task_index)
        (void)gmp_scheduler_add_task(&cpu1_scheduler, &cpu1_tasks[task_index]);
}

void mainloop(void)
{
    gmp_scheduler_dispatch(&cpu1_scheduler);
}
