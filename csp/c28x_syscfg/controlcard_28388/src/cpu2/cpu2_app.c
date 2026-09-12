/** @file cpu2_app.c CPU2 scheduler and deterministic sine-wave computation. */

#include <gmp_core.h>
#include <core/pm/function_scheduler/function_scheduler.h>
#include <math.h>

#include "device.h"
#include "driverlib.h"
#include "tricore_shared.h"

#pragma DATA_SECTION(cpu1_to_cpu2_command, "GMP_MSGRAM_CPU1_TO_CPU2")
volatile gmp_wave_command_t cpu1_to_cpu2_command;
#pragma DATA_SECTION(cm_to_cpu2_command, "GMP_MSGRAM_CM_TO_CPU")
volatile gmp_wave_command_t cm_to_cpu2_command;
#pragma DATA_SECTION(cpu2_to_cpu1_snapshot, "GMP_MSGRAM_CPU2_TO_CPU1")
volatile gmp_wave_snapshot_t cpu2_to_cpu1_snapshot;
#pragma DATA_SECTION(cpu2_to_cm_snapshot, "GMP_MSGRAM_CPU_TO_CM")
volatile gmp_wave_snapshot_t cpu2_to_cm_snapshot;

static gmp_scheduler_t cpu2_scheduler;
static volatile float wave_sine;
static volatile float wave_cosine = 1.0F;
static volatile float wave_step_sine;
static volatile float wave_step_cosine = 1.0F;
static volatile float wave_gain = 1.0F;
static volatile float wave_offset;
static volatile uint32_t wave_sample_count;
volatile uint32_t cpu2_scheduler_heartbeats;
volatile uint32_t cpu2_command_updates;

static uint32_t cpu2_read_command(const volatile gmp_wave_command_t *command,
                                  float *frequency, float *gain, float *offset)
{
    uint32_t begin = command->sequence_begin;
    float local_frequency = command->frequency_hz;
    float local_gain = command->gain;
    float local_offset = command->offset;
    uint32_t end = command->sequence_end;
    if ((begin != end) || ((begin & 1UL) != 0UL) ||
        (command->magic != GMP_TRICORE_MAGIC))
        return 0UL;
    *frequency = local_frequency;
    *gain = local_gain;
    *offset = local_offset;
    return end;
}

static gmp_task_status_t cpu2_command_task(gmp_task_t *task)
{
    static uint32_t last_cpu1_sequence;
    static uint32_t last_cm_sequence;
    float frequency;
    float gain;
    float offset;
    uint32_t sequence;
    GMP_UNUSED_VAR(task);

    sequence = cpu2_read_command(&cm_to_cpu2_command, &frequency, &gain, &offset);
    if ((sequence != 0UL) && (sequence != last_cm_sequence))
        last_cm_sequence = sequence;
    else
    {
        sequence = cpu2_read_command(&cpu1_to_cpu2_command,
                                     &frequency, &gain, &offset);
        if ((sequence == 0UL) || (sequence == last_cpu1_sequence))
            return GMP_TASK_DONE;
        last_cpu1_sequence = sequence;
    }

    if (frequency < 1.0F) frequency = 1.0F;
    if (frequency > 200.0F) frequency = 200.0F;
    if (gain < 0.0F) gain = 0.0F;
    if (gain > 10.0F) gain = 10.0F;
    if (offset < -10.0F) offset = -10.0F;
    if (offset > 10.0F) offset = 10.0F;
    frequency *= 6.2831853071795864769F / GMP_TRICORE_SAMPLE_RATE_HZ;
    DINT;
    wave_step_sine = sinf(frequency);
    wave_step_cosine = cosf(frequency);
    wave_gain = gain;
    wave_offset = offset;
    EINT;
    cpu2_command_updates++;
    return GMP_TASK_DONE;
}

static gmp_task_status_t cpu2_heartbeat_task(gmp_task_t *task)
{
    GMP_UNUSED_VAR(task);
    cpu2_scheduler_heartbeats++;
    return GMP_TASK_DONE;
}

static gmp_task_t cpu2_tasks[] = {
    {"command", cpu2_command_task, 1U, 0U, 1, NULL},
    {"heartbeat", cpu2_heartbeat_task, 500U, 0U, 1, NULL}
};

static void cpu2_publish(volatile gmp_wave_snapshot_t *snapshot,
                         float sine, float cosine, float scaled_sine,
                         float scaled_cosine, uint32_t count)
{
    uint32_t sequence = snapshot->sequence_end + 2UL;
    snapshot->sequence_begin = sequence | 1UL;
    snapshot->magic = GMP_TRICORE_MAGIC;
    snapshot->sine = sine;
    snapshot->cosine = cosine;
    snapshot->scaled_sine = scaled_sine;
    snapshot->scaled_cosine = scaled_cosine;
    snapshot->sample_count = count;
    snapshot->source_core = 2UL;
    snapshot->sequence_end = sequence;
    snapshot->sequence_begin = sequence;
}

__interrupt static void cpu2_timer_isr(void)
{
    float sine = wave_sine;
    float cosine = wave_cosine;
    float next_sine = sine * wave_step_cosine + cosine * wave_step_sine;
    float next_cosine = cosine * wave_step_cosine - sine * wave_step_sine;
    uint32_t count = ++wave_sample_count;
    wave_sine = next_sine;
    wave_cosine = next_cosine;
    cpu2_publish(&cpu2_to_cpu1_snapshot, sine, cosine,
                 sine * wave_gain + wave_offset,
                 cosine * wave_gain + wave_offset, count);
    cpu2_publish(&cpu2_to_cm_snapshot, sine, cosine,
                 sine * wave_gain + wave_offset,
                 cosine * wave_gain + wave_offset, count);
    gmp_step_system_tick();
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

void setup_peripheral(void)
{
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    /* CPU1 owns the PLL setup. CPU2 consumes the shared 200 MHz board clock
     * contract instead of asking its pin-free SysConfig context to emit a
     * second, potentially divergent clock tree. */
    CPUTimer_setPeriod(CPUTIMER0_BASE,
                       (GMP_F28388D_CPU_CLOCK_HZ / 1000UL) - 1UL);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_register(INT_TIMER0, &cpu2_timer_isr);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_startTimer(CPUTIMER0_BASE);
}

void init(void)
{
    size_gt index;
    gmp_scheduler_init(&cpu2_scheduler);
    for (index = 0U; index < sizeof(cpu2_tasks) / sizeof(cpu2_tasks[0]); ++index)
        (void)gmp_scheduler_add_task(&cpu2_scheduler, &cpu2_tasks[index]);
}

void mainloop(void)
{
    gmp_scheduler_dispatch(&cpu2_scheduler);
}
