/** @file user_main.c CPU2 scheduler and deterministic waveform application. */

#include <gmp_core.h>
#include <core/pm/function_scheduler/function_scheduler.h>
#include <math.h>

#include "tricore_shared.h"
#include "user_main.h"
#include <xplt.peripheral.h>

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
    uint32_t magic = command->magic;
    float local_frequency = command->frequency_hz;
    float local_gain = command->gain;
    float local_offset = command->offset;
    uint32_t end = command->sequence_end;
    if ((begin != end) || ((begin & 1UL) != 0UL) ||
        (magic != GMP_TRICORE_MAGIC))
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

void user_cpu2_control_step(void)
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
