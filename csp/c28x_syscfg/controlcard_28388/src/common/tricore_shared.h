#ifndef GMP_F28388D_TRICORE_SHARED_H
#define GMP_F28388D_TRICORE_SHARED_H

#include <stdint.h>
#include <limits.h>

#define GMP_TRICORE_MAGIC            (0x47323838UL)
#define GMP_TRICORE_DEFAULT_FREQ_HZ  (50.0F)
#define GMP_TRICORE_SAMPLE_RATE_HZ   (1000.0F)

/* Every field is 32 bits so the C28x and Cortex-M views have identical layout. */
typedef struct
{
    volatile uint32_t sequence_begin;
    volatile uint32_t magic;
    volatile float frequency_hz;
    volatile float gain;
    volatile float offset;
    volatile uint32_t sequence_end;
} gmp_wave_command_t;

typedef struct
{
    volatile uint32_t sequence_begin;
    volatile uint32_t magic;
    volatile float sine;
    volatile float cosine;
    volatile float scaled_sine;
    volatile float scaled_cosine;
    volatile uint32_t sample_count;
    volatile uint32_t source_core;
    volatile uint32_t sequence_end;
} gmp_wave_snapshot_t;

/* sizeof() counts 16-bit C bytes on C28x and 8-bit C bytes on CM. */
typedef char gmp_wave_command_layout_must_be_192_bits[
    (sizeof(gmp_wave_command_t) * CHAR_BIT == 192U) ? 1 : -1];
typedef char gmp_wave_snapshot_layout_must_be_288_bits[
    (sizeof(gmp_wave_snapshot_t) * CHAR_BIT == 288U) ? 1 : -1];

static inline uint32_t gmp_wave_snapshot_valid(const volatile gmp_wave_snapshot_t *snapshot)
{
    uint32_t begin = snapshot->sequence_begin;
    uint32_t end = snapshot->sequence_end;
    return ((begin == end) && ((begin & 1UL) == 0UL) &&
            (snapshot->magic == GMP_TRICORE_MAGIC));
}

/* Read a shared snapshot through its seqlock.  Checking validity and then
 * reading fields separately is unsafe because the producer can begin the next
 * update between those operations. */
static inline uint32_t gmp_wave_snapshot_read(
    const volatile gmp_wave_snapshot_t *source, gmp_wave_snapshot_t *result)
{
    uint32_t begin;
    uint32_t end;
    uint32_t magic;
    if (source == (const volatile gmp_wave_snapshot_t *)0 ||
        result == (gmp_wave_snapshot_t *)0)
        return 0UL;
    begin = source->sequence_begin;
    magic = source->magic;
    result->sine = source->sine;
    result->cosine = source->cosine;
    result->scaled_sine = source->scaled_sine;
    result->scaled_cosine = source->scaled_cosine;
    result->sample_count = source->sample_count;
    result->source_core = source->source_core;
    end = source->sequence_end;
    result->sequence_begin = begin;
    result->magic = magic;
    result->sequence_end = end;
    return ((begin == end) && ((begin & 1UL) == 0UL) &&
            (magic == GMP_TRICORE_MAGIC));
}

#endif
