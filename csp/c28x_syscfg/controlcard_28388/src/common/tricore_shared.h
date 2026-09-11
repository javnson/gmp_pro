#ifndef GMP_F28388D_TRICORE_SHARED_H
#define GMP_F28388D_TRICORE_SHARED_H

#include <stdint.h>

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

static inline uint32_t gmp_wave_snapshot_valid(const volatile gmp_wave_snapshot_t *snapshot)
{
    uint32_t begin = snapshot->sequence_begin;
    uint32_t end = snapshot->sequence_end;
    return ((begin == end) && ((begin & 1UL) == 0UL) &&
            (snapshot->magic == GMP_TRICORE_MAGIC));
}

#endif
