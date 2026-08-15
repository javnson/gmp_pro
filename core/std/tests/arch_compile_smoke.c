#include <limits.h>

#include <core/std/gmp.std.h>

#define GMP_COMPILE_ASSERT(name, condition) typedef char name[(condition) ? 1 : -1]

GMP_COMPILE_ASSERT(gmp_arch_must_be_detected, GMP_ARCH_TYPE != GMP_ARCH_UNKNOWN);
GMP_COMPILE_ASSERT(gmp_byte_width_matches_contract,
                   sizeof(byte_gt) * CHAR_BIT == GMP_PORT_DATA_SIZE_PER_BITS);
GMP_COMPILE_ASSERT(gmp_fast_width_matches_contract,
                   sizeof(fast_gt) * CHAR_BIT >= GMP_PORT_FAST_SIZE_PER_BITS);
GMP_COMPILE_ASSERT(gmp_time_width_matches_contract,
                   sizeof(time_gt) * CHAR_BIT == GMP_PORT_TIME_SIZE_PER_BITS);
GMP_COMPILE_ASSERT(gmp_size_width_matches_contract,
                   sizeof(size_gt) * CHAR_BIT == GMP_PORT_SIZE_SIZE_PER_BITS);

int gmp_arch_compile_smoke(void)
{
    byte_gt data = (byte_gt)0;
    fast_gt state = (fast_gt)0;
    time_gt tick = (time_gt)0;
    size_gt count = (size_gt)0;

    return (int)(data + state + (fast_gt)tick + (fast_gt)count);
}
