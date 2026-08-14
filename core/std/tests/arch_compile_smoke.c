/**
 * @file arch_compile_smoke.c
 * @brief Compiler-target smoke test for GMP architecture type selection.
 */

#include <limits.h>

#include <gmp_type.h>

#define GMP_ARCH_TEST_JOIN_IMPL(a, b) a##b
#define GMP_ARCH_TEST_JOIN(a, b)      GMP_ARCH_TEST_JOIN_IMPL(a, b)
#define GMP_ARCH_TEST_ASSERT(expr)                                                                  \
    typedef char GMP_ARCH_TEST_JOIN(gmp_arch_test_assert_, __LINE__)[(expr) ? 1 : -1]

GMP_ARCH_TEST_ASSERT(sizeof(data_gt) * CHAR_BIT == GMP_PORT_DATA_SIZE_PER_BITS);
GMP_ARCH_TEST_ASSERT(sizeof(fast_gt) * CHAR_BIT >= GMP_PORT_FAST_SIZE_PER_BITS);
GMP_ARCH_TEST_ASSERT(sizeof(time_gt) * CHAR_BIT == GMP_PORT_TIME_SIZE_PER_BITS);
GMP_ARCH_TEST_ASSERT(sizeof(size_gt) * CHAR_BIT == GMP_PORT_SIZE_SIZE_PER_BITS);
GMP_ARCH_TEST_ASSERT(sizeof(pwm_gt) * CHAR_BIT == GMP_PORT_PWM_SIZE_PER_BITS);

#if defined(__TMS320C28XX__)
GMP_ARCH_TEST_ASSERT(GMP_ARCH_TYPE == GMP_ARCH_C28X);
GMP_ARCH_TEST_ASSERT(CHAR_BIT == 16);
GMP_ARCH_TEST_ASSERT(GMP_PORT_DATA_SIZE_PER_BITS == 16);
#elif defined(__C29__) || defined(__c29__)
GMP_ARCH_TEST_ASSERT(GMP_ARCH_TYPE == GMP_ARCH_C29X);
GMP_ARCH_TEST_ASSERT(CHAR_BIT == 8);
GMP_ARCH_TEST_ASSERT(GMP_PORT_DATA_SIZE_PER_BITS == 8);
#elif defined(__ARM_ARCH_PROFILE) && (__ARM_ARCH_PROFILE == 'M')
GMP_ARCH_TEST_ASSERT(GMP_ARCH_TYPE == GMP_ARCH_ARM_M);
GMP_ARCH_TEST_ASSERT(CHAR_BIT == 8);
GMP_ARCH_TEST_ASSERT(GMP_PORT_DATA_SIZE_PER_BITS == 8);
#endif

int gmp_arch_compile_smoke(void)
{
    return (int)(sizeof(data_gt) + sizeof(fast_gt) + sizeof(time_gt));
}
