/**
 * @file vs_test_clock.cpp
 * @brief Portable clock stub shared by hosted CTL unit-test projects.
 */

#include <ctl/portable/gmp_ctl_portable.h>

/** @brief Return a deterministic tick for unit tests that do not advance time. */
time_gt gmp_ctl_portable_get_tick(void)
{
    return 0U;
}
