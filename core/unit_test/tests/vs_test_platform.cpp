/**
 * @file vs_test_platform.cpp
 * @brief Minimal host platform services required by tested GMP components.
 */

#include "vs_test_platform.h"

#include <cstdarg>

extern "C"
{
time_gt gmp_test_system_tick = 0;

time_gt gmp_base_get_system_tick(void)
{
    return gmp_test_system_tick;
}

size_gt gmp_base_print_internal(const char* format, ...)
{
    (void)format;
    return 0;
}
}

