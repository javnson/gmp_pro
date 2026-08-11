/**
 * @file gmp_tunable.c
 * @brief Compile the GMP Tunable backend selected by the platform data unit.
 */

#include <gmp_core.h>
#include <core/dev/tunable.h>

#include <string.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES == 1
#include <core/dev/datalink/gmp_tunable_u8.inc>
#elif GMP_PORT_DATA_SIZE_PER_BYTES == 2
#include <core/dev/datalink/gmp_tunable_u16.inc>
#else
#error "GMP Tunable supports only 1-byte and 2-byte addressable data units"
#endif
