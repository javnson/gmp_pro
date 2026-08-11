/**
 * @file gmp_mem_presp.c
 * @brief Compile the Memory Perspective backend selected by the platform data unit.
 */

#include <gmp_core.h>
#include <core/dev/mem_presp.h>

#include <stdint.h>
#include <string.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES == 1
#include <core/dev/datalink/gmp_mem_presp_u8.inc>
#elif GMP_PORT_DATA_SIZE_PER_BYTES == 2
#include <core/dev/datalink/gmp_mem_presp_u16.inc>
#else
#error "GMP Memory Perspective supports only 1-byte and 2-byte data units"
#endif
