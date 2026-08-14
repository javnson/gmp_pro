/**
 * @file gmp_datalink.c
 * @brief Compile the GMP Data Link backend selected by the platform data unit.
 */

#include <gmp_type.h>
#include <core/base/gmp_base.h>
#include <core/dev/datalink/datalink.h>
#include <core/base/checksum/crc16.h>

#include <string.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES == 1
#include <core/dev/datalink/datalink_u8.inc>
#elif GMP_PORT_DATA_SIZE_PER_BYTES == 2
#include <core/dev/datalink/datalink_u16.inc>
#else
#error "GMP Data Link supports only 1-byte and 2-byte addressable data units"
#endif
