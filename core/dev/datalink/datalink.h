/**
 * @file datalink.h
 * @brief Select the GMP Data Link API for the platform data unit.
 */

#ifndef _FILE_GMP_DATALINK_SELECTOR_H
#define _FILE_GMP_DATALINK_SELECTOR_H

#include <gmp_type.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES == 1
#include <core/dev/datalink/datalink_u8.h>
#elif GMP_PORT_DATA_SIZE_PER_BYTES == 2
#include <core/dev/datalink/datalink_u16.h>
#else
#error "GMP Data Link supports only 1-byte and 2-byte addressable data units"
#endif

#endif // _FILE_GMP_DATALINK_SELECTOR_H
