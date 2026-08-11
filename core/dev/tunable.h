/**
 * @file tunable.h
 * @brief Select the GMP Tunable API for the platform data unit.
 */

#ifndef _FILE_GMP_TUNABLE_SELECTOR_H
#define _FILE_GMP_TUNABLE_SELECTOR_H

#if GMP_PORT_DATA_SIZE_PER_BYTES == 1
#include <core/dev/datalink/tunable_u8.h>
#elif GMP_PORT_DATA_SIZE_PER_BYTES == 2
#include <core/dev/datalink/tunable_u16.h>
#else
#error "GMP Tunable supports only 1-byte and 2-byte addressable data units"
#endif

#endif // _FILE_GMP_TUNABLE_SELECTOR_H
