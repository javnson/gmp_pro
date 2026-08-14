/**
 * @file mem_presp.h
 * @brief Select the GMP Memory Perspective API for the platform data unit.
 */

#ifndef _FILE_GMP_MEM_PERSP_SELECTOR_H
#define _FILE_GMP_MEM_PERSP_SELECTOR_H

#if GMP_PORT_DATA_SIZE_PER_BYTES == 1
#include <core/dev/datalink/mem_presp_u8.h>
#elif GMP_PORT_DATA_SIZE_PER_BYTES == 2
#include <core/dev/datalink/mem_presp_u16.h>
#else
#error "GMP Memory Perspective supports only 1-byte and 2-byte data units"
#endif

#endif // _FILE_GMP_MEM_PERSP_SELECTOR_H
