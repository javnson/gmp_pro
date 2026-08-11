/**
 * @file scope_u8.h
 * @brief Data Link Scope service for byte-addressed CPUs.
 */

#ifndef _FILE_GMP_SCOPE_U8_H
#define _FILE_GMP_SCOPE_U8_H

#if GMP_PORT_DATA_SIZE_PER_BYTES != 1
#error "scope_u8.h requires GMP_PORT_DATA_SIZE_PER_BYTES == 1"
#endif

#include <core/dev/datalink.h>
#include <core/dev/datalink/scope_protocol.h>

#endif // _FILE_GMP_SCOPE_U8_H
