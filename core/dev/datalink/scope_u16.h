/**
 * @file scope_u16.h
 * @brief Data Link Scope service for 16-bit-addressed DSPs.
 */

#ifndef _FILE_GMP_SCOPE_U16_H
#define _FILE_GMP_SCOPE_U16_H

#if GMP_PORT_DATA_SIZE_PER_BYTES != 2
#error "scope_u16.h requires GMP_PORT_DATA_SIZE_PER_BYTES == 2"
#endif

#include <core/dev/datalink/datalink.h>
#include <core/dev/datalink/scope_protocol.h>

#endif // _FILE_GMP_SCOPE_U16_H
