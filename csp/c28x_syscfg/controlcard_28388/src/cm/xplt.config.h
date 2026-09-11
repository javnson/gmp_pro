#ifndef GMP_F28388D_CM_XPLT_CONFIG_H
#define GMP_F28388D_CM_XPLT_CONFIG_H

#include "ctrl_settings.h"

/* ARM execution model; the CM CSP inherits the C28x-family u16 data unit. */
#define GMP_ARCH_TYPE (1)
#define SPECIFY_DISABLE_GMP_CTL
#define SPECIFY_DISABLE_GMP_LOGO
#define SPECIFY_BASE_PRINT_NOT_IMPL

#define GMP_F28388D_CM_DL_TCP (1)
#define GMP_F28388D_CM_DL_UDP (2)
#ifndef GMP_F28388D_CM_DL_TRANSPORT
#define GMP_F28388D_CM_DL_TRANSPORT GMP_F28388D_CM_DL_TCP
#endif

#if GMP_F28388D_DATA_UNIT_BITS != 16U
#error "F28388D CPU1, CPU2 and CM must share the system-u16 data model"
#endif

#endif
