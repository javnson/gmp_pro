/**
 * @file gmp.std.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// This file provide all GMP basic standard types

#ifndef _FILE_GMP_STD_H_
#define _FILE_GMP_STD_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// ....................................................................//
// invoke all the configurations
//

// Include all options for config
#include <core/std/cfg/options.cfg.h>

// invoke user config file
//
#include "xplt.config.h"

#ifndef SPECIFY_DISABLE_CSP
// invoke CSP default config
//
#include <csp.config.h>
#endif // SPECIFY_DISABLE_CSP

// invoke system default config
//
#include <core/std/cfg/gmp.cfg.h>

// invoke validate configuration
//
#include <core/std/cfg/validate.cfg.h>

// ....................................................................//
// invoke all the necessary system headers
#if !defined(__TMS320C28XX_CLA__)
#include <assert.h>
#else
// patch a null definition for assert function
#define assert(_ignore) ((void)0)
#endif // __TMS320C28XX_CLA__

#if !defined(__TMS320C28XX_CLA__)
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif // __TMS320C28XX_CLA__

#include <stddef.h>
#include <stdint.h>

// ....................................................................//
// invoke all the GMP STD (standard support)
//

// Compiler support
//
#include <core/std/cfg/compiler.cfg.h>

// GMP error code support
//
#include <core/std/cfg/errorcode.cfg.h>

// ....................................................................//
// CSP related
#ifndef SPECIFY_DISABLE_CSP
// CSP basic type
#include <csp.typedef.h>
#endif // SPECIFY_DISABLE_CSP

// Invoke default type
//
#include <core/std/cfg/types.cfg.h>

// Invoke endian config
//
#include <core/std/cfg/endian.cfg.h>

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_STD_H_
