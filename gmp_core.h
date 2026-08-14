/**
 * @file gmp_core.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_GMP_CORE_HEADER_H_
#define _FILE_GMP_CORE_HEADER_H_

// This file is GMP library C source header

// GMP_CTL_PORTABLE is the lightweight, no-CSP entry for applications that only
// consume CTL algorithms.  It deliberately skips xplt.config.h, csp.config.h,
// runtime assembly, peripheral management, and the GMP entry framework.
#if defined(GMP_CTL_PORTABLE)
#include <ctl/portable/gmp_ctl_portable.h>
#else
// Standard configuration and portable types.
#include <gmp_type.h>

// Peripheral handle types are kept separate from the standard type layer.
#include <core/dev/peripheral_types.h>

#ifndef SPECIFY_DISABLE_CSP
#include <csp.general.h>
#include <core/rt/csp_port.h>
#endif // SPECIFY_DISABLE_CSP

// Portable base services and peripheral service contracts.
#include <core/base/gmp_base.h>
#include <core/dev/peripheral_port.h>
#endif // GMP_CTL_PORTABLE

// TI fixed library
#ifdef ENABLE_IQMATH_HEADER_DIREDCT
#ifndef __IQMATHLIB_H_INCLUDED__

#include <third_party\iqmath\IQmathLib.h>
// #include "sw/modules/iqmath/src/32b/IQmathLib.h"
#endif
#endif // USING_FIXED_LIBRARY

//////////////////////////////////////////////////////////////////////////
// Step III GMP basement
//

// <csp> default chip type definition
// #include <csp/chip_port.h>

// default peripheral types
// This header may be seized in `chip_port.h`
// #include <core/std/default_peripheral.config.h>

// public C ports function
// memory management support
// #include <core/base/gmp_base.h>

// public C CSP ports functions
// peripheral functions
// #include <core/rt/csp_port.h>

//////////////////////////////////////////////////////////////////////////
// Step IV other C core modules
//

// (MM) Memory Management module
#if defined SPECIFY_GMP_BLOCK_MEMORY_ENABLE
#include <core/mm/block_mem.h>
#endif // SPECIFY_GMP_BLOCK_MEMORY_ENABLE

// + (WF) Workflow module

// + (SCH) Scheduling module

// Optional controller framework assembly.
#if defined SPECIFY_ENABLE_GMP_CTL
#include <ctl/ctl.config.h>
#include <ctl/math_block/gmp_math.h>
#include <ctl_main.h>
#include <xplt.ctl_interface.h>
#include <ctl/framework/ctl_dispatch.h>

#if defined SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
#include <ctl/framework/ctl_nano.h>
#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
#endif // SPECIFY_ENABLE_GMP_CTL

// Full GMP runtime entry and dispatch helpers.
#include <core/rt/gmp_runtime.h>

// #ifdef __cplusplus
// extern "C"
//{
// #endif // __cplusplus

// This function would be called by user in entry function.
// And this function would not return.
//
// You may find definition of this function in `gmp_std_port.c`.
//
// extern void gmp_base_entry(void);

// This function would be called by main ISR function, by user.
// User should call this function, in your ctl_main.cpp or just ignore it.
// When you need to simulate your controller, this function would be invoked.
//
// You may find the base service declarations in `core/base/gmp_base.h`.
//
// extern void gmp_base_ctl_step(void);

// #ifdef __cplusplus
// }
// #endif // __cplusplus

#endif // _FILE_GMP_CORE_HEADER_H_
