/**
 * @file arch.cfg.h
 * @brief Selects repository architecture defaults for the active compiler target.
 *
 * Selection and override order:
 *   1. xplt.config.h may define GMP_ARCH_TYPE and any GMP_PORT_* override.
 *   2. csp.config.h/csp.typedef.h may provide platform-specific overrides.
 *   3. this file selects an architecture and fills only missing GMP_PORT_* items.
 *   4. types.cfg.h supplies the final architecture-independent fallbacks.
 */

#ifndef _FILE_GMP_ARCH_CFG_H_
#define _FILE_GMP_ARCH_CFG_H_

#define GMP_ARCH_UNKNOWN  (0)
#define GMP_ARCH_ARM_M    (1)
#define GMP_ARCH_C28X     (2)
#define GMP_ARCH_X86      (3)
#define GMP_ARCH_X86_64   (4)
#define GMP_ARCH_RISCV_32 (5)
#define GMP_ARCH_C29X     (6)

#ifndef GMP_ARCH_TYPE
#if defined(__TMS320C28XX__) || defined(__TMS320C28XX_CLA__)
#define GMP_ARCH_TYPE GMP_ARCH_C28X
#elif defined(__C29__) || defined(__c29__)
#define GMP_ARCH_TYPE GMP_ARCH_C29X
#elif defined(__ARM_ARCH_PROFILE) && (__ARM_ARCH_PROFILE == 'M')
#define GMP_ARCH_TYPE GMP_ARCH_ARM_M
#elif defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) ||                         \
    defined(__ARM_ARCH_8M_BASE__) || defined(__ARM_ARCH_8M_MAIN__)
#define GMP_ARCH_TYPE GMP_ARCH_ARM_M
#elif defined(_M_X64) || defined(__x86_64__) || defined(__amd64__)
#define GMP_ARCH_TYPE GMP_ARCH_X86_64
#elif defined(_M_IX86) || defined(__i386__)
#define GMP_ARCH_TYPE GMP_ARCH_X86
#elif defined(__riscv) && defined(__riscv_xlen) && (__riscv_xlen == 32)
#define GMP_ARCH_TYPE GMP_ARCH_RISCV_32
#else
#define GMP_ARCH_TYPE GMP_ARCH_UNKNOWN
#endif
#endif

#if GMP_ARCH_TYPE == GMP_ARCH_ARM_M
#include <core/std/arch/arm_m_general.h>
#elif GMP_ARCH_TYPE == GMP_ARCH_C28X
#include <core/std/arch/c28x.h>
#elif GMP_ARCH_TYPE == GMP_ARCH_C29X
#include <core/std/arch/c29x.h>
#elif GMP_ARCH_TYPE == GMP_ARCH_X86
#include <core/std/arch/x86.h>
#elif GMP_ARCH_TYPE == GMP_ARCH_X86_64
#include <core/std/arch/x86_64.h>
#elif GMP_ARCH_TYPE == GMP_ARCH_RISCV_32
#include <core/std/arch/risc_v_32.h>
#elif GMP_ARCH_TYPE != GMP_ARCH_UNKNOWN
#error "Unsupported GMP_ARCH_TYPE selection."
#endif

#endif // _FILE_GMP_ARCH_CFG_H_
