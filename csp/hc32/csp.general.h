/**
 * @file csp.general.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <csp.config.h>

//
// Instert a software breakpoint right here
// GMP library Debug Software Break Point Macro
// This instruction is valid in Cotex-M kernel chip.
//
#define GMP_DBG_SWBP __asm volatile("BKPT #0")

//////////////////////////////////////////////////////////////////////////
// Step II: Invoke all the HC32 general headers.
//

// HDSC HC32 Series Device Driver Library(HC32DDL)
#include "hc32_ddl.h"
