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
// Step II: Invoke all the STM32 general headers.
//

// STM32 System core support
#include <csp/stm32/common/sys_model.stm32.h>

// STM32 System Computing support
#include <csp/stm32/common/computing_model.stm32.h>

// STM32 GPIO support
#include <csp/stm32/common/gpio_model.stm32.h>

// STM32 general peripheral
#include <csp/stm32/common/peripheral_model.stm32.h>
