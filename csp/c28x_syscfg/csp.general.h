/**
 * @file csp.general.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2025-01-18
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_CSP_GENERAL_H_
#define _FILE_CSP_GENERAL_H_

// Basement from C2000ware library.
//#include "device.h"
#include "driverlib.h"

//
// ClockTree Tool
//
#include "clocktree.h"

//
// C2000Ware SysConfig
//  - Software - Device Support
//
#ifndef _C2000_CSP_DISABLE_DEVICE_SUPPORT_
//#include "device.h"
#endif // _C2000_CSP_DISABLE_DEVICE_SUPPORT_

//
// C2000Ware SysConfig
//
#include "board.h"

//
// Linker CMD Tool
//
#ifndef _C2000_CSP_DISBALE_LINKER_CMD_TOOL_
#include "device_cmd.h"
#endif // _C2000_CSP_DISBALE_LINKER_CMD_TOOL_

//
// C200Ware Libraries
//
#ifndef _C2000_CSP_DISABLE_C2000WARE_LIBRARY_
#include "c2000ware_libraries.h"
#endif // _C2000_CSP_DISABLE_C2000WARE_LIBRARY_

//
// C2000 CLB Configuration Tool
//
#ifndef _C2000_CSP_DISABLE_CLB_CONFIG_TOOL_
#include "clb_config.h"
#endif // _C2000_CSP_DISABLE_CLB_CONFIG_TOOL_

#include <c28x_peripheral_driver.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifndef __TMS320C28XX_CLA__
//
// Instert a software breakpoint right here
// GMP library Debug Software Break Point Macro
//
// This instruction is shown in spru430f chapter 1.2.2
// TMS320C28x CPU and Instruction Set Reference Guide
// ref: https://www.ti.com.cn/cn/lit/ug/spru430f/spru430f.pdf
//
#define GMP_DBG_SWBP asm(" ESTOP0")

#else
//
// For CLA debug mode, only this instruction is counted.
// GMP library Debug Software Break Point Macro for CLA debug
//
#define GMP_DBG_SWBP __mdebugstop()

#endif // __TMS320C28XX_CLA__

// debug port
extern uart_halt debug_uart;

GMP_STATIC_INLINE void gmp_base_enter_critical()
{
    DINT;
}

GMP_STATIC_INLINE void gmp_base_leave_critical()
{
    EINT;
}

//////////////////////////////////////////////////////////////////////////
// GPIO definition
// #define GMP_PORT_GPIO_T uint32_t

//#define gmp_hal_gpio_write(pin_num, value) GPIO_writePin((pin_num), (value))
//#define gmp_hal_gpio_read(pin_num)         GPIO_readPin((pin_num))
//#define gmp_hal_gpio_set(pin_num)          GPIO_writePin((pin_num), (1))
//#define gmp_hal_gpio_reset(pin_num)        GPIO_writePin((pin_num), (0))
//#define gmp_hal_gpio_toggle(pin_num)       GPIO_togglePin((pin_num))

//////////////////////////////////////////////////////////////////////////
// Watch Dog module
GMP_STATIC_INLINE void gmp_hal_wd_feed(void)
{
}

//////////////////////////////////////////////////////////////////////////
// SPI module

// Debug Print function
size_gt gmp_base_print_c28xsyscfg(const char* p_fmt, ...);

// Step System tick
void gmp_step_system_tick(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CSP_GENERAL_H_
