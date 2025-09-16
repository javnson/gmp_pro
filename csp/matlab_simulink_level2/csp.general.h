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

//////////////////////////////////////////////////////////////////////////
// Step I: Select HAL library
//

//////////////////////////////////////////////////////////////////////////
// Step II: Invoke default GMP CSP headers
#include <core/std/gmp.std.h>

// invoke GMP Simulink Communication buffer type
// #include <csp/windows_simulink/simulink_buffer.h>

//
// Instert a software breakpoint right here
// GMP library Debug Software Break Point Macro
// This is a Microsoft intrinsic function to create a software BP here.
//
#define GMP_DBG_SWBP __debugbreak()

//////////////////////////////////////////////////////////////////////////
// Step III: Invoke all the other CSP headers
//

// ec_gt windows_print_function(uint32_t *handle, half_duplex_ift *port);

// Simulink Disable/Enable output
void csp_sl_enable_output(void);
void csp_sl_disable_output(void);

// Simulink Input Panel PORT
double csp_sl_get_panel_input(fast_gt channel);
