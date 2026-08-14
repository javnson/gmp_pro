/**
 * @file .config.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// Software release: GMP Core Library
// Release date:
// Author:           JsScript
// Copyright
// Source repository: https://github.com/javnson/gmp_pro

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
// implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//// This file contains all the leading macro definitions.
//// This file may configure the operating mode of the whole library.
//
//// User should config this file first.
//// The following macro definitions will applied to all the modules.
//
// #ifndef GENERAL_MOTOR_PLAT
//
//// GMP library version definition
// #define GENERAL_MOTOR_PLAT
//
//// Library Serial numbers
// #define GMP_SERIAL_NUM ((00006ul))
//
// #define GMP_AUTO
// #define GMP_AUTO_STM32          1
// #define GMP_AUTO_TIC2000        2
// #define GMP_AUTO_WINDOWS        3
// #define GMP_AUTO_LINUX          4
// #define GMP_AUTO_WINDOWS_MATLAB 5
//
//// This macro shold be defined in <user.config.h>
//// GMP support chip select
//// #define MASTERCHIP GMP_AUTO_STM32
//
//// Compiler options
//// using CCS C2000 compiler
//// #define COMPILER_CCS_C2000
//
//// Enable unimplemented function warning
// #define SPECIFY_ENABLE_UNIMPL_FUNC_WARNING
//
//// When meet an unimplemented function just stuck the program
// #define SPECIFY_STUCK_WHEN_UNIMPL_FUNC
//
//// enable the test environment
//// This switch is only for developing
//// As a common user, we strongly suggest you disable the macro
//// #define SPECIFY_ENABLE_TEST_ENVIRONMENT
//
//// Specify the test environment is PC operation system
//// #define SPECIFY_PC_TEST_ENV
//
//// specify GMP should feed watch dog
//// The program will invoke feed watch dog
//// #define SPECIFY_ENABLE_FEED_WATCHDOG
//
//// Disable the GMP LOGO output
//// The code size will greatly lessen
//// #define SPECIFY_DISABLE_GMP_LOGO
//
////////////////////////////////////////////////////////////////////////////
//// This module permit GMP auto detect what chip is in use and configure it automatically.
// #if defined GMP_AUTO
//
// #if (MASTERCHIP == GMP_AUTO_WINDOWS)
//
//// redirect debug print to std C library `printf`
// #define gmp_dbg_prt printf
//
//// Specify the environment is PC test environment
// #ifndef SPECIFY_PC_TEST_ENV
// #define SPECIFY_PC_TEST_ENV
// #endif // SPECIFY_PC_TEST_ENV
//
//// Simulation settings maximum iterations
// #ifndef MAX_ITERATION_LOOPS
// #define MAX_ITERATION_LOOPS (1000000)
// #endif // MAX_ITERATION_LOOPS
//
//// In this mode, CTL and Main loop will not run in two different threads
// #define CTL_DISABLE_MULTITHREAD
//
//// When main loop has invoked `CTL_MAIN_LOOP_RUNNING_RATIO` times,
//// the ct_main function will be called once.
// #define CTL_MAIN_LOOP_RUNNING_RATIO 2
//
//// Enable GMP LOGO
// #undef SPECIFY_DISABLE_GMP_LOGO
//// #define SPECIFY_DISABLE_GMP_LOGO
//
// #endif // MASTERCHIP
//
// #endif // GMP_AUTO
//
//// This macro disable the chip initial functions invoke, including
//// gmp_csp_startup, gmp_setup_peripheral, gmp_init_peripheral_tree
//// These three functions may be complete by an automation toools.
//// #define DISABLE_GMP_INITIALIZATION_FUNCTION_INVOKE
//
////////////////////////////////////////////////////////////////////////////
//// MEMORY CONTROLLER SETTINGS
//// Memory Management Deposit
////
//// GMP/core/mm
//// enable the block memory management
//// if you disable the block, the block memory will not active.
// #define SPECIFY_GMP_BLOCK_MEMORY_ENABLE
//
//// specify the initial global memory heap bank.
// #define GMP_GLOBAL_MEMORY_HEAP_HANDLE default_mem_heap
//
//// specify the default memory control function, allocation and free
// #define GMP_BLOCK_ALLOC_FUNC(size) gmp_block_alloc(default_mem_heap, size)
// #define GMP_BLOCK_FREE_FUNC(ptr)   gmp_block_free(default_mem_heap, ptr)
//
//// #define GMP_BLOCK_ALLOC_FUNC(size) alloc(size)
//// #define GMP_BLOCK_FREE_FUNC(ptr) free(ptr)
//
//// Specify the default heap size
// #define GMP_DEFAULT_HEAP_SIZE ((1536))
//
//// The default print function rely on the allocation function,
//// so the alloc_function and free_function is necessary
// #define SPECIFY_ENABLE_DEFUALT_DEBUG_PRINT_FUNC
//
//// In default case, the print function may use gmp_malloc function,
//// if you have to avoid the malloc and free you should enable the following function
// #define SPECIFY_DISABLE_DYNAMIC_ALLOC_OF_DBGPTR
//
// #define GMP_TIMER_BASE_FREQ 64000000
//
////////////////////////////////////////////////////////////////////////////
//// Device and IO device controller Settings
//// Device and Peripheral Management Deposit
////
//// NOTICE: if you disable the module you will lose the ability of ext module
////
//// GMP/core/dev
// #define GMP_PERIPHERAL_IO
//
//// GMP IO device extention
//// GMP/ext
//
////////////////////////////////////////////////////////////////////////////
//// Timer of the STM32 device
//
//// The base frequency of timer peripheral
//// generally, this macro will be implemented by chip selection and auto config.
//// #define GMP_TIMER_BASE_FREQ ((72000000U))
//
////////////////////////////////////////////////////////////////////////////
//// GMP WF settings
//// Work flow Management Deposit
////
//// GMP/core/workflow
//
//// GMP/core/workflow_scheduling
//
////////////////////////////////////////////////////////////////////////////
//// GMP CTL(Controller Template Library)
//
//// Enable GMP CTL module
//// #define SPECIFY_ENABLE_GMP_CTL
//// Keep DataLink and other optional adapters independent from CTL math
//// #define SPECIFY_DISABLE_GMP_MATH
// #define ENABLE_TI_IQMATH
// #define ENABLE_IQMATH_HEADER_DIREDCT
//
//// The rest of GMP CTL config please complete in <ctrl_config.h>
//
// #endif // GENERAL_MOTOR_PLAT
//
//
////////////////////////////////////////////////////////////////////////////
//// Global config selections

//////////////////////////////////////////////////////////////////////////
// GMP Library Core Settings

// ....................................................................//
// Basic & Common Settings

// Specify this is the PC test environment.
// #define SPECIFY_PC_ENVIRONMENT

// Specify the maximum loop times
#ifndef PC_ENV_MAX_ITERATION
#define PC_ENV_MAX_ITERATION ((100000))
#endif // PC_ENV_MAX_ITERATION

// Specify disable CSP (Chip Support Package)
// #define SPECIFY_DISABLE_CSP

// specify what happened when error happened.
// When you enable the flag, the system will stuck when error occurred.
// Or, when error occurred, the program will continue running.
// #define SPECIFY_STUCK_WHEN_ERROR

// Specify disable GMP logo function
// #define SPECIFY_DISABLE_GMP_LOGO

// specify runtime environment endian
#ifndef GMP_CHIP_ENDIAN
#define GMP_CHIP_ENDIAN GMP_CHIP_LITTLE_ENDIAN
#endif // GMP_CHIP_ENDIAN

// enable int64_t & uint64_t
#define SPECIFY_ENABLE_INTEGER64

// Specify system tick frequency
// 1000 ticks per second
#ifndef SPECIFY_SYSTEM_TICK_FREQUENCY
#define SPECIFY_SYSTEM_TICK_FREQUENCY ((1000))
#endif // SPECIFY_SYSTEM_TICK_FREQUENCY

//....................................................................//
// Print function implement

// Specify the base print function is disabled.
// #define SPECIFY_BASE_PRINT_NOT_IMPL

#ifndef GMP_BASE_PRINT_CHAR_EXT
#define GMP_BASE_PRINT_CHAR_EXT ((128))
#endif // _GMP_CHAR_EXT

// Default handle of GMP base print function
#ifndef GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE
#define GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE void*
#endif // GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE

// default transmit function of GMP base print function
#ifndef GMP_BASE_PRINT_FUNCTION
#define GMP_BASE_PRINT_FUNCTION gmp_hal_uart_send
#endif // GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE

//....................................................................//
// Memory controller

// Select Default Memory controller functions
// options reference <options.cfg.h>
// + USING_DEFAULT_SYSTEM_DEFAULT_FUNCTION, using system <memory.h> functions
//   such as, `malloc()`, `free()`
// + USING_GMP_BLOCK_DEFAULT_FUNCTION, using gmp basic functions from <block_mem.h> functions
//   such as, `gmp_mm_block_alloc()`, `gmp_mm_block_free()`
// + USING_MANUAL_SPECIFY_FUNCTION, using gmp basic functions via user specify
#ifndef SPECIFY_GMP_DEFAULT_ALLOC
#define SPECIFY_GMP_DEFAULT_ALLOC USING_DEFAULT_SYSTEM_DEFAULT_FUNCTION
#endif // SPECIFY_GMP_DEFAULT_ALLOC

// if user select USING_MANUAL_SPECIFY_FUNCTION, user should specify the following two macros.
// SPECIFY_GMP_USER_ALLOC point to a function is a memory alloc function,
//  which prototype is void* func(size_gt size);
// SPECIFY_GMP_USER_FREE point to a function is a memory free function,
//  which prototype is void func(void* memory);
#ifndef SPECIFY_GMP_USER_ALLOC
#define SPECIFY_GMP_USER_ALLOC malloc
#endif // SPECIFY_GMP_USER_ALLOC
#ifndef SPECIFY_GMP_USER_FREE
#define SPECIFY_GMP_USER_FREE free
#endif // SPECIFY_GMP_USER_FREE

// GMP memory magic number
//
#ifndef GMP_MEM_MAGIC_NUMBER
#define GMP_MEM_MAGIC_NUMBER (0xFC6A)
#endif // GMP_MEM_MAGIC_NUMBER

// Default heap memory space
//
#ifndef GMP_DEFAULT_HEAP_SIZE
#define GMP_DEFAULT_HEAP_SIZE ((1536))
#endif // GMP_DEFAULT_HEAP_SIZE

// Default heap size, byte(s)
//
#ifndef GMP_DEFAULT_BUFFER_SIZE
#define GMP_DEFAULT_BUFFER_SIZE ((32))
#endif // GMP_DEFAULT_BUFFER_SIZE

// Block Memory controller configuration
//
#ifndef GMP_MEM_BLOCK_SIZE
#define GMP_MEM_BLOCK_SIZE (0x0010)
#endif // GMP_MEM_BLOCK_SIZE

//////////////////////////////////////////////////////////////////////////
// Controller Template Library Settings

// Specify disable GMP CTL module
// #define SPECIFY_DISABLE_GMP_CTL

// Specify enable GMP CTL Nano Framework
// #define SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

// Select Default Controller calculating type
// options reference <options.cfg.h>
// implements is in <core/std/cfg/default.types.h>
// + USING_FIXED_TI_IQ_LIBRARY: Fixed number library, TI-IQ math library
// + USING_FIXED_ARM_CMSIS_Q_LIBRARY: Fixed number library, ARM-CMSIS math library
// + USING_QFPLIB_FLOAT: QFP Float library, ARM support
// + USING_FLOAT_FPU: Float number, using Float FPU
// + USING_DOUBLE_FPU: Float number, using Double FPU
#ifndef SPECIFY_CTRL_GT_TYPE
// #define SPECIFY_CTRL_GT_TYPE USING_DOUBLE_FPU
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
// #define SPECIFY_CTRL_GT_TYPE USING_FIXED_TI_IQ_LIBRARY
#endif // SPECIFY_CTRL_GT_TYPE

// Select the high-precision type used by initialization and parameter tools.
// parameter_gt is not intended for high-frequency control-step arithmetic.
#ifndef SPECIFY_PARAMETER_GT_TYPE
#define SPECIFY_PARAMETER_GT_TYPE USING_FLOAT_FPU
#endif // SPECIFY_PARAMETER_GT_TYPE

// The default macro to define a string.
#ifndef TEXT_STRING
#define TEXT_STRING(A) (A)
#endif // TEXT_STRING

// Startup logo selection. The legacy disable switch remains supported.
#ifndef SPECIFY_GMP_LOGO_MODE
#ifdef SPECIFY_DISABLE_GMP_LOGO
#define SPECIFY_GMP_LOGO_MODE GMP_LOGO_MODE_DISABLED
#else
#define SPECIFY_GMP_LOGO_MODE GMP_LOGO_MODE_FULL
#endif // SPECIFY_DISABLE_GMP_LOGO
#endif // SPECIFY_GMP_LOGO_MODE
