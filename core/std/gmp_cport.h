/**
 * @file gmp_cport.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief This file should ensure that the user may compile it as a C source file.
 * User may include this file and access the GMP library.
 * @version 0.1
 * @date 2024-09-28
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_GLOBAL_H
#define _FILE_GLOBAL_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////
// Step I: User function prototypes
//
// The following functions should be implemented by user
//

// ....................................................................//
// User should implement this three functions
// These function is weak function
//

// This function would execute only once.
// User should implement all the initialization code in this function.
//
void init(void);

// This function would be the endless loop.
// User should implement all the loop tasks and round-robin tasks.
//
void mainloop(void);

// This function should setup all the peripherals.
// In this function the code could be platform related.
//
void setup_peripheral(void);

// ....................................................................//
// User should implement this 3 functions
//

// This function would be implemented in ctl_main.c
// This function would execute only once.
// User should implement all the controller related initialization code in this function.
// That means user init process may isolate with the controller init process.
//
void ctl_init(void);

// This function would be implemented in ctl_main.c
// This function would be called by main ISR function.
// User should call this function, in your ctl_main.cpp or just ignore it.
// When you need to simulate your controller, this function would be invoked.
// return 0 is normal, and any non-zero value means error.
//
void ctl_mainloop(void);

// This function is a standard interface for testing if the controller meets online criterion.
// 1 means reached criterion.
// 0 means don't reached.
fast_gt ctl_ready_mainloop(void);

// This function would be implemented in ctl_main.c
// This function would be called in every controller loop
// This function would be called by `gmp_base_ctl_step`
//
//    void ctl_dispatch(void);

// ....................................................................//
// The following function would be called by user.

// Key this function must do implement, otherwise the time-based part may invalid.
// This function would return the system tick point right now.
//
#ifndef gmp_base_get_system_tick
time_gt gmp_base_get_system_tick(void);
#endif // gmp_base_get_system_tick

// Derived function for system tick
// Calculate the time difference from the specified time
//
GMP_STATIC_INLINE time_gt gmp_base_get_diff_system_tick(time_gt t0)
{
    time_gt current_tick = gmp_base_get_system_tick();
    return current_tick - t0;
}

// judge if current time meets elapse conditions
// return 1 if delay elapsed.
GMP_STATIC_INLINE time_gt gmp_base_is_delay_elapsed(time_gt t0, uint32_t delay_t)
{
    time_gt current_tick = gmp_base_get_system_tick();
    return ((current_tick - t0) >= delay_t);
}

// The function should be called by user or system when fatal error happened.
// So the function must own the ability of stop the program.
//
void gmp_base_system_stuck(void);

// This function implement a basic assert function.
// user may call this function to ensure if program is correct.
// This function has a basic version in gmp_std_port.c

#if defined USE_GMP_SELF_BASE_ASSERT
void gmp_base_assert(void* condition);

#elif defined DISABLE_GMP_BASE_ASSERT
#define gmp_base_assert(assert_condition) (void)(assert_condition)
#else
#include <assert.h>
#define gmp_base_assert(assert_condition) assert(assert_condition)
#endif // USE_GMP_BASE_ASSERT
// When a function is unimplemented, the function would be invoke.
//
void gmp_base_not_impl(const char* file, uint32_t line);

// System debug print function
//
// size_gt gmp_base_print(const char *msg, ...);

#if defined USER_SPECIFIED_PRINT_FUNCTION

#if defined SPECIFY_BASE_PRINT_NOT_IMPL
#define gmp_base_print(x, ...)
#else // SPECIFY_BASE_PRINT_NOT_IMPL
#define gmp_base_print USER_SPECIFIED_PRINT_FUNCTION
#endif // SPECIFY_BASE_PRINT_NOT_IMPL

#else // USER_SPECIFIED_PRINT_FUNCTION
// define GMP base function print
#define gmp_base_print(A, ...) gmp_base_print_internal(A, ##__VA_ARGS__)
#endif // USER_SPECIFIED_PRINT_FUNCTION

// Show GMP setup label
// This function contain a label show of GMP.
//
void gmp_base_show_label(void);

// The memory controller functions
//
void* gmp_base_malloc(size_gt size);
void gmp_base_free(void* ptr);

// This function would be called by user in entry function.
// And this function would not return.
// This function should be called only once.
// This is a inline function, defined in gmp_core_func.h
//
// void gmp_base_entry(void);

#ifdef SPECIFY_ENABLE_GMP_CTL
#ifdef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
// extern controller nano object
extern ctl_object_nano_t* ctl_nano_handle;

#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
#endif // SPECIFY_ENABLE_GMP_CTL

// This function would be called by main ISR function, by user.
// User should call this function, in your ctl_main.cpp or just ignore it.
// When you need to simulate your controller, this function would be invoked.
//
// GMP_STATIC_INLINE
// void gmp_base_ctl_step(void);

//////////////////////////////////////////////////////////////////////////
// Step II: These function have implemented by GMP
//

// ....................................................................//
// Error Handle function
//

// This function would be called when system meets error.
// In order to referenced by error handling,
// the function prototype is redefined in `error_code.h`
//
// void gmp_system_stuck(void);

// ....................................................................//
// GMP default initialization function and loop function
//

// This function is the GMP loop function,
// `user_loop()` would be called in this function,
// and other GMP deposit would be invoked in this function.
//
//    void gmp_loop(void);

// This function is the GMP initialization function,
// `user_loop()` would be called in the function,
// and other GMP deposit would be invoked in this function.
//
//    void gmp_init(void);

// Default heap memory set
//
// #if defined SPECIFY_GMP_BLOCK_MEMORY_ENABLE
//     GMP_MEM_ALIGN
//     extern data_gt default_heap[GMP_DEFAULT_HEAP_SIZE];
// #endif

// #ifdef SPECIFY_ENABLE_DEFUALT_DEBUG_PRINT_FUNC
//
//     // debug print function prototype
//     //
//     size_gt gmp_dbg_prt_fn(const char *p_fmt, ...);
//
// #endif // SPECIFY_ENABLE_DEFUALT_DEBUG_PRINT_FUNC
//
//     // debug print function alias
//     // user should invoke this function instead of any direct function name.
//     //
// #ifndef gmp_dbg_prt
// #define gmp_dbg_prt(x, ...)
// #endif // debug output mode disabled

//////////////////////////////////////////////////////////////////////////
// Step V: global variables
//  The following variables will be defined in <core/std/global.c>
//

// ....................................................................//
// delay parameters
//

// Timeout counter
//
extern size_gt g_delay;

// Timeout timer
//
extern size_gt g_delay_ms;

// Default print device handle
//
extern GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE* default_debug_dev;

#ifdef __cplusplus
}
#endif

#endif // _FILE_GLOBAL_H
