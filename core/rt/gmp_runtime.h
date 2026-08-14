
#ifndef _FILE_GMP_CORE_H_
#define _FILE_GMP_CORE_H_

#include <gmp_type.h>
#include <core/base/gmp_base.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/** @brief Initialize platform-owned peripherals required by GMP. */
void setup_peripheral(void);

/** @brief Initialize the user application after platform startup. */
void init(void);

/** @brief Execute one user application background iteration. */
void mainloop(void);

/** @brief Initialize the optional CTL application layer. */
void ctl_init(void);

/** @brief Execute one optional CTL application iteration. */
void ctl_mainloop(void);

/** @brief Report whether the optional CTL application is ready. */
fast_gt ctl_ready_mainloop(void);

// extern ctl_object_nano_t *ctl_nano_handle;

// This function is the basic init part of GMP entry.
// User may call this function to init GMP library.
// If you need GMP to manage the function call you may not call this function.
GMP_STATIC_INLINE
void gmp_base_init(void)
{

#ifndef SPECIFY_DISABLE_CSP
    // CSP Startup function
    //
    gmp_csp_startup();
#endif // SPECIFY_DISABLE_CSP

    // platform related function, initialize peripheral
    // This function is defined by user.
    // if CSP is disabled, user should implement chip setup routine
    // in this function.
    //
    setup_peripheral();

    // TODO: setup GMP library memory management module
    //

#if SPECIFY_GMP_LOGO_MODE != GMP_LOGO_MODE_DISABLED
    // Debug information print
    //
    gmp_base_show_label();
#endif // SPECIFY_GMP_LOGO_MODE != GMP_LOGO_MODE_DISABLED

#if !defined SPECIFY_DISABLE_GMP_CTL
    // Call CTL(Controller template library) initialization function
    //
    ctl_init();

#endif // SPECIFY_DISABLE_GMP_CTL

    // Call user initialization function
    //
    init();

#ifndef SPECIFY_DISABLE_CSP
    // latest function before Main loop, CSP may use this function to enable interrupt.
    //
    gmp_csp_post_process();
#endif // SPECIFY_DISABLE_CSP

#if !defined SPECIFY_DISABLE_GMP_CTL
#ifdef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
    ctl_fm_controller_inspection(ctl_nano_handle);
#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
#endif // SPECIFY_DISABLE_GMP_CTL
}

// This is the loop part of GMP library.
// This function will return without blockage
// If you need GMP to manage the function call you may not call this function.
GMP_STATIC_INLINE
void gmp_base_loop(void)
{
#ifndef SPECIFY_DISABLE_CSP
    // Call GMP CSP module loop routine
    //
    gmp_csp_loop();

#endif // SPECIFY_DISABLE_CSP

#if !defined SPECIFY_DISABLE_GMP_CTL
#ifdef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
    ctl_fm_state_dispatch(ctl_nano_handle);
#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
#endif // SPECIFY_DISABLE_GMP_CTL

    // Call user general loop routine
    //
    mainloop();

#if !defined SPECIFY_DISABLE_GMP_CTL
    // Call controller loop routine
    //
    ctl_mainloop();
#endif // SPECIFY_DISABLE_GMP_CTL
}

// This function should be called when Chip setup is completed.
GMP_STATIC_INLINE
void gmp_base_entry(void)
{
#ifdef SPECIFY_PC_ENVIRONMENT
    size_t loop_tick;
#endif // SPECIFY_PC_ENVIRONMENT

    gmp_base_init();

#ifdef SPECIFY_PC_ENVIRONMENT
    // PC simulate environment, finite iteration
    for (loop_tick = 0; loop_tick < PC_ENV_MAX_ITERATION; ++loop_tick)
#else  // SPECIFY_PC_ENVIRONMENT
    // real processor routine
    // for (;;)
    while (1)
#endif // SPECIFY_PC_ENVIRONMENT
    {
        gmp_base_loop();
    }

#if !defined SPECIFY_DISABLE_CSP
#if !defined SPECIFY_DISABLE_CSP_EXIT
    // This function is unreachable.
    gmp_csp_exit();
#endif // SPECIFY_DISABLE_CSP_EXIT
#endif // SPECIFY_DISABLE_CSP
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_CORE_H_
