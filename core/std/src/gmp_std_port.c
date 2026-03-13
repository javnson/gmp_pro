
// #include <core/std/gmp.std.h>

#include <gmp_core.h>

// This function should be called when System fatal error happened.
void gmp_base_system_stuck(void)
{
}

#if !defined USER_SPECIFIED_PRINT_FUNCTION

GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE default_debug_dev = NULL;

// implement the gmp_debug_print routine.
size_gt gmp_base_print_internal(const char *p_fmt, ...)
{
    size_gt ret = 0;

#if defined SPECIFY_BASE_PRINT_NOT_IMPL
    return ret;
#else // SPECIFY_BASE_PRINT_NOT_IMPL

    half_duplex_ift ptr;

    // if no one was specified to output, just ignore the request.
    if (default_debug_dev == NULL)
    {
        return ret;
    }

    size_gt size = (size_gt)strlen(p_fmt);

    static data_gt str[GMP_BASE_PRINT_CHAR_EXT];
    memset(str, 0, GMP_BASE_PRINT_CHAR_EXT);

    va_list vArgs;
    va_start(vArgs, p_fmt);
    vsprintf((char *)str, (char const *)p_fmt, vArgs);
    va_end(vArgs);

    ptr.buf = str;
    ptr.length = (size_gt)strlen((char *)str);
    ptr.capacity = GMP_BASE_PRINT_CHAR_EXT;

    GMP_BASE_PRINT_FUNCTION(default_debug_dev, &ptr);

    ret = ptr.length;

    return ret;

#endif // SPECIFY_BASE_PRINT_NOT_IMPL
}

// define GMP base function print
#define gmp_base_print gmp_base_print_internal

#endif // USER_SPECIFIED_PRINT_FUNCTION

#ifndef SPECIFY_DISABLE_GMP_LOGO
// This function would print a GMP label
void gmp_base_show_label(void)
{

#include <core/std/gmp_logo.inl>

    gmp_base_print(TEXT_STRING("[okay] General motor platform ready.\r\n"));
}
#endif // SPECIFY_DISABLE_GMP_LOGO

//////////////////////////////////////////////////////////////////////////
// A function is not implement, this is just a place holder.

void gmp_base_not_impl(const char *file, uint32_t line)
{
    // mark and allow the function parameter can be ignored
    UNUSED_PARAMETER(file);
    UNUSED_PARAMETER(line);

#if defined SPECIFY_ENABLE_UNIMPL_FUNC_WARNING
    // print the error information
    gmp_dbg_prt(TEXT_STRING("[Erro] invoke unimplemented function: [%s, %d].\r\n"), file, line);
#endif // SPECIFY_ENABLE_UNIMPL_FUNC_WARNING

#if defined SPECIFY_STUCK_WHEN_UNIMPL_FUNC
#if defined SPECIFY_ENABLE_UNIMPL_FUNC_WARNING
    gmp_dbg_prt("[INFO] Program has stuck!\r\n");
#endif // SPECIFY_ENABLE_UNIMPL_FUNC_WARNING

    // Stop program right now
    gmp_system_stuck();

#endif // SPECIFY_STUCK_WHEN_UNIMPL_FUNC
}

//////////////////////////////////////////////////////////////////////////
// Memory management

// The following function would be called by libraries in the default case.

// unit byte
void *gmp_base_malloc(size_gt size)
{
#if SPECIFY_GMP_DEFAULT_ALLOC == USING_DEFAULT_SYSTEM_DEFAULT_FUNCTION
    return malloc(size);
#elif SPECIFY_GMP_DEFAULT_ALLOC == USING_GMP_BLOCK_DEFAULT_FUNCTION
    return gmp_mm_block_alloc(default_gmp_area_mem_handle, size);
#elif SPECIFY_GMP_DEFAULT_ALLOC == USING_MANUAL_SPECIFY_FUNCTION
    return SPECIFY_GMP_USER_ALLOC(size);
#else //  not implement

#endif // SPECIFY_GMP_DEFAULT_ALLOC
}

void gmp_base_free(void *ptr)
{
#if SPECIFY_GMP_DEFAULT_ALLOC == USING_DEFAULT_SYSTEM_DEFAULT_FUNCTION
    free(ptr);
#elif SPECIFY_GMP_DEFAULT_ALLOC == USING_GMP_BLOCK_DEFAULT_FUNCTION
    gmp_mm_block_free(default_gmp_area_mem_handle, ptr);
#elif SPECIFY_GMP_DEFAULT_ALLOC == USING_MANUAL_SPECIFY_FUNCTION
    return SPECIFY_GMP_USER_FREE(ptr);
#else //  not implement

#endif // SPECIFY_GMP_DEFAULT_ALLOC
}

#if defined USE_GMP_SELF_BASE_ASSERT
void gmp_base_assert(void *condition)
{
    if (condition)
    {
        gmp_base_print("[ASSERT] occurrd, program will stuck here.\r\n");
        gmp_base_print("[NOTE] You may continue in debug mode, to ignore this event.\r\n");

        GMP_DBG_SWBP;
    }
}
#endif // USE_GMP_SELF_BASE_ASSERT

//////////////////////////////////////////////////////////////////////////
// Weak function definition

#ifdef _MSC_VER
// This function would execute only once.
// User should implement all the initialization code in this function.
//
#pragma comment(linker, "/alternatename:init=gmp_defualt_msvc_init")
void gmp_defualt_msvc_init(void)
{
    // not implement
}

// This function would be the endless loop.
// User should implement all the loop tasks and round-robin tasks.
//
#pragma comment(linker, "/alternatename:mainloop=gmp_defualt_msvc_mainloop")
void gmp_defualt_msvc_mainloop(void)
{
    // not implement
}

// This function should setup all the peripherals.
// In this function the code could be platform related.
//
#pragma comment(linker, "/alternatename:setup_peripheral=gmp_defualt_msvc_setup_peripheral")
void gmp_defualt_msvc_setup_peripheral(void)
{
    // not implement
}

// This function would be implemented in ctl_main.c
// This function would execute only once.
// User should implement all the controller related initialization code in this function.
// That means user init process may isolate with the controller init process.
//
#pragma comment(linker, "/alternatename:ctl_init=gmp_defualt_msvc_ctl_init")
void gmp_defualt_msvc_ctl_init(void)
{
}

// This function would be implemented in ctl_main.c
// This function would be called by main ISR function.
// User should call this function, in your ctl_main.cpp or just ignore it.
// When you need to simulate your controller, this function would be invoked.
// return 0 is normal, and any non-zero value means error.
//
#pragma comment(linker, "/alternatename:ctl_mainloop=gmp_defualt_msvc_ctl_mainloop")
void gmp_defualt_msvc_ctl_mainloop(void)
{
    // not implement
}

// This function would be implemented in ctl_main.c
// This function would be called in every controller loop
// This function would be called by @gmp_base_ctl_step
//
#pragma comment(linker, "/alternatename:ctl_dispatch=gmp_defualt_msvc_ctl_dispatch")
void gmp_defualt_msvc_ctl_dispatch(void)
{
    // not implement
}

#elif defined __TI_COMPILER_VERSION__

#else // other compiler support weak symbol

// This function would execute only once.
// User should implement all the initialization code in this function.
//
GMP_WEAK_FUNC_PREFIX
void init(void) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
}

// This function would be the endless loop.
// User should implement all the loop tasks and round-robin tasks.
//
GMP_WEAK_FUNC_PREFIX
void mainloop(void) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
}

// This function should setup all the peripherals.
// In this function the code could be platform related.
//
GMP_WEAK_FUNC_PREFIX
void setup_peripheral(void) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
}

// This function would be implemented in ctl_main.c
// This function would execute only once.
// User should implement all the controller related initialization code in this function.
// That means user init process may isolate with the controller init process.
//
GMP_WEAK_FUNC_PREFIX
void ctl_init(void) GMP_WEAK_FUNC_SUFFIX
{
}

// This function would be implemented in ctl_main.c
// This function would be called by main ISR function.
// User should call this function, in your ctl_main.cpp or just ignore it.
// When you need to simulate your controller, this function would be invoked.
// return 0 is normal, and any non-zero value means error.
//
GMP_WEAK_FUNC_PREFIX
void ctl_mainloop(void) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
}

// This function would be implemented in ctl_main.c
// This function would be called in every controller loop
// This function would be called by @gmp_base_ctl_step
//
// GMP_WEAK_FUNC_PREFIX
// void ctl_dispatch(void) GMP_WEAK_FUNC_SUFFIX
//{
//    // not implement
//}

#ifdef SPECIFY_DISABLE_CSP
GMP_WEAK_FUNC_PREFIX
ec_gt gmp_hal_uart_send(GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE *placeholder1,
                        half_duplex_ift *placeholder2) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
    return 0;
}
#endif

#endif // other compiler support weak symbol
