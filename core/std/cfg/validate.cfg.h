
// This file validate configurations

#ifndef _FILE_GMP_CORE_VALIDATE_CFG_H_
#define _FILE_GMP_CORE_VALIDATE_CFG_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////
// validate parameters
// Note: In C++ environments, some compilers automatically define BIG_ENDIAN/LITTLE_ENDIAN
// So we only check if these macros are defined by user (before including system headers)
#if defined(__cplusplus)
// In C++ mode, we ignore automatically defined endian macros
// and use GMP's own endian configuration instead
#else
// In C mode, check if both endian macros are defined
#if defined BIG_ENDIAN && defined LITTLE_ENDIAN
#error "You should chose big-endian or little-endian."
#endif // BIG_ENDIAN && LITTLE_ENDIAN
#endif // __cplusplus

#if SPECIFY_GMP_DEFAULT_ALLOC == USING_MANUAL_SPECIFY_FUNCTION
#if !((defined SPECIFY_GMP_USER_ALLOC) && (defined SPECIFY_GMP_USER_FREE))
#error "You should specify user_alloc and user_free function, via config files."
#endif
#endif // USING_MANUAL_SPECIFY_FUNCTION

	// To mark a unused param
#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(x) ((void)(x))
#endif // UNUSED_PARAMETER

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_CORE_VALIDATE_CFG_H_
