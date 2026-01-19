/**
 * @file cc.c2000.inl
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

//////////////////////////////////////////////////////////////////////////
// Step I language patch
//
// Wrong C++ version
#ifdef __cplusplus

#if __cplusplus < 199711L
#error This library needs at least a C++03 compliant compiler.
#endif // __cplusplus <= 199711L

// Lower version compiler patch
#if __cplusplus < 201103L
#define constexpr const
#define override
#define final

#ifndef nullptr
#define nullptr (0x00000000)
#endif // nullptr alternative definition

#endif // __cplusplus <= 201103L

#endif // __cplusplus

#ifndef __STATIC_INLINE
#define __STATIC_INLINE static inline
#endif //__STATIC_INLINE

//////////////////////////////////////////////////////////////////////////
// Step II system library

//////////////////////////////////////////////////////////////////////////
// Step III library support macro

// + weak function Modifier
 #define GMP_WEAK_FUNC_PREFIX __attribute__((weak))
//#define GMP_WEAK_FUNC_PREFIX __attribute((weak))
// #define GMP_WEAK_FUNC_PREFIX _weak
// #define GMP_WEAK_FUNC_PREFIX //_Pragma("weak")
#define GMP_WEAK_FUNC_SUFFIX //__attribute__((weak))

// + disable optimization
 #define GMP_NO_OPT_PREFIX 
 //#define GMP_NO_OPT_PREFIX _Pragma("FUNCTION_OPTIONS(\"--opt_level=0\")")
//#define GMP_NO_OPT_PREFIX _Pragma("FUNC_OPTIMIZE_OPTLEVEL(\"0\")")
//#define GMP_NO_OPT_PREFIX _Pragma("OPTIMIZE(\" off \")")
//#define GMP_NO_OPT_PREFIX __attribute__((optimize("O0")))
//#define GMP_NO_OPT_PREFIX __attribute__((optimize("off")));
#define GMP_NO_OPT_SUFFIX 

// + variables aligned
#define GMP_MEM_ALIGN _Pragma("DATA_ALIGN(4)")

// + inline modifier
#define GMP_INLINE _Pragma("FUNC_ALWAYS_INLINE")

// + static inline modifier
#define GMP_FORCE_INLINE _Pragma("FORCEINLINE")

// + NOP
#define GMP_INSTRUCTION_NOP asm("nop")
