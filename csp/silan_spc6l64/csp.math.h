// This math file is for Silan Motor Controller

// #ifndef GLOBAL_Q
//// Specify global Q value
// #define GLOBAL_Q (24)
// #endif // GLOBAL_Q

// include iqmath library
// #include <third_party/iqmath/IQmathLib.h>

#ifndef _FILE_IQMATH_MACROS_H_
#define _FILE_IQMATH_MACROS_H_

#ifndef saturation_macro
#define saturation_macro(_a, _min, _max) ((_a) <= (_min)) ? (_min) : (((_a) >= (_max)) ? (_max) : (_a))
#endif

// Type conversion function
#define GMP_CTRL_FROM_REAL(x) (_IQ(x))
#define GMP_CTRL_TO_REAL(x)   (_IQtoF(x))
#define int2ctrl(x)   (_IQ(x))
#define ctrl2int(x)   (_IQint(x))
#define ctrl_mod_1(x) (_IQfrac(x))

// Calculation
#define ctl_mul(A, B)        (silan_mult_Iq24(A, B))
#define pwm_mul(A, B)        (silan_mult_Iq24(A, B))
#define ctl_div(A, B)        (silan_div_Iq24(A, B))
#define ctl_sat(A, Pos, Neg) saturation_macro((A), (Neg), (Pos))
#define pwm_sat(A, Pos, Neg) saturation_macro((A), (Neg), (Pos))

// Extension Calculation
#define ctl_div2(A) (_IQdiv2(A))
#define ctl_div4(A) (_IQdiv4(A))

// Add and sub
#define ctl_add(A, B) ((A + B))
#define ctl_sub(A, B) ((A - B))

// Nonlinear support
#define ctl_sin(A)      (_IQsinPU(A))
#define ctl_cos(A)      (_IQcosPU(A))
#define ctl_tan(A)      (_IQdiv(_IQsinPU(A), _IQcosPU(A)))
#define ctl_atan2(Y, X) (_IQatan2PU((Y), (X)))
#define ctl_exp(A)      (_IQexp(A))
#define ctl_ln(A)       (_IQlog(A))
#define ctl_sqrt(A)     (_IQsqrt(A))
#define ctl_isqrt(A)    (_IQisqrt(A))

// Specify ctrl_gt is a fixed number
#define CTRL_GT_IS_FIXED

#endif // _FILE_IQMATH_MACROS_H_
