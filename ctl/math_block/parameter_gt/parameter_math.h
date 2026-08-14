/**
 * @file parameter_math.h
 * @brief Non-linear math operations for the parameter_gt domain.
 *
 * parameter_gt is always a native floating-point type. Basic arithmetic uses
 * the C operators directly; this file only owns operations that otherwise
 * expose a concrete float/double libm function in CTL algorithms.
 *
 * Unlike ctl_sin()/ctl_cos(), trigonometric arguments here are in radians.
 */

#ifndef FILE_GMP_PARAMETER_MATH_H_
#define FILE_GMP_PARAMETER_MATH_H_

#include <math.h>

#if (SPECIFY_PARAMETER_GT_TYPE == USING_DOUBLE_FPU)
#define param_abs(value)          ((parameter_gt)fabs((double)(value)))
#define param_sin(value)          ((parameter_gt)sin((double)(value)))
#define param_cos(value)          ((parameter_gt)cos((double)(value)))
#define param_tan(value)          ((parameter_gt)tan((double)(value)))
#define param_asin(value)         ((parameter_gt)asin((double)(value)))
#define param_acos(value)         ((parameter_gt)acos((double)(value)))
#define param_atan(value)         ((parameter_gt)atan((double)(value)))
#define param_atan2(y, x)         ((parameter_gt)atan2((double)(y), (double)(x)))
#define param_exp(value)          ((parameter_gt)exp((double)(value)))
#define param_ln(value)           ((parameter_gt)log((double)(value)))
#define param_log10(value)        ((parameter_gt)log10((double)(value)))
#define param_pow(base, exponent) ((parameter_gt)pow((double)(base), (double)(exponent)))
#define param_sqrt(value)         ((parameter_gt)sqrt((double)(value)))
#define param_floor(value)        ((parameter_gt)floor((double)(value)))
#define param_ceil(value)         ((parameter_gt)ceil((double)(value)))
#define param_mod(value, divisor) ((parameter_gt)fmod((double)(value), (double)(divisor)))
#else
#define param_abs(value)          ((parameter_gt)fabsf((float)(value)))
#define param_sin(value)          ((parameter_gt)sinf((float)(value)))
#define param_cos(value)          ((parameter_gt)cosf((float)(value)))
#define param_tan(value)          ((parameter_gt)tanf((float)(value)))
#define param_asin(value)         ((parameter_gt)asinf((float)(value)))
#define param_acos(value)         ((parameter_gt)acosf((float)(value)))
#define param_atan(value)         ((parameter_gt)atanf((float)(value)))
#define param_atan2(y, x)         ((parameter_gt)atan2f((float)(y), (float)(x)))
#define param_exp(value)          ((parameter_gt)expf((float)(value)))
#define param_ln(value)           ((parameter_gt)logf((float)(value)))
#define param_log10(value)        ((parameter_gt)log10f((float)(value)))
#define param_pow(base, exponent) ((parameter_gt)powf((float)(base), (float)(exponent)))
#define param_sqrt(value)         ((parameter_gt)sqrtf((float)(value)))
#define param_floor(value)        ((parameter_gt)floorf((float)(value)))
#define param_ceil(value)         ((parameter_gt)ceilf((float)(value)))
#define param_mod(value, divisor) ((parameter_gt)fmodf((float)(value), (float)(divisor)))
#endif

/* Explicit PU-angle variants. One PU turn equals 2*pi radians. */
#define param_sin_pu(value) param_sin(CTL_PARAM_CONST_2PI * (value))
#define param_cos_pu(value) param_cos(CTL_PARAM_CONST_2PI * (value))

#endif /* FILE_GMP_PARAMETER_MATH_H_ */
