/**
 * @file default.types.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// This file provide the default type of the whole environment.
// This file should only be included to a headers after the chip configure is included.

#ifndef _FILE_CHIP_DEFAULT_TYPE_H_
#define _FILE_CHIP_DEFAULT_TYPE_H_

//////////////////////////////////////////////////////////////////////////
// Step I Necessary macros
#ifndef NULL
#define NULL ((0))
#endif // NULL

// patch for stdint definitions
#ifndef UINT32_MAX
#define UINT32_MAX ((0xFFFFFFFF))
#endif // UINT32_MAX

//////////////////////////////////////////////////////////////////////////
// Step III Basic element type

// ....................................................................//
// basic element data type
// This type is smallest unit of the chip
// generally, it's a 8-bit number.
//
#ifndef GMP_PORT_DATA_T
#define GMP_PORT_DATA_T              char
#define GMP_PORT_DATA_SIZE_PER_BITS  (8)
#define GMP_PORT_DATA_SIZE_PER_BYTES (1)
#endif // GMP_PORT_DATA_T

typedef GMP_PORT_DATA_T data_gt;

// ....................................................................//
// basic element data type which is fast one
// This type is determined by the width of the chip data bus.
// This type is only designed for flags,
// so user should treat it as a 8-bit type.
//
#ifndef GMP_PORT_FAST_T
#define GMP_PORT_FAST_T              int_fast32_t
#define GMP_PORT_FAST_SIZE_PER_BITS  (32)
#define GMP_PORT_FAST_SIZE_PER_BYTES (4)
#endif // GMP_PORT_FAST_T

typedef GMP_PORT_FAST_T fast_gt;

// This type is disabled, and user should use fast_gt instead.
// // ....................................................................//
// // basic element data type, which is for fast at least 8-bit type
// // Any code may only user the first 8 bits contents
// //
// #ifndef GMP_PORT_FAST8_T
// #define GMP_PORT_FAST8_T              int_fast8_t
// #define GMP_PORT_FAST8_SIZE_PER_BITS  (16)
// #define GMP_PORT_FAST8_SIZE_PER_BYTES (2)
// #endif // GMP_PORT_FAST8_T

// typedef GMP_PORT_FAST8_T fast8_gt;

// ....................................................................//
// basic element data type, which is for fast at least 16-bit type
// Any code may only user the first 16 bits contents
//
#ifndef GMP_PORT_FAST16_T
#define GMP_PORT_FAST16_T              int_fast16_t
#define GMP_PORT_FAST16_SIZE_PER_BITS  (16)
#define GMP_PORT_FAST16_SIZE_PER_BYTES (2)
#endif // GMP_PORT_FAST16_T

typedef GMP_PORT_FAST16_T fast16_gt;

// ....................................................................//
// basic element data type, which is for fast at least 32-bit type
// Any code may only user the first 32 bits contents
//
#ifndef GMP_PORT_FAST32_T
#define GMP_PORT_FAST32_T              int_fast32_t
#define GMP_PORT_FAST32_SIZE_PER_BITS  (32)
#define GMP_PORT_FAST32_SIZE_PER_BYTES (4)
#endif // GMP_PORT_FAST32_T

typedef GMP_PORT_FAST32_T fast32_gt;

// ....................................................................//
// basic element of timing containing
// at least 32 bit
//
#ifndef GMP_PORT_TIME_T
#define GMP_PORT_TIME_T              uint32_t
#define GMP_PORT_TIME_SIZE_PER_BITS  (32)
#define GMP_PORT_TIME_SIZE_PER_BYTES (4)

// This is an addition item for time type
// This value will used to calculate time.
//
#define GMP_PORT_TIME_MAXIMUM (UINT32_MAX)
#endif // GMP_PORT_TIME_T

typedef GMP_PORT_TIME_T time_gt;

// ....................................................................//
// basic element of size containing
//
#ifndef GMP_PORT_SIZE_T
#define GMP_PORT_SIZE_T              uint32_t
#define GMP_PORT_SIZE_SIZE_PER_BITS  (32)
#define GMP_PORT_SIZE_SIZE_PER_BYTES (4)
#endif // GMP_PORT_SIZE_T

typedef GMP_PORT_SIZE_T size_gt;

// ....................................................................//
// basic element of address containing
//
#ifndef GMP_PORT_ADDR_T
#define GMP_PORT_ADDR_T              uint32_t
#define GMP_PORT_ADDR_SIZE_PER_BITS  (32)
#define GMP_PORT_ADDR_SIZE_PER_BYTES (4)
#endif // GMP_PORT_ADDR_T

typedef GMP_PORT_ADDR_T addr_gt;

// ....................................................................//
// basic element of address containing with at least 32-bit size
//
#ifndef GMP_PORT_ADDR32_T
#define GMP_PORT_ADDR32_T              uint32_t
#define GMP_PORT_ADDR32_SIZE_PER_BITS  (32)
#define GMP_PORT_ADDR32_SIZE_PER_BYTES (4)
#endif // GMP_PORT_ADDR_T

typedef GMP_PORT_ADDR32_T addr32_gt;

// ....................................................................//
// basic element of address containing with at least 16-bit size
//
#ifndef GMP_PORT_ADDR16_T
#define GMP_PORT_ADDR16_T              uint16_t
#define GMP_PORT_ADDR16_SIZE_PER_BITS  (16)
#define GMP_PORT_ADDR16_SIZE_PER_BYTES (2)
#endif // GMP_PORT_ADDR_T

typedef GMP_PORT_ADDR16_T addr16_gt;

// ....................................................................//
// basic element of address offset containing
//
#ifndef GMP_PORT_DIFF_T
#define GMP_PORT_DIFF_T              int32_t
#define GMP_PORT_DIFF_SIZE_PER_BITS  (32)
#define GMP_PORT_DIFF_SIZE_PER_BYTES (4)
#endif // GMP_PORT_DIFF_T

typedef GMP_PORT_DIFF_T diff_gt;

// ....................................................................//
// basic container of parameters
//
#ifndef GMP_PORT_PARAM_T
#define GMP_PORT_PARAM_T              int32_t
#define GMP_PORT_PARAM_SIZE_PER_BITS  (32)
#define GMP_PORT_PARAM_SIZE_PER_BYTES (4)
#endif // GMP_PORT_PARAM_T

typedef GMP_PORT_PARAM_T param_gt;

// ....................................................................//
// basic container of ADC results
//
#ifndef GMP_PORT_ADC_T
#define GMP_PORT_ADC_T              uint32_t
#define GMP_PORT_ADC_SIZE_PER_BITS  (32)
#define GMP_PORT_ADC_SIZE_PER_BYTES (4)
#endif // GMP_PORT_ADC_T

typedef GMP_PORT_ADC_T adc_gt;

// ....................................................................//
// basic container of ADC results
//
#ifndef GMP_PORT_DAC_T
#define GMP_PORT_DAC_T              int32_t
#define GMP_PORT_DAC_SIZE_PER_BITS  (32)
#define GMP_PORT_DAC_SIZE_PER_BYTES (4)
#endif // GMP_PORT_DAC_T

typedef GMP_PORT_DAC_T dac_gt;

// ....................................................................//
// basic container of PWM results
// output type must be signed type
//
#ifndef GMP_PORT_PWM_T
#define GMP_PORT_PWM_T              int32_t
#define GMP_PORT_PWM_SIZE_PER_BITS  (32)
#define GMP_PORT_PWM_SIZE_PER_BYTES (4)
#endif // GMP_PORT_PWM_T

typedef GMP_PORT_ADC_T pwm_gt;

#endif // _FILE_CHIP_DEFAULT_TYPE_H_
