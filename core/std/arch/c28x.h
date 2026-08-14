#ifndef _FILE_ARCH_C28X_H_
#define _FILE_ARCH_C28X_H_

/* C28x has a 16-bit addressable C byte and therefore does not provide
 * int8_t/uint8_t.  Reusable code must use data_gt or fast_gt for small state
 * values instead of assuming an 8-bit native type. */
#ifndef ARCH_NAME
#define ARCH_NAME "C28x"
#endif

#ifndef SIZEOF_UNIT
#define SIZEOF_UNIT (2)
#endif

#ifndef GMP_PORT_DATA_T
#define GMP_PORT_DATA_T              int16_t
#define GMP_PORT_DATA_SIZE_PER_BITS  (16)
#define GMP_PORT_DATA_SIZE_PER_BYTES (2)
#endif

#ifndef GMP_PORT_FAST_T
#define GMP_PORT_FAST_T              int_fast16_t
#define GMP_PORT_FAST_SIZE_PER_BITS  (16)
#define GMP_PORT_FAST_SIZE_PER_BYTES (2)
#endif

#ifndef GMP_PORT_FAST8_T
#define GMP_PORT_FAST8_T               int16_t
#define GMP_PORT_FAST8_SIZE_PER_BITS   (16)
#define GMP_PORT_FAST8_SIZE_PER_BYTES  (2)
#endif

#ifndef GMP_PORT_FAST16_T
#define GMP_PORT_FAST16_T               int16_t
#define GMP_PORT_FAST16_SIZE_PER_BITS   (16)
#define GMP_PORT_FAST16_SIZE_PER_BYTES  (2)
#endif

#ifndef GMP_PORT_FAST32_T
#define GMP_PORT_FAST32_T               int_fast32_t
#define GMP_PORT_FAST32_SIZE_PER_BITS   (32)
#define GMP_PORT_FAST32_SIZE_PER_BYTES  (4)
#endif

#ifndef GMP_PORT_TIME_T
#define GMP_PORT_TIME_T              uint32_t
#define GMP_PORT_TIME_SIZE_PER_BITS  (32)
#define GMP_PORT_TIME_SIZE_PER_BYTES (4)
#define GMP_PORT_TIME_MAXIMUM        (UINT32_MAX)
#endif

#ifndef GMP_PORT_SIZE_T
#define GMP_PORT_SIZE_T              uint32_t
#define GMP_PORT_SIZE_SIZE_PER_BITS  (32)
#define GMP_PORT_SIZE_SIZE_PER_BYTES (4)
#endif

#ifndef GMP_PORT_ADDR_T
#define GMP_PORT_ADDR_T              uint32_t
#define GMP_PORT_ADDR_SIZE_PER_BITS  (32)
#define GMP_PORT_ADDR_SIZE_PER_BYTES (4)
#endif

#ifndef GMP_PORT_ADDR32_T
#define GMP_PORT_ADDR32_T              uint32_t
#define GMP_PORT_ADDR32_SIZE_PER_BITS  (32)
#define GMP_PORT_ADDR32_SIZE_PER_BYTES (4)
#endif

#ifndef GMP_PORT_ADDR16_T
#define GMP_PORT_ADDR16_T              uint32_t
#define GMP_PORT_ADDR16_SIZE_PER_BITS  (32)
#define GMP_PORT_ADDR16_SIZE_PER_BYTES (4)
#endif

#ifndef GMP_PORT_DIFF_T
#define GMP_PORT_DIFF_T              int32_t
#define GMP_PORT_DIFF_SIZE_PER_BITS  (32)
#define GMP_PORT_DIFF_SIZE_PER_BYTES (4)
#endif

#ifndef GMP_PORT_PARAM_T
#define GMP_PORT_PARAM_T              int32_t
#define GMP_PORT_PARAM_SIZE_PER_BITS  (32)
#define GMP_PORT_PARAM_SIZE_PER_BYTES (4)
#endif

#ifndef GMP_PORT_ADC_T
#define GMP_PORT_ADC_T              uint32_t
#define GMP_PORT_ADC_SIZE_PER_BITS  (32)
#define GMP_PORT_ADC_SIZE_PER_BYTES (4)
#endif

#ifndef GMP_PORT_DAC_T
#define GMP_PORT_DAC_T              uint32_t
#define GMP_PORT_DAC_SIZE_PER_BITS  (32)
#define GMP_PORT_DAC_SIZE_PER_BYTES (4)
#endif

#ifndef GMP_PORT_PWM_T
#define GMP_PORT_PWM_T              uint32_t
#define GMP_PORT_PWM_SIZE_PER_BITS  (32)
#define GMP_PORT_PWM_SIZE_PER_BYTES (4)
#endif

#endif // _FILE_ARCH_C28X_H_
