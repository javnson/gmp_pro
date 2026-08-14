#ifndef _FILE_ARCH_C29X_H_
#define _FILE_ARCH_C29X_H_

/* TI C29 is byte-addressed and optimized for 32-bit arithmetic. */
#ifndef ARCH_NAME
#define ARCH_NAME "C29x"
#endif

#ifndef SIZEOF_UNIT
#define SIZEOF_UNIT (1)
#endif

#ifndef GMP_PORT_DATA_T
#define GMP_PORT_DATA_T              uint8_t
#define GMP_PORT_DATA_SIZE_PER_BITS  (8)
#define GMP_PORT_DATA_SIZE_PER_BYTES (1)
#endif

#ifndef GMP_PORT_FAST_T
#define GMP_PORT_FAST_T              int_fast32_t
#define GMP_PORT_FAST_SIZE_PER_BITS  (32)
#define GMP_PORT_FAST_SIZE_PER_BYTES (4)
#endif

#ifndef GMP_PORT_FAST8_T
#define GMP_PORT_FAST8_T               int_fast32_t
#define GMP_PORT_FAST8_SIZE_PER_BITS   (32)
#define GMP_PORT_FAST8_SIZE_PER_BYTES  (4)
#endif

#ifndef GMP_PORT_FAST16_T
#define GMP_PORT_FAST16_T               int_fast32_t
#define GMP_PORT_FAST16_SIZE_PER_BITS   (32)
#define GMP_PORT_FAST16_SIZE_PER_BYTES  (4)
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

#endif // _FILE_ARCH_C29X_H_
