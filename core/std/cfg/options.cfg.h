// This file provide series of macros options definition.

// Specify chip byte endian type
#define GMP_CHIP_LITTLE_ENDIAN ((0x01))
#define GMP_CHIP_BIG_ENDIAN    ((0x02))

// GMP startup logo selections
#define GMP_LOGO_MODE_FULL     ((0x00))
#define GMP_LOGO_MODE_LITE     ((0x01))
#define GMP_LOGO_MODE_DISABLED ((0xFF))

// GMP execution environment selections. The OS owns scheduling whenever an
// RTOS backend is selected; GMP only contributes a non-blocking service task.
#define GMP_OS_BACKEND_NONE     ((0x00))
#define GMP_OS_BACKEND_FREERTOS ((0x01))

// Selections of Memory controller functions
#define USING_DEFAULT_SYSTEM_DEFAULT_FUNCTION ((0x01))
#define USING_GMP_BLOCK_DEFAULT_FUNCTION      ((0x02))
#define USING_MANUAL_SPECIFY_FUNCTION         ((0xFF))

// Selections of `ctrl_gt` type
#define USING_FIXED_TI_IQ_LIBRARY       ((0x01))
#define USING_FIXED_ARM_CMSIS_Q_LIBRARY ((0x02))
#define USING_QFPLIB_FLOAT              ((0x03))

#define USING_FLOAT_TI_IQ_LIBRRARY      ((0x80))
#define USING_FLOAT_FPU                 ((0x81))
#define USING_DOUBLE_FPU                ((0x82))
#define USING_LONG_DOUBLE               ((0x83))

#define USING_FLOAT_CLA_LIBRARY         ((0xAA))
