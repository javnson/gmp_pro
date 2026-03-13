//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all necessary GMP config macro in this file.
//
// WARNING: This file must be kept in the include search path during compilation.
//

//////////////////////////////////////////////////////////////////////////
// GMP core config module

// Disable CSP
// #define SPECIFY_DISABLE_CSP

// Disable Base print function
// #define SPECIFY_BASE_PRINT_NOT_IMPL

// using user specified default log print function
#define USER_SPECIFIED_PRINT_FUNCTION printf_s

// Disable GMP startup screen
// #define SPECIFY_DISABLE_GMP_LOGO

//////////////////////////////////////////////////////////////////////////
// CTL config module
//

// Disable GMP CTL module
// #define SPECIFY_DISABLE_GMP_CTL

// Specify enable CTL library
#define SPECIFY_ENABLE_GMP_CTL

// Specify enable CTL framework nano
// #define SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

// Specify GMP CTL default type

// #define SPECIFY_CTRL_GT_TYPE USING_DOUBLE_FPU
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
// #define SPECIFY_CTRL_GT_TYPE USING_FIXED_TI_IQ_LIBRARY

//////////////////////////////////////////////////////////////////////////
// Windows Simulate platform config

// PC environment setup
#define SPECIFY_PC_ENVIRONMENT

// PC environment maximum loop counter
#define PC_ENV_MAX_ITERATION ((100000000))

// specify ASIO config .json file
#define GMP_ASIO_CONFIG_JSON "network.json"

// specify Simulink Communicate buffer
#define gmp_pc_simulink_rx_buffer_t dp_sil_rx_buf_t
#define gmp_pc_simulink_tx_buffer_t dp_sil_tx_buf_t

// Invoke Controller Settings
#include <ctrl_settings.h>
