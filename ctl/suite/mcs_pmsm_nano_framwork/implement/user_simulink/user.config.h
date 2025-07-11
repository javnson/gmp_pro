// This is the example of user.config.h

// Specify the running environment
//#define MASTERCHIP GMP_AUTO_STM32

// Disable CSP
//#define SPECIFY_DISABLE_CSP

// Disable Base print function
//#define SPECIFY_BASE_PRINT_NOT_IMPL

// using user specified default log print function
#define USER_SPECIFIED_PRINT_FUNCTION printf

// Disable GMP startup screen
//#define SPECIFY_DISABLE_GMP_LOGO





//////////////////////////////////////////////////////////////////////////
// CTL config module
// 
// Disable GMP CTL module
//#define SPECIFY_DISABLE_GMP_CTL

// Specify enable CTL framework nano
//#define SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

// Specify GMP CTL default type
// 
// #define SPECIFY_CTRL_GT_TYPE USING_DOUBLE_FPU
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
// #define SPECIFY_CTRL_GT_TYPE USING_FIXED_TI_IQ_LIBRARY

// Specify enable CTL library, this macro is disabled
//#define SPECIFY_ENABLE_GMP_CTL

//// Specify enable CTL framework nano
//#define SPECIFY_ENABLE_CTL_FRAMEWORK_NANO


//////////////////////////////////////////////////////////////////////////
// Windows Simulate platform

// PC environment setup
#define SPECIFY_PC_ENVIRONMENT

// PC environment maximum loop counter
#define PC_ENV_MAX_ITERATION ((100000000))

// specify ASIO config .json file
#define GMP_ASIO_CONFIG_JSON "network.json"



