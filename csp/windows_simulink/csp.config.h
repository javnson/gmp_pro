/**
 * @file csp.config.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// This file provide a Chip Support Package configuration for an example device

// PC environment
#define SPECIFY_PC_ENVIRONMENT

// Max iteration
#ifndef PC_ENV_MAX_ITERATION
#define PC_ENV_MAX_ITERATION ((100000000))
#endif

// Connection Timeout
#ifndef GMP_ASIO_UDP_LINK_TIMEOUT
#define GMP_ASIO_UDP_LINK_TIMEOUT ((10000))
#endif // GMP_ASIO_UDP_LINK_TIMEOUT

// Controller tick per main loop tick
// controller routine would be called when every N mainloop tick happened.
#define GMP_PC_CONTROLLER_DIV_PER_MAINLOOP ((5))

// Speicfy Simulink communication TX structure type
#define GMP_PC_SIMULINK_TX_STRUCT tx_buf_t

// Specify Simulink communication RX structure type
#define GMP_PC_SIMULINK_RX_STRUCT rx_buf_t

// ASIO network config file path/name
#define GMP_ASIO_CONFIG_JSON "network.json"

// Specify enable "Stop Command"
#define GMP_ASIO_ENABLE_STOP_CMD

// Default handle of GMP base print function
#ifndef GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE
#define GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE uint32_t
#endif // GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE

// default transmit function of GMP base print function
#ifndef GMP_BASE_PRINT_FUNCTION
#define GMP_BASE_PRINT_FUNCTION(A, B) windows_print_function(A, B)
#endif // GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE

// define GMP Base time tick resolution
// tick timer will pulse GMP_BASE_TIME_TICK_RESOLUTION times in 1 second.
#ifndef GMP_BASE_TIME_TICK_RESOLUTION
#define GMP_BASE_TIME_TICK_RESOLUTION 1000
#endif // GMP_BASE_TIME_TICK_RESOLUTION

// Enable GMP LOGO
#ifdef SPECIFY_DISABLE_GMP_LOGO
#undef SPECIFY_DISABLE_GMP_LOGO
#endif // SPECIFY_DISABLE_GMP_LOGO

// Enable GMP CTL
// #ifndef SPECIFY_ENABLE_GMP_CTL
// #define SPECIFY_ENABLE_GMP_CTL
// #endif // SPECIFY_ENABLE_GMP_CTL

// Enable GMP CTL Controller Framework Nano
// #ifndef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
// #define SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
// #endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

// Simulation condition stop
// if connection is setup but no message comes in 2 sec,
// stop simulation.
#ifndef PC_SIMULATE_STOP_CONDITION
#define PC_SIMULATE_STOP_CONDITION
#endif // PC_SIMULATE_STOP_CONDITION

// Select Little endian default
#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN
#endif // LITTLE_ENDIAN

// Simulation will not stop automatically.
// #define DISABLE_ASIO_HELPER_TIMEOUT_OPTION
