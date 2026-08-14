/**
 * @file csp.config.h
 * @brief Default GMP configuration for native Linux SIL/PIL processes.
 */

#ifndef GMP_LINUX_SIMULINK_CSP_CONFIG_H
#define GMP_LINUX_SIMULINK_CSP_CONFIG_H

#define SPECIFY_PC_ENVIRONMENT

#ifndef PC_ENV_MAX_ITERATION
#define PC_ENV_MAX_ITERATION ((100000000))
#endif

#ifndef GMP_ASIO_UDP_LINK_TIMEOUT
#define GMP_ASIO_UDP_LINK_TIMEOUT ((10000))
#endif

#ifndef GMP_PC_CONTROLLER_DIV_PER_MAINLOOP
#define GMP_PC_CONTROLLER_DIV_PER_MAINLOOP ((5))
#endif

#ifndef GMP_PC_SIMULINK_TX_STRUCT
#define GMP_PC_SIMULINK_TX_STRUCT tx_buf_t
#endif

#ifndef GMP_PC_SIMULINK_RX_STRUCT
#define GMP_PC_SIMULINK_RX_STRUCT rx_buf_t
#endif

#ifndef gmp_pc_simulink_tx_buffer_t
#define gmp_pc_simulink_tx_buffer_t GMP_PC_SIMULINK_TX_STRUCT
#endif

#ifndef gmp_pc_simulink_rx_buffer_t
#define gmp_pc_simulink_rx_buffer_t GMP_PC_SIMULINK_RX_STRUCT
#endif

#ifndef GMP_ASIO_CONFIG_JSON
#define GMP_ASIO_CONFIG_JSON "network.json"
#endif

#define GMP_ASIO_ENABLE_STOP_CMD

/* A hosted target writes diagnostic output directly to standard output. */
#ifndef USER_SPECIFIED_PRINT_FUNCTION
#define USER_SPECIFIED_PRINT_FUNCTION printf
#endif

#ifndef GMP_BASE_TIME_TICK_RESOLUTION
#define GMP_BASE_TIME_TICK_RESOLUTION ((1000))
#endif

#ifndef PC_SIMULATE_STOP_CONDITION
#define PC_SIMULATE_STOP_CONDITION
#endif

#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN
#endif

#endif /* GMP_LINUX_SIMULINK_CSP_CONFIG_H */
