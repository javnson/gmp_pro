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

// This file provide a Chip Support Package configuration for RP PICO

#define USER_SPECIFIED_PRINT_FUNCTION(A, ...) gmp_base_print_default(A, ##__VA_ARGS__)

#define SPECIFY_DISABLE_CSP_EXIT
