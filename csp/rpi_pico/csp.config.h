/**
 * @file csp.config.h
 * @brief GMP chip support package configuration for Raspberry Pi Pico boards.
 */

#ifndef GMP_CSP_RPI_PICO_CONFIG_H
#define GMP_CSP_RPI_PICO_CONFIG_H

#define USER_SPECIFIED_PRINT_FUNCTION(A, ...) gmp_base_print_default(A, ##__VA_ARGS__)

/* Bare-metal applications do not return from gmp_base_entry(). */
#define SPECIFY_DISABLE_CSP_EXIT

#endif /* GMP_CSP_RPI_PICO_CONFIG_H */
