/**
 * @file csp.typedef.h
 * @brief Fundamental GMP port type selections for hosted Linux targets.
 */

#ifndef GMP_LINUX_SIMULINK_CSP_TYPEDEF_H
#define GMP_LINUX_SIMULINK_CSP_TYPEDEF_H

/* Linux hosts expose an addressable eight-bit byte. */
#define GMP_PORT_DATA_T              unsigned char
#define GMP_PORT_DATA_SIZE_PER_BITS  (8)
#define GMP_PORT_DATA_SIZE_PER_BYTES (1)

#endif /* GMP_LINUX_SIMULINK_CSP_TYPEDEF_H */
