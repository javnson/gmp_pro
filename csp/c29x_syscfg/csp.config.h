/**
 * @file csp.config.h
 * @brief GMP configuration for TI C29x devices configured by SysConfig.
 */

#ifndef GMP_C29X_CSP_CONFIG_H
#define GMP_C29X_CSP_CONFIG_H

#define SPECIFY_DISABLE_CSP_EXIT
#define USER_SPECIFIED_PRINT_FUNCTION(format, ...) \
    gmp_base_print_c29xsyscfg((format), ##__VA_ARGS__)
#define GMP_USER_PRINT_FUNCTION_DECLARATION size_gt gmp_base_print_c29xsyscfg(const char *format, ...);

#endif /* GMP_C29X_CSP_CONFIG_H */
