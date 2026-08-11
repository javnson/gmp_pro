/**
 * @file sdpe_gmp_ctl_portable_settings.h
 * @brief SDPE project bindings for GMP CTL Portable for TI C2000 DSP.
 * @note SDPE-managed compile-time contract for the no-CSP TI C28x/C29x CTL integration.
 */

#ifndef _PROJECT_SDPE_GMP_CTL_PORTABLE_SETTINGS_H_
#define _PROJECT_SDPE_GMP_CTL_PORTABLE_SETTINGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
// SDPE extension point: add after_extern_open code in the Project Requirement Code page if needed.

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define GMP_CTL_PORTABLE_TI_DSP_SDPE_PROJECT_ID "gmp_ctl_portable_ti_dsp"
#define GMP_CTL_PORTABLE_TI_DSP_SDPE_PROJECT_SUITE "ctl_portable"
#define GMP_CTL_PORTABLE_TI_DSP_SDPE_PROJECT_VERSION "1.0.0"
#define GMP_CTL_PORTABLE_TI_DSP_SDPE_PROJECT_UPDATED_AT "2026-08-11"

//=================================================================================================
/**
 * @brief Portable Mode.
 */

/**
 * @brief Bypass the GMP runtime and CSP when gmp_core.h is included. Also define this symbol globally or force-include the generated SDPE header.
 */
#ifndef GMP_CTL_PORTABLE
#define GMP_CTL_PORTABLE
#endif // GMP_CTL_PORTABLE

//=================================================================================================
/**
 * @brief Numeric Backend.
 */

/**
 * @brief CTL real-time numeric backend. IQmath also requires its headers and library.
 *        Options: USING_FLOAT_FPU, USING_DOUBLE_FPU, USING_FIXED_TI_IQ_LIBRARY
 */
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU

/**
 * @brief Physical parameter storage type.
 *        Options: USING_FLOAT_FPU, USING_DOUBLE_FPU
 */
#define SPECIFY_PARAMETER_GT_TYPE USING_FLOAT_FPU

// User project tail code
// SDPE extension point: add before_footer code in the Project Requirement Code page if needed.

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_GMP_CTL_PORTABLE_SETTINGS_H_
