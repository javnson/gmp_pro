/**
 * @file ctrl_gt_patch.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a common header patch for the ctrl_gt type and related macros.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file is intended to provide compatibility patches or aliases for macros
 * used throughout the control library. It can help resolve naming conflicts or
 * ensure consistent behavior across different modules.
 */

#ifndef _FILT_CTRL_GT_PATCH_H_
#define _FILT_CTRL_GT_PATCH_H_

/**
 * @defgroup MC_PATCHES Compatibility Patches
 * @ingroup MC_CTRL_GT_IMPL
 * @brief A collection of macros for ensuring compatibility.
 * @{
 */

/**
 * @brief An alias for the `ctrl_mod_1` macro.
 *
 * This macro simply maps to itself. It might be used to ensure that a specific
 * version of the macro is used, or to prevent redefinition warnings in certain
 * build configurations.
 */
#define ctl_mod_1(x) ctrl_mod_1(x)

/** @} */ // end of MC_PATCHES group

#endif //_FILT_CTRL_GT_PATCH_H_
