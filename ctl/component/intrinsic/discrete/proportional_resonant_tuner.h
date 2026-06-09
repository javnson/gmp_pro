/**
 * @file proportional_resonant_tuner.h
 * @brief Non-Intrusive Runtime Parametric Tuning Companions Framework SDK.
 * @details Establishes shadow-register bridge nodes for both ideal and quasi-resonant 
 * blocks to support asynchronous background tuning compiles without disrupting active loops.
 * @version 0.2
 * @date 2026-06-09
 * * @copyright Copyright GMP(c) 2026
 */

#ifndef PROPORTIONAL_RESONANT_TUNER_H
#define PROPORTIONAL_RESONANT_TUNER_H

#include "proportional_resonant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Discretization Mode Specifier for Quasi-Resonant Loops                    */
/*---------------------------------------------------------------------------*/

/**
 * @brief Discretization mapping strategies for Quasi-Resonant Tuners.
 */
typedef enum
{
    CTL_TUNE_QR_TUSTIN = 0, /**< Standard Bilinear Tustin transformation mapping (K = 2*Fs). */
    CTL_TUNE_QR_PREWARPED   /**< Modified Bilinear transform matching resonant frequency warp. */
} ctl_tune_qr_mode_e;

/*---------------------------------------------------------------------------*/
/* Pure Resonant (R) Tuner Component                                         */
/*---------------------------------------------------------------------------*/

/**
 * @brief External Tuning Bridge Manager for pure Resonant (R) Controllers.
 */
typedef struct _tag_ctl_resonant_tuner
{
    parameter_gt target_kr;            /**< Target resonant gain command workspace. */
    parameter_gt target_freq_resonant; /**< Target resonant frequency center command (Hz). */

    ctl_resonant_coef_t shadow_coef; /**< Staged discrete parameters computed block. */
    fast_gt flag_update_pending;     /**< Safe inter-boundary data transition indicator flag. */
} ctl_resonant_tuner_t;

/**
 * @brief Compiles physical parameters inside the ideal R tuner into staging digital shadow coefficients.
 * @details Performs background algebraic discretization without affecting the running loop.
 * * @param[in,out] tuner Pointer to the background pure resonant tuner companion instance.
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_tune_resonant_compile(ctl_resonant_tuner_t* tuner, parameter_gt fs);

/**
 * @brief Inputs new physical objectives into the ideal R tuner workspace and triggers automatic compilation.
 * * @param[out] tuner Pointer to the background pure resonant tuner companion instance.
 * @param[in] kr Target resonant gain.
 * @param[in] freq_resonant Target resonant center frequency (Hz).
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_tune_resonant(ctl_resonant_tuner_t* tuner, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs);

/**
 * @brief Decodes an active running ideal R controller back into user-friendly fields to initialize the tuner.
 * @note Designed for hot-plugging diagnostic toolsets to sync parameters smoothly.
 * * @param[out] tuner Pointer to the background pure resonant tuner companion instance to be backward initialized.
 * @param[in] active_ctrl Pointer to the currently running unmodified execution pure resonant controller.
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_init_resonant_tuner_from_ctrl(ctl_resonant_tuner_t* tuner, const resonant_ctrl_t* active_ctrl,
                                       parameter_gt fs);

/**
 * @brief Joint initialization routine configuring both the ideal R controller core and tuner companion simultaneously.
 * * @param[out] r Pointer to the running pure resonant controller core instance.
 * @param[out] tuner Pointer to the companion pure resonant tuner manager instance.
 * @param[in] kr Initial resonant gain.
 * @param[in] freq_resonant Initial resonant center frequency (Hz).
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_init_tunable_resonant_controller(resonant_ctrl_t* r, ctl_resonant_tuner_t* tuner, parameter_gt kr,
                                          parameter_gt freq_resonant, parameter_gt fs);

/**
 * @brief Rigid Shock-Free Coefficient Swapper for Primary Resonant Loop.
 * @param[in,out] tuner Pointer to the background tuner companion instance.
 * @param[out] r Pointer to the running execution controller instance.
 */
GMP_STATIC_INLINE void ctl_tune_resonant_deploy(ctl_resonant_tuner_t* tuner, resonant_ctrl_t* r)
{
    if (tuner->flag_update_pending)
    {
        r->coef = tuner->shadow_coef; /* Atomic structural assignment block move */
        tuner->flag_update_pending = 0;
    }
}

/*---------------------------------------------------------------------------*/
/* Quasi-Resonant (QR) Tuner Component                                       */
/*---------------------------------------------------------------------------*/

/**
 * @brief External Tuning Bridge Manager for Quasi-Resonant (QR) Controllers.
 */
typedef struct _tag_ctl_qr_tuner
{
    parameter_gt target_kr;            /**< Target quasi-resonant gain command workspace. */
    parameter_gt target_freq_resonant; /**< Target resonant center frequency command (Hz). */
    parameter_gt target_freq_cut;      /**< Target filter damping cutoff bandwidth command (Hz). */

    ctl_tune_qr_mode_e method_mode; /**< Discretization mapping operational strategy flag. */

    ctl_qr_coef_t shadow_coef;   /**< Staged discrete parameters computed block. */
    fast_gt flag_update_pending; /**< Safe inter-boundary data transition indicator flag. */
} ctl_qr_tuner_t;

/**
 * @brief Compiles physical parameters inside the QR tuner into staging digital shadow coefficients.
 * @details Performs background algebraic discretization without affecting the running loop.
 * @param[in,out] tuner Pointer to the background QR tuner companion instance.
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_tune_qr_compile(ctl_qr_tuner_t* tuner, parameter_gt fs);

/**
 * @brief Inputs new physical objectives into the QR tuner workspace and triggers automatic compilation.
 * @param[out] tuner Pointer to the background QR tuner companion instance.
 * @param[in] kr Target quasi-resonant gain.
 * @param[in] freq_resonant Target resonant center frequency (Hz).
 * @param[in] freq_cut Target filter cutoff damping bandwidth (Hz).
 * @param[in] mode Discretization mapping operational strategy flag (Tustin or Prewarped).
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_tune_qr(ctl_qr_tuner_t* tuner, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                 ctl_tune_qr_mode_e mode, parameter_gt fs);

/**
 * @brief Decodes an anonymous active active QR controller back into user-friendly fields to initialize the tuner.
 * @note Designed for plug-and-play tuning tools to mirror existing controller running contexts shock-free.
 * @param[out] tuner Pointer to the background QR tuner companion instance to be backward initialized.
 * @param[in] active_ctrl Pointer to the currently running unmodified execution QR controller.
 * @param[in] mode Discretization mapping strategy under which active_ctrl was historically compiled.
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_init_qr_tuner_from_ctrl(ctl_qr_tuner_t* tuner, const qr_ctrl_t* active_ctrl, ctl_tune_qr_mode_e mode,
                                 parameter_gt fs);

/**
 * @brief Joint initialization routine configuring both the QR controller core and tuner companion simultaneously.
 * @param[out] r Pointer to the running QR controller core instance.
 * @param[out] tuner Pointer to the companion QR tuner manager instance.
 * @param[in] kr Initial quasi-resonant gain.
 * @param[in] freq_resonant Initial resonant center frequency (Hz).
 * @param[in] freq_cut Initial filter cutoff damping bandwidth (Hz).
 * @param[in] mode Discretization mapping operational strategy flag (Tustin or Prewarped).
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_init_tunable_qr_controller(qr_ctrl_t* r, ctl_qr_tuner_t* tuner, parameter_gt kr, parameter_gt freq_resonant,
                                    parameter_gt freq_cut, ctl_tune_qr_mode_e mode, parameter_gt fs);

/**
 * @brief Rigid Shock-Free Coefficient Swapper for Primary Quasi-Resonant Loop.
 * @param[in,out] tuner Pointer to the background quasi-resonant tuner companion instance.
 * @param[out] r Pointer to the running execution quasi-resonant controller instance.
 */
GMP_STATIC_INLINE void ctl_tune_qr_deploy(ctl_qr_tuner_t* tuner, qr_ctrl_t* r)
{
    if (tuner->flag_update_pending)
    {
        r->coef = tuner->shadow_coef; /* Atomic structural assignment block move */
        tuner->flag_update_pending = 0;
    }
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // PROPORTIONAL_RESONANT_TUNER_H
