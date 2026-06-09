/**
 * @file ctl_resonant_tuner.h
 * @brief Non-Intrusive Runtime Tuning Plugin with Normalized Structural Mapping.
 */

#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

#ifndef CTL_RESONANT_TUNER_H
#define CTL_RESONANT_TUNER_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Pure Resonant (R) Tuner Component                                         */
/*---------------------------------------------------------------------------*/

/**
 * @brief Standardized External Tuning Manager Workspace (Tuner Component).
 */
typedef struct _tag_ctl_resonant_tuner
{
    /* Flat User Target Workspace */
    parameter_gt target_kr;            /**< Target resonant gain command. */
    parameter_gt target_freq_resonant; /**< Target resonant frequency command (Hz). */

    /* Synchronization Bridge Layer */
    ctl_resonant_coef_t shadow_coef; /**< Pre-calculated discrete parameters ready for the ISR. */
    fast_gt flag_update_pending;     /**< Inter-boundary trigger flag: Non-zero when shadow is ready. */
} ctl_resonant_tuner_t;

/*---------------------------------------------------------------------------*/
/* Standardized Tuner Architecture API Matrix                                */
/*---------------------------------------------------------------------------*/

void ctl_tune_resonant_compile(ctl_resonant_tuner_t* tuner, parameter_gt fs);
void ctl_tune_resonant(ctl_resonant_tuner_t* tuner, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs);

/**
 * @brief Standardized API to initialize the tuner backward from an active running controller.
 */
void ctl_init_resonant_tuner_from_ctrl(ctl_resonant_tuner_t* tuner, const resonant_ctrl_t* active_ctrl,
                                       parameter_gt fs);

/**
 * @brief Joint initializer routine to configure both entity blocks simultaneously.
 */
void ctl_init_tunable_resonant_controller(resonant_ctrl_t* r, ctl_resonant_tuner_t* tuner, parameter_gt kr,
                                          parameter_gt freq_resonant, parameter_gt fs);

/**
 * @brief Rigid Shock-Free Structural Parameter Deployment Agent.
 */
GMP_STATIC_INLINE void ctl_tune_resonant_deploy(ctl_resonant_tuner_t* tuner, resonant_ctrl_t* r)
{
    if (tuner->flag_update_pending)
    {
        /* Unified structural assignment copy: transfers all coefficients atomically in 1 block execution */
        r->coef = tuner->shadow_coef;
        tuner->flag_update_pending = 0;
    }
}

/*---------------------------------------------------------------------------*/
/* Quasi-Resonant (QR) Tuner Component                                       */
/*---------------------------------------------------------------------------*/

/**
 * @brief Discretization method mode for QR Controller tuning.
 */
typedef enum
{
    CTL_TUNE_QR_TUSTIN = 0, /**< Standard Bilinear Tustin transformation (K = 2*Fs). */
    CTL_TUNE_QR_PREWARPED   /**< Tustin transformation with center frequency pre-warping. */
} ctl_tune_qr_mode_e;

/**
 * @brief Standardized External Tuning Manager Workspace for Quasi-Resonant (QR) Controller.
 */
typedef struct _tag_ctl_qr_tuner
{
    /* Flat User Target Workspace */
    parameter_gt target_kr;            /**< Target quasi-resonant gain command. */
    parameter_gt target_freq_resonant; /**< Target resonant center frequency command (Hz). */
    parameter_gt target_freq_cut;      /**< Target filter cutoff frequency/bandwidth command (Hz). */

    ctl_tune_qr_mode_e method_mode; /**< Mapping operator mode selector (Standard Tustin or Prewarped). */

    /* Synchronization Bridge Layer */
    ctl_qr_coef_t shadow_coef;   /**< Pre-calculated discrete parameters ready for the ISR. */
    fast_gt flag_update_pending; /**< Inter-boundary trigger flag: Non-zero when shadow is ready. */
} ctl_qr_tuner_t;

/**
 * @brief Compiles the continuous physical targets inside the QR tuner into digital shadow coefficients.
 * @param[in,out] tuner Pointer to the background QR tuner companion instance.
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_tune_qr_compile(ctl_qr_tuner_t* tuner, parameter_gt fs);

/**
 * @brief Inputs new QR tuning parameters into the workspace and automatically triggers compilation.
 * @param[out] tuner Pointer to the background QR tuner companion instance.
 * @param[in] kr Target quasi-resonant gain.
 * @param[in] freq_resonant Target center resonant frequency (Hz).
 * @param[in] freq_cut Target filter cutoff bandwidth frequency (Hz).
 * @param[in] mode Discretization mapping strategy (Tustin or Prewarped).
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_tune_qr(ctl_qr_tuner_t* tuner, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                 ctl_tune_qr_mode_e mode, parameter_gt fs);

/**
 * @brief Standardized API to initialize the QR tuner backward from an active running QR controller.
 * @param[out] tuner Pointer to the background QR tuner companion instance to be initialized.
 * @param[in] active_ctrl Pointer to the currently running unmodified execution QR controller.
 * @param[in] mode Discretization strategy under which active_ctrl was compiled.
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_init_qr_tuner_from_ctrl(ctl_qr_tuner_t* tuner, const qr_ctrl_t* active_ctrl, ctl_tune_qr_mode_e mode,
                                 parameter_gt fs);

/**
 * @brief Joint initializer routine to configure both the QR controller core and tuner companion simultaneously.
 * @param[out] r Pointer to the running QR controller instance.
 * @param[out] tuner Pointer to the associated QR tuner companion instance.
 * @param[in] kr Initial resonant gain.
 * @param[in] freq_resonant Initial resonant frequency center (Hz).
 * @param[in] freq_cut Initial filter cutoff frequency (Hz).
 * @param[in] mode Discretization mapping strategy (Tustin or Prewarped).
 * @param[in] fs Control loop execution sampling frequency (Hz).
 */
void ctl_init_tunable_qr_controller(qr_ctrl_t* r, ctl_qr_tuner_t* tuner, parameter_gt kr, parameter_gt freq_resonant,
                                    parameter_gt freq_cut, ctl_tune_qr_mode_e mode, parameter_gt fs);

/**
 * @brief Rigid Shock-Free Structural Parameter Deployment Agent for QR Controller.
 * @param[in,out] tuner Pointer to the background QR tuner companion instance.
 * @param[out] r Pointer to the running QR controller instance to receive parameters.
 */
GMP_STATIC_INLINE void ctl_tune_qr_deploy(ctl_qr_tuner_t* tuner, qr_ctrl_t* r)
{
    if (tuner->flag_update_pending)
    {
        r->coef = tuner->shadow_coef; /* Atomic structural assignment block copy */
        tuner->flag_update_pending = 0;
    }
}


#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CTL_RESONANT_TUNER_H
