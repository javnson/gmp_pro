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

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CTL_RESONANT_TUNER_H
