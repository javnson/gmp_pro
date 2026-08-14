/**
 * @file inv_zero_ctrl.h
 * @brief Tunable QPR zero-sequence current controller for four-wire inverters.
 */

#ifndef _FILE_DP_INV_ZERO_CTRL_H_
#define _FILE_DP_INV_ZERO_CTRL_H_

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/interface/interface_base.h>

#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/discrete/biquad_filter.h>
#include <ctl/component/intrinsic/discrete/proportional_resonant_tuner.h>

#include <ctl/component/digital_power/inv/gfl_core.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Zero-sequence current controller.
 * @details Regulates Clarke i0 in the stationary frame. The QPR resonant center
 * is normally the fundamental output frequency, allowing the controller to
 * reject fundamental zero-sequence current caused by unbalanced loads.
 */
typedef struct _tag_inv_zero_ctrl
{
    ctrl_gt* i0;      //!< Attached zero-sequence current feedback.
    ctrl_gt* v0_sink; //!< Attached zero-sequence modulation-voltage destination.

    ctrl_gt i0_set;
    ctrl_gt i0_fbk;
    ctrl_gt v0_raw;
    ctrl_gt v0_out;

    qpr_ctrl_t qpr;
    ctl_qr_tuner_t tuner;

    ctrl_gt kp_shadow;
    ctrl_gt output_limit_max;
    ctrl_gt output_limit_min;

    fast_gt flag_enable;
    fast_gt flag_tune_pending;
} inv_zero_ctrl_t;

typedef struct _tag_inv_zero_ctrl_init
{
    parameter_gt fs;
    parameter_gt kp;
    parameter_gt kr;
    parameter_gt freq_resonant;
    parameter_gt freq_cut;
    ctl_tune_qr_mode_e tune_mode;
    parameter_gt output_limit_max;
    parameter_gt output_limit_min;
} inv_zero_ctrl_init_t;

void ctl_auto_tuning_zero_inv(inv_zero_ctrl_init_t* zero_init, const gfl_inv_ctrl_init_t* gfl_init);
void ctl_update_zero_inv_coeff(inv_zero_ctrl_t* zero, const inv_zero_ctrl_init_t* init);
void ctl_init_zero_inv(inv_zero_ctrl_t* zero, const inv_zero_ctrl_init_t* init);

/**
 * @brief Compiles new QPR coefficients into a background shadow bank.
 * @details Call from a background task. ctl_step_zero_inv_ctrl() performs the
 * small atomic deployment at the control boundary.
 */
void ctl_tune_zero_inv_ctrl(inv_zero_ctrl_t* zero, parameter_gt kp, parameter_gt kr,
                            parameter_gt freq_resonant, parameter_gt freq_cut,
                            ctl_tune_qr_mode_e mode, parameter_gt fs);

GMP_STATIC_INLINE void ctl_clear_zero_inv(inv_zero_ctrl_t* zero)
{
    ctl_clear_qpr_controller(&zero->qpr);
    zero->i0_fbk = CTL_CTRL_CONST_ZERO;
    zero->v0_raw = CTL_CTRL_CONST_ZERO;
    zero->v0_out = CTL_CTRL_CONST_ZERO;
}

GMP_STATIC_INLINE void ctl_deploy_zero_inv_tuning(inv_zero_ctrl_t* zero)
{
    if (zero->flag_tune_pending)
    {
        zero->qpr.kp = zero->kp_shadow;
        ctl_tune_qr_deploy(&zero->tuner, &zero->qpr.resonant_part);
        zero->flag_tune_pending = 0;
    }
}

GMP_STATIC_INLINE void ctl_step_zero_inv_ctrl(inv_zero_ctrl_t* zero)
{
    gmp_ctl_assert(zero);

    ctl_deploy_zero_inv_tuning(zero);

    if (!zero->flag_enable)
    {
        zero->v0_raw = CTL_CTRL_CONST_ZERO;
        zero->v0_out = CTL_CTRL_CONST_ZERO;
        if (zero->v0_sink != NULL)
            *zero->v0_sink = CTL_CTRL_CONST_ZERO;
        return;
    }

    gmp_ctl_assert(zero->i0);
    gmp_ctl_assert(zero->v0_sink);

    zero->i0_fbk = *zero->i0;
    zero->v0_raw = ctl_step_qpr_controller(&zero->qpr, zero->i0_set - zero->i0_fbk);
    zero->v0_out = ctl_sat(zero->v0_raw, zero->output_limit_max, zero->output_limit_min);
    *zero->v0_sink = zero->v0_out;
}

GMP_STATIC_INLINE void ctl_attach_zero_inv(inv_zero_ctrl_t* zero, ctrl_gt* i0, ctrl_gt* v0_sink)
{
    gmp_ctl_assert(zero);
    zero->i0 = i0;
    zero->v0_sink = v0_sink;
}

/**
 * @brief Attaches i0 feedback and zero-axis voltage injection to a GFL core.
 */
GMP_STATIC_INLINE void ctl_attach_zero_inv_to_gfl(inv_zero_ctrl_t* zero, gfl_inv_ctrl_t* current)
{
    gmp_ctl_assert(current);
    ctl_attach_zero_inv(zero, &current->iab0.dat[phase_0], &current->vab0_ff_external.dat[phase_0]);
}

GMP_STATIC_INLINE void ctl_set_zero_inv_current(inv_zero_ctrl_t* zero, ctrl_gt i0_set)
{
    zero->i0_set = i0_set;
}

GMP_STATIC_INLINE void ctl_enable_zero_inv(inv_zero_ctrl_t* zero)
{
    zero->flag_enable = 1;
}

GMP_STATIC_INLINE void ctl_disable_zero_inv(inv_zero_ctrl_t* zero)
{
    zero->flag_enable = 0;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_DP_INV_ZERO_CTRL_H_
