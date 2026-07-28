/**
 * @file dq_pi.h
 * @brief Decoupled d-q PI control with feedforward and vector limiting.
 * @version 0.1
 * @date 2026-07-28
 */

#ifndef _FILE_CTL_DQ_PI_H_
#define _FILE_CTL_DQ_PI_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/math_block/vector_lite/vector2.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup dq_pi_controller Decoupled D-Q PI Controller
 * @ingroup CTL_INTRINSIC_COMPLEX
 * @brief Two independent PI axes with optional feedforward, circular limiting,
 * rectangular limiting, and external-limit anti-windup.
 * @{
 */

typedef struct _tag_ctl_dq_pi_t
{
    ctl_pid_t axis[2]; //!< Independent d-axis and q-axis PI controllers.

    ctl_vector2_t error;    //!< Current reference tracking error.
    ctl_vector2_t ctrl_out; //!< PI output before feedforward and vector limiting.
    ctl_vector2_t ff_out;   //!< Feedforward selected by the enable switch.
    ctl_vector2_t out;      //!< Final limited d-q output.

    ctrl_gt circle_limit_sq;      //!< Squared radius used by the circular limiter.
    ctl_vector2_t rect_limit_max; //!< Per-axis rectangular upper limits.
    ctl_vector2_t rect_limit_min; //!< Per-axis rectangular lower limits.

    fast_gt flag_enable_feedforward; //!< Enables addition of the feedforward vector.
    fast_gt flag_enable_circle_limit; //!< Enables circular limiting.
    fast_gt flag_enable_rect_limit;   //!< Enables rectangular limiting.
} ctl_dq_pi_t;

/**
 * @brief Initializes a d-q PI controller in parallel form.
 * @details Feedforward and circular limiting default to disabled. Rectangular
 * limiting defaults to enabled with +/-1 limits on both axes.
 */
void ctl_init_dq_pi(ctl_dq_pi_t* dq, parameter_gt kp_d, parameter_gt ki_d, parameter_gt kp_q,
                    parameter_gt ki_q, parameter_gt fs);

/**
 * @brief Clears controller states and intermediate outputs without changing parameters.
 */
GMP_STATIC_INLINE void ctl_clear_dq_pi(ctl_dq_pi_t* dq)
{
    ctl_clear_pid(&dq->axis[0]);
    ctl_clear_pid(&dq->axis[1]);
    ctl_vector2_clear(&dq->error);
    ctl_vector2_clear(&dq->ctrl_out);
    ctl_vector2_clear(&dq->ff_out);
    ctl_vector2_clear(&dq->out);
}

/**
 * @brief Executes one d-q PI control step.
 * @details Processing order is PI control, optional feedforward, optional circular
 * limiting, optional rectangular limiting, and integrator back-calculation.
 * `output` may be NULL when the caller reads `dq->out` directly. `feedforward`
 * may be NULL only while feedforward is disabled.
 */
GMP_STATIC_INLINE void ctl_step_dq_pi(ctl_dq_pi_t* dq, const ctl_vector2_t* target,
                                      const ctl_vector2_t* feedback, const ctl_vector2_t* feedforward,
                                      ctl_vector2_t* output)
{
    ctl_vector2_t command;
    uint_fast8_t axis;

    gmp_base_assert(dq);
    gmp_base_assert(target);
    gmp_base_assert(feedback);

    for (axis = 0; axis < 2; ++axis)
    {
        dq->error.dat[axis] = target->dat[axis] - feedback->dat[axis];
        dq->ctrl_out.dat[axis] = ctl_step_pid_par_raw(&dq->axis[axis], dq->error.dat[axis]);
    }

    if (dq->flag_enable_feedforward)
    {
        gmp_base_assert(feedforward);
        ctl_vector2_copy(&dq->ff_out, feedforward);
    }
    else
    {
        ctl_vector2_clear(&dq->ff_out);
    }
    ctl_vector2_add(&command, &dq->ctrl_out, &dq->ff_out);

    if (dq->flag_enable_circle_limit)
        ctl_vector2_sat_circle_sq(&command, &command, dq->circle_limit_sq);

    if (dq->flag_enable_rect_limit)
        ctl_vector2_sat_rect(&command, &command, &dq->rect_limit_max, &dq->rect_limit_min);

    ctl_vector2_copy(&dq->out, &command);

    for (axis = 0; axis < 2; ++axis)
    {
        ctrl_gt applied_ctrl = dq->out.dat[axis] - dq->ff_out.dat[axis];
        ctl_pid_clamping_correction_using_real_output(&dq->axis[axis], applied_ctrl);
        dq->axis[axis].out = applied_ctrl;
    }

    if (output)
        ctl_vector2_copy(output, &dq->out);
}

GMP_STATIC_INLINE void ctl_set_dq_pi_circle_limit(ctl_dq_pi_t* dq, ctrl_gt radius)
{
    gmp_base_assert(radius >= float2ctrl(0.0f));
    dq->circle_limit_sq = ctl_mul(radius, radius);
}

GMP_STATIC_INLINE void ctl_set_dq_pi_circle_limit_sq(ctl_dq_pi_t* dq, ctrl_gt radius_sq)
{
    gmp_base_assert(radius_sq >= float2ctrl(0.0f));
    dq->circle_limit_sq = radius_sq;
}

GMP_STATIC_INLINE void ctl_set_dq_pi_rect_limit(ctl_dq_pi_t* dq, const ctl_vector2_t* limit_max,
                                                const ctl_vector2_t* limit_min)
{
    gmp_base_assert(limit_max);
    gmp_base_assert(limit_min);
    gmp_base_assert(limit_max->dat[0] >= limit_min->dat[0]);
    gmp_base_assert(limit_max->dat[1] >= limit_min->dat[1]);
    ctl_vector2_copy(&dq->rect_limit_max, limit_max);
    ctl_vector2_copy(&dq->rect_limit_min, limit_min);
}

GMP_STATIC_INLINE void ctl_enable_dq_pi_feedforward(ctl_dq_pi_t* dq)
{
    dq->flag_enable_feedforward = 1;
}
GMP_STATIC_INLINE void ctl_disable_dq_pi_feedforward(ctl_dq_pi_t* dq)
{
    dq->flag_enable_feedforward = 0;
}
GMP_STATIC_INLINE void ctl_enable_dq_pi_circle_limit(ctl_dq_pi_t* dq)
{
    dq->flag_enable_circle_limit = 1;
}
GMP_STATIC_INLINE void ctl_disable_dq_pi_circle_limit(ctl_dq_pi_t* dq)
{
    dq->flag_enable_circle_limit = 0;
}
GMP_STATIC_INLINE void ctl_enable_dq_pi_rect_limit(ctl_dq_pi_t* dq)
{
    dq->flag_enable_rect_limit = 1;
}
GMP_STATIC_INLINE void ctl_disable_dq_pi_rect_limit(ctl_dq_pi_t* dq)
{
    dq->flag_enable_rect_limit = 0;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_DQ_PI_H_
