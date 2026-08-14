/**
 * @file dq_ladrc1.h
 * @brief Decoupled d-q LADRC1 control with feedforward and vector limiting.
 * @version 0.1
 * @date 2026-07-28
 */

#ifndef _FILE_CTL_DQ_LADRC1_H_
#define _FILE_CTL_DQ_LADRC1_H_

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/intrinsic/continuous/ladrc1.h>
#include <ctl/math_block/vector_lite/vector2.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup dq_ladrc1_controller Decoupled D-Q LADRC1 Controller
 * @ingroup CTL_INTRINSIC_COMPLEX
 * @brief Two independent LADRC1 axes with optional feedforward, circular limiting,
 * rectangular limiting, and applied-command observer feedback.
 * @{
 */

typedef struct _tag_ctl_dq_ladrc1_t
{
    ctl_ladrc1_t axis[2]; //!< Independent d-axis and q-axis LADRC controllers.

    ctl_vector2_t ctrl_out; //!< LADRC output before feedforward and vector limiting.
    ctl_vector2_t ff_out;   //!< Feedforward selected by the enable switch.
    ctl_vector2_t out;      //!< Final limited d-q output.

    ctrl_gt circle_limit_sq;      //!< Squared radius used by the circular limiter.
    ctl_vector2_t rect_limit_max; //!< Per-axis rectangular upper limits.
    ctl_vector2_t rect_limit_min; //!< Per-axis rectangular lower limits.

    fast_gt flag_enable_feedforward; //!< Enables addition of the feedforward vector.
    fast_gt flag_enable_circle_limit; //!< Enables circular limiting.
    fast_gt flag_enable_rect_limit;   //!< Enables rectangular limiting.
} ctl_dq_ladrc1_t;

/**
 * @brief Initializes a d-q first-order LADRC controller.
 * @details Feedforward and circular limiting default to disabled. Rectangular
 * limiting defaults to enabled with +/-1 limits on both axes.
 */
void ctl_init_dq_ladrc1(ctl_dq_ladrc1_t* dq, parameter_gt b0_d, parameter_gt fc_d, parameter_gt fo_d,
                        parameter_gt b0_q, parameter_gt fc_q, parameter_gt fo_q, parameter_gt fs);

/**
 * @brief Clears observer states and intermediate outputs without changing parameters.
 */
GMP_STATIC_INLINE void ctl_clear_dq_ladrc1(ctl_dq_ladrc1_t* dq)
{
    ctl_clear_ladrc1(&dq->axis[0]);
    ctl_clear_ladrc1(&dq->axis[1]);
    ctl_vector2_clear(&dq->ctrl_out);
    ctl_vector2_clear(&dq->ff_out);
    ctl_vector2_clear(&dq->out);
}

/**
 * @brief Executes one d-q LADRC control step.
 * @details Processing order is LADRC control, optional feedforward, optional
 * circular limiting, optional rectangular limiting, and observer input
 * back-calculation. The observer receives the complete command actually applied
 * to the plant, including feedforward.
 */
GMP_STATIC_INLINE void ctl_step_dq_ladrc1(ctl_dq_ladrc1_t* dq, const ctl_vector2_t* target,
                                          const ctl_vector2_t* feedback,
                                          const ctl_vector2_t* feedforward, ctl_vector2_t* output)
{
    ctl_vector2_t command;
    uint_fast8_t axis;

    gmp_ctl_assert(dq);
    gmp_ctl_assert(target);
    gmp_ctl_assert(feedback);

    for (axis = 0; axis < 2; ++axis)
        dq->ctrl_out.dat[axis] = ctl_step_ladrc1_raw(&dq->axis[axis], target->dat[axis], feedback->dat[axis]);

    if (dq->flag_enable_feedforward)
    {
        gmp_ctl_assert(feedforward);
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
        dq->axis[axis].out = dq->out.dat[axis];
        dq->axis[axis].u_prev = dq->out.dat[axis];
    }

    if (output)
        ctl_vector2_copy(output, &dq->out);
}

GMP_STATIC_INLINE void ctl_set_dq_ladrc1_circle_limit(ctl_dq_ladrc1_t* dq, ctrl_gt radius)
{
    gmp_ctl_assert(radius >= float2ctrl(0.0f));
    dq->circle_limit_sq = ctl_mul(radius, radius);
}

GMP_STATIC_INLINE void ctl_set_dq_ladrc1_circle_limit_sq(ctl_dq_ladrc1_t* dq, ctrl_gt radius_sq)
{
    gmp_ctl_assert(radius_sq >= float2ctrl(0.0f));
    dq->circle_limit_sq = radius_sq;
}

GMP_STATIC_INLINE void ctl_set_dq_ladrc1_rect_limit(ctl_dq_ladrc1_t* dq, const ctl_vector2_t* limit_max,
                                                    const ctl_vector2_t* limit_min)
{
    gmp_ctl_assert(limit_max);
    gmp_ctl_assert(limit_min);
    gmp_ctl_assert(limit_max->dat[0] >= limit_min->dat[0]);
    gmp_ctl_assert(limit_max->dat[1] >= limit_min->dat[1]);
    ctl_vector2_copy(&dq->rect_limit_max, limit_max);
    ctl_vector2_copy(&dq->rect_limit_min, limit_min);
}

GMP_STATIC_INLINE void ctl_enable_dq_ladrc1_feedforward(ctl_dq_ladrc1_t* dq)
{
    dq->flag_enable_feedforward = 1;
}
GMP_STATIC_INLINE void ctl_disable_dq_ladrc1_feedforward(ctl_dq_ladrc1_t* dq)
{
    dq->flag_enable_feedforward = 0;
}
GMP_STATIC_INLINE void ctl_enable_dq_ladrc1_circle_limit(ctl_dq_ladrc1_t* dq)
{
    dq->flag_enable_circle_limit = 1;
}
GMP_STATIC_INLINE void ctl_disable_dq_ladrc1_circle_limit(ctl_dq_ladrc1_t* dq)
{
    dq->flag_enable_circle_limit = 0;
}
GMP_STATIC_INLINE void ctl_enable_dq_ladrc1_rect_limit(ctl_dq_ladrc1_t* dq)
{
    dq->flag_enable_rect_limit = 1;
}
GMP_STATIC_INLINE void ctl_disable_dq_ladrc1_rect_limit(ctl_dq_ladrc1_t* dq)
{
    dq->flag_enable_rect_limit = 0;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_DQ_LADRC1_H_
