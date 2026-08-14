/**
 * @file encoder_switcher.h
 * @brief Bumpless electrical-angle and same-frame speed source handover.
 *
 * @details The switcher captures the phase offset between its current output
 * and a newly selected live source.  During transfer that offset is ramped to
 * zero while the selected source continues to move.  This avoids the artificial
 * angular-velocity pulse caused by directly interpolating two moving angles.
 *
 * All angles are turns-per-unit: 1 pu = 360 degrees = 2*pi rad.  If velocity
 * interfaces are attached, both speeds must describe the same rotating frame
 * and use the same PU base.  In particular, an ACIM synchronous rotor-flux
 * speed must never be routed to the mechanical rotor-speed loop.
 */

#ifndef _FILE_ENCODER_SWITCHER_H_
#define _FILE_ENCODER_SWITCHER_H_

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/motor_control/interface/motor_universal_interface.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum _tag_angle_switch_state
{
    ANGLE_SWITCH_IDLE_A = 0,
    ANGLE_SWITCH_IDLE_B,
    ANGLE_SWITCH_TRANS_A2B,
    ANGLE_SWITCH_TRANS_B2A
} angle_switch_state_e;

typedef struct _tag_angle_switcher
{
    rotation_ift* src_a;
    rotation_ift* src_b;
    velocity_ift* spd_a;
    velocity_ift* spd_b;

    rotation_ift out_enc;
    velocity_ift out_spd;

    angle_switch_state_e state;
    ctrl_gt weight;              //!< Source-B ownership: 0 = A, 1 = B.
    ctrl_gt weight_step;         //!< Transfer progress per ISR tick.
    ctrl_gt transition_progress; //!< Current transfer progress, [0, 1].
    ctrl_gt angle_offset_pu;     //!< Captured output minus target angle, shortest path.
    ctrl_gt transition_speed_start;

    ctrl_gt speed_match_enter_pu; //!< Enter tolerance for |speed A - speed B|.
    ctrl_gt speed_match_exit_pu;  //!< Larger exit tolerance providing hysteresis.
    uint32_t qualify_ticks_required;
    uint32_t qualify_ticks;
    fast_gt flag_speed_qualification;
    fast_gt requested_source_b;
    fast_gt request_pending;
} ctl_angle_switcher_t;

GMP_STATIC_INLINE ctrl_gt ctl_helper_wrap_angle_pu(ctrl_gt angle)
{
    ctrl_gt result = ctl_mod_1(angle);
    if (result < float2ctrl(0.0f))
        result += float2ctrl(1.0f);
    return result;
}

GMP_STATIC_INLINE ctrl_gt ctl_helper_diff_angle_pu(ctrl_gt target, ctrl_gt current)
{
    ctrl_gt difference = ctl_mod_1(target - current);
    if (difference >= float2ctrl(0.5f))
        difference -= float2ctrl(1.0f);
    else if (difference < float2ctrl(-0.5f))
        difference += float2ctrl(1.0f);
    return difference;
}

GMP_STATIC_INLINE void ctl_set_angle_switcher_duration(ctl_angle_switcher_t* ctx,
                                                        parameter_gt transition_time_s,
                                                        parameter_gt isr_frequency_hz)
{
    parameter_gt ticks = transition_time_s * isr_frequency_hz;
    ctx->weight_step = float2ctrl((ticks > 1.0f) ? (1.0f / ticks) : 1.0f);
}

void ctl_init_angle_switcher(ctl_angle_switcher_t* ctx, parameter_gt transition_time_s,
                             parameter_gt isr_frequency_hz);

/**
 * @brief Enables noise-tolerant speed matching before a requested handover.
 * @param enter_tolerance_pu Speed mismatch below which qualification accumulates.
 * @param exit_tolerance_pu Speed mismatch above which qualification resets.
 * @param qualify_time_s Required continuous/held qualification duration.
 * @note The two speed ports must have identical physical meaning and PU base.
 */
GMP_STATIC_INLINE void ctl_configure_angle_switcher_speed_qualification(
    ctl_angle_switcher_t* ctx, parameter_gt enter_tolerance_pu,
    parameter_gt exit_tolerance_pu, parameter_gt qualify_time_s,
    parameter_gt isr_frequency_hz)
{
    parameter_gt enter_abs = (enter_tolerance_pu >= 0.0f) ? enter_tolerance_pu : -enter_tolerance_pu;
    parameter_gt exit_abs = (exit_tolerance_pu >= 0.0f) ? exit_tolerance_pu : -exit_tolerance_pu;
    parameter_gt ticks = qualify_time_s * isr_frequency_hz;

    if (exit_abs < enter_abs)
        exit_abs = enter_abs;
    ctx->speed_match_enter_pu = float2ctrl(enter_abs);
    ctx->speed_match_exit_pu = float2ctrl(exit_abs);
    ctx->qualify_ticks_required = (ticks > 1.0f) ? (uint32_t)ticks : 1U;
    ctx->qualify_ticks = 0U;
    ctx->flag_speed_qualification = 1;
}

GMP_STATIC_INLINE void ctl_disable_angle_switcher_speed_qualification(ctl_angle_switcher_t* ctx)
{
    ctx->flag_speed_qualification = 0;
    ctx->qualify_ticks = 0U;
}

GMP_STATIC_INLINE void ctl_attach_angle_switcher(ctl_angle_switcher_t* ctx,
                                                  rotation_ift* source_a,
                                                  rotation_ift* source_b)
{
    ctx->src_a = source_a;
    ctx->src_b = source_b;
    if (source_a != NULL)
        ctx->out_enc = *source_a;
}

/** Attach optional speeds belonging to the exact same frame as each angle. */
GMP_STATIC_INLINE void ctl_attach_angle_switcher_speed(ctl_angle_switcher_t* ctx,
                                                       velocity_ift* speed_a,
                                                       velocity_ift* speed_b)
{
    ctx->spd_a = speed_a;
    ctx->spd_b = speed_b;
    if (speed_a != NULL)
        ctx->out_spd = *speed_a;
}

/** Immediately selects one source; intended for initialization and reset only. */
GMP_STATIC_INLINE void ctl_select_angle_source_immediate(ctl_angle_switcher_t* ctx,
                                                         fast_gt select_source_b)
{
    rotation_ift* position = select_source_b ? ctx->src_b : ctx->src_a;
    velocity_ift* speed = select_source_b ? ctx->spd_b : ctx->spd_a;

    ctx->state = select_source_b ? ANGLE_SWITCH_IDLE_B : ANGLE_SWITCH_IDLE_A;
    ctx->weight = select_source_b ? float2ctrl(1.0f) : float2ctrl(0.0f);
    ctx->transition_progress = float2ctrl(1.0f);
    ctx->angle_offset_pu = float2ctrl(0.0f);
    ctx->request_pending = 0;
    ctx->qualify_ticks = 0U;
    if (position != NULL)
        ctx->out_enc = *position;
    if (speed != NULL)
        ctx->out_spd = *speed;
}

/**
 * @brief Requests a bumpless handover.
 * @param target_source_b Nonzero selects B; zero selects A.
 * @details With speed qualification enabled, the request remains pending until
 * the two speed sources satisfy the configured tolerance, hysteresis and
 * debounce contract. Without qualification, transfer starts immediately.
 */
GMP_STATIC_INLINE void ctl_begin_angle_transition(ctl_angle_switcher_t* ctx,
                                                   fast_gt target_source_b)
{
    rotation_ift* target_position = target_source_b ? ctx->src_b : ctx->src_a;
    if (target_position == NULL)
        return;

    if ((target_source_b && ctx->state == ANGLE_SWITCH_IDLE_B) ||
        (!target_source_b && ctx->state == ANGLE_SWITCH_IDLE_A))
        return;

    ctx->angle_offset_pu = ctl_helper_diff_angle_pu(ctx->out_enc.elec_position,
                                                    target_position->elec_position);
    ctx->transition_speed_start = ctx->out_spd.speed;
    ctx->transition_progress = float2ctrl(0.0f);
    ctx->state = target_source_b ? ANGLE_SWITCH_TRANS_A2B : ANGLE_SWITCH_TRANS_B2A;
    ctx->request_pending = 0;
    ctx->qualify_ticks = 0U;
}

GMP_STATIC_INLINE void ctl_trigger_angle_transition(ctl_angle_switcher_t* ctx,
                                                     fast_gt target_source_b)
{
    if ((target_source_b && ctx->state == ANGLE_SWITCH_IDLE_B) ||
        (!target_source_b && ctx->state == ANGLE_SWITCH_IDLE_A))
    {
        ctx->request_pending = 0;
        ctx->qualify_ticks = 0U;
        return;
    }

    ctx->requested_source_b = target_source_b ? 1 : 0;
    ctx->request_pending = 1;
    ctx->qualify_ticks = 0U;
    if (!ctx->flag_speed_qualification)
        ctl_begin_angle_transition(ctx, ctx->requested_source_b);
}

/** Cancel a not-yet-started qualified request; an active transfer is unchanged. */
GMP_STATIC_INLINE void ctl_cancel_angle_transition_request(ctl_angle_switcher_t* ctx)
{
    if ((ctx->state == ANGLE_SWITCH_IDLE_A) || (ctx->state == ANGLE_SWITCH_IDLE_B))
    {
        ctx->request_pending = 0;
        ctx->qualify_ticks = 0U;
    }
}

GMP_STATIC_INLINE ctrl_gt ctl_step_angle_switcher(ctl_angle_switcher_t* ctx)
{
    rotation_ift* target_position;
    velocity_ift* target_speed;
    fast_gt target_is_b;

    gmp_ctl_assert(ctx != NULL);
    gmp_ctl_assert(ctx->src_a != NULL);
    gmp_ctl_assert(ctx->src_b != NULL);

    if (ctx->request_pending)
    {
        if (!ctx->flag_speed_qualification)
        {
            ctl_begin_angle_transition(ctx, ctx->requested_source_b);
        }
        else if ((ctx->spd_a != NULL) && (ctx->spd_b != NULL))
        {
            ctrl_gt speed_error = ctl_abs(ctx->spd_a->speed - ctx->spd_b->speed);
            if (speed_error <= ctx->speed_match_enter_pu)
            {
                if (ctx->qualify_ticks < ctx->qualify_ticks_required)
                    ++ctx->qualify_ticks;
            }
            else if (speed_error >= ctx->speed_match_exit_pu)
            {
                ctx->qualify_ticks = 0U;
            }

            if (ctx->qualify_ticks >= ctx->qualify_ticks_required)
                ctl_begin_angle_transition(ctx, ctx->requested_source_b);
        }
    }

    if (ctx->state == ANGLE_SWITCH_IDLE_A || ctx->state == ANGLE_SWITCH_IDLE_B)
    {
        target_is_b = (ctx->state == ANGLE_SWITCH_IDLE_B);
        target_position = target_is_b ? ctx->src_b : ctx->src_a;
        target_speed = target_is_b ? ctx->spd_b : ctx->spd_a;
        ctx->out_enc = *target_position;
        if (target_speed != NULL)
            ctx->out_spd = *target_speed;
        ctx->weight = target_is_b ? float2ctrl(1.0f) : float2ctrl(0.0f);
        return ctx->out_enc.elec_position;
    }

    target_is_b = (ctx->state == ANGLE_SWITCH_TRANS_A2B);
    target_position = target_is_b ? ctx->src_b : ctx->src_a;
    target_speed = target_is_b ? ctx->spd_b : ctx->spd_a;

    ctx->transition_progress += ctx->weight_step;
    if (ctx->transition_progress > float2ctrl(1.0f))
        ctx->transition_progress = float2ctrl(1.0f);

    {
        ctrl_gt remaining = float2ctrl(1.0f) - ctx->transition_progress;
        ctx->out_enc.elec_position = ctl_helper_wrap_angle_pu(
            target_position->elec_position + ctl_mul(remaining, ctx->angle_offset_pu));
        ctx->weight = target_is_b ? ctx->transition_progress : remaining;

        if (target_speed != NULL)
            ctx->out_spd.speed = ctl_mul(remaining, ctx->transition_speed_start) +
                                 ctl_mul(ctx->transition_progress, target_speed->speed);
    }

    if (ctx->weight < float2ctrl(0.5f))
    {
        ctx->out_enc.position = ctx->src_a->position;
        ctx->out_enc.revolutions = ctx->src_a->revolutions;
    }
    else
    {
        ctx->out_enc.position = ctx->src_b->position;
        ctx->out_enc.revolutions = ctx->src_b->revolutions;
    }

    if (ctx->transition_progress >= float2ctrl(1.0f))
    {
        ctx->state = target_is_b ? ANGLE_SWITCH_IDLE_B : ANGLE_SWITCH_IDLE_A;
        ctx->angle_offset_pu = float2ctrl(0.0f);
        ctx->out_enc = *target_position;
        if (target_speed != NULL)
            ctx->out_spd = *target_speed;
    }

    return ctx->out_enc.elec_position;
}

#ifdef __cplusplus
}
#endif

#endif
