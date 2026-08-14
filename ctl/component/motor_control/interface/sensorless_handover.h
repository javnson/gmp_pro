/**
 * @file sensorless_handover.h
 * @brief Coordinated sensorless angle handover and startup Id release.
 *
 * @details This application-neutral orchestrator owns no observer.  It combines
 * a bumpless angle/same-frame-speed switcher with a speed-scheduled startup
 * excitation reference.  During source A operation Id first decreases from
 * the forced-start value toward the handover value.  During A-to-B angle
 * transfer the exact same ownership weight blends Id toward its closed-loop
 * value.  Reverse transfer raises Id along the reciprocal path.
 */

#ifndef _FILE_SENSORLESS_HANDOVER_H_
#define _FILE_SENSORLESS_HANDOVER_H_

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/motor_control/interface/encoder_switcher.h>
#include <ctl/component/motor_control/interface/startup_excitation.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _tag_sensorless_handover
{
    ctl_angle_switcher_t angle;
    ctl_startup_excitation_t if_excitation;
    ctrl_gt closed_loop_id_ref;
    ctrl_gt id_ref_out;
} ctl_sensorless_handover_t;

GMP_STATIC_INLINE void ctl_init_sensorless_handover(
    ctl_sensorless_handover_t* handover,
    parameter_gt transition_time_s, parameter_gt isr_frequency_hz,
    ctrl_gt if_startup_id_ref, ctrl_gt if_handover_id_ref,
    ctrl_gt closed_loop_id_ref, ctrl_gt id_fade_start_speed_pu,
    ctrl_gt id_fade_end_speed_pu)
{
    ctl_init_angle_switcher(&handover->angle, transition_time_s, isr_frequency_hz);
    ctl_init_startup_excitation(&handover->if_excitation, if_startup_id_ref,
                                if_handover_id_ref, id_fade_start_speed_pu,
                                id_fade_end_speed_pu);
    handover->closed_loop_id_ref = closed_loop_id_ref;
    handover->id_ref_out = if_startup_id_ref;
}

GMP_STATIC_INLINE void ctl_attach_sensorless_handover(
    ctl_sensorless_handover_t* handover,
    rotation_ift* if_position, velocity_ift* if_speed,
    rotation_ift* closed_position, velocity_ift* closed_speed)
{
    ctl_attach_angle_switcher(&handover->angle, if_position, closed_position);
    ctl_attach_angle_switcher_speed(&handover->angle, if_speed, closed_speed);
}

GMP_STATIC_INLINE void ctl_clear_sensorless_handover(ctl_sensorless_handover_t* handover)
{
    ctl_select_angle_source_immediate(&handover->angle, 0);
    handover->if_excitation.id_ref_out = handover->if_excitation.startup_id_ref;
    handover->if_excitation.closed_loop_weight = CTL_CTRL_CONST_ZERO;
    handover->id_ref_out = handover->if_excitation.startup_id_ref;
}

GMP_STATIC_INLINE void ctl_configure_sensorless_handover_speed_qualification(
    ctl_sensorless_handover_t* handover,
    parameter_gt enter_tolerance_pu, parameter_gt exit_tolerance_pu,
    parameter_gt qualify_time_s, parameter_gt isr_frequency_hz)
{
    ctl_configure_angle_switcher_speed_qualification(
        &handover->angle, enter_tolerance_pu, exit_tolerance_pu,
        qualify_time_s, isr_frequency_hz);
}

GMP_STATIC_INLINE void ctl_request_sensorless_handover(
    ctl_sensorless_handover_t* handover, fast_gt closed_loop_source)
{
    ctl_trigger_angle_transition(&handover->angle, closed_loop_source);
}

GMP_STATIC_INLINE void ctl_cancel_sensorless_handover_request(
    ctl_sensorless_handover_t* handover)
{
    ctl_cancel_angle_transition_request(&handover->angle);
}

/** Bypass qualification, primarily for a confirmed observer-loss fallback. */
GMP_STATIC_INLINE void ctl_force_sensorless_handover(
    ctl_sensorless_handover_t* handover, fast_gt closed_loop_source)
{
    ctl_begin_angle_transition(&handover->angle, closed_loop_source);
}

GMP_STATIC_INLINE ctrl_gt ctl_step_sensorless_handover(
    ctl_sensorless_handover_t* handover, ctrl_gt if_speed_pu)
{
    ctrl_gt if_id_ref = ctl_step_startup_excitation(&handover->if_excitation,
                                                     if_speed_pu);
    ctrl_gt closed_weight;

    (void)ctl_step_angle_switcher(&handover->angle);
    closed_weight = handover->angle.weight;
    handover->id_ref_out = if_id_ref + ctl_mul(
        closed_weight, handover->closed_loop_id_ref - if_id_ref);
    return handover->id_ref_out;
}

GMP_STATIC_INLINE rotation_ift* ctl_get_sensorless_handover_position(
    ctl_sensorless_handover_t* handover)
{
    return &handover->angle.out_enc;
}

GMP_STATIC_INLINE velocity_ift* ctl_get_sensorless_handover_speed(
    ctl_sensorless_handover_t* handover)
{
    return &handover->angle.out_spd;
}

#ifdef __cplusplus
}
#endif

#endif
