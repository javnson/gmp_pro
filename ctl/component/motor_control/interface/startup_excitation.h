/**
 * @file startup_excitation.h
 * @brief Speed-scheduled startup excitation current reference.
 *
 * @details This module only schedules a d-axis current command.  Observer
 * qualification and angle ownership remain separate responsibilities.  It is
 * suitable for reducing a deliberately high I/F startup excitation toward a
 * normal closed-loop value as the machine enters an observable speed region.
 *
 * For PMSM the run value may be zero or supplied by an MTPA policy.  For ACIM
 * it must normally remain nonzero because rotor flux requires magnetizing
 * current even after load current has risen.
 */

#ifndef _FILE_STARTUP_EXCITATION_H_
#define _FILE_STARTUP_EXCITATION_H_

#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _tag_startup_excitation
{
    ctrl_gt startup_id_ref;
    ctrl_gt closed_loop_id_ref;
    ctrl_gt fade_start_speed_pu;
    ctrl_gt fade_end_speed_pu;
    ctrl_gt id_ref_out;
    ctrl_gt closed_loop_weight;
} ctl_startup_excitation_t;

GMP_STATIC_INLINE void ctl_init_startup_excitation(ctl_startup_excitation_t* scheduler,
                                                   ctrl_gt startup_id_ref,
                                                   ctrl_gt closed_loop_id_ref,
                                                   ctrl_gt fade_start_speed_pu,
                                                   ctrl_gt fade_end_speed_pu)
{
    scheduler->startup_id_ref = startup_id_ref;
    scheduler->closed_loop_id_ref = closed_loop_id_ref;
    scheduler->fade_start_speed_pu = ctl_abs(fade_start_speed_pu);
    scheduler->fade_end_speed_pu = ctl_abs(fade_end_speed_pu);
    scheduler->id_ref_out = startup_id_ref;
    scheduler->closed_loop_weight = float2ctrl(0.0f);
}

GMP_STATIC_INLINE ctrl_gt ctl_step_startup_excitation(ctl_startup_excitation_t* scheduler,
                                                       ctrl_gt speed_pu)
{
    ctrl_gt absolute_speed = ctl_abs(speed_pu);
    ctrl_gt speed_span = scheduler->fade_end_speed_pu - scheduler->fade_start_speed_pu;

    if (absolute_speed <= scheduler->fade_start_speed_pu)
        scheduler->closed_loop_weight = float2ctrl(0.0f);
    else if ((absolute_speed >= scheduler->fade_end_speed_pu) ||
             (speed_span <= float2ctrl(0.0f)))
        scheduler->closed_loop_weight = float2ctrl(1.0f);
    else
        scheduler->closed_loop_weight = ctl_div(
            absolute_speed - scheduler->fade_start_speed_pu, speed_span);

    scheduler->id_ref_out = scheduler->startup_id_ref +
        ctl_mul(scheduler->closed_loop_weight,
                scheduler->closed_loop_id_ref - scheduler->startup_id_ref);
    return scheduler->id_ref_out;
}

#ifdef __cplusplus
}
#endif

#endif
