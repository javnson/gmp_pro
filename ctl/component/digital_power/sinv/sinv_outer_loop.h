/**
 * @file sinv_outer_loop.h
 * @brief Cascaded active-power and DC-link voltage controllers for a single-phase converter.
 */
#ifndef _CTL_SINV_OUTER_LOOP_H_
#define _CTL_SINV_OUTER_LOOP_H_

#include <ctl/component/intrinsic/continuous/continuous_pi.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _tag_sinv_outer_loop {
    ctl_pi_t power_pi;
    ctl_pi_t dc_bus_pi;
    ctrl_gt active_power_cmd;
    ctrl_gt dc_bus_ref;
    ctrl_gt output_limit;
    uint32_t execution_divider;
    uint32_t execution_tick;
} ctl_sinv_outer_loop_t;

GMP_STATIC_INLINE void ctl_init_sinv_outer_loop(ctl_sinv_outer_loop_t* loop,
    parameter_gt power_kp, parameter_gt power_ki,
    parameter_gt bus_kp, parameter_gt bus_ki,
    parameter_gt execution_frequency, parameter_gt controller_frequency,
    parameter_gt output_limit)
{
    ctl_init_pi(&loop->power_pi, power_kp, power_ki, execution_frequency);
    ctl_init_pi(&loop->dc_bus_pi, bus_kp, bus_ki, execution_frequency);
    loop->output_limit = float2ctrl(output_limit);
    ctl_set_pi_limit(&loop->power_pi, loop->output_limit, -loop->output_limit);
    ctl_set_pi_int_limit(&loop->power_pi, loop->output_limit, -loop->output_limit);
    ctl_set_pi_limit(&loop->dc_bus_pi, loop->output_limit, -loop->output_limit);
    ctl_set_pi_int_limit(&loop->dc_bus_pi, loop->output_limit, -loop->output_limit);
    loop->execution_divider = (uint32_t)(controller_frequency / execution_frequency);
    if (loop->execution_divider == 0U)
        loop->execution_divider = 1U;
    loop->execution_tick = 0U;
    loop->dc_bus_ref = float2ctrl(0.0f);
    loop->active_power_cmd = float2ctrl(0.0f);
}

GMP_STATIC_INLINE void ctl_clear_sinv_outer_loop(ctl_sinv_outer_loop_t* loop)
{
    ctl_clear_pi(&loop->power_pi);
    ctl_clear_pi(&loop->dc_bus_pi);
    loop->execution_tick = 0U;
    loop->active_power_cmd = float2ctrl(0.0f);
}

GMP_STATIC_INLINE fast_gt ctl_sinv_outer_loop_due(ctl_sinv_outer_loop_t* loop)
{
    if (++loop->execution_tick >= loop->execution_divider) {
        loop->execution_tick = 0U;
        return 1;
    }
    return 0;
}

GMP_STATIC_INLINE ctrl_gt ctl_step_sinv_power_loop(ctl_sinv_outer_loop_t* loop,
                                                    ctrl_gt power_ref, ctrl_gt power_feedback)
{
    if (ctl_sinv_outer_loop_due(loop))
        loop->active_power_cmd = ctl_step_pi_par(&loop->power_pi, power_ref - power_feedback);
    return loop->active_power_cmd;
}

GMP_STATIC_INLINE ctrl_gt ctl_step_sinv_dc_bus_loop(ctl_sinv_outer_loop_t* loop,
                                                     ctrl_gt bus_ref, ctrl_gt bus_feedback,
                                                     ctrl_gt rectifier_polarity)
{
    loop->dc_bus_ref = bus_ref;
    if (ctl_sinv_outer_loop_due(loop)) {
        ctrl_gt magnitude = ctl_step_pi_par(&loop->dc_bus_pi, bus_ref - bus_feedback);
        loop->active_power_cmd = ctl_mul(rectifier_polarity, magnitude);
    }
    return loop->active_power_cmd;
}

#ifdef __cplusplus
}
#endif
#endif
