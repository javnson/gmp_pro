/**
 * @file dsa_dl_scope.c
 * @brief Four-channel DSA capture backend for the GMP Data Link Scope service.
 */

#include <ctl/math_block/gmp_math.h>
#ifndef SPECIFY_DISABLE_CSP
// The DataLink bridge is platform-bound. Include the CSP inline critical
// section implementation without pulling the complete GMP runtime into CTL.
#include <core/dev/peripheral_types.h>
#include <csp.general.h>
#endif
#include <core/base/gmp_base.h>

#include <ctl/component/dsa/dsa_dl_scope.h>

/** @brief Return the wire sample type used by the configured control scalar. */
static gmp_scope_sample_type_t ctl_dsa_dl_scope_sample_type(void)
{
#if defined CTRL_GT_IS_DOUBLE
    return GMP_SCOPE_SAMPLE_F64;
#elif defined CTRL_GT_IS_FLOAT || defined CTRL_GT_IS_CMSIS_FLOAT || defined CTRL_GT_IS_QFP_FLOAT
    return GMP_SCOPE_SAMPLE_F32;
#else
    return GMP_SCOPE_SAMPLE_RAW;
#endif
}

/** @brief Return the requested number of pre-trigger samples. */
static uint32_t ctl_dsa_dl_scope_pretrigger(const ctl_dsa_dl_scope_t* scope)
{
    uint32_t samples = (scope->depth * scope->position_permille) / 1000U;
    if (samples >= scope->depth)
        samples = scope->depth - 1U;
    return samples;
}

/** @brief Store one sample tuple in the circular history. */
static void ctl_dsa_dl_scope_store_history(ctl_dsa_dl_scope_t* scope,
                                           const ctrl_gt channels[CTL_DSA_DL_SCOPE_CHANNELS])
{
    uint16_t channel;
    for (channel = 0U; channel < CTL_DSA_DL_SCOPE_CHANNELS; ++channel)
        scope->history[channel * scope->depth + scope->history_write] = channels[channel];

    scope->history_write++;
    if (scope->history_write >= scope->depth)
        scope->history_write = 0U;
    if (scope->history_count < scope->depth)
        scope->history_count++;
}

/** @brief Copy chronological pre-trigger samples into the published buffer. */
static void ctl_dsa_dl_scope_copy_history(ctl_dsa_dl_scope_t* scope,
                                          uint32_t sample_count)
{
    uint16_t channel;
    uint32_t index;
    uint32_t source = (scope->history_write + scope->depth - sample_count) % scope->depth;

    for (channel = 0U; channel < CTL_DSA_DL_SCOPE_CHANNELS; ++channel)
    {
        uint32_t read_index = source;
        for (index = 0U; index < sample_count; ++index)
        {
            scope->buffer[channel * scope->depth + index] =
                scope->history[channel * scope->depth + read_index];
            read_index++;
            if (read_index >= scope->depth)
                read_index = 0U;
        }
    }
}

/** @brief Arm one acquisition while the control interrupt is excluded. */
static void ctl_dsa_dl_scope_arm_internal(ctl_dsa_dl_scope_t* scope)
{
    uint32_t timeout_ticks;
    uint32_t effective_sample_rate_hz;

    ctl_clear_dsa_trigger(&scope->trigger);
    scope->trigger.option = scope->trigger_mode;
    scope->trigger.trigger_level = real2ctrl(scope->trigger_level);
    effective_sample_rate_hz = scope->sample_rate_hz /
                               ((uint32_t)scope->sample_divider + 1U);
    timeout_ticks = (scope->auto_timeout_ms * effective_sample_rate_hz) / 1000U;
    scope->trigger.auto_timeout_ticks = (timeout_ticks == 0U) ? 1U : timeout_ticks;
    scope->trigger.flag_is_force_trigger = 0;
    ctl_reset_dsa_scope_tracker(&scope->recorder);
    scope->history_write = 0U;
    scope->history_count = 0U;
    scope->sample_divider_counter = 0U;
    scope->state = GMP_SCOPE_STATE_WAITING;
}

/** @brief Apply one configuration received from the desktop debugger. */
static fast_gt ctl_dsa_dl_scope_configure(void* user_context,
                                          const gmp_scope_config_t* config)
{
    ctl_dsa_dl_scope_t* scope = (ctl_dsa_dl_scope_t*)user_context;
    if (scope == NULL || config == NULL ||
        config->mode > DSA_TRIGGER_OPTION_FALLING_EDGE_AUTO ||
        config->channel >= CTL_DSA_DL_SCOPE_CHANNELS ||
        config->position_permille > 1000U)
        return 0;

    gmp_base_enter_critical();
    scope->trigger_mode = (ctl_dsa_trigger_option_t)config->mode;
    scope->trigger_channel = config->channel;
    scope->position_permille = config->position_permille;
    scope->trigger_level = config->level;
    scope->auto_timeout_ms = config->auto_timeout_ms;
    scope->sample_divider = config->sample_divider;
    scope->sample_divider_counter = 0U;
    gmp_base_leave_critical();
    return 1;
}

/** @brief Arm callback exported through the generic Scope service. */
static fast_gt ctl_dsa_dl_scope_arm(void* user_context)
{
    ctl_dsa_dl_scope_t* scope = (ctl_dsa_dl_scope_t*)user_context;
    if (scope == NULL)
        return 0;

    gmp_base_enter_critical();
    ctl_dsa_dl_scope_arm_internal(scope);
    gmp_base_leave_critical();
    return 1;
}

/** @brief Status callback exported through the generic Scope service. */
static gmp_scope_capture_state_t ctl_dsa_dl_scope_status(void* user_context,
                                                         uint32_t* generation)
{
    ctl_dsa_dl_scope_t* scope = (ctl_dsa_dl_scope_t*)user_context;
    if (scope == NULL)
        return GMP_SCOPE_STATE_WAITING;
    if (generation != NULL)
        *generation = scope->generation;
    return scope->state;
}

void ctl_init_dsa_dl_scope(ctl_dsa_dl_scope_t* scope, gmp_datalink_t* dl,
                           uint16_t command, const char* name,
                           ctrl_gt* buffer, ctrl_gt* history,
                           uint32_t depth, uint32_t sample_rate_hz)
{
    if (scope == NULL || dl == NULL || buffer == NULL || history == NULL ||
        depth == 0U || sample_rate_hz == 0U)
        return;

    scope->buffer = buffer;
    scope->history = history;
    scope->depth = depth;
    scope->sample_rate_hz = sample_rate_hz;
    scope->generation = 0U;
    scope->position_permille = 500U;
    scope->trigger_channel = 0;
    scope->trigger_mode = DSA_TRIGGER_OPTION_RISING_EDGE_AUTO;
    scope->auto_timeout_ms = 1000U;
    scope->sample_divider = 0U;
    scope->sample_divider_counter = 0U;
    scope->trigger_level = 0.0F;

    scope->resource.name = (name == NULL) ? "Control Scope" : name;
    scope->resource.buffer = buffer;
    scope->resource.byte_length = depth * CTL_DSA_DL_SCOPE_CHANNELS *
                                  sizeof(ctrl_gt) * GMP_PORT_DATA_SIZE_PER_BYTES;
    scope->resource.sample_type = ctl_dsa_dl_scope_sample_type();
    scope->resource.layout = GMP_SCOPE_LAYOUT_SOA;
    scope->resource.channels = CTL_DSA_DL_SCOPE_CHANNELS;
    scope->resource.depth = depth;
    scope->resource.sample_rate_hz = sample_rate_hz;
    scope->resource.configure = ctl_dsa_dl_scope_configure;
    scope->resource.arm = ctl_dsa_dl_scope_arm;
    scope->resource.get_status = ctl_dsa_dl_scope_status;
    scope->resource.user_context = scope;

    gmp_scope_init(&scope->service, dl, command, &scope->resource, 1);
    ctl_init_dsa_trigger(&scope->trigger, DSA_TRIGGER_OPTION_RISING_EDGE_AUTO,
                         0.0F, 1.0F, (parameter_gt)sample_rate_hz);
    ctl_init_dsa_scope(&scope->recorder, buffer,
                       depth * CTL_DSA_DL_SCOPE_CHANNELS,
                       (parameter_gt)sample_rate_hz);
    ctl_config_dsa_scope(&scope->recorder, CTL_DSA_DL_SCOPE_CHANNELS, 1U);
    ctl_dsa_dl_scope_arm_internal(scope);
}

void ctl_step_dsa_dl_scope_4ch(ctl_dsa_dl_scope_t* scope,
                               ctrl_gt channel_0, ctrl_gt channel_1,
                               ctrl_gt channel_2, ctrl_gt channel_3)
{
    ctrl_gt channels[CTL_DSA_DL_SCOPE_CHANNELS];
    ctrl_gt trigger_sample;
    uint32_t pretrigger_samples;
    uint16_t channel;

    if (scope == NULL || scope->buffer == NULL || scope->history == NULL)
        return;

    if (scope->sample_divider_counter < scope->sample_divider)
    {
        scope->sample_divider_counter++;
        return;
    }
    scope->sample_divider_counter = 0U;

    channels[0] = channel_0;
    channels[1] = channel_1;
    channels[2] = channel_2;
    channels[3] = channel_3;
    trigger_sample = channels[scope->trigger_channel];
    pretrigger_samples = ctl_dsa_dl_scope_pretrigger(scope);

    if (scope->state == GMP_SCOPE_STATE_WAITING)
    {
        fast_gt triggered = ctl_step_dsa_trigger(&scope->trigger, trigger_sample);
        if (triggered && scope->history_count >= pretrigger_samples)
        {
            ctl_dsa_dl_scope_copy_history(scope, pretrigger_samples);
            for (channel = 0U; channel < CTL_DSA_DL_SCOPE_CHANNELS; ++channel)
                scope->buffer[channel * scope->depth + pretrigger_samples] = channels[channel];
            scope->recorder.current_idx = pretrigger_samples + 1U;
            ctl_clear_divider(&scope->recorder.divider);
            if (scope->recorder.current_idx >= scope->recorder.depth)
            {
                scope->generation++;
                scope->state = GMP_SCOPE_STATE_READY;
            }
            else
            {
                scope->state = GMP_SCOPE_STATE_CAPTURING;
            }
        }
        else
        {
            ctl_dsa_dl_scope_store_history(scope, channels);
        }
    }
    else if (scope->state == GMP_SCOPE_STATE_CAPTURING)
    {
        (void)ctl_step_dsa_scope_4ch(&scope->recorder, channel_0, channel_1,
                                     channel_2, channel_3);
        if (scope->recorder.current_idx >= scope->recorder.depth)
        {
            scope->generation++;
            scope->state = GMP_SCOPE_STATE_READY;
        }
    }
}

fast_gt ctl_dsa_dl_scope_rx_cb(ctl_dsa_dl_scope_t* scope)
{
    if (scope == NULL)
        return 0;
    return gmp_scope_rx_cb(&scope->service);
}
