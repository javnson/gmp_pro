#include <gmp_core.h>

#include <ctl/component/motor_control/interface/encoder_switcher.h>

/**
 * @brief Initializes the angle switcher context to safe defaults.
 * * @param[out] ctx          Pointer to the angle switcher structure to be initialized.
 * @param[in]  trans_time_s Initial duration of the transition in seconds.
 * @param[in]  isr_freq     The execution frequency of the ISR in Hz.
 */
void ctl_init_angle_switcher(ctl_angle_switcher_t* ctx, parameter_gt transition_time_s,
                             parameter_gt isr_frequency_hz)
{
    ctx->src_a = NULL;
    ctx->src_b = NULL;
    ctx->spd_a = NULL;
    ctx->spd_b = NULL;
    ctx->out_enc.position = float2ctrl(0.0f);
    ctx->out_enc.elec_position = float2ctrl(0.0f);
    ctx->out_enc.revolutions = 0;
    ctx->out_spd.speed = float2ctrl(0.0f);
    ctx->state = ANGLE_SWITCH_IDLE_A;
    ctx->weight = float2ctrl(0.0f);
    ctx->transition_progress = float2ctrl(1.0f);
    ctx->angle_offset_pu = float2ctrl(0.0f);
    ctx->transition_speed_start = float2ctrl(0.0f);
    ctx->speed_match_enter_pu = float2ctrl(0.0f);
    ctx->speed_match_exit_pu = float2ctrl(0.0f);
    ctx->qualify_ticks_required = 1U;
    ctx->qualify_ticks = 0U;
    ctx->flag_speed_qualification = 0;
    ctx->requested_source_b = 0;
    ctx->request_pending = 0;
    ctl_set_angle_switcher_duration(ctx, transition_time_s, isr_frequency_hz);
}
