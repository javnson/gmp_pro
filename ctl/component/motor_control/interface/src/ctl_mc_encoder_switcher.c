#include <gmp_core.h>

#include <ctl/component/motor_control/interface/encoder_switcher.h>

/**
 * @brief Initializes the angle switcher context to safe defaults.
 * * @param[out] ctx          Pointer to the angle switcher structure to be initialized.
 * @param[in]  trans_time_s Initial duration of the transition in seconds.
 * @param[in]  isr_freq     The execution frequency of the ISR in Hz.
 */
void ctl_init_angle_switcher(ctl_angle_switcher_t* ctx, parameter_gt trans_time_s, parameter_gt isr_freq)
{
    ctx->out_enc.position = 0;
    ctx->out_enc.elec_position = 0;
    ctx->out_enc.revolutions = 0;

    ctx->state = ANGLE_SWITCH_IDLE_A; // Default to Source A
    ctx->weight = float2ctrl(0.0f);
    ctl_set_angle_switcher_duration(ctx, trans_time_s, isr_freq);
}
