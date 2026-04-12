/**
 * @file angle_switcher.h
 * @brief Smooth angle transition module for motor control encoder sources.
 */

#include <ctl/component/motor_control/interface/motor_universal_interface.h>

#ifndef _FILE_ENCODER_SWITCHER_H_
#define _FILE_ENCODER_SWITCHER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief State enumeration for the angle switcher.
 */
typedef enum _tag_angle_switch_state
{
    ANGLE_SWITCH_IDLE_A = 0, /*!< Purely outputting Source A. */
    ANGLE_SWITCH_IDLE_B,     /*!< Purely outputting Source B. */
    ANGLE_SWITCH_TRANS_A2B,  /*!< Transitioning from Source A to Source B. */
    ANGLE_SWITCH_TRANS_B2A   /*!< Transitioning from Source B to Source A. */
} angle_switch_state_e;

/**
 * @brief A module to smoothly transition between two rotation encoder sources.
 */
typedef struct _tag_angle_switcher
{
    // --- Input Interfaces ---
    rotation_ift* src_a; /*!< Pointer to rotation source A (e.g., IF Generator). */
    rotation_ift* src_b; /*!< Pointer to rotation source B (e.g., SMO or Real Encoder). */

    // --- Output Interface ---
    rotation_ift out_enc; /*!< The blended output rotation interface. */

    // --- State & Control ---
    angle_switch_state_e state; /*!< Current state of the switcher. */

    ctrl_gt weight;      /*!< Transition weight (0.0 to 1.0). 0.0 = Pure A, 1.0 = Pure B. */
    ctrl_gt weight_step; /*!< Increment added to weight per ISR tick. */

} ctl_angle_switcher_t;

/**
 * @brief Wraps a per-unit angle to the range [0.0, 1.0).
 * @param[in] angle The input angle in per-unit (PU).
 * @return The wrapped angle bounded to the range [0.0, 1.0).
 */
GMP_STATIC_INLINE ctrl_gt ctl_helper_wrap_angle_pu(ctrl_gt angle)
{
    ctrl_gt res = ctl_mod_1(angle);
    if (res < float2ctrl(0.0f))
    {
        res += 1.0f;
    }
    return res;
}

/**
 * @brief Calculates the shortest path difference between two PU angles.
 * @details Computes (target - current) and normalizes the result via shortest path.
 * @param[in] target  The target angle in per-unit (PU).
 * @param[in] current The current angle in per-unit (PU).
 * @return The shortest angular difference bounded to the range [-0.5, 0.5).
 */
GMP_STATIC_INLINE ctrl_gt ctl_helper_diff_angle_pu(ctrl_gt target, ctrl_gt current)
{
    ctrl_gt diff = target - current;
    diff = ctl_mod_1(diff);

    if (diff > float2ctrl(0.5f))
    {
        diff -= float2ctrl(1.0f);
    }
    else if (diff < float2ctrl(-0.5f))
    {
        diff += float2ctrl(1.0f);
    }
    return diff;
}

/**
 * @brief Sets or updates the transition duration for the angle switcher.
 * * @param[in,out] ctx          Pointer to the angle switcher structure.
 * @param[in]     trans_time_s The desired duration of the transition in seconds.
 * @param[in]     isr_freq     The execution frequency of the ISR in Hz.
 */
GMP_STATIC_INLINE void ctl_set_angle_switcher_duration(ctl_angle_switcher_t* ctx, parameter_gt trans_time_s,
                                                       parameter_gt isr_freq)
{
    ctx->weight_step = float2ctrl(1.0f / (trans_time_s * isr_freq));
}

/**
 * @brief Initializes the angle switcher context to safe defaults.
 * * @param[out] ctx          Pointer to the angle switcher structure to be initialized.
 * @param[in]  trans_time_s Initial duration of the transition in seconds.
 * @param[in]  isr_freq     The execution frequency of the ISR in Hz.
 */
void ctl_init_angle_switcher(ctl_angle_switcher_t* ctx, parameter_gt trans_time_s, parameter_gt isr_freq);

/**
 * @brief Attaches rotation sources to the angle switcher interface.
 * @param ctx   Pointer to the angle switcher structure.
 * @param src_a Pointer to the first angle source.
 * @param src_b Pointer to the second angle source.
 */
GMP_STATIC_INLINE void ctl_attach_angle_switcher(ctl_angle_switcher_t* ctx, rotation_ift* _src_a, rotation_ift* _src_b)
{
    ctx->src_a = _src_a;
    ctx->src_b = _src_b;
}

/**
 * @brief Triggers a transition to the target source over a specified duration.
 * @param ctx       Pointer to the angle switcher structure.
 * @param target  If true (1), transition to Source B. If false (0), transition to Source A.
 * @param trans_time_s Duration of the transition in seconds.
 * @param isr_freq  The frequency of the ISR in Hz.
 */
GMP_STATIC_INLINE void ctl_trigger_angle_transition(ctl_angle_switcher_t* ctx, fast_gt target)
{
    if (target)
    {
        if (ctx->state == ANGLE_SWITCH_IDLE_A || ctx->state == ANGLE_SWITCH_TRANS_B2A)
        {
            ctx->state = ANGLE_SWITCH_TRANS_A2B;
        }
    }
    else
    {
        if (ctx->state == ANGLE_SWITCH_IDLE_B || ctx->state == ANGLE_SWITCH_TRANS_A2B)
        {
            ctx->state = ANGLE_SWITCH_TRANS_B2A;
        }
    }
}

/**
 * @brief Executes one step of the angle blending logic.
 * @details Interpolates between the two attached encoder sources based on the current transition weight.
 * This function must be called in the high-frequency control ISR before the FOC core executes.
 * @param[in,out] ctx Pointer to the angle switcher structure.
 * @return The current blended electrical angle in per-unit (PU).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_angle_switcher(ctl_angle_switcher_t* ctx)
{
    ctrl_gt pos_a = ctx->src_a->elec_position;
    ctrl_gt pos_b = ctx->src_b->elec_position;
    ctrl_gt diff;

    switch (ctx->state)
    {
    case ANGLE_SWITCH_IDLE_A:
        ctx->weight = float2ctrl(0.0f);
        ctx->out_enc.elec_position = pos_a;
        break;

    case ANGLE_SWITCH_IDLE_B:
        ctx->weight = float2ctrl(1.0f);
        ctx->out_enc.elec_position = pos_b;
        break;

    case ANGLE_SWITCH_TRANS_A2B:
        ctx->weight += ctx->weight_step;
        if (ctx->weight >= float2ctrl(1.0f))
        {
            ctx->weight = float2ctrl(1.0f);
            ctx->state = ANGLE_SWITCH_IDLE_B;
            ctx->out_enc.elec_position = pos_b;
        }
        else
        {
            // Shortest path logic: out = A + weight * (B - A)
            diff = ctl_helper_diff_angle_pu(pos_b, pos_a);
            ctx->out_enc.elec_position = ctl_helper_wrap_angle_pu(pos_a + ctl_mul(ctx->weight, diff));
        }
        break;

    case ANGLE_SWITCH_TRANS_B2A:
        ctx->weight -= ctx->weight_step;
        if (ctx->weight <= float2ctrl(0.0f))
        {
            ctx->weight = float2ctrl(0.0f);
            ctx->state = ANGLE_SWITCH_IDLE_A;
            ctx->out_enc.elec_position = pos_a;
        }
        else
        {
            // Shortest path logic: out = A + weight * (B - A)
            // Note: Even when moving B->A, the formula holds true based on weight decreasing to 0
            diff = ctl_helper_diff_angle_pu(pos_b, pos_a);
            ctx->out_enc.elec_position = ctl_helper_wrap_angle_pu(pos_a + ctl_mul(ctx->weight, diff));
        }
        break;
    }

    // Pass through mechanical position & revolutions from the currently dominant source
    // (Optional: You could also blend mechanical position if needed, but usually elec_position is the critical one)
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

    return ctx->out_enc.elec_position;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif //_FILE_ENCODER_SWITCHER_H_
