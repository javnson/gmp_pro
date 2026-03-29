
#include <gmp_core.h>

#include <ctl/component/intrinsic/basic/state_sequencer.h>

#include <ctl/component/dsa/dsa_scope.h>

#include <ctl/component/motor_control/basic/mtr_protection.h>
#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/current_loop/foc_core.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/component/motor_control/interface/encoder_switcher.h>
#include <ctl/component/motor_control/observer/pmsm_esmo.h>

#include <ctl/component/motor_control/consultant/mech_consultant.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>
#include <ctl/component/motor_control/consultant/pu_consultant.h>

#include <ctl/component/motor_control/pmsm_offline_id/pmsm_offline_id_sm.h>

//
// --- Resistance & Dead-Time (RS_DT) ---
//

#pragma region RS_DT_MODULE

/**
 * @brief Initializes the Rs & DT identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_rs_dt(ctl_pmsm_offline_id_t* ctx)
{
    ctx->sub_rs_dt.sm = PMSM_ID_RSDT_INIT;
    ctx->sub_rs_dt.angle_idx = 0;
    ctx->sub_rs_dt.step_idx = 0;
    ctx->sub_rs_dt.angle_pu = float2ctrl(0.0f);
    ctx->sub_rs_dt.current_ref_pu = float2ctrl(0.0f);

    // Arm the sequencer so the background loop catches FIRST_ENTRY for INIT
    ctl_clear_state_seq(&ctx->seq, 0);
}

/**
 * @brief ISR step function for Rs & DT identification.
 * @details STRICT RULE: ISR acts as the "Data Path" only. 
 * It executes actions based on the current state and sequence phase, but NEVER changes the state.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_rs_dt_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_rs_dt_t* sub = &ctx->sub_rs_dt;
    pmsm_oid_cfg_rs_dt_t* cfg = &sub->cfg;

    // Call the sequencer strictly in every active state
    ctl_state_seq_e seq_phase = ctl_step_state_seq(&ctx->seq);

    switch (sub->sm)
    {
    case PMSM_ID_RSDT_DISABLED:
    case PMSM_ID_RSDT_INIT:
    case PMSM_ID_RSDT_STEP_EVALUATE:
    case PMSM_ID_RSDT_CALCULATE:
    case PMSM_ID_RSDT_COMPLETE:
    case PMSM_ID_RSDT_FAULT:
        // Passive states: Keep stepping the sequencer for diagnostic timing, but do nothing physically.
        break;

    case PMSM_ID_RSDT_ALIGN_SETTLE:
        switch (seq_phase)
        {
        case CTL_ST_FIRST_ENTRY:
            if (sub->angle_idx < 6)
            {
                sub->angle_pu = sub->angle_pu_array[sub->angle_idx];
            }
            ctl_id_set_static_angle(ctx, sub->angle_pu);
            ctl_id_apply_dc_current(ctx, cfg->max_current_pu, float2ctrl(0.0f));
            break;
        case CTL_ST_KEEP:
        case CTL_ST_LEAVE:
            // Hold output. Waiting for loop to transition.
            break;
        }
        break;

    case PMSM_ID_RSDT_STEP_DELAY:
        switch (seq_phase)
        {
        case CTL_ST_FIRST_ENTRY:
            sub->current_ref_pu += sub->step_size_pu; // Pure math addition
            ctl_id_apply_dc_current(ctx, sub->current_ref_pu, float2ctrl(0.0f));
            break;
        case CTL_ST_KEEP:
        case CTL_ST_LEAVE:
            // Hold transient decay. Waiting for loop.
            break;
        }
        break;

    case PMSM_ID_RSDT_MEASURE:
        switch (seq_phase)
        {
        case CTL_ST_FIRST_ENTRY:
            sub->sum_u = float2ctrl(0.0f);
            sub->sum_i = float2ctrl(0.0f);
            // Intentionally NO break. Accumulate the very first point!
        case CTL_ST_KEEP:
            sub->sum_u += ctl_id_get_vdq(ctx, phase_d);
            sub->sum_i += ctl_id_get_idq(ctx, phase_d);
            break;
        case CTL_ST_LEAVE:
            // Stop accumulating. Hold safe state and wait for loop to process the sum.
            break;
        }
        break;
    }
}

/**
 * @brief Background loop function for Rs & DT identification.
 * @details STRICT RULE: Loop acts as the "Control Path". 
 * It exclusively manages state transitions (sub->sm = xxx), handles heavy math, 
 * and interacts with the DSA Scope.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_rs_dt(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_rs_dt_t* sub = &ctx->sub_rs_dt;
    pmsm_oid_cfg_rs_dt_t* cfg = &sub->cfg;
    uint16_t i;

    // Safety Rule: Zero output in passive states
    if (sub->sm == PMSM_ID_RSDT_CALCULATE || sub->sm == PMSM_ID_RSDT_COMPLETE || sub->sm == PMSM_ID_RSDT_FAULT)
    {
        ctl_id_disable_output(ctx);
    }

    // Call the sequencer strictly to evaluate loop lifecycle
    ctl_state_seq_e loop_phase = ctl_loop_state_seq(&ctx->seq);

    switch (sub->sm)
    {
    case PMSM_ID_RSDT_INIT:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
            // Pre-calculate constants
            sub->align_ticks = SEC_TO_TICKS(cfg->align_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->measure_delay_ticks = SEC_TO_TICKS(cfg->measure_delay_s, ctx->cfg_basic.isr_freq_hz);

            if (cfg->measure_points > 0)
                sub->inv_measure_points = ctl_div(float2ctrl(1.0f), float2ctrl((float)cfg->measure_points));
            else
                sub->inv_measure_points = float2ctrl(1.0f);

            if (cfg->steps > 1)
                sub->step_size_pu =
                    ctl_div((cfg->max_current_pu - cfg->min_current_pu), float2ctrl((float)(cfg->steps - 1)));
            else
                sub->step_size_pu = float2ctrl(0.0f);

            for (i = 0; i < 6; i++)
            {
                sub->angle_pu_array[i] = float2ctrl((float)i * 0.1666667f);
            }

            sub->current_ref_pu = float2ctrl(cfg->min_current_pu) - sub->step_size_pu;

            ctl_id_route_foc_angle(ctx, PMSM_ID_ANGLE_SRC_STATIC);
            ctl_id_set_foc_state(ctx, PMSM_ID_CURRENT_CLOSELOOP);

            ctl_wipe_dsa_scope_memory(&ctx->analyzer);
            ctl_config_dsa_scope(&ctx->analyzer, 2, 1);

            // TRANSITION -> ALIGN_SETTLE
            sub->sm = PMSM_ID_RSDT_ALIGN_SETTLE;
            ctl_clear_state_seq(&ctx->seq, sub->align_ticks);
        }
        break;

    case PMSM_ID_RSDT_ALIGN_SETTLE:
        if (loop_phase == CTL_ST_LEAVE)
        {
            // TRANSITION -> STEP_DELAY
            sub->sm = PMSM_ID_RSDT_STEP_DELAY;
            ctl_clear_state_seq(&ctx->seq, sub->measure_delay_ticks);
        }
        break;

    case PMSM_ID_RSDT_STEP_DELAY:
        if (loop_phase == CTL_ST_LEAVE)
        {
            // TRANSITION -> MEASURE
            sub->sm = PMSM_ID_RSDT_MEASURE;
            ctl_clear_state_seq(&ctx->seq, cfg->measure_points);
        }
        break;

    case PMSM_ID_RSDT_MEASURE:
        if (loop_phase == CTL_ST_LEAVE)
        {
            // ISR has safely finished accumulating points. Loop calculates the average.
            ctrl_gt avg_u = ctl_mul(sub->sum_u, sub->inv_measure_points);
            ctrl_gt avg_i = ctl_mul(sub->sum_i, sub->inv_measure_points);

            // Push to DSA Scope (Perfectly safe to do in the loop here!)
            ctl_step_dsa_scope_2ch(&ctx->analyzer, avg_i, avg_u);

            // TRANSITION -> STEP_EVALUATE
            sub->sm = PMSM_ID_RSDT_STEP_EVALUATE;
            ctl_clear_state_seq(&ctx->seq, 0); // 0-tick logical state
        }
        break;

    case PMSM_ID_RSDT_STEP_EVALUATE:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
            sub->step_idx++;

            if (sub->step_idx >= cfg->steps)
            {
                sub->step_idx = 0;
                sub->angle_idx++;

                if (sub->angle_idx >= 6)
                {
                    // TRANSITION -> CALCULATE
                    sub->sm = PMSM_ID_RSDT_CALCULATE;
                    ctl_clear_state_seq(&ctx->seq, 0);
                }
                else
                {
                    // TRANSITION -> ALIGN_SETTLE
                    sub->sm = PMSM_ID_RSDT_ALIGN_SETTLE;
                    ctl_clear_state_seq(&ctx->seq, sub->align_ticks);
                }
            }
            else
            {
                // TRANSITION -> STEP_DELAY
                sub->sm = PMSM_ID_RSDT_STEP_DELAY;
                ctl_clear_state_seq(&ctx->seq, sub->measure_delay_ticks);
            }
        }
        break;

    case PMSM_ID_RSDT_CALCULATE:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
            parameter_gt rs_sum = 0.0f, vcomp_sum = 0.0f;

            for (i = 0; i < 6; i++)
            {
                uint32_t start_idx = i * cfg->steps;
                uint32_t end_idx = start_idx + cfg->steps - 1;
                parameter_gt slope = 0.0f, intercept = 0.0f;

                fast_gt fit_ok = ctl_dsa_fit_vs_dim(&ctx->analyzer, 0, 1, start_idx, end_idx, &slope, &intercept);

                if (fit_ok)
                {
                    sub->rs_array[i] = slope;
                    sub->vcomp_array[i] = intercept;
                }
                else
                {
                    sub->sm = PMSM_ID_RSDT_FAULT;
                    return;
                }

                rs_sum += sub->rs_array[i];
                vcomp_sum += sub->vcomp_array[i];
            }

            sub->rs_mean = rs_sum / 6.0f;
            sub->vcomp_mean = vcomp_sum / 6.0f;

            parameter_gt rs_var_sum = 0.0f, vcomp_var_sum = 0.0f;
            for (i = 0; i < 6; i++)
            {
                parameter_gt d_rs = sub->rs_array[i] - sub->rs_mean;
                parameter_gt d_vc = sub->vcomp_array[i] - sub->vcomp_mean;
                rs_var_sum += (d_rs * d_rs);
                vcomp_var_sum += (d_vc * d_vc);
            }
            sub->rs_var = rs_var_sum / 6.0f;
            sub->vcomp_var = vcomp_var_sum / 6.0f;

            parameter_gt Z_base = ctx->identified_pu.V_base / ctx->identified_pu.I_base;
            ctx->pmsm_param.Rs = sub->rs_mean * Z_base;

            ctl_wipe_dsa_scope_memory(&ctx->analyzer);

            // TRANSITION -> COMPLETE
            sub->sm = PMSM_ID_RSDT_COMPLETE;
            ctl_clear_state_seq(&ctx->seq, 0);
        }
        break;

    default:
        break;
    }
}

#pragma endregion

//
// --- Inductance (LD_LQ) ---
//

#pragma region LD_LQ_MDOULE

/**
 * @brief Initializes the Inductance (Ld, Lq) identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_ldq(ctl_pmsm_offline_id_t* ctx)
{
    ctx->sub_ldq.sm = PMSM_ID_LDQ_INIT;
    ctx->sub_ldq.bias_step_idx = 0;
    ctx->sub_ldq.is_measuring_q_axis = 0;
    ctx->sub_ldq.bias_curr_ref_pu = float2ctrl(0.0f);

    // Arm the sequencer so the background loop catches FIRST_ENTRY for INIT
    ctl_clear_state_seq(&ctx->seq, 0);
}

/**
 * @brief ISR step function for Ld & Lq identification.
 * @details STRICT DATA PATH: Executes pulse actions and data recording via DSA.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_ldq_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_ldq_t* sub = &ctx->sub_ldq;
    pmsm_oid_cfg_ld_lq_t* cfg = &sub->cfg;

    ctl_state_seq_e seq_phase = ctl_step_state_seq(&ctx->seq);

    switch (sub->sm)
    {
    case PMSM_ID_LDQ_DISABLED:
    case PMSM_ID_LDQ_INIT:
    case PMSM_ID_LDQ_STEP_EVALUATE:
    case PMSM_ID_LDQ_CALCULATE:
    case PMSM_ID_LDQ_COMPLETE:
    case PMSM_ID_LDQ_FAULT:
        break;

    case PMSM_ID_LDQ_BIAS_SETTLE:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            if (sub->is_measuring_q_axis == 0)
            {
                ctl_id_apply_dc_current(ctx, sub->bias_curr_ref_pu, float2ctrl(0.0f));
            }
            else
            {
                ctl_id_apply_dc_current(ctx, float2ctrl(cfg->align_current_pu), sub->bias_curr_ref_pu);
            }
        }
        break;

    case PMSM_ID_LDQ_PULSE_MEASURE:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            // 1. Freeze PI voltages
            sub->frozen_vd_pu = ctl_id_get_vdq(ctx, 0);
            sub->frozen_vq_pu = ctl_id_get_vdq(ctx, 1);

            // 2. Freeze steady-state currents (I_0) to calculate Delta I later
            sub->frozen_id_pu = ctl_id_get_idq(ctx, 0);
            sub->frozen_iq_pu = ctl_id_get_idq(ctx, 1);

            // 3. Open Loop Pulse Injection
            if (sub->is_measuring_q_axis == 0)
            {
                ctl_id_apply_voltage_pulse(ctx, sub->frozen_vd_pu + float2ctrl(cfg->pulse_voltage_pu),
                                           sub->frozen_vq_pu);
            }
            else
            {
                ctl_id_apply_voltage_pulse(ctx, sub->frozen_vd_pu,
                                           sub->frozen_vq_pu + float2ctrl(cfg->pulse_voltage_pu));
            }
        }

        // Keep pushing high-speed current points during the active pulse
        if (seq_phase == CTL_ST_KEEP || seq_phase == CTL_ST_LEAVE)
        {
            ctrl_gt active_i = (sub->is_measuring_q_axis == 0) ? ctl_id_get_idq(ctx, 0) : ctl_id_get_idq(ctx, 1);
            ctl_step_dsa_scope_1ch(&ctx->analyzer, active_i);
        }
        break;

    case PMSM_ID_LDQ_COOLDOWN:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            // Remove pulse, let inductive energy decay safely
            ctl_id_apply_voltage_pulse(ctx, sub->frozen_vd_pu, sub->frozen_vq_pu);
        }
        break;
    }
}

/**
 * @brief Background loop function for Ld & Lq identification.
 * @details STRICT CONTROL PATH: Manages lifecycle, computes R-L Integral evaluations, 
 * and controls state transitions safely.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_ldq(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_ldq_t* sub = &ctx->sub_ldq;
    pmsm_oid_cfg_ld_lq_t* cfg = &sub->cfg;

    if (sub->sm == PMSM_ID_LDQ_CALCULATE || sub->sm == PMSM_ID_LDQ_COMPLETE || sub->sm == PMSM_ID_LDQ_FAULT)
    {
        ctl_id_disable_output(ctx);
    }

    ctl_state_seq_e loop_phase = ctl_loop_state_seq(&ctx->seq);

    switch (sub->sm)
    {
    case PMSM_ID_LDQ_INIT:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
            sub->settle_ticks = SEC_TO_TICKS(cfg->settle_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->pulse_ticks = SEC_TO_TICKS(cfg->pulse_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->cooldown_ticks = SEC_TO_TICKS(cfg->cooldown_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->dt_sec = 1.0f / ctx->cfg_basic.isr_freq_hz;

            if (cfg->bias_steps > 1)
            {
                sub->step_size_pu = float2ctrl(cfg->max_bias_curr_pu / (float)(cfg->bias_steps - 1));
            }
            else
            {
                sub->step_size_pu = float2ctrl(0.0f);
            }

            ctl_id_set_static_angle(ctx, float2ctrl(0.0f));
            ctl_id_set_foc_state(ctx, PMSM_ID_CURRENT_CLOSELOOP);

            // DA Only needs to hold ONE pulse length now!
            ctl_wipe_dsa_scope_memory(&ctx->analyzer);
            ctl_config_dsa_scope(&ctx->analyzer, 1, 1);

            sub->bias_curr_ref_pu = float2ctrl(0.0f);
            sub->sm = PMSM_ID_LDQ_BIAS_SETTLE;
            ctl_clear_state_seq(&ctx->seq, sub->settle_ticks);
        }
        break;

    case PMSM_ID_LDQ_BIAS_SETTLE:
        if (loop_phase == CTL_ST_LEAVE)
        {
            sub->sm = PMSM_ID_LDQ_PULSE_MEASURE;
            ctl_clear_state_seq(&ctx->seq, sub->pulse_ticks);
        }
        break;

    case PMSM_ID_LDQ_PULSE_MEASURE:
        if (loop_phase == CTL_ST_LEAVE)
        {
            // === NEW: IMMEDIATE R-L INTEGRAL CALCULATION ===
            // Get physical Rs previously identified, convert back to PU
            parameter_gt Z_base = ctx->identified_pu.V_base / ctx->identified_pu.I_base;
            parameter_gt rs_pu = ctx->pmsm_param.Rs / Z_base;

            parameter_gt V_p = cfg->pulse_voltage_pu;
            parameter_gt Ts = sub->dt_sec;
            parameter_gt I_0 =
                (sub->is_measuring_q_axis == 0) ? ctrl2float(sub->frozen_id_pu) : ctrl2float(sub->frozen_iq_pu);

            uint32_t N = ctx->analyzer.current_idx;
            parameter_gt sum_delta_i = 0.0f;
            parameter_gt delta_i_N = 0.0f;

            // Integrate over the recorded pulse
            for (uint32_t i = 0; i < N; i++)
            {
                parameter_gt I_k = ctrl2float(ctl_mem_get_2d_soa(&ctx->analyzer.mem, 0, i, ctx->analyzer.depth));
                parameter_gt delta_i = I_k - I_0;

                sum_delta_i += delta_i; // Integration Area
                if (i == N - 1)
                    delta_i_N = delta_i; // Final Delta I
            }

            // Safety guard: prevent division by zero if current didn't move
            if (delta_i_N < 1e-6f)
                delta_i_N = 1e-6f;

            // Integral Equation: L = (V_p * N * Ts - R_s * Ts * sum_delta_i) / delta_i_N
            parameter_gt volt_time_area = V_p * (parameter_gt)N * Ts;
            parameter_gt res_drop_area = rs_pu * Ts * sum_delta_i;
            parameter_gt L_pu = (volt_time_area - res_drop_area) / delta_i_N;

            // Store the immediate result
            if (sub->is_measuring_q_axis == 0)
            {
                sub->ld_array[sub->bias_step_idx] = L_pu;
            }
            else
            {
                sub->lq_array[sub->bias_step_idx] = L_pu;
            }

            // WIPE the DSA Scope ready for the next pulse!
            ctl_wipe_dsa_scope_memory(&ctx->analyzer);

            sub->sm = PMSM_ID_LDQ_COOLDOWN;
            ctl_clear_state_seq(&ctx->seq, sub->cooldown_ticks);
        }
        break;

    case PMSM_ID_LDQ_COOLDOWN:
        if (loop_phase == CTL_ST_LEAVE)
        {
            sub->sm = PMSM_ID_LDQ_STEP_EVALUATE;
            ctl_clear_state_seq(&ctx->seq, 0);
        }
        break;

    case PMSM_ID_LDQ_STEP_EVALUATE:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
            sub->bias_step_idx++;
            if (sub->bias_step_idx >= cfg->bias_steps)
            {
                if (sub->is_measuring_q_axis == 0)
                {
                    sub->is_measuring_q_axis = 1;
                    sub->bias_step_idx = 0;
                    sub->bias_curr_ref_pu = float2ctrl(0.0f);
                    sub->sm = PMSM_ID_LDQ_BIAS_SETTLE;
                    ctl_clear_state_seq(&ctx->seq, sub->settle_ticks);
                }
                else
                {
                    sub->sm = PMSM_ID_LDQ_CALCULATE;
                    ctl_clear_state_seq(&ctx->seq, 0);
                }
            }
            else
            {
                sub->bias_curr_ref_pu += sub->step_size_pu;
                sub->sm = PMSM_ID_LDQ_BIAS_SETTLE;
                ctl_clear_state_seq(&ctx->seq, sub->settle_ticks);
            }
        }
        break;

    case PMSM_ID_LDQ_CALCULATE:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
            // All arrays are already filled with PU inductances!
            // We just need to do physical base conversion.
            parameter_gt Z_base = ctx->identified_pu.V_base / ctx->identified_pu.I_base;
            parameter_gt L_base = Z_base / ctx->identified_pu.W_base;

            // Store Nominal L (at 0A bias, i.e., index 0)
            ctx->pmsm_param.Ld = sub->ld_array[0] * L_base;
            ctx->pmsm_param.Lq = sub->lq_array[0] * L_base;

            ctx->pmsm_param.saliency_ratio = ctx->pmsm_param.Lq / ctx->pmsm_param.Ld;
            ctx->pmsm_param.is_ipm = (ctx->pmsm_param.saliency_ratio > 1.05f) ? 1 : 0;

            sub->sm = PMSM_ID_LDQ_COMPLETE;
            ctl_clear_state_seq(&ctx->seq, 0);
        }
        break;

    default:
        break;
    }
}
#pragma endregion

//
// --- Flux Linkage (FLUX) ---
//

#pragma region FLUX

/**
 * @brief Initializes the Flux Linkage identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_flux(ctl_pmsm_offline_id_t* ctx)
{
    ctx->sub_flux.sm = PMSM_ID_FLUX_INIT;
    ctx->sub_flux.is_first_entry = 1;

    ctx->sub_flux.tick_timer = 0;
    ctx->sub_flux.step_idx = 0;
    ctx->sub_flux.target_w_pu = float2ctrl(0.0f);
}

/**
 * @brief ISR step function for Flux Linkage identification.
 * @details Executes V/F ramping, I/F dragging, and pushes averaged data to the DSA Scope.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_flux_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_flux_t* sub = &ctx->sub_flux;
    pmsm_oid_cfg_flux_t* cfg = &sub->cfg;

    // Global action: Continue V/F angle generation and maintain drag current
    if (sub->sm >= PMSM_ID_FLUX_RAMP_SPEED && sub->sm <= PMSM_ID_FLUX_RAMP_STOP)
    {
        ctl_id_step_vf_generator(ctx);
        ctl_id_apply_dc_current(ctx, cfg->if_current_pu, float2ctrl(0.0f));
    }

    switch (sub->sm)
    {
    case PMSM_ID_FLUX_DISABLED:
    case PMSM_ID_FLUX_INIT:
    case PMSM_ID_FLUX_CALCULATE:
    case PMSM_ID_FLUX_COMPLETE:
    case PMSM_ID_FLUX_FAULT:
        break;

    case PMSM_ID_FLUX_RAMP_SPEED:
        if (sub->is_first_entry)
        {
            // Dynamic target calculation (Addition logic instead of multiplication)
            if (sub->step_idx == 0)
            {
                sub->target_w_pu = cfg->min_target_speed_pu;
            }
            else
            {
                sub->target_w_pu += sub->step_size_pu;
            }

            ctl_id_set_vf_target_speed(ctx, sub->target_w_pu);
            sub->is_first_entry = 0;
        }

        // Wait for V/F generator to reach target
        {
            ctrl_gt err = sub->target_w_pu - ctx->vf_gen.current_freq_pu;
            if (err < float2ctrl(0.001f) && err > float2ctrl(-0.001f))
            {
                sub->sm = PMSM_ID_FLUX_SETTLE;
                sub->is_first_entry = 1;
            }
        }
        break;

    case PMSM_ID_FLUX_SETTLE:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            sub->is_first_entry = 0;
        }

        sub->tick_timer++;
        if (sub->tick_timer >= sub->settle_ticks)
        {
            sub->sm = PMSM_ID_FLUX_MEASURE;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_ID_FLUX_MEASURE:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            sub->sum_ud = float2ctrl(0.0f);
            sub->sum_uq = float2ctrl(0.0f);
            sub->sum_id = float2ctrl(0.0f);
            sub->sum_iq = float2ctrl(0.0f);
            sub->sum_w = float2ctrl(0.0f);
            sub->is_first_entry = 0;
        }

        // Accumulate data
        sub->sum_ud += ctx->foc_core.vdq_ref.dat[0];
        sub->sum_uq += ctx->foc_core.vdq_ref.dat[1];
        sub->sum_id += ctx->foc_core.idq0.dat[0];
        sub->sum_iq += ctx->foc_core.idq0.dat[1];
        sub->sum_w += ctx->vf_gen.current_freq_pu;

        sub->tick_timer++;
        if (sub->tick_timer >= cfg->measure_points)
        {
            // Calculate averages
            ctrl_gt avg_ud = ctl_mul(sub->sum_ud, sub->inv_measure_points);
            ctrl_gt avg_uq = ctl_mul(sub->sum_uq, sub->inv_measure_points);
            ctrl_gt avg_id = ctl_mul(sub->sum_id, sub->inv_measure_points);
            ctrl_gt avg_iq = ctl_mul(sub->sum_iq, sub->inv_measure_points);
            ctrl_gt avg_w = ctl_mul(sub->sum_w, sub->inv_measure_points);

            // Direct DSA Memory Write (Dimension 0 to 4)
            // Since this is 5 variables, we bypass the 4ch limit by writing directly to SoA memory
            uint32_t idx = ctx->analyzer.current_idx;
            uint32_t depth = ctx->analyzer.depth;

            if (idx < depth)
            {
                ctl_mem_set_2d_soa(&ctx->analyzer.mem, 0, idx, depth, avg_ud);
                ctl_mem_set_2d_soa(&ctx->analyzer.mem, 1, idx, depth, avg_uq);
                ctl_mem_set_2d_soa(&ctx->analyzer.mem, 2, idx, depth, avg_id);
                ctl_mem_set_2d_soa(&ctx->analyzer.mem, 3, idx, depth, avg_iq);
                ctl_mem_set_2d_soa(&ctx->analyzer.mem, 4, idx, depth, avg_w);
                ctx->analyzer.current_idx++;
            }

            sub->sm = PMSM_ID_FLUX_STEP_EVALUATE;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_ID_FLUX_STEP_EVALUATE:
        if (sub->is_first_entry)
        {
            sub->step_idx++;
            if (sub->step_idx >= cfg->steps)
            {
                sub->sm = PMSM_ID_FLUX_RAMP_STOP;
            }
            else
            {
                sub->sm = PMSM_ID_FLUX_RAMP_SPEED;
            }
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_ID_FLUX_RAMP_STOP:
        if (sub->is_first_entry)
        {
            ctl_id_set_vf_target_speed(ctx, float2ctrl(0.0f));
            sub->is_first_entry = 0;
        }

        if (ctx->vf_gen.current_freq_pu <= float2ctrl(0.005f))
        {
            sub->sm = PMSM_ID_FLUX_CALCULATE;
            sub->is_first_entry = 1;
        }
        break;
    }
}

/**
 * @brief Background loop function for Flux Linkage identification.
 * @details Computes pre-requisites and performs the linear regression (|E| vs W) using the DA.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_flux(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_flux_t* sub = &ctx->sub_flux;
    pmsm_oid_cfg_flux_t* cfg = &sub->cfg;
    uint32_t i;

    // --- Safety Rule: Zero output in passive states ---
    if (sub->sm == PMSM_ID_FLUX_CALCULATE || sub->sm == PMSM_ID_FLUX_COMPLETE || sub->sm == PMSM_ID_FLUX_FAULT)
    {
        ctl_id_disable_output(ctx);
    }

    if (sub->sm == PMSM_ID_FLUX_INIT)
    {
        if (sub->is_first_entry)
        {
            // 1. Pre-calculate ISR constants
            sub->settle_ticks = SEC_TO_TICKS(cfg->settle_time_s, ctx->cfg_basic.isr_freq_hz);

            if (cfg->measure_points > 0)
            {
                sub->inv_measure_points = ctl_div(float2ctrl(1.0f), float2ctrl((float)cfg->measure_points));
            }
            else
            {
                sub->inv_measure_points = float2ctrl(1.0f);
            }

            if (cfg->steps > 1)
            {
                sub->step_size_pu =
                    ctl_div((cfg->max_target_speed_pu - cfg->min_target_speed_pu), float2ctrl((float)(cfg->steps - 1)));
            }
            else
            {
                sub->step_size_pu = float2ctrl(0.0f);
            }

            // 2. Configure FOC core for I/F mode
            ctl_id_route_foc_angle(ctx, PMSM_ID_ANGLE_SRC_VF_GEN);
            ctl_enable_mtr_current_ctrl(&ctx->foc_core);
            ctx->foc_core.flag_enable_decouple = 0;
            ctx->foc_core.flag_enable_vdq_feedforward = 0;

            ctl_clear_slope_f_pu(&ctx->vf_gen);

            // 3. Configure DSA Scope
            // Dimensions = 6 (Ud, Uq, Id, Iq, W, |E|). Divider = 1 (Manual trigger).
            ctl_wipe_dsa_scope_memory(&ctx->analyzer);
            ctl_config_dsa_scope(&ctx->analyzer, 6, 1);

            sub->is_first_entry = 0;
            sub->sm = PMSM_ID_FLUX_RAMP_SPEED;
            sub->is_first_entry = 1;
        }
    }
    else if (sub->sm == PMSM_ID_FLUX_CALCULATE)
    {
        if (sub->is_first_entry)
        {
            // 1. Retrieve identified parameters from previous stages (In PU)
            parameter_gt rs_pu = ctx->pmsm_param.Rs / ctx->identified_pu.Z_base;
            parameter_gt ld_pu = ctx->pmsm_param.Ld / ctx->identified_pu.L_base;
            parameter_gt lq_pu = ctx->pmsm_param.Lq / ctx->identified_pu.L_base;

            uint32_t depth = ctx->analyzer.depth;

            // 2. Compute Back-EMF magnitude for each speed step
            for (i = 0; i < cfg->steps && i < depth; i++)
            {
                parameter_gt ud = (parameter_gt)ctl_mem_get_2d_soa(&ctx->analyzer.mem, 0, i, depth);
                parameter_gt uq = (parameter_gt)ctl_mem_get_2d_soa(&ctx->analyzer.mem, 1, i, depth);
                parameter_gt id = (parameter_gt)ctl_mem_get_2d_soa(&ctx->analyzer.mem, 2, i, depth);
                parameter_gt iq = (parameter_gt)ctl_mem_get_2d_soa(&ctx->analyzer.mem, 3, i, depth);
                parameter_gt w = (parameter_gt)ctl_mem_get_2d_soa(&ctx->analyzer.mem, 4, i, depth);

                // E_d = U_d - R_s * I_d + W * L_q * I_q
                parameter_gt ed = ud - (rs_pu * id) + (w * lq_pu * iq);
                // E_q = U_q - R_s * I_q - W * L_d * I_d
                parameter_gt eq = uq - (rs_pu * iq) - (w * ld_pu * id);

                // Magnitude |E| = sqrt(Ed^2 + Eq^2)
                parameter_gt e_mag = sqrtf((ed * ed) + (eq * eq));

                // Save |E| to Dimension 5
                ctl_mem_set_2d_soa(&ctx->analyzer.mem, 5, i, depth, float2ctrl(e_mag));
            }

            // 3. Perform Linear Regression: Dimension 5 (|E|) against Dimension 4 (W)
            parameter_gt flux_pu = 0.0f, intercept = 0.0f;
            fast_gt fit_ok = ctl_dsa_fit_vs_dim(&ctx->analyzer, 4, 5, 0, cfg->steps - 1, &flux_pu, &intercept);

            if (!fit_ok)
            {
                sub->sm = PMSM_ID_FLUX_FAULT;
                return;
            }

            // 4. Convert PU Flux to Physical Units (Weber)
            parameter_gt flux_base = ctx->identified_pu.V_base / ctx->identified_pu.W_base;
            ctx->pmsm_param.flux_linkage = flux_pu * flux_base;
            ctx->identified_pu.Flux_base = float2ctrl(flux_base);

            // 5. Calculate Motor Characteristic Current (if IPM)
            if (ctx->pmsm_param.is_ipm)
            {
                parameter_gt delta_l = ctx->pmsm_param.Lq - ctx->pmsm_param.Ld;
                if (delta_l > 0.0001f)
                {
                    ctx->pmsm_param.char_current = ctx->pmsm_param.flux_linkage / delta_l;
                }
            }
            else
            {
                ctx->pmsm_param.char_current = 9999.0f;
            }

            ctl_wipe_dsa_scope_memory(&ctx->analyzer);

            sub->is_first_entry = 0;
            sub->sm = PMSM_ID_FLUX_COMPLETE;
        }
    }
}

#pragma endregion

//
// --- Mechanical Parameters (MECH) ---
//

#pragma region MECH

/**
 * @brief Initializes the Mechanical Parameters identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_mech(ctl_pmsm_offline_id_t* ctx)
{
    ctx->sub_mech.sm = PMSM_ID_MECH_INIT;
    ctx->sub_mech.is_first_entry = 1;

    ctx->sub_mech.tick_timer = 0;
    ctx->sub_mech.active_iq_ref_pu = float2ctrl(0.0f);
    ctx->sub_mech.active_id_ref_pu = float2ctrl(0.0f);
}

/**
 * @brief ISR step function for Mechanical Parameters identification.
 * @details Highly optimized state machine managing V/F start, closed-loop handover, 
 * localized speed control, and dual-curve high-speed recording via DSA Scope.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_mech_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_mech_t* sub = &ctx->sub_mech;
    pmsm_oid_cfg_mech_t* cfg = &sub->cfg;

    // 始终更新 V/F 发生器以备随时切回
    ctl_step_slope_f_pu(&ctx->vf_gen);
    ctrl_gt current_speed_pu = ctx->foc_core.spd_if->speed;

    switch (sub->sm)
    {
    case PMSM_ID_MECH_DISABLED:
    case PMSM_ID_MECH_INIT:
    case PMSM_ID_MECH_CALCULATE:
    case PMSM_ID_MECH_COMPLETE:
    case PMSM_ID_MECH_FAULT:
        break; // Loop handles these.

    // --- (IF_START 和 HANDOVER_TO_CLOSED 保持你提供的完美逻辑，此处略写以突出核心) ---
    case PMSM_ID_MECH_IF_START:
        if (sub->is_first_entry)
        {
            ctl_id_set_vf_target_speed(ctx, cfg->low_speed_pu);
            ctl_id_apply_dc_current(ctx, cfg->if_current_pu, float2ctrl(0.0f));
            sub->is_first_entry = 0;
        }
        if (ctx->vf_gen.current_freq_pu >= cfg->low_speed_pu)
        {
            sub->sm = PMSM_ID_MECH_HANDOVER_TO_CLOSED;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_ID_MECH_HANDOVER_TO_CLOSED:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            ctl_trigger_angle_transition(&ctx->angle_switcher, 1);
            sub->is_first_entry = 0;
        }
        {
            ctrl_gt w = ctx->angle_switcher.weight;
            sub->active_id_ref_pu = ctl_mul(cfg->if_current_pu, float2ctrl(1.0f) - w);
            sub->active_iq_ref_pu = ctl_mul(cfg->low_speed_pu, w);
            ctl_id_apply_dc_current(ctx, sub->active_id_ref_pu, sub->active_iq_ref_pu);
        }
        sub->tick_timer++;
        if (sub->tick_timer >= sub->transition_ticks)
        {
            sub->sm = PMSM_ID_MECH_STEADY_LOW;
            sub->is_first_entry = 1;
        }
        break;

    // -------------------------------------------------------------
    // Steady State (Low & High) for Friction Measurement
    // -------------------------------------------------------------
    case PMSM_ID_MECH_STEADY_LOW:
    case PMSM_ID_MECH_STEADY_HIGH:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            sub->sum_iq_steady = float2ctrl(0.0f);
            sub->is_first_entry = 0;
        }

        // Mini I-Controller for Speed Hold
        {
            ctrl_gt target_spd = (sub->sm == PMSM_ID_MECH_STEADY_LOW) ? cfg->low_speed_pu : cfg->high_speed_pu;
            ctrl_gt err = target_spd - current_speed_pu;
            sub->active_iq_ref_pu += ctl_mul(err, float2ctrl(0.001f)); // Soft I-gain
            sub->active_iq_ref_pu = ctl_sat(sub->active_iq_ref_pu, float2ctrl(0.5f), float2ctrl(-0.5f));
            ctl_id_apply_dc_current(ctx, float2ctrl(0.0f), sub->active_iq_ref_pu);
        }

        sub->sum_iq_steady += ctx->foc_core.idq0.dat[1];

        sub->tick_timer++;
        if (sub->tick_timer >= sub->settle_ticks)
        {
            parameter_gt avg_iq = (parameter_gt)ctl_mul(sub->sum_iq_steady, sub->inv_settle_ticks);

            if (sub->sm == PMSM_ID_MECH_STEADY_LOW)
            {
                sub->iq_steady_low_pu = avg_iq;
                sub->sm = PMSM_ID_MECH_ACCEL_TEST;
            }
            else
            {
                sub->iq_steady_high_pu = avg_iq;
                sub->sm = PMSM_ID_MECH_DECEL_TEST;
            }
            sub->is_first_entry = 1;
        }
        break;

    // -------------------------------------------------------------
    // Acceleration Test
    // -------------------------------------------------------------
    case PMSM_ID_MECH_ACCEL_TEST:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            ctl_id_apply_dc_current(ctx, float2ctrl(0.0f), cfg->accel_iq_pu);

            // 记录加速段在 DA 内存中的起始索引
            sub->da_idx_accel_start = ctx->analyzer.current_idx;
            sub->is_first_entry = 0;
        }

        // DSA Integration: Push 1-channel data (Speed)
        ctl_step_dsa_scope_1ch(&ctx->analyzer, current_speed_pu);

        sub->tick_timer++;
        // If speed reached OR DA is full (prevention of overwrite)
        if (current_speed_pu >= cfg->high_speed_pu || ctx->analyzer.current_idx >= ctx->analyzer.depth)
        {
            // 记录结束索引
            sub->da_idx_accel_end = (ctx->analyzer.current_idx > 0) ? ctx->analyzer.current_idx - 1 : 0;

            sub->active_iq_ref_pu = cfg->accel_iq_pu; // Handover initial value for STEADY_HIGH PI
            sub->sm = PMSM_ID_MECH_STEADY_HIGH;
            sub->is_first_entry = 1;
        }
        break;

    // -------------------------------------------------------------
    // Deceleration Test
    // -------------------------------------------------------------
    case PMSM_ID_MECH_DECEL_TEST:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            ctl_id_apply_dc_current(ctx, float2ctrl(0.0f), cfg->decel_iq_pu);

            // 记录减速段在 DA 内存中的起始索引
            sub->da_idx_decel_start = ctx->analyzer.current_idx;
            sub->is_first_entry = 0;
        }

        // OVER-VOLTAGE PROTECTION (CRITICAL)
        if (ctx->foc_core.udc > cfg->max_vbus_pu)
        {
            ctl_id_disable_output(ctx);
            sub->sm = PMSM_ID_MECH_FAULT;
            sub->is_first_entry = 1;
            break;
        }

        // DSA Integration: Push 1-channel data (Speed)
        ctl_step_dsa_scope_1ch(&ctx->analyzer, current_speed_pu);

        sub->tick_timer++;
        if (current_speed_pu <= cfg->low_speed_pu || ctx->analyzer.current_idx >= ctx->analyzer.depth)
        {
            // 记录结束索引
            sub->da_idx_decel_end = (ctx->analyzer.current_idx > 0) ? ctx->analyzer.current_idx - 1 : 0;

            sub->sm = PMSM_ID_MECH_HANDOVER_TO_IF;
            sub->is_first_entry = 1;
        }
        break;

    // --- (HANDOVER_TO_IF 和 IF_STOP 保持原逻辑) ---
    case PMSM_ID_MECH_HANDOVER_TO_IF:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            ctl_id_set_vf_target_speed(ctx, current_speed_pu);
            ctx->vf_gen.current_freq_pu = current_speed_pu;
            ctl_trigger_angle_transition(&ctx->angle_switcher, 0);
            sub->is_first_entry = 0;
        }
        {
            ctrl_gt w = ctx->angle_switcher.weight;
            sub->active_id_ref_pu = ctl_mul(cfg->if_current_pu, float2ctrl(1.0f) - w);
            sub->active_iq_ref_pu = ctl_mul(sub->active_iq_ref_pu, w);
            ctl_id_apply_dc_current(ctx, sub->active_id_ref_pu, sub->active_iq_ref_pu);
        }
        sub->tick_timer++;
        if (sub->tick_timer >= sub->transition_ticks)
        {
            sub->sm = PMSM_ID_MECH_IF_STOP;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_ID_MECH_IF_STOP:
        if (sub->is_first_entry)
        {
            ctl_id_set_vf_target_speed(ctx, float2ctrl(0.0f));
            sub->is_first_entry = 0;
        }
        if (ctx->vf_gen.current_freq_pu <= float2ctrl(0.005f))
        {
            sub->sm = PMSM_ID_MECH_CALCULATE;
            sub->is_first_entry = 1;
        }
        break;
    }
}

/**
 * @brief Background loop function for Mechanical Parameters identification.
 * @details Safely executes pre-calculations and dual-curve linear regressions.
 * MUST be called from a low-priority task or main loop.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_mech(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_mech_t* sub = &ctx->sub_mech;
    pmsm_oid_cfg_mech_t* cfg = &sub->cfg;

    // --- Safety Rule ---
    if (sub->sm == PMSM_ID_MECH_CALCULATE || sub->sm == PMSM_ID_MECH_COMPLETE || sub->sm == PMSM_ID_MECH_FAULT)
    {
        ctl_id_disable_output(ctx);
    }

    if (sub->sm == PMSM_ID_MECH_INIT)
    {
        if (sub->is_first_entry)
        {
            // 1. Pre-calculate ISR constants
            sub->dt_sec = 1.0f / ctx->cfg_basic.isr_freq_hz;
            sub->settle_ticks = SEC_TO_TICKS(cfg->settle_time_s, ctx->cfg_basic.isr_freq_hz);
            // 补充 transition_ticks 的安全计算
            sub->transition_ticks = SEC_TO_TICKS(0.5f, ctx->cfg_basic.isr_freq_hz); // default 0.5s handover
            sub->inv_settle_ticks = ctl_div(float2ctrl(1.0f), float2ctrl((float)sub->settle_ticks));

            // 2. Configure FOC core and Angle Switcher
            ctx->foc_core.flag_enable_decouple = 0;
            ctx->foc_core.flag_enable_vdq_feedforward = 0;
            ctl_init_angle_switcher(&ctx->angle_switcher, 0.5f, ctx->cfg_basic.isr_freq_hz);
            ctl_attach_angle_switcher(&ctx->angle_switcher, &ctx->vf_gen.enc, ctx->enc);
            ctx->foc_core.pos_if = &ctx->angle_switcher.out_enc;

            // 3. Configure DSA Scope for MECH
            // Estimate max duration for BOTH accel and decel (e.g., 6.0 seconds total)
            parameter_gt max_duration_s = 6.0f;
            // Calculate necessary divider to prevent buffer overflow (1 Dimension: Speed)
            uint32_t div =
                ctl_dsa_calc_min_divider(ctx->analyzer.mem.capacity, 1, max_duration_s, ctx->cfg_basic.isr_freq_hz);

            ctl_wipe_dsa_scope_memory(&ctx->analyzer);
            ctl_config_dsa_scope(&ctx->analyzer, 1, div);

            sub->is_first_entry = 0;
            sub->sm = PMSM_ID_MECH_IF_START;
            sub->is_first_entry = 1;
        }
    }
    else if (sub->sm == PMSM_ID_MECH_CALCULATE)
    {
        if (sub->is_first_entry)
        {
            parameter_gt alpha_acc_pu_s = 0.0f, initial_w_acc = 0.0f;
            parameter_gt alpha_dec_pu_s = 0.0f, initial_w_dec = 0.0f;

            // 1. Perform Linear Fitting on DA Segments: Speed = alpha * Time + initial_speed
            // 提取加速段角加速度
            ctl_dsa_fit_vs_time(&ctx->analyzer, 0, sub->da_idx_accel_start, sub->da_idx_accel_end, &alpha_acc_pu_s,
                                &initial_w_acc);

            // 提取减速段角加速度
            ctl_dsa_fit_vs_time(&ctx->analyzer, 0, sub->da_idx_decel_start, sub->da_idx_decel_end, &alpha_dec_pu_s,
                                &initial_w_dec);

            // Safety catch for singularities
            if (alpha_acc_pu_s < 0.001f)
                alpha_acc_pu_s = 0.001f;
            if (alpha_dec_pu_s > -0.001f)
                alpha_dec_pu_s = -0.001f;

            // 2. Base Conversions
            parameter_gt I_base = ctx->identified_pu.I_base;
            parameter_gt W_mech_base = ctx->identified_pu.W_base / (parameter_gt)ctx->cfg_basic.pole_pairs;

            // 3. Convert PU data to Physical Units
            parameter_gt alpha_acc_rads2 = alpha_acc_pu_s * W_mech_base;
            parameter_gt alpha_dec_rads2 = alpha_dec_pu_s * W_mech_base;
            parameter_gt w_mech_low = cfg->low_speed_pu * W_mech_base;
            parameter_gt w_mech_high = cfg->high_speed_pu * W_mech_base;

            // 4. Calculate Electromagnetic Torque Constant Kt (Nm/A)
            parameter_gt Kt = 1.5f * (parameter_gt)ctx->pmsm_param.pole_pairs * ctx->pmsm_param.flux_linkage;

            parameter_gt T_acc = Kt * (cfg->accel_iq_pu * I_base);
            parameter_gt T_dec = Kt * (cfg->decel_iq_pu * I_base);
            parameter_gt T_fric_low = Kt * (sub->iq_steady_low_pu * I_base);
            parameter_gt T_fric_high = Kt * (sub->iq_steady_high_pu * I_base);

            // 5. Calculate Inertia J (kg*m^2)
            parameter_gt delta_T = T_acc - T_dec;
            parameter_gt delta_alpha = alpha_acc_rads2 - alpha_dec_rads2;

            if (delta_alpha > 0.001f)
            {
                ctx->pmsm_mech_param.J_total = delta_T / delta_alpha;
            }
            else
            {
                ctx->pmsm_mech_param.J_total = 0.0001f; // Safeguard
            }

            // 6. Calculate Viscous Damping B (Nm / (rad/s))
            parameter_gt delta_T_fric = T_fric_high - T_fric_low;
            parameter_gt delta_w_mech = w_mech_high - w_mech_low;

            if (delta_w_mech > 0.001f)
            {
                ctx->pmsm_mech_param.B_viscous = delta_T_fric / delta_w_mech;
            }
            else
            {
                ctx->pmsm_mech_param.B_viscous = 0.0f;
            }

            // 7. Calculate Derived Mechanical Time Constant
            if (ctx->pmsm_mech_param.B_viscous > 0.00001f)
            {
                ctx->pmsm_mech_param.tau_m = ctx->pmsm_mech_param.J_total / ctx->pmsm_mech_param.B_viscous;
            }
            else
            {
                ctx->pmsm_mech_param.tau_m = 9999.0f;
            }

            // Cleanup memory bounds for next use
            ctl_wipe_dsa_scope_memory(&ctx->analyzer);

            sub->is_first_entry = 0;
            sub->sm = PMSM_ID_MECH_COMPLETE;
        }
    }
}

#pragma endregion

/**
 * @brief High-frequency ISR step function for PMSM Offline Identification.
 * @details Routes execution to the active sub-task's ISR, steps angle switcher, and executes FOC.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
{
    // 1. Safety check: Do nothing if passive.
    if (ctx->sm == PMSM_OFFLINE_ID_DISABLED || ctx->sm == PMSM_OFFLINE_ID_FAULT || ctx->sm == PMSM_OFFLINE_ID_READY ||
        ctx->sm == PMSM_OFFLINE_ID_COMPLETE)
    {
        return;
    }

    // Execute High-Speed Protection Check
    if (ctl_step_mtr_protect_fast(&ctx->protect))
    {
        // Immediate hardware shutdown action should happen here (e.g., disable PWM)
        ctl_disable_mtr_current_ctrl(&ctx->foc_core);
        ctx->sm = PMSM_OFFLINE_ID_FAULT;
        return; // Abort further execution!
    }

    // 2. Dispatch ISR logic
    switch (ctx->sm)
    {
    case PMSM_OFFLINE_ID_PREPARE:
        // Execute user-defined ADC calib or Enc alignment step here
        break;

    case PMSM_OFFLINE_ID_RS_DT:
        ctl_step_oid_rs_dt_isr(ctx);
        if (ctx->sub_rs_dt.sm == PMSM_ID_RSDT_FAULT)
            ctx->sm = PMSM_OFFLINE_ID_FAULT;
        break;

    case PMSM_OFFLINE_ID_LD_LQ:
        ctl_step_oid_ldq_isr(ctx);
        if (ctx->sub_ldq.sm == PMSM_ID_LDQ_FAULT)
            ctx->sm = PMSM_OFFLINE_ID_FAULT;
        break;

    case PMSM_OFFLINE_ID_FLUX:
        ctl_step_oid_flux_isr(ctx);
        if (ctx->sub_flux.sm == PMSM_ID_FLUX_FAULT)
            ctx->sm = PMSM_OFFLINE_ID_FAULT;
        break;

    case PMSM_OFFLINE_ID_MECH:
        ctl_step_oid_mech_isr(ctx);
        if (ctx->sub_mech.sm == PMSM_ID_MECH_FAULT)
            ctx->sm = PMSM_OFFLINE_ID_FAULT;
        break;

    default:
        break;
    }

    // 3. Step Core Embedded Components
    ctl_step_angle_switcher(&ctx->angle_switcher);
    ctl_step_current_controller(&ctx->foc_core);
}

/**
 * @brief Background loop function for PMSM Offline Identification.
 * @details Manages heavy calculations, timeout checking, and state transitions using the Move Next router.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
{
    switch (ctx->sm)
    {
    case PMSM_OFFLINE_ID_DISABLED:
        break;

    case PMSM_OFFLINE_ID_READY:
        // Transition based on a user command (e.g., START_ID flag)
        // if (user_command == START_ID) {
        //     ctx->sm = ctl_oid_get_next_state(ctx, PMSM_OFFLINE_ID_READY);
        //     ctl_oid_init_target_state(ctx);
        // }
        break;

    case PMSM_OFFLINE_ID_PREPARE:
        // Wait for external user logic (ADC Calib / Enc Align) to finish
        // if (user_prepare_done) {
        //     ctx->sm = ctl_oid_get_next_state(ctx, PMSM_OFFLINE_ID_PREPARE);
        //     ctl_oid_init_target_state(ctx);
        // }
        break;

    // ---------------------------------------------------------------------
    // Core Sub-Tasks
    // ---------------------------------------------------------------------
    case PMSM_OFFLINE_ID_RS_DT:
        ctl_loop_oid_rs_dt(ctx);
        if (ctx->sub_rs_dt.sm == PMSM_ID_RSDT_COMPLETE)
        {
            ctx->sm = ctl_oid_get_next_state(ctx, PMSM_OFFLINE_ID_RS_DT);
            ctl_oid_init_target_state(ctx);
        }
        break;

    case PMSM_OFFLINE_ID_LD_LQ:
        ctl_loop_oid_ldq(ctx);
        if (ctx->sub_ldq.sm == PMSM_ID_LDQ_COMPLETE)
        {
            ctx->sm = ctl_oid_get_next_state(ctx, PMSM_OFFLINE_ID_LD_LQ);
            ctl_oid_init_target_state(ctx);
        }
        break;

    case PMSM_OFFLINE_ID_FLUX:
        ctl_loop_oid_flux(ctx);
        if (ctx->sub_flux.sm == PMSM_ID_FLUX_COMPLETE)
        {
            ctx->sm = ctl_oid_get_next_state(ctx, PMSM_OFFLINE_ID_FLUX);
            ctl_oid_init_target_state(ctx);
        }
        break;

    case PMSM_OFFLINE_ID_MECH:
        ctl_loop_oid_mech(ctx);
        if (ctx->sub_mech.sm == PMSM_ID_MECH_COMPLETE)
        {
            ctx->sm = ctl_oid_get_next_state(ctx, PMSM_OFFLINE_ID_MECH);
            ctl_oid_init_target_state(ctx);
        }
        break;

    // ---------------------------------------------------------------------
    // Finalization & Fault
    // ---------------------------------------------------------------------
    case PMSM_OFFLINE_ID_COMPLETE:
        // 1. Safely disable FOC output
        ctl_disable_mtr_current_ctrl(&ctx->foc_core);

        // 2. Format final Consultant Structures if needed
        // ctl_finalize_offline_id_report(ctx);

        // 3. Hold in this state. The host application can read parameters now.
        // It's up to the user to command ctx->sm = PMSM_OFFLINE_ID_READY or DISABLED to restart.
        break;

    case PMSM_OFFLINE_ID_FAULT:
        ctl_disable_mtr_current_ctrl(&ctx->foc_core);
        // Wait for user fault reset
        break;

    default:
        break;
    }
}

/**
 * @brief Initializes the complete PMSM Offline Identification Master State Machine.
 * @param[out] ctx          Pointer to the master offline ID context.
 * @param[in]  init_cfg     Pointer to the configuration "checkup form".
 * @param[in]  dsa_buffer   Pointer to the memory pool for the DSA Scope.
 * @param[in]  dsa_capacity Total capacity of the DSA Scope memory buffer.
 */
void ctl_init_pmsm_offline_id_sm(ctl_pmsm_offline_id_t* ctx, const ctl_pmsm_offline_id_init_t* init_cfg,
                                 ctrl_gt* dsa_buffer, uint32_t dsa_capacity)
{
    // =========================================================================
    // 1. Copy Configurations & Establish Base Values
    // =========================================================================
    ctx->cfg_basic = init_cfg->cfg_basic;

    ctx->sub_rs_dt.cfg = init_cfg->cfg_rs_dt;
    ctx->sub_ldq.cfg = init_cfg->cfg_ld_lq;
    ctx->sub_flux.cfg = init_cfg->cfg_flux;
    ctx->sub_mech.cfg = init_cfg->cfg_mech;

    // Initialize Base Values for PU conversions
    ctl_consultant_pu_pmsm_init(&ctx->identified_pu, init_cfg->v_base, init_cfg->i_base, init_cfg->w_base,
                                init_cfg->cfg_basic.pole_pairs);

    // =========================================================================
    // 2. Initialize Core Embedded Components
    // =========================================================================

    // 2.1 V/F Generator
    ctl_clear_slope_f_pu(&ctx->vf_gen);

    // 2.2 FOC Core Basic Init
    ctl_init_mtr_current_ctrl_basic(&ctx->foc_core, init_cfg->kp, init_cfg->ki, init_cfg->max_vs_pu,
                                    init_cfg->cfg_basic.isr_freq_hz);

    // 2.3 Angle Switcher (Default to 0.5s transition)
    ctl_init_angle_switcher(&ctx->angle_switcher, 0.5f, ctx->cfg_basic.isr_freq_hz);

    // 2.4 DSA Scope (Data Analyzer)
    ctl_init_dsa_scope(&ctx->analyzer, dsa_buffer, dsa_capacity, ctx->cfg_basic.isr_freq_hz);

    // 2.5 Motor Protection Module
    ctl_init_mtr_protect(&ctx->protect, ctx->cfg_basic.isr_freq_hz);

    // --- BINDING PROTECTION PORTS ---
    ctl_attach_mtr_protect_port(&ctx->protect,
                                &ctx->foc_core.udc,                    // Bus voltage
                                (ctl_vector2_t*)(&ctx->foc_core.idq0), // Measured actual Idq
                                &ctx->foc_core.idq_ref,                // Reference target Idq
                                NULL,                                  // Motor Temp (Optional)
                                NULL                                   // Inverter Temp (Optional)
    );

    // Clear protection mask (enable all protections)
    ctl_set_mtr_protect_mask(&ctx->protect, MTR_PROT_NONE);

    // =========================================================================
    // 3. Reset Sub-Process Trackers
    // =========================================================================
    ctx->sub_rs_dt.sm = PMSM_ID_RSDT_DISABLED;
    ctx->sub_ldq.sm = PMSM_ID_LDQ_DISABLED;
    ctx->sub_flux.sm = PMSM_ID_FLUX_DISABLED;
    ctx->sub_mech.sm = PMSM_ID_MECH_DISABLED;

    // Clear parameter structures
    ctx->pmsm_param.Rs = 0.0f;
    ctx->pmsm_param.Ld = 0.0f;
    ctx->pmsm_param.Lq = 0.0f;
    ctx->pmsm_param.flux_linkage = 0.0f;
    ctx->pmsm_param.pole_pairs = init_cfg->cfg_basic.pole_pairs;

    ctx->pmsm_mech_param.J_total = 0.0f;
    ctx->pmsm_mech_param.B_viscous = 0.0f;

    // =========================================================================
    // 4. Set Master State Machine
    // =========================================================================
    ctx->enc = NULL; // Must be explicitly bound by the user later if not sensorless

    // Boot directly into READY state, awaiting the user's START command
    ctx->sm = PMSM_OFFLINE_ID_READY;
}

///**
// * @brief Enables the Offline Identification process.
// * @details Commands the state machine to transition from READY to the first
// * active test stage (PREPARE or otherwise). If the system is not in READY, this is ignored.
// * @param[in,out] ctx Pointer to the master offline ID context.
// */
//GMP_STATIC_INLINE void ctl_enable_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
//{
//    if (ctx->sm == PMSM_OFFLINE_ID_READY)
//    {
//        // Leverage the Move Next router to automatically jump to the first enabled step
//        ctx->sm = ctl_oid_get_next_state(ctx, PMSM_OFFLINE_ID_READY);
//        ctl_oid_init_target_state(ctx);
//    }
//}
//
///**
// * @brief Safely disables the Offline Identification process.
// * @details Immediately turns off the FOC PWM outputs and forces the master state
// * machine into the DISABLED state. Can be used as a soft E-Stop.
// * @param[in,out] ctx Pointer to the master offline ID context.
// */
//GMP_STATIC_INLINE void ctl_disable_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
//{
//    ctl_disable_mtr_current_ctrl(&ctx->foc_core);
//    ctx->sm = PMSM_OFFLINE_ID_DISABLED;
//}

///**
// * @brief Clears the operational state and faults of the Offline Identification module.
// * @details Safely stops the motor, wipes the data analyzer memory, resets protection faults,
// * clears FOC internal integrals, and puts the state machine back into the READY state.
// * @param[in,out] ctx Pointer to the master offline ID context.
// */
//GMP_STATIC_INLINE void ctl_clear_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
//{
//    // 1. Safe Hardware Shutdown
//    ctl_disable_mtr_current_ctrl(&ctx->foc_core);
//    ctl_clear_mtr_current_ctrl(&ctx->foc_core);
//
//    // 2. Reset Core Components
//    ctl_clear_mtr_protect(&ctx->protect);
//    ctl_wipe_dsa_scope_memory(&ctx->analyzer);
//    ctl_clear_slope_f_pu(&ctx->vf_gen);
//
//    // 3. Reset Sub-state machines
//    ctx->sub_rs_dt.sm = PMSM_ID_RSDT_DISABLED;
//    ctx->sub_ldq.sm = PMSM_ID_LDQ_DISABLED;
//    ctx->sub_flux.sm = PMSM_ID_FLUX_DISABLED;
//    ctx->sub_mech.sm = PMSM_ID_MECH_DISABLED;
//
//    // 4. Return to Staging Ground
//    ctx->sm = PMSM_OFFLINE_ID_READY;
//}
