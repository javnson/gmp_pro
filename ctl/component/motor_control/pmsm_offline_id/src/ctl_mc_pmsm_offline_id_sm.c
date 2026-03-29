
#include <gmp_core.h>

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
 * Safely freezes data pushing upon reaching LEAVE to prevent asynchronous loop overflow.
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
        switch (seq_phase)
        {
        case CTL_ST_FIRST_ENTRY:
            if (sub->is_measuring_q_axis == 0)
            {
                ctl_id_apply_dc_current(ctx, sub->bias_curr_ref_pu, float2ctrl(0.0f));
            }
            else
            {
                ctl_id_apply_dc_current(ctx, float2ctrl(cfg->align_current_pu), sub->bias_curr_ref_pu);
            }
            break;
        case CTL_ST_KEEP:
        case CTL_ST_LEAVE:
            // Wait for the PI controller to settle
            break;
        }
        break;

    case PMSM_ID_LDQ_PULSE_MEASURE:
        switch (seq_phase)
        {
        case CTL_ST_FIRST_ENTRY:
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
            // Intentionally NO break to record the starting current point!

        case CTL_ST_KEEP: {
            // ONLY push high-speed current points during the active pulse
            ctrl_gt active_i = (sub->is_measuring_q_axis == 0) ? ctl_id_get_idq(ctx, 0) : ctl_id_get_idq(ctx, 1);
            ctl_step_dsa_scope_1ch(&ctx->analyzer, active_i);
            break;
        }

        case CTL_ST_LEAVE:
            // CRITICAL SAFETY WINDOW:
            // Do NOT push to DSA! Stop accumulation.
            // The pulse time is up, hold the voltage output and wait for loop to process the DSA buffer.
            break;
        }
        break;

    case PMSM_ID_LDQ_COOLDOWN:
        switch (seq_phase)
        {
        case CTL_ST_FIRST_ENTRY:
            // Remove pulse, let inductive energy decay safely
            ctl_id_apply_voltage_pulse(ctx, sub->frozen_vd_pu, sub->frozen_vq_pu);
            break;
        case CTL_ST_KEEP:
        case CTL_ST_LEAVE:
            break;
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
            // 1. Setup physical constants
            parameter_gt Z_base = ctx->identified_pu.V_base / ctx->identified_pu.I_base;
            parameter_gt rs_pu = ctx->pmsm_param.Rs / Z_base;

            // 2. Calculate inputs for the First-Order Solver
            // The theoretical steady-state current delta = Voltage Pulse / Resistance
            parameter_gt target_delta_i = cfg->pulse_voltage_pu / rs_pu;

            // Initial current baseline
            parameter_gt baseline_i =
                (sub->is_measuring_q_axis == 0) ? ctrl2float(sub->frozen_id_pu) : ctrl2float(sub->frozen_iq_pu);

            parameter_gt tau = 0.0f;
            uint32_t end_idx = (ctx->analyzer.current_idx > 0) ? ctx->analyzer.current_idx - 1 : 0;

            // 3. Call the generic DSA First-Order Time Constant solver
            fast_gt fit_ok =
                ctl_dsa_fit_first_order_tau(&ctx->analyzer, 0, 0, end_idx, baseline_i, target_delta_i, &tau);

            if (fit_ok)
            {
                // 4. Convert time constant to Inductance: L = tau * R
                parameter_gt L_pu = tau * rs_pu;

                // Store the immediate result
                if (sub->is_measuring_q_axis == 0)
                {
                    sub->ld_array[sub->bias_step_idx] = L_pu;
                }
                else
                {
                    sub->lq_array[sub->bias_step_idx] = L_pu;
                }
            }
            else
            {
                sub->sm = PMSM_ID_LDQ_FAULT; // Matrix or data error
                return;
            }

            // WIPE the DSA Scope ready for the next pulse!
            ctl_wipe_dsa_scope_memory(&ctx->analyzer);

            // TRANSITION -> COOLDOWN
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
    ctx->sub_flux.step_idx = 0;
    ctx->sub_flux.target_w_pu = float2ctrl(0.0f);

    // Arm the sequencer
    ctl_clear_state_seq(&ctx->seq, 0);
}

/**
 * @brief ISR step function for Flux Linkage identification.
 * @details STRICT DATA PATH: Executes V/F ramping, I/F dragging, and data accumulation.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_flux_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_flux_t* sub = &ctx->sub_flux;
    pmsm_oid_cfg_flux_t* cfg = &sub->cfg;

    ctl_state_seq_e seq_phase = ctl_step_state_seq(&ctx->seq);

    // Global Action: During active dynamic states, maintain V/F generator and drag current
    if (sub->sm >= PMSM_ID_FLUX_RAMP_SPEED && sub->sm <= PMSM_ID_FLUX_RAMP_STOP)
    {
        ctl_id_step_vf_generator(ctx);
        ctl_id_apply_dc_current(ctx, float2ctrl(cfg->if_current_pu), float2ctrl(0.0f));
    }

    switch (sub->sm)
    {
    case PMSM_ID_FLUX_DISABLED:
    case PMSM_ID_FLUX_INIT:
    case PMSM_ID_FLUX_SETTLE:
    case PMSM_ID_FLUX_STEP_EVALUATE:
    case PMSM_ID_FLUX_CALCULATE:
    case PMSM_ID_FLUX_COMPLETE:
    case PMSM_ID_FLUX_FAULT:
        break; // Passive or handled purely by loop

    case PMSM_ID_FLUX_RAMP_SPEED:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            if (sub->step_idx == 0)
            {
                sub->target_w_pu = float2ctrl(cfg->min_target_speed_pu);
            }
            else
            {
                sub->target_w_pu += sub->step_size_pu;
            }
            ctl_id_set_vf_target_speed(ctx, sub->target_w_pu);
        }
        break;

    case PMSM_ID_FLUX_MEASURE:
        switch (seq_phase)
        {
        case CTL_ST_FIRST_ENTRY:
            sub->sum_ud = float2ctrl(0.0f);
            sub->sum_uq = float2ctrl(0.0f);
            sub->sum_id = float2ctrl(0.0f);
            sub->sum_iq = float2ctrl(0.0f);
            sub->sum_w = float2ctrl(0.0f);
            // Intentionally NO break here! Accumulate the 0-th point.

        case CTL_ST_KEEP:
            // Fast accumulation: Only happens EXACTLY `measure_points` times.
            sub->sum_ud += ctl_id_get_vdq(ctx, phase_d);
            sub->sum_uq += ctl_id_get_vdq(ctx, phase_q);
            sub->sum_id += ctl_id_get_idq(ctx, phase_d);
            sub->sum_iq += ctl_id_get_idq(ctx, phase_q);
            sub->sum_w += ctx->vf_gen.current_freq_pu;
            break;

        case CTL_ST_LEAVE:
            // CRITICAL SAFETY WINDOW:
            // Stop accumulating! The data is now frozen.
            // We hold here safely until the background loop reads the data and switches the state.
            break;
        }
        break;

    case PMSM_ID_FLUX_RAMP_STOP:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            ctl_id_set_vf_target_speed(ctx, float2ctrl(0.0f));
        }
        break;
    }
}

/**
 * @brief Background loop function for Flux Linkage identification.
 * @details STRICT CONTROL PATH: Manages both time-based and condition-based transitions.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_flux(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_flux_t* sub = &ctx->sub_flux;
    pmsm_oid_cfg_flux_t* cfg = &sub->cfg;
    uint32_t i;

    if (sub->sm == PMSM_ID_FLUX_CALCULATE || sub->sm == PMSM_ID_FLUX_COMPLETE || sub->sm == PMSM_ID_FLUX_FAULT)
    {
        ctl_id_disable_output(ctx);
    }

    ctl_state_seq_e loop_phase = ctl_loop_state_seq(&ctx->seq);

    switch (sub->sm)
    {
    case PMSM_ID_FLUX_INIT:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
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
                    float2ctrl((cfg->max_target_speed_pu - cfg->min_target_speed_pu) / (float)(cfg->steps - 1));
            }
            else
            {
                sub->step_size_pu = float2ctrl(0.0f);
            }

            ctl_id_route_foc_angle(ctx, PMSM_ID_ANGLE_SRC_VF_GEN);
            ctl_id_set_foc_state(ctx, PMSM_ID_CURRENT_CLOSELOOP);
            ctl_clear_slope_f_pu(&ctx->vf_gen);

            ctl_wipe_dsa_scope_memory(&ctx->analyzer);
            ctl_config_dsa_scope(&ctx->analyzer, 6, 1);

            sub->sm = PMSM_ID_FLUX_RAMP_SPEED;
            // Use port maximum time for conditional wait
            ctl_clear_state_seq(&ctx->seq, GMP_PORT_TIME_MAXIMUM);
        }
        break;

    case PMSM_ID_FLUX_RAMP_SPEED: {
        ctrl_gt err = sub->target_w_pu - ctx->vf_gen.current_freq_pu;
        if (err < float2ctrl(0.001f) && err > float2ctrl(-0.001f))
        {
            sub->sm = PMSM_ID_FLUX_SETTLE;
            ctl_clear_state_seq(&ctx->seq, sub->settle_ticks);
        }
    }
    break;

    case PMSM_ID_FLUX_SETTLE:
        if (loop_phase == CTL_ST_LEAVE)
        {
            sub->sm = PMSM_ID_FLUX_MEASURE;
            ctl_clear_state_seq(&ctx->seq, cfg->measure_points);
        }
        break;

    case PMSM_ID_FLUX_MEASURE:
        // Wait for ISR to hit LEAVE. At this exact moment, we know for a fact
        // that ISR has stopped accumulating and the data is perfectly clean.
        if (loop_phase == CTL_ST_LEAVE)
        {
            ctrl_gt avg_ud = ctl_mul(sub->sum_ud, sub->inv_measure_points);
            ctrl_gt avg_uq = ctl_mul(sub->sum_uq, sub->inv_measure_points);
            ctrl_gt avg_id = ctl_mul(sub->sum_id, sub->inv_measure_points);
            ctrl_gt avg_iq = ctl_mul(sub->sum_iq, sub->inv_measure_points);
            ctrl_gt avg_w = ctl_mul(sub->sum_w, sub->inv_measure_points);

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
            ctl_clear_state_seq(&ctx->seq, 0);
        }
        break;

    case PMSM_ID_FLUX_STEP_EVALUATE:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
            sub->step_idx++;
            if (sub->step_idx >= cfg->steps)
            {
                sub->sm = PMSM_ID_FLUX_RAMP_STOP;
                ctl_clear_state_seq(&ctx->seq, GMP_PORT_TIME_MAXIMUM);
            }
            else
            {
                sub->sm = PMSM_ID_FLUX_RAMP_SPEED;
                ctl_clear_state_seq(&ctx->seq, GMP_PORT_TIME_MAXIMUM);
            }
        }
        break;

    case PMSM_ID_FLUX_RAMP_STOP:
        if (ctx->vf_gen.current_freq_pu <= float2ctrl(0.005f))
        {
            sub->sm = PMSM_ID_FLUX_CALCULATE;
            ctl_clear_state_seq(&ctx->seq, 0);
        }
        break;

    case PMSM_ID_FLUX_CALCULATE:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
            parameter_gt Z_base = ctx->identified_pu.V_base / ctx->identified_pu.I_base;
            parameter_gt L_base = Z_base / ctx->identified_pu.W_base;

            parameter_gt rs_pu = ctx->pmsm_param.Rs / Z_base;
            parameter_gt ld_pu = ctx->pmsm_param.Ld / L_base;
            parameter_gt lq_pu = ctx->pmsm_param.Lq / L_base;

            uint32_t depth = ctx->analyzer.depth;

            for (i = 0; i < cfg->steps && i < depth; i++)
            {
                parameter_gt ud = ctrl2float(ctl_mem_get_2d_soa(&ctx->analyzer.mem, 0, i, depth));
                parameter_gt uq = ctrl2float(ctl_mem_get_2d_soa(&ctx->analyzer.mem, 1, i, depth));
                parameter_gt id = ctrl2float(ctl_mem_get_2d_soa(&ctx->analyzer.mem, 2, i, depth));
                parameter_gt iq = ctrl2float(ctl_mem_get_2d_soa(&ctx->analyzer.mem, 3, i, depth));
                parameter_gt w = ctrl2float(ctl_mem_get_2d_soa(&ctx->analyzer.mem, 4, i, depth));

                parameter_gt ed = ud - (rs_pu * id) + (w * lq_pu * iq);
                parameter_gt eq = uq - (rs_pu * iq) - (w * ld_pu * id);

                parameter_gt e_mag = ctl_math_sqrt((ed * ed) + (eq * eq));
                ctl_mem_set_2d_soa(&ctx->analyzer.mem, 5, i, depth, float2ctrl(e_mag));
            }

            parameter_gt flux_pu = 0.0f, intercept = 0.0f;
            fast_gt fit_ok = ctl_dsa_fit_vs_dim(&ctx->analyzer, 4, 5, 0, cfg->steps - 1, &flux_pu, &intercept);

            if (!fit_ok)
            {
                sub->sm = PMSM_ID_FLUX_FAULT;
                return;
            }

            parameter_gt flux_base = ctx->identified_pu.V_base / ctx->identified_pu.W_base;
            ctx->pmsm_param.flux_linkage = flux_pu * flux_base;
            ctx->identified_pu.Flux_base = float2ctrl(flux_base);

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

            sub->sm = PMSM_ID_FLUX_COMPLETE;
            ctl_clear_state_seq(&ctx->seq, 0);
        }
        break;

    default:
        break;
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
    ctx->sub_mech.active_iq_ref_pu = float2ctrl(0.0f);
    ctx->sub_mech.active_id_ref_pu = float2ctrl(0.0f);

    ctl_clear_state_seq(&ctx->seq, 0);
}

/**
 * @brief ISR step function for Mechanical Parameters identification.
 * @details STRICT DATA PATH: Executes closed-loop handovers, localized speed control, 
 * and DSA data pushing. Never changes FSM states.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_mech_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_mech_t* sub = &ctx->sub_mech;
    pmsm_oid_cfg_mech_t* cfg = &sub->cfg;

    // Always step V/F generator to keep the internal angle running
    ctl_id_step_vf_generator(ctx);
    ctrl_gt current_speed_pu = ctx->foc_core.spd_if->speed;

    ctl_state_seq_e seq_phase = ctl_step_state_seq(&ctx->seq);

    switch (sub->sm)
    {
    case PMSM_ID_MECH_DISABLED:
    case PMSM_ID_MECH_INIT:
    case PMSM_ID_MECH_CALCULATE:
    case PMSM_ID_MECH_COMPLETE:
    case PMSM_ID_MECH_FAULT:
        break;

    case PMSM_ID_MECH_IF_START:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            ctl_id_set_vf_target_speed(ctx, float2ctrl(cfg->low_speed_pu));
            ctl_id_apply_dc_current(ctx, float2ctrl(cfg->if_current_pu), float2ctrl(0.0f));
        }
        break;

    case PMSM_ID_MECH_HANDOVER_TO_CLOSED:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            ctl_trigger_angle_transition(&ctx->angle_switcher, 1);
        }
        if (seq_phase == CTL_ST_KEEP || seq_phase == CTL_ST_FIRST_ENTRY)
        {
            ctrl_gt w = ctx->angle_switcher.weight;
            sub->active_id_ref_pu = ctl_mul(float2ctrl(cfg->if_current_pu), float2ctrl(1.0f) - w);
            sub->active_iq_ref_pu = ctl_mul(float2ctrl(cfg->low_speed_pu), w);
            ctl_id_apply_dc_current(ctx, sub->active_id_ref_pu, sub->active_iq_ref_pu);
        }
        break;

    case PMSM_ID_MECH_STEADY_LOW:
    case PMSM_ID_MECH_STEADY_HIGH:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            sub->sum_iq_steady = float2ctrl(0.0f);
        }
        if (seq_phase == CTL_ST_KEEP || seq_phase == CTL_ST_FIRST_ENTRY)
        {
            // Mini I-Controller for Speed Hold
            ctrl_gt target_spd =
                float2ctrl((sub->sm == PMSM_ID_MECH_STEADY_LOW) ? cfg->low_speed_pu : cfg->high_speed_pu);
            ctrl_gt err = target_spd - current_speed_pu;
            sub->active_iq_ref_pu += ctl_mul(err, float2ctrl(0.001f));
            sub->active_iq_ref_pu = ctl_sat(sub->active_iq_ref_pu, float2ctrl(0.5f), float2ctrl(-0.5f));

            ctl_id_apply_dc_current(ctx, float2ctrl(0.0f), sub->active_iq_ref_pu);
            sub->sum_iq_steady += ctl_id_get_idq(ctx, phase_q);
        }
        break;

    case PMSM_ID_MECH_ACCEL_TEST:
    case PMSM_ID_MECH_DECEL_TEST:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            sub->active_iq_ref_pu =
                float2ctrl((sub->sm == PMSM_ID_MECH_ACCEL_TEST) ? cfg->accel_iq_pu : cfg->decel_iq_pu);
            ctl_id_apply_dc_current(ctx, float2ctrl(0.0f), sub->active_iq_ref_pu);
        }
        if (seq_phase == CTL_ST_KEEP || seq_phase == CTL_ST_FIRST_ENTRY)
        {
            // DSA Integration: Push 1-channel data (Speed)
            ctl_step_dsa_scope_1ch(&ctx->analyzer, current_speed_pu);
        }
        // If LEAVE is reached (either by time limit or forced by loop), STOP pushing.
        break;

    case PMSM_ID_MECH_HANDOVER_TO_IF:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            ctl_id_set_vf_target_speed(ctx, current_speed_pu);
            ctx->vf_gen.current_freq_pu = current_speed_pu;
            ctl_trigger_angle_transition(&ctx->angle_switcher, 0);
        }
        if (seq_phase == CTL_ST_KEEP || seq_phase == CTL_ST_FIRST_ENTRY)
        {
            ctrl_gt w = ctx->angle_switcher.weight;
            sub->active_id_ref_pu = ctl_mul(float2ctrl(cfg->if_current_pu), float2ctrl(1.0f) - w);
            sub->active_iq_ref_pu = ctl_mul(sub->active_iq_ref_pu, w);
            ctl_id_apply_dc_current(ctx, sub->active_id_ref_pu, sub->active_iq_ref_pu);
        }
        break;

    case PMSM_ID_MECH_IF_STOP:
        if (seq_phase == CTL_ST_FIRST_ENTRY)
        {
            ctl_id_set_vf_target_speed(ctx, float2ctrl(0.0f));
        }
        break;
    }
}

/**
 * @brief Background loop function for Mechanical Parameters identification.
 * @details STRICT CONTROL PATH: Safely executes pre-calculations, dynamic bounds checking, 
 * and dual-curve linear regressions.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_mech(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_mech_t* sub = &ctx->sub_mech;
    pmsm_oid_cfg_mech_t* cfg = &sub->cfg;

    // Safety Rule
    if (sub->sm == PMSM_ID_MECH_CALCULATE || sub->sm == PMSM_ID_MECH_COMPLETE || sub->sm == PMSM_ID_MECH_FAULT)
    {
        ctl_id_disable_output(ctx);
    }

    ctl_state_seq_e loop_phase = ctl_loop_state_seq(&ctx->seq);
    parameter_gt current_speed_real = ctrl2float(ctl_id_get_speed(ctx));

    switch (sub->sm)
    {
    case PMSM_ID_MECH_INIT:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
            sub->settle_ticks = SEC_TO_TICKS(cfg->settle_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->transition_ticks = SEC_TO_TICKS(cfg->transition_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->inv_settle_ticks = ctl_div(float2ctrl(1.0f), float2ctrl((float)sub->settle_ticks));

            // Configure interfaces
            ctl_id_set_foc_state(ctx, PMSM_ID_CURRENT_CLOSELOOP);
            ctl_init_angle_switcher(&ctx->angle_switcher, cfg->transition_time_s, ctx->cfg_basic.isr_freq_hz);
            ctl_attach_angle_switcher(&ctx->angle_switcher, &ctx->vf_gen.enc, ctx->enc);
            ctl_id_route_foc_angle(ctx, PMSM_ID_ANGLE_SRC_SWITCHER);

            // Configure DSA Scope (Estimate e.g., 6.0 seconds max for dynamic tests)
            uint32_t div = ctl_dsa_calc_min_divider(ctx->analyzer.mem.capacity, 1, 6.0f, ctx->cfg_basic.isr_freq_hz);
            ctl_wipe_dsa_scope_memory(&ctx->analyzer);
            ctl_config_dsa_scope(&ctx->analyzer, 1, div);

            // TRANSITION -> IF_START (Condition-based)
            sub->sm = PMSM_ID_MECH_IF_START;
            ctl_clear_state_seq(&ctx->seq, GMP_PORT_TIME_MAXIMUM);
        }
        break;

    case PMSM_ID_MECH_IF_START:
        if (current_speed_real >= cfg->low_speed_pu)
        {
            sub->sm = PMSM_ID_MECH_HANDOVER_TO_CLOSED;
            ctl_clear_state_seq(&ctx->seq, sub->transition_ticks);
        }
        break;

    case PMSM_ID_MECH_HANDOVER_TO_CLOSED:
        if (loop_phase == CTL_ST_LEAVE)
        {
            sub->sm = PMSM_ID_MECH_STEADY_LOW;
            ctl_clear_state_seq(&ctx->seq, sub->settle_ticks);
        }
        break;

    case PMSM_ID_MECH_STEADY_LOW:
        if (loop_phase == CTL_ST_LEAVE)
        {
            sub->iq_steady_low_pu = ctrl2float(ctl_mul(sub->sum_iq_steady, sub->inv_settle_ticks));

            // Record DA starting index before transition
            sub->da_idx_accel_start = ctx->analyzer.current_idx;

            sub->sm = PMSM_ID_MECH_ACCEL_TEST;
            ctl_clear_state_seq(&ctx->seq, GMP_PORT_TIME_MAXIMUM);
        }
        break;

    case PMSM_ID_MECH_ACCEL_TEST:
        // Conditional Transition: Speed reached OR DA buffer full
        if (current_speed_real >= cfg->high_speed_pu || ctx->analyzer.current_idx >= ctx->analyzer.depth)
        {
            sub->da_idx_accel_end = (ctx->analyzer.current_idx > 0) ? ctx->analyzer.current_idx - 1 : 0;

            sub->sm = PMSM_ID_MECH_STEADY_HIGH;
            ctl_clear_state_seq(&ctx->seq, sub->settle_ticks);
        }
        break;

    case PMSM_ID_MECH_STEADY_HIGH:
        if (loop_phase == CTL_ST_LEAVE)
        {
            sub->iq_steady_high_pu = ctrl2float(ctl_mul(sub->sum_iq_steady, sub->inv_settle_ticks));

            sub->da_idx_decel_start = ctx->analyzer.current_idx;

            sub->sm = PMSM_ID_MECH_DECEL_TEST;
            ctl_clear_state_seq(&ctx->seq, GMP_PORT_TIME_MAXIMUM);
        }
        break;

    case PMSM_ID_MECH_DECEL_TEST:
        // OVER-VOLTAGE PROTECTION (Control Path intercepts and throws FAULT)
        if (ctrl2float(ctl_id_get_udc(ctx)) > cfg->max_vbus_pu)
        {
            sub->sm = PMSM_ID_MECH_FAULT;
            return;
        }

        if (current_speed_real <= cfg->low_speed_pu || ctx->analyzer.current_idx >= ctx->analyzer.depth)
        {
            sub->da_idx_decel_end = (ctx->analyzer.current_idx > 0) ? ctx->analyzer.current_idx - 1 : 0;

            sub->sm = PMSM_ID_MECH_HANDOVER_TO_IF;
            ctl_clear_state_seq(&ctx->seq, sub->transition_ticks);
        }
        break;

    case PMSM_ID_MECH_HANDOVER_TO_IF:
        if (loop_phase == CTL_ST_LEAVE)
        {
            sub->sm = PMSM_ID_MECH_IF_STOP;
            ctl_clear_state_seq(&ctx->seq, GMP_PORT_TIME_MAXIMUM);
        }
        break;

    case PMSM_ID_MECH_IF_STOP:
        if (current_speed_real <= 0.005f)
        {
            sub->sm = PMSM_ID_MECH_CALCULATE;
            ctl_clear_state_seq(&ctx->seq, 0);
        }
        break;

    case PMSM_ID_MECH_CALCULATE:
        if (loop_phase == CTL_ST_FIRST_ENTRY)
        {
            parameter_gt alpha_acc_pu_s = 0.0f, initial_w_acc = 0.0f;
            parameter_gt alpha_dec_pu_s = 0.0f, initial_w_dec = 0.0f;

            // Fit Acceleration
            ctl_dsa_fit_vs_time(&ctx->analyzer, 0, sub->da_idx_accel_start, sub->da_idx_accel_end, &alpha_acc_pu_s,
                                &initial_w_acc);
            // Fit Deceleration
            ctl_dsa_fit_vs_time(&ctx->analyzer, 0, sub->da_idx_decel_start, sub->da_idx_decel_end, &alpha_dec_pu_s,
                                &initial_w_dec);

            if (alpha_acc_pu_s < 0.001f)
                alpha_acc_pu_s = 0.001f;
            if (alpha_dec_pu_s > -0.001f)
                alpha_dec_pu_s = -0.001f;

            parameter_gt I_base = ctrl2float(ctx->identified_pu.I_base);
            parameter_gt W_mech_base = ctrl2float(ctx->identified_pu.W_base) / (parameter_gt)ctx->cfg_basic.pole_pairs;

            parameter_gt alpha_acc_rads2 = alpha_acc_pu_s * W_mech_base;
            parameter_gt alpha_dec_rads2 = alpha_dec_pu_s * W_mech_base;
            parameter_gt w_mech_low = cfg->low_speed_pu * W_mech_base;
            parameter_gt w_mech_high = cfg->high_speed_pu * W_mech_base;

            parameter_gt Kt = 1.5f * (parameter_gt)ctx->pmsm_param.pole_pairs * ctx->pmsm_param.flux_linkage;

            parameter_gt T_acc = Kt * (cfg->accel_iq_pu * I_base);
            parameter_gt T_dec = Kt * (cfg->decel_iq_pu * I_base);
            parameter_gt T_fric_low = Kt * (sub->iq_steady_low_pu * I_base);
            parameter_gt T_fric_high = Kt * (sub->iq_steady_high_pu * I_base);

            parameter_gt delta_T = T_acc - T_dec;
            parameter_gt delta_alpha = alpha_acc_rads2 - alpha_dec_rads2;

            ctx->pmsm_mech_param.J_total = (delta_alpha > 0.001f) ? (delta_T / delta_alpha) : 0.0001f;

            parameter_gt delta_T_fric = T_fric_high - T_fric_low;
            parameter_gt delta_w_mech = w_mech_high - w_mech_low;

            ctx->pmsm_mech_param.B_viscous = (delta_w_mech > 0.001f) ? (delta_T_fric / delta_w_mech) : 0.0f;

            ctx->pmsm_mech_param.tau_m = (ctx->pmsm_mech_param.B_viscous > 0.00001f)
                                             ? (ctx->pmsm_mech_param.J_total / ctx->pmsm_mech_param.B_viscous)
                                             : 9999.0f;

            ctl_wipe_dsa_scope_memory(&ctx->analyzer);

            sub->sm = PMSM_ID_MECH_COMPLETE;
            ctl_clear_state_seq(&ctx->seq, 0);
        }
        break;
    }
}
#pragma endregion

//
// --- Global State machine. ---
//

/**
 * @brief High-frequency ISR step function for PMSM Offline Identification.
 * @details Routes execution to the active sub-task's ISR and steps the angle switcher.
 * NOTE: The external FOC Core and Protection module MUST be stepped independently 
 * in the main motor ISR by the host application.
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

    // 3. Step Core Embedded Components owned by this module
    ctl_step_angle_switcher(&ctx->angle_switcher);
}

/**
 * @brief Background loop function for PMSM Offline Identification.
 * @details Manages heavy calculations, timeout checking, and state transitions.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
{
    switch (ctx->sm)
    {
    case PMSM_OFFLINE_ID_DISABLED:
    case PMSM_OFFLINE_ID_READY:
    case PMSM_OFFLINE_ID_PREPARE:
        // Transition based on a user command handled externally
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
        // 1. Safely disable FOC output via adapter interface
        ctl_id_disable_output(ctx);
        // 2. Hold in this state. The host application can read parameters now.
        break;

    case PMSM_OFFLINE_ID_FAULT:
        // Safely disable FOC output via adapter interface
        ctl_id_disable_output(ctx);
        break;

    default:
        break;
    }
}

/**
 * @brief Initializes the complete PMSM Offline Identification Master State Machine.
 * @details Only initializes the sub-components strictly owned by the Offline ID context.
 * The global FOC core and Protection modules must be initialized by the host application.
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
    // 2. Initialize Core Embedded Components Owned by ID Module
    // =========================================================================

    // 2.1 V/F Generator
    ctl_clear_slope_f_pu(&ctx->vf_gen);

    // 2.2 Angle Switcher (Default to 0.5s transition)
    ctl_init_angle_switcher(&ctx->angle_switcher, 0.5f, ctx->cfg_basic.isr_freq_hz);

    // 2.3 DSA Scope (Data Analyzer)
    ctl_init_dsa_scope(&ctx->analyzer, dsa_buffer, dsa_capacity, ctx->cfg_basic.isr_freq_hz);

    // 2.4 State Sequencer
    ctl_clear_state_seq(&ctx->seq, 0);

    // Note: FOC Core and MTR Protect initialization are explicitly removed from here.
    // They are global external dependencies and should be managed by the system bootloader.

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
    ctx->pmsm_param.is_ipm = 0;

    ctx->pmsm_mech_param.J_total = 0.0f;
    ctx->pmsm_mech_param.B_viscous = 0.0f;

    // =========================================================================
    // 4. Set Master State Machine
    // =========================================================================
    ctx->enc = NULL; // Must be explicitly bound by the user later if not sensorless

    // Boot directly into READY state, awaiting the user's START command
    ctx->sm = PMSM_OFFLINE_ID_READY;
}
