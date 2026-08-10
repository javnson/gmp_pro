#include <gmp_core.h>

#include <ctl/component/motor_control/pmsm_offline_id/pmsm_offline_id_sm.h>

static parameter_gt ctl_oid_wrap_delta(parameter_gt delta)
{
    if (delta > 0.5f)
        delta -= 1.0f;
    else if (delta < -0.5f)
        delta += 1.0f;
    return delta;
}

static parameter_gt ctl_oid_abs(parameter_gt value)
{
    return (value >= 0.0f) ? value : -value;
}

static void ctl_oid_encoder_fault(ctl_pmsm_offline_id_t* ctx, pmsm_offline_id_encoder_fault_t fault)
{
    ctx->sub_encoder.fault = fault;
    ctx->sub_encoder.sm = PMSM_ID_ENCODER_FAULT;
    ctl_id_disable_output(ctx);
    ctl_id_set_pwm_output(ctx, 0);
}

void ctl_init_oid_encoder(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_encoder_t* sub = &ctx->sub_encoder;
    uint32_t i;

    sub->sm = PMSM_ID_ENCODER_INIT;
    sub->fault = PMSM_ID_ENCODER_FAULT_NONE;
    sub->cycle_count = 0U;
    sub->sweep_tick = 0U;
    sub->cycle_ready = 0;
    sub->identified_pole_pairs = 0U;
    sub->encoder_offset_pu = 0.0f;
    sub->cycle_net_motion_pu = float2ctrl(0.0f);
    sub->command_angle_pu = float2ctrl(0.0f);
    for (i = 0U; i < PMSM_ID_ENCODER_MAX_ANCHORS; ++i)
        sub->cycle_motion_pu[i] = 0.0f;
    ctl_clear_state_seq(&ctx->seq, 0U);
}

void ctl_step_oid_encoder_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_encoder_t* sub = &ctx->sub_encoder;
    pmsm_oid_cfg_encoder_t* cfg = &sub->cfg;
    parameter_gt current_position;
    parameter_gt delta;

    (void)ctl_step_state_seq(&ctx->seq);

    if (sub->sm == PMSM_ID_ENCODER_DISABLED || sub->sm == PMSM_ID_ENCODER_INIT ||
        sub->sm == PMSM_ID_ENCODER_CALCULATE || sub->sm == PMSM_ID_ENCODER_COMPLETE ||
        sub->sm == PMSM_ID_ENCODER_FAULT || ctx->enc == NULL)
        return;

    current_position = ctrl2float(ctl_get_encoder_position(ctx->enc));
    delta = ctl_oid_wrap_delta(current_position - ctrl2float(sub->last_position_pu));
    sub->last_position_pu = float2ctrl(current_position);

    if (ctl_oid_abs(delta) > cfg->max_sample_jump_pu)
    {
        ctl_oid_encoder_fault(ctx, PMSM_ID_ENCODER_FAULT_RANDOM_JUMP);
        return;
    }

    if (sub->sm == PMSM_ID_ENCODER_NOISE_CHECK)
    {
        parameter_gt unwrapped = ctrl2float(sub->cycle_net_motion_pu) + delta;
        sub->cycle_net_motion_pu = float2ctrl(unwrapped);
        if (unwrapped < ctrl2float(sub->stationary_min_pu))
            sub->stationary_min_pu = float2ctrl(unwrapped);
        if (unwrapped > ctrl2float(sub->stationary_max_pu))
            sub->stationary_max_pu = float2ctrl(unwrapped);
        return;
    }

    if (sub->sm == PMSM_ID_ENCODER_ALIGN_ZERO || sub->sm == PMSM_ID_ENCODER_ANCHOR_SETTLE)
    {
        ctl_id_set_static_angle(ctx, float2ctrl(0.0f));
        ctl_id_apply_dc_current(ctx, float2ctrl(cfg->align_current_pu), float2ctrl(0.0f));
        if (sub->sm == PMSM_ID_ENCODER_ANCHOR_SETTLE)
            sub->cycle_net_motion_pu += float2ctrl(delta);
        return;
    }

    if (sub->sm == PMSM_ID_ENCODER_SWEEP)
    {
        parameter_gt command = ctrl2float(sub->command_angle_pu);
        parameter_gt step = cfg->sweep_elec_hz / ctx->cfg_basic.isr_freq_hz;

        sub->cycle_net_motion_pu += float2ctrl(delta);
        if (!sub->cycle_ready)
        {
            command += step;
            sub->sweep_tick++;
            if (command >= 1.0f)
            {
                command = 0.0f;
                sub->cycle_ready = 1;
            }
            else if (sub->sweep_tick >= sub->sweep_timeout_ticks)
            {
                ctl_oid_encoder_fault(ctx, PMSM_ID_ENCODER_FAULT_TIMEOUT);
                return;
            }
            sub->command_angle_pu = float2ctrl(command);
        }
        ctl_id_set_static_angle(ctx, sub->command_angle_pu);
        ctl_id_apply_dc_current(ctx, float2ctrl(cfg->align_current_pu), float2ctrl(0.0f));
    }
}

void ctl_loop_oid_encoder(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_encoder_t* sub = &ctx->sub_encoder;
    pmsm_oid_cfg_encoder_t* cfg = &sub->cfg;
    ctl_state_seq_e phase = ctl_loop_state_seq(&ctx->seq);
    parameter_gt current_position = (ctx->enc != NULL) ? ctrl2float(ctl_get_encoder_position(ctx->enc)) : 0.0f;

    switch (sub->sm)
    {
    case PMSM_ID_ENCODER_INIT:
        if (phase == CTL_ST_FIRST_ENTRY)
        {
            if (ctx->enc == NULL || ctx->cfg_basic.is_sensorless || cfg->align_current_pu <= 0.0f ||
                cfg->sweep_elec_hz <= 0.0f || cfg->max_sample_jump_pu <= 0.0f ||
                cfg->max_pole_pairs == 0U || cfg->max_pole_pairs >= PMSM_ID_ENCODER_MAX_ANCHORS)
            {
                ctl_oid_encoder_fault(ctx, PMSM_ID_ENCODER_FAULT_INVALID_CONFIG);
                break;
            }
            sub->noise_ticks = SEC_TO_TICKS(cfg->noise_check_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->align_ticks = SEC_TO_TICKS(cfg->align_settle_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->anchor_ticks = SEC_TO_TICKS(cfg->anchor_settle_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->sweep_timeout_ticks =
                (uint32_t)(2.0f * ctx->cfg_basic.isr_freq_hz / cfg->sweep_elec_hz) + 1U;
            sub->last_position_pu = float2ctrl(current_position);
            sub->cycle_net_motion_pu = float2ctrl(0.0f);
            sub->stationary_min_pu = float2ctrl(0.0f);
            sub->stationary_max_pu = float2ctrl(0.0f);
            ctl_id_disable_output(ctx);
            ctl_id_set_pwm_output(ctx, 0);
            sub->sm = PMSM_ID_ENCODER_NOISE_CHECK;
            ctl_clear_state_seq(&ctx->seq, sub->noise_ticks);
        }
        break;

    case PMSM_ID_ENCODER_NOISE_CHECK:
        if (phase == CTL_ST_LEAVE)
        {
            parameter_gt span = ctrl2float(sub->stationary_max_pu - sub->stationary_min_pu);
            if (span > cfg->max_stationary_span_pu)
            {
                ctl_oid_encoder_fault(ctx, PMSM_ID_ENCODER_FAULT_RANDOM_JUMP);
                break;
            }
            ctl_id_set_pwm_output(ctx, 1);
            ctl_id_set_foc_state(ctx, PMSM_ID_CURRENT_CLOSELOOP);
            ctl_id_route_foc_angle(ctx, PMSM_ID_ANGLE_SRC_STATIC);
            ctl_id_set_static_angle(ctx, float2ctrl(0.0f));
            sub->last_position_pu = float2ctrl(current_position);
            sub->sm = PMSM_ID_ENCODER_ALIGN_ZERO;
            ctl_clear_state_seq(&ctx->seq, sub->align_ticks);
        }
        break;

    case PMSM_ID_ENCODER_ALIGN_ZERO:
        if (phase == CTL_ST_LEAVE)
        {
            sub->first_anchor_pu = float2ctrl(current_position);
            sub->encoder_offset_pu = current_position;
            sub->last_position_pu = float2ctrl(current_position);
            sub->cycle_net_motion_pu = float2ctrl(0.0f);
            sub->command_angle_pu = float2ctrl(0.0f);
            sub->sweep_tick = 0U;
            sub->cycle_ready = 0;
            sub->sm = PMSM_ID_ENCODER_SWEEP;
            ctl_clear_state_seq(&ctx->seq, GMP_PORT_TIME_MAXIMUM);
        }
        break;

    case PMSM_ID_ENCODER_SWEEP:
        if (sub->cycle_ready)
        {
            sub->sm = PMSM_ID_ENCODER_ANCHOR_SETTLE;
            ctl_clear_state_seq(&ctx->seq, sub->anchor_ticks);
        }
        break;

    case PMSM_ID_ENCODER_ANCHOR_SETTLE:
        if (phase == CTL_ST_LEAVE)
        {
            parameter_gt motion = ctrl2float(sub->cycle_net_motion_pu);
            parameter_gt return_error = ctl_oid_abs(ctl_oid_wrap_delta(current_position -
                                                        ctrl2float(sub->first_anchor_pu)));
            if (ctl_oid_abs(motion) < cfg->min_cycle_motion_pu)
            {
                ctl_oid_encoder_fault(ctx, PMSM_ID_ENCODER_FAULT_NO_MOTION);
                break;
            }
            sub->cycle_motion_pu[sub->cycle_count] = motion;
            sub->cycle_count++;

            if (return_error <= cfg->zero_return_tolerance_pu)
            {
                sub->sm = PMSM_ID_ENCODER_CALCULATE;
                ctl_clear_state_seq(&ctx->seq, 0U);
            }
            else if (sub->cycle_count >= cfg->max_pole_pairs)
            {
                ctl_oid_encoder_fault(ctx, PMSM_ID_ENCODER_FAULT_ZERO_RETURN);
            }
            else
            {
                sub->last_position_pu = float2ctrl(current_position);
                sub->cycle_net_motion_pu = float2ctrl(0.0f);
                sub->command_angle_pu = float2ctrl(0.0f);
                sub->sweep_tick = 0U;
                sub->cycle_ready = 0;
                sub->sm = PMSM_ID_ENCODER_SWEEP;
                ctl_clear_state_seq(&ctx->seq, GMP_PORT_TIME_MAXIMUM);
            }
        }
        break;

    case PMSM_ID_ENCODER_CALCULATE:
        if (phase == CTL_ST_FIRST_ENTRY)
        {
            parameter_gt mean = 0.0f;
            uint32_t i;
            for (i = 0U; i < sub->cycle_count; ++i)
                mean += sub->cycle_motion_pu[i];
            mean /= (parameter_gt)sub->cycle_count;
            for (i = 0U; i < sub->cycle_count; ++i)
            {
                if (ctl_oid_abs(sub->cycle_motion_pu[i] - mean) > cfg->max_cycle_deviation_pu)
                {
                    ctl_oid_encoder_fault(ctx, PMSM_ID_ENCODER_FAULT_NONUNIFORM);
                    return;
                }
            }
            sub->identified_pole_pairs = (uint16_t)sub->cycle_count;
            ctx->cfg_basic.pole_pairs = (parameter_gt)sub->identified_pole_pairs;
            ctx->pmsm_param.pole_pairs = sub->identified_pole_pairs;
            ctl_id_commit_encoder_calibration(ctx, sub->identified_pole_pairs,
                                              float2ctrl(sub->encoder_offset_pu));
            ctl_id_disable_output(ctx);
            sub->sm = PMSM_ID_ENCODER_COMPLETE;
            ctl_clear_state_seq(&ctx->seq, 0U);
        }
        break;

    default:
        break;
    }
}

static fast_gt ctl_oid_fit_alpha_vs_speed(ctl_dsa_scope_t* scope, uint32_t start_idx, uint32_t end_idx,
                                           parameter_gt mech_speed_base, uint32_t min_samples,
                                           parameter_gt* slope, parameter_gt* intercept, parameter_gt* r2)
{
    uint32_t i;
    uint32_t n;
    parameter_gt sum_ii = 0.0f, sum_it = 0.0f, sum_tt = 0.0f;
    parameter_gt sum_iy = 0.0f, sum_ty = 0.0f;
    parameter_gt integral_w = 0.0f, sum_y = 0.0f;
    parameter_gt w_prev, w_initial;
    parameter_gt dt = scope->effective_dt_sec;

    if (end_idx <= start_idx + 1U || end_idx >= scope->depth || dt <= 0.0f)
        return 0;
    n = end_idx - start_idx;
    if (n < min_samples)
        return 0;

    w_initial = ctrl2float(ctl_mem_get_2d_soa(&scope->mem, 0U, start_idx, scope->depth)) * mech_speed_base;
    w_prev = w_initial;
    for (i = start_idx + 1U; i <= end_idx; ++i)
    {
        parameter_gt wm = ctrl2float(ctl_mem_get_2d_soa(&scope->mem, 0U, i, scope->depth)) * mech_speed_base;
        parameter_gt time = (parameter_gt)(i - start_idx) * dt;
        parameter_gt y = wm - w_initial;
        integral_w += 0.5f * (w_prev + wm) * dt;
        sum_ii += integral_w * integral_w;
        sum_it += integral_w * time;
        sum_tt += time * time;
        sum_iy += integral_w * y;
        sum_ty += time * y;
        sum_y += y;
        w_prev = wm;
    }

    {
        parameter_gt determinant = sum_ii * sum_tt - sum_it * sum_it;
        parameter_gt mean_y = sum_y / (parameter_gt)n;
        parameter_gt sse = 0.0f, sst = 0.0f;
        if (ctl_oid_abs(determinant) < 1e-12f)
            return 0;
        *slope = (sum_iy * sum_tt - sum_it * sum_ty) / determinant;
        *intercept = (sum_ii * sum_ty - sum_it * sum_iy) / determinant;

        integral_w = 0.0f;
        w_prev = w_initial;
        for (i = start_idx + 1U; i <= end_idx; ++i)
        {
            parameter_gt wm = ctrl2float(ctl_mem_get_2d_soa(&scope->mem, 0U, i, scope->depth)) * mech_speed_base;
            parameter_gt time = (parameter_gt)(i - start_idx) * dt;
            parameter_gt y = wm - w_initial;
            parameter_gt residual;
            integral_w += 0.5f * (w_prev + wm) * dt;
            residual = y - ((*slope) * integral_w + (*intercept) * time);
            sse += residual * residual;
            sst += (y - mean_y) * (y - mean_y);
            w_prev = wm;
        }
        if (sst <= 1e-12f)
            return 0;
        *r2 = 1.0f - sse / sst;
    }
    return 1;
}

void ctl_init_oid_mech(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_mech_t* sub = &ctx->sub_mech;
    sub->sm = PMSM_ID_MECH_INIT;
    sub->elapsed_ticks = 0U;
    sub->load_torque_Nm = 0.0f;
    ctl_clear_state_seq(&ctx->seq, 0U);
}

void ctl_step_oid_mech_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_mech_t* sub = &ctx->sub_mech;

    if (sub->sm == PMSM_ID_MECH_DISABLED || sub->sm == PMSM_ID_MECH_INIT ||
        sub->sm == PMSM_ID_MECH_CALCULATE || sub->sm == PMSM_ID_MECH_COMPLETE ||
        sub->sm == PMSM_ID_MECH_FAULT)
        return;

    sub->elapsed_ticks++;
    if (sub->elapsed_ticks >= sub->timeout_ticks)
    {
        sub->sm = PMSM_ID_MECH_FAULT;
        ctl_id_disable_output(ctx);
        ctl_id_set_pwm_output(ctx, 0);
        return;
    }

    if (sub->sm == PMSM_ID_MECH_ACCEL_TO_WINDOW || sub->sm == PMSM_ID_MECH_ACCEL_RECORD ||
        sub->sm == PMSM_ID_MECH_ACCEL_TO_PWM_OFF)
        ctl_id_apply_dc_current(ctx, float2ctrl(0.0f), float2ctrl(sub->cfg.accel_iq_pu));

    if (sub->sm == PMSM_ID_MECH_ACCEL_RECORD || sub->sm == PMSM_ID_MECH_COAST_RECORD)
        ctl_step_dsa_scope_2ch(&ctx->analyzer, ctl_id_get_speed(ctx), ctl_id_get_idq(ctx, phase_q));
}

void ctl_loop_oid_mech(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_mech_t* sub = &ctx->sub_mech;
    pmsm_oid_cfg_mech_t* cfg = &sub->cfg;
    parameter_gt speed = ctrl2float(ctl_id_get_speed(ctx));
    parameter_gt low = cfg->target_speed_pu * cfg->fit_low_ratio;
    parameter_gt high = cfg->target_speed_pu * cfg->fit_high_ratio;
    parameter_gt pwm_off = cfg->target_speed_pu * cfg->pwm_off_ratio;

    switch (sub->sm)
    {
    case PMSM_ID_MECH_INIT:
        if (ctx->enc == NULL || ctx->cfg_basic.is_sensorless || cfg->target_speed_pu <= 0.0f ||
            cfg->accel_iq_pu <= 0.0f || cfg->fit_low_ratio <= 0.0f ||
            cfg->fit_low_ratio >= cfg->fit_high_ratio || cfg->fit_high_ratio >= cfg->pwm_off_ratio ||
            cfg->pwm_off_ratio > 1.0f || ctx->pmsm_param.flux_linkage <= 0.0f)
        {
            sub->sm = PMSM_ID_MECH_FAULT;
            break;
        }
        sub->timeout_ticks = SEC_TO_TICKS(cfg->max_test_time_s, ctx->cfg_basic.isr_freq_hz);
        sub->elapsed_ticks = 0U;
        ctl_wipe_dsa_scope_memory(&ctx->analyzer);
        ctl_config_dsa_scope(&ctx->analyzer, 2U,
                             ctl_dsa_calc_min_divider(ctx->analyzer.mem.capacity, 2U,
                                                      cfg->record_time_s, ctx->cfg_basic.isr_freq_hz));
        ctl_id_route_foc_angle(ctx, PMSM_ID_ANGLE_SRC_REAL_ENC);
        ctl_id_set_foc_state(ctx, PMSM_ID_CURRENT_CLOSELOOP);
        ctl_id_set_pwm_output(ctx, 1);
        ctl_id_apply_dc_current(ctx, float2ctrl(0.0f), float2ctrl(cfg->accel_iq_pu));
        sub->sm = PMSM_ID_MECH_ACCEL_TO_WINDOW;
        break;

    case PMSM_ID_MECH_ACCEL_TO_WINDOW:
        if (speed >= low)
        {
            sub->da_idx_accel_start = ctx->analyzer.current_idx;
            sub->sm = PMSM_ID_MECH_ACCEL_RECORD;
        }
        break;

    case PMSM_ID_MECH_ACCEL_RECORD:
        if (ctx->analyzer.current_idx >= ctx->analyzer.depth)
            sub->sm = PMSM_ID_MECH_FAULT;
        else if (speed >= high)
        {
            sub->da_idx_accel_end = (ctx->analyzer.current_idx > 0U) ? ctx->analyzer.current_idx - 1U : 0U;
            sub->sm = PMSM_ID_MECH_ACCEL_TO_PWM_OFF;
        }
        break;

    case PMSM_ID_MECH_ACCEL_TO_PWM_OFF:
        if (speed >= pwm_off)
        {
            ctl_id_disable_output(ctx);
            ctl_id_set_pwm_output(ctx, 0);
            sub->sm = PMSM_ID_MECH_COAST_TO_WINDOW;
        }
        break;

    case PMSM_ID_MECH_COAST_TO_WINDOW:
        if (speed <= high)
        {
            sub->da_idx_decel_start = ctx->analyzer.current_idx;
            sub->sm = PMSM_ID_MECH_COAST_RECORD;
        }
        break;

    case PMSM_ID_MECH_COAST_RECORD:
        if (ctx->analyzer.current_idx >= ctx->analyzer.depth)
            sub->sm = PMSM_ID_MECH_FAULT;
        else if (speed <= low)
        {
            sub->da_idx_decel_end = (ctx->analyzer.current_idx > 0U) ? ctx->analyzer.current_idx - 1U : 0U;
            sub->sm = PMSM_ID_MECH_CALCULATE;
            ctl_clear_state_seq(&ctx->seq, 0U);
        }
        break;

    case PMSM_ID_MECH_CALCULATE:
    {
        parameter_gt mech_speed_base = ctrl2float(ctx->identified_pu.W_base) /
                                       (parameter_gt)ctx->pmsm_param.pole_pairs;
        parameter_gt iq_sum = 0.0f;
        parameter_gt iq_count;
        parameter_gt torque;
        parameter_gt common_slope;
        parameter_gt intercept_delta;
        uint32_t i;
        fast_gt accel_ok = ctl_oid_fit_alpha_vs_speed(&ctx->analyzer, sub->da_idx_accel_start,
                                                       sub->da_idx_accel_end, mech_speed_base,
                                                       cfg->min_fit_samples, &sub->accel_slope,
                                                       &sub->accel_intercept, &sub->accel_r2);
        fast_gt coast_ok = ctl_oid_fit_alpha_vs_speed(&ctx->analyzer, sub->da_idx_decel_start,
                                                       sub->da_idx_decel_end, mech_speed_base,
                                                       cfg->min_fit_samples, &sub->coast_slope,
                                                       &sub->coast_intercept, &sub->coast_r2);
        if (!accel_ok || !coast_ok || sub->accel_r2 < cfg->min_fit_r2 || sub->coast_r2 < cfg->min_fit_r2)
        {
            sub->sm = PMSM_ID_MECH_FAULT;
            break;
        }
        for (i = sub->da_idx_accel_start; i <= sub->da_idx_accel_end; ++i)
            iq_sum += ctrl2float(ctl_mem_get_2d_soa(&ctx->analyzer.mem, 1U, i, ctx->analyzer.depth));
        iq_count = (parameter_gt)(sub->da_idx_accel_end - sub->da_idx_accel_start + 1U);
        sub->average_accel_iq_pu = iq_sum / iq_count;
        torque = 1.5f * (parameter_gt)ctx->pmsm_param.pole_pairs * ctx->pmsm_param.flux_linkage *
                 sub->average_accel_iq_pu * ctrl2float(ctx->identified_pu.I_base);
        intercept_delta = sub->accel_intercept - sub->coast_intercept;
        common_slope = 0.5f * (sub->accel_slope + sub->coast_slope);
        if (torque <= 0.0f || intercept_delta <= 1e-6f || common_slope >= 0.0f)
        {
            sub->sm = PMSM_ID_MECH_FAULT;
            break;
        }
        ctx->pmsm_mech_param.J_total = torque / intercept_delta;
        ctx->pmsm_mech_param.B_viscous = -common_slope * ctx->pmsm_mech_param.J_total;
        ctx->pmsm_mech_param.tau_m = ctx->pmsm_mech_param.J_total / ctx->pmsm_mech_param.B_viscous;
        sub->load_torque_Nm = -sub->coast_intercept * ctx->pmsm_mech_param.J_total;
        ctl_wipe_dsa_scope_memory(&ctx->analyzer);
        sub->sm = PMSM_ID_MECH_COMPLETE;
        break;
    }

    case PMSM_ID_MECH_FAULT:
        ctl_id_disable_output(ctx);
        ctl_id_set_pwm_output(ctx, 0);
        break;

    default:
        break;
    }
}
