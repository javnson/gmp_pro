
#include <gmp_core.h>

#include <ctl_main.h>

// Offline ID Instances
ctl_pmsm_offline_id_t pmsm_oid;
ctl_pmsm_offline_id_init_t pmsm_oid_cfg;

void init_pmsm_offline_id()
{
    // =========================================================================
    // PMSM Offline ID Configuration
    // =========================================================================
    pmsm_oid_cfg.v_base = CTRL_VOLTAGE_BASE;
    pmsm_oid_cfg.i_base = CTRL_CURRENT_BASE;
    pmsm_oid_cfg.w_base = MOTOR_PARAM_RATED_FREQUENCY * 2.0f * 3.1415926f;

    // Basic execution configuration
    pmsm_oid_cfg.cfg_basic.isr_freq_hz = CONTROLLER_FREQUENCY;
    pmsm_oid_cfg.cfg_basic.pole_pairs = MOTOR_PARAM_POLE_PAIRS;
    pmsm_oid_cfg.cfg_basic.is_sensorless = 0;

    // Identification stage enables
    pmsm_oid_cfg.cfg_basic.flag_enable_prepare = MCS_PMSM_ID_ENABLE_PREPARE;
    pmsm_oid_cfg.cfg_basic.flag_enable_encoder_calibration = MCS_PMSM_ID_ENABLE_ENCODER_CALIBRATION;
    pmsm_oid_cfg.cfg_basic.flag_enable_rs_dt = MCS_PMSM_ID_ENABLE_RS_DT;
    pmsm_oid_cfg.cfg_basic.flag_enable_ldq = MCS_PMSM_ID_ENABLE_LDQ;
    pmsm_oid_cfg.cfg_basic.flag_enable_flux = MCS_PMSM_ID_ENABLE_FLUX;
    pmsm_oid_cfg.cfg_basic.flag_enable_mech_id = MCS_PMSM_ID_ENABLE_MECHANICAL_ID;

    // Identification submodule settings
    // Rs & DT
    pmsm_oid_cfg.cfg_rs_dt.max_current_pu = MCS_PMSM_ID_RSDT_MAX_CURRENT_A / CTRL_CURRENT_BASE;
    pmsm_oid_cfg.cfg_rs_dt.min_current_pu = MCS_PMSM_ID_RSDT_MIN_CURRENT_A / CTRL_CURRENT_BASE;
    pmsm_oid_cfg.cfg_rs_dt.steps = MCS_PMSM_ID_RSDT_STEPS;
    pmsm_oid_cfg.cfg_rs_dt.align_time_s = MCS_PMSM_ID_RSDT_ALIGN_TIME_S;
    pmsm_oid_cfg.cfg_rs_dt.measure_delay_s = MCS_PMSM_ID_RSDT_MEASURE_DELAY_S;
    pmsm_oid_cfg.cfg_rs_dt.measure_points = MCS_PMSM_ID_RSDT_MEASURE_POINTS;

    // Ld & Lq
    pmsm_oid_cfg.cfg_ld_lq.pulse_voltage_pu = MCS_PMSM_ID_LDQ_PULSE_VOLTAGE_V / CTRL_VOLTAGE_BASE;
    pmsm_oid_cfg.cfg_ld_lq.max_bias_curr_pu = MCS_PMSM_ID_LDQ_MAX_BIAS_CURRENT_A / CTRL_CURRENT_BASE;
    pmsm_oid_cfg.cfg_ld_lq.bias_steps = MCS_PMSM_ID_LDQ_BIAS_STEPS;
    pmsm_oid_cfg.cfg_ld_lq.align_current_pu = MCS_PMSM_ID_LDQ_ALIGN_CURRENT_A / CTRL_CURRENT_BASE;
    pmsm_oid_cfg.cfg_ld_lq.settle_time_s = MCS_PMSM_ID_LDQ_SETTLE_TIME_S;
    pmsm_oid_cfg.cfg_ld_lq.pulse_time_s = MCS_PMSM_ID_LDQ_PULSE_TIME_S;
    pmsm_oid_cfg.cfg_ld_lq.cooldown_time_s = MCS_PMSM_ID_LDQ_COOLDOWN_TIME_S;

    // Flux Linkage
    pmsm_oid_cfg.cfg_flux.min_target_speed_pu = MCS_PMSM_ID_FLUX_MIN_SPEED_RPM / MOTOR_PARAM_MAX_SPEED;
    pmsm_oid_cfg.cfg_flux.max_target_speed_pu = MCS_PMSM_ID_FLUX_MAX_SPEED_RPM / MOTOR_PARAM_MAX_SPEED;
    pmsm_oid_cfg.cfg_flux.steps = MCS_PMSM_ID_FLUX_STEPS;
    pmsm_oid_cfg.cfg_flux.if_current_pu = MCS_PMSM_ID_FLUX_IF_CURRENT_A / CTRL_CURRENT_BASE;
    pmsm_oid_cfg.cfg_flux.settle_time_s = MCS_PMSM_ID_FLUX_SETTLE_TIME_S;
    pmsm_oid_cfg.cfg_flux.measure_points = MCS_PMSM_ID_FLUX_MEASURE_POINTS;

    // Mechanical identification (real encoder + real current loop)
    pmsm_oid_cfg.cfg_mech.target_speed_pu = MCS_PMSM_ID_MECH_TARGET_SPEED_RPM / MOTOR_PARAM_MAX_SPEED;
    pmsm_oid_cfg.cfg_mech.fit_low_ratio = MCS_PMSM_ID_MECH_FIT_LOW_RATIO;
    pmsm_oid_cfg.cfg_mech.fit_high_ratio = MCS_PMSM_ID_MECH_FIT_HIGH_RATIO;
    pmsm_oid_cfg.cfg_mech.pwm_off_ratio = MCS_PMSM_ID_MECH_PWM_OFF_RATIO;
    pmsm_oid_cfg.cfg_mech.accel_iq_pu = MCS_PMSM_ID_MECH_ACCEL_CURRENT_A / CTRL_CURRENT_BASE;
    pmsm_oid_cfg.cfg_mech.max_test_time_s = MCS_PMSM_ID_MECH_MAX_TEST_TIME_S;
    pmsm_oid_cfg.cfg_mech.record_time_s = MCS_PMSM_ID_MECH_RECORD_TIME_S;
    pmsm_oid_cfg.cfg_mech.min_fit_r2 = MCS_PMSM_ID_MECH_MIN_FIT_R2;
    pmsm_oid_cfg.cfg_mech.min_fit_samples = MCS_PMSM_ID_MECH_MIN_FIT_SAMPLES;

    // Sensored encoder calibration
    pmsm_oid_cfg.cfg_encoder.align_current_pu = MCS_PMSM_ID_ENCODER_ALIGN_CURRENT_A / CTRL_CURRENT_BASE;
    pmsm_oid_cfg.cfg_encoder.noise_check_time_s = MCS_PMSM_ID_ENCODER_NOISE_CHECK_TIME_S;
    pmsm_oid_cfg.cfg_encoder.align_settle_time_s = MCS_PMSM_ID_ENCODER_ALIGN_SETTLE_TIME_S;
    pmsm_oid_cfg.cfg_encoder.sweep_elec_hz = MCS_PMSM_ID_ENCODER_SWEEP_ELEC_HZ;
    pmsm_oid_cfg.cfg_encoder.anchor_settle_time_s = MCS_PMSM_ID_ENCODER_ANCHOR_SETTLE_TIME_S;
    pmsm_oid_cfg.cfg_encoder.max_sample_jump_pu = MCS_PMSM_ID_ENCODER_MAX_SAMPLE_JUMP_PU;
    pmsm_oid_cfg.cfg_encoder.max_stationary_span_pu = MCS_PMSM_ID_ENCODER_MAX_STATIONARY_SPAN_PU;
    pmsm_oid_cfg.cfg_encoder.min_cycle_motion_pu = MCS_PMSM_ID_ENCODER_MIN_CYCLE_MOTION_PU;
    pmsm_oid_cfg.cfg_encoder.max_cycle_deviation_pu = MCS_PMSM_ID_ENCODER_MAX_CYCLE_DEVIATION_PU;
    pmsm_oid_cfg.cfg_encoder.zero_return_tolerance_pu = MCS_PMSM_ID_ENCODER_ZERO_RETURN_TOLERANCE_PU;
    pmsm_oid_cfg.cfg_encoder.max_pole_pairs = MCS_PMSM_ID_ENCODER_MAX_POLE_PAIRS;

    // Initialize the identification engine.
    ctl_init_pmsm_offline_id_sm(&pmsm_oid, &pmsm_oid_cfg, dsa_buffer, DSA_BUFFER_SIZE);

    // The encoder stage is hosted by the suite while the legacy master source
    // retains the target-specific PREPARE/ADC handshake.
    pmsm_oid.sub_encoder.cfg = pmsm_oid_cfg.cfg_encoder;
    pmsm_oid.sub_encoder.sm = PMSM_ID_ENCODER_DISABLED;

    pmsm_oid.enc = &pos_enc.encif;
}

void loop_pmsm_offline_id()
{
    // =========================================================================
    // Offline ID Phase 1: Prepare (ADC Calibration Handshake)
    // =========================================================================
    if (pmsm_oid.sm == PMSM_OFFLINE_ID_PREPARE)
    {

        // Wait for the target ADC calibration handshake.
        if (flag_enable_adc_calibrator == 0 && index_adc_calibrator > 7)
        {
            // Calibration completed; advance to the next enabled stage.
            pmsm_oid.sm = ctl_oid_get_next_state(&pmsm_oid, PMSM_OFFLINE_ID_PREPARE);
            ctl_oid_init_target_state(&pmsm_oid);
        }
    }

    if (pmsm_oid.sm == PMSM_OFFLINE_ID_ENCODER_CALIB)
    {
        ctl_loop_oid_encoder(&pmsm_oid);
        if (pmsm_oid.sub_encoder.sm == PMSM_ID_ENCODER_FAULT)
        {
            pmsm_oid.sm = PMSM_OFFLINE_ID_FAULT;
        }
        else if (pmsm_oid.sub_encoder.sm == PMSM_ID_ENCODER_COMPLETE)
        {
            pmsm_oid.sm = ctl_oid_get_next_state(&pmsm_oid, PMSM_OFFLINE_ID_ENCODER_CALIB);
            ctl_oid_init_target_state(&pmsm_oid);
        }
    }

    // =========================================================================
    // OID Background Loop: Runs heavy math (Linear Regression, Regulators)
    // =========================================================================
    ctl_loop_pmsm_offline_id(&pmsm_oid);
}

/**
 * @brief Routes the FOC core's angle input to a specific internal/external source.
 * @param[in,out] ctx Pointer to the master offline ID context.
 * @param[in]     src The target angle source enum.
 */
void ctl_id_route_foc_angle(ctl_pmsm_offline_id_t* ctx, pmsm_oid_angle_src_e src)
{
    switch (src)
    {
    case PMSM_ID_ANGLE_SRC_STATIC:
        mtr_ctrl.pos_if = &ctx->static_angle;
        break;

    case PMSM_ID_ANGLE_SRC_VF_GEN:
        mtr_ctrl.pos_if = &ctx->vf_gen.enc;
        break;

    case PMSM_ID_ANGLE_SRC_REAL_ENC:
        mtr_ctrl.pos_if = ctx->enc;
        break;

    case PMSM_ID_ANGLE_SRC_SWITCHER:
        // Route FOC angle directly to the blended output of the angle switcher
        mtr_ctrl.pos_if = &ctx->angle_switcher.out_enc;
        break;

    default:
        break;
    }
}

/**
 * @brief Configures the operating state of the external FOC core.
 * @details Safely switches the FOC core between open-loop voltage injection and
 * closed-loop current regulation. Automatically disables advanced features like
 * cross-coupling decoupling and feedforward to ensure pure fundamental responses during ID.
 * @param[in,out] ctx   Pointer to the master offline ID context.
 * @param[in]     state The target FOC operating state (Open-loop or Closed-loop).
 */
void ctl_id_set_foc_state(ctl_pmsm_offline_id_t* ctx, pmsm_id_foc_state_e state)
{
    switch (state) // Fixed: Changed from 'src' to 'state'
    {
    case PMSM_ID_VOLTAGE_OPENLOOP:
        ctl_disable_foc_core_current_ctrl(&mtr_ctrl);
        ctl_disable_foc_core_decouple(&mtr_ctrl);
        ctl_disable_foc_core_vdq_ff(&mtr_ctrl);
        break;
    case PMSM_ID_CURRENT_CLOSELOOP:
        ctl_enable_foc_core_current_ctrl(&mtr_ctrl);
        ctl_disable_foc_core_decouple(&mtr_ctrl);
        ctl_disable_foc_core_vdq_ff(&mtr_ctrl);
        break;
    default:
        break;
    }
}

/**
 * @brief Retrieves the measured actual current (Id or Iq) from the FOC core.
 * @param[in] ctx   Pointer to the master offline ID context.
 * @param[in] index 0 for D-axis current (Id), 1 for Q-axis current (Iq).
 * @return ctrl_gt  The measured current in PU.
 */
ctrl_gt ctl_id_get_idq(ctl_pmsm_offline_id_t* ctx, fast_gt index)
{
    return mtr_ctrl.idq0.dat[index];
}

/**
 * @brief Retrieves the applied voltage reference (Vd or Vq) from the FOC core.
 * @details In closed-loop, this is the PI output. In open-loop, this is the injected voltage.
 * @param[in] ctx   Pointer to the master offline ID context.
 * @param[in] index 0 for D-axis voltage (Vd), 1 for Q-axis voltage (Vq).
 * @return ctrl_gt  The applied voltage reference in PU.
 */
ctrl_gt ctl_id_get_vdq(ctl_pmsm_offline_id_t* ctx, fast_gt index)
{
    return mtr_ctrl.vdq_ref.dat[index];
}

/**
 * @brief Retrieves the measured DC bus voltage from the FOC core.
 * @param[in] ctx  Pointer to the master offline ID context.
 * @return ctrl_gt The DC bus voltage in PU.
 */
ctrl_gt ctl_id_get_udc(ctl_pmsm_offline_id_t* ctx)
{
    return mtr_ctrl.udc;
}

/**
 * @brief Retrieves the current electrical speed from the FOC core's position interface.
 * @details Depending on the active angle routing (V/F, SMO, or Encoder),
 * this returns the synchronized speed of that specific source.
 * @param[in] ctx  Pointer to the master offline ID context.
 * @return ctrl_gt The electrical speed in PU.
 */
ctrl_gt ctl_id_get_speed(ctl_pmsm_offline_id_t* ctx)
{
    return mtr_ctrl.spd_if->speed;
}

/**
 * @brief Safely shuts down the FOC output (Zero current/voltage injection).
 * @details Instantly disables PI controllers and commands 0V on both axes.
 * Used for transitioning into safe passive states or upon fault detection.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_id_disable_output(ctl_pmsm_offline_id_t* ctx)
{
    ctl_disable_foc_core_current_ctrl(&mtr_ctrl);
    ctl_set_foc_core_idq_ref(&mtr_ctrl, float2ctrl(0.0f), float2ctrl(0.0f));
    ctl_set_foc_core_vdq_ref(&mtr_ctrl, float2ctrl(0.0f), float2ctrl(0.0f));
}

void ctl_id_set_pwm_output(ctl_pmsm_offline_id_t* ctx, fast_gt enable)
{
    GMP_UNUSED_VAR(ctx);
    if (enable)
    {
        flag_oid_pwm_inhibit = 0;
        ctl_fast_enable_output();
    }
    else
    {
        flag_oid_pwm_inhibit = 1;
        ctl_fast_disable_output();
    }
}

void ctl_id_commit_encoder_calibration(ctl_pmsm_offline_id_t* ctx, uint16_t pole_pairs,
                                       ctrl_gt encoder_offset_pu)
{
    GMP_UNUSED_VAR(ctx);
    pos_enc.pole_pairs = pole_pairs;
    ctl_set_autoturn_pos_encoder_mech_offset(&pos_enc, encoder_offset_pu);
}

/**
 * @brief Applies a constant closed-loop DC current vector.
 * @details Re-enables the FOC PI controllers if they were disabled, and tracks the target Id/Iq.
 * Exclusively used in Rs, Encoder Alignment, and steady-state dragging (Flux/Mech).
 * @param[in,out] ctx   Pointer to the master offline ID context.
 * @param[in]     id_pu D-axis current reference in PU.
 * @param[in]     iq_pu Q-axis current reference in PU.
 */
void ctl_id_apply_dc_current(ctl_pmsm_offline_id_t* ctx, ctrl_gt id_pu, ctrl_gt iq_pu)
{
    ctl_enable_foc_core_current_ctrl(&mtr_ctrl);
    ctl_set_foc_core_idq_ref(&mtr_ctrl, id_pu, iq_pu);
}

/**
 * @brief Applies an open-loop voltage pulse.
 * @details Disables the FOC PI controllers and directly injects raw Vd/Vq voltages.
 * Exclusively used for high-frequency pulse injection during Ld/Lq measurement.
 * @param[in,out] ctx   Pointer to the master offline ID context.
 * @param[in]     vd_pu D-axis voltage reference in PU.
 * @param[in]     vq_pu Q-axis voltage reference in PU.
 */
void ctl_id_apply_voltage_pulse(ctl_pmsm_offline_id_t* ctx, ctrl_gt vd_pu, ctrl_gt vq_pu)
{
    ctl_disable_foc_core_current_ctrl(&mtr_ctrl); // Disable PI regulation
    ctl_set_foc_core_vdq_ref(&mtr_ctrl, vd_pu, vq_pu);
}

/**
 * @brief Safely disables the Offline Identification process.
 * @details Immediately turns off the FOC PWM outputs and forces the master state
 * machine into the DISABLED state. Can be used as a soft E-Stop.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_disable_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
{
    GMP_UNUSED_VAR(ctx);

    ctl_disable_foc_core_current_ctrl(&mtr_ctrl);
    ctx->sm = PMSM_OFFLINE_ID_DISABLED;
}

//////////////////////////////////////////////////////////////////////////
