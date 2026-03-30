
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

        // 基础开关配置
        pmsm_oid_cfg.cfg_basic.isr_freq_hz = CONTROLLER_FREQUENCY;
        pmsm_oid_cfg.cfg_basic.pole_pairs = MOTOR_PARAM_POLE_PAIRS;
        pmsm_oid_cfg.cfg_basic.is_sensorless = 0;

        // 使能辨识的流程 (全开)
        pmsm_oid_cfg.cfg_basic.flag_enable_prepare = 1; // 打开准备阶段 (连接 ADC 校准)
        pmsm_oid_cfg.cfg_basic.flag_enable_rs_dt   = 1;
        pmsm_oid_cfg.cfg_basic.flag_enable_ldq     = 1;
        pmsm_oid_cfg.cfg_basic.flag_enable_flux    = 1;
        pmsm_oid_cfg.cfg_basic.flag_enable_mech_id = 0;

        // --- 子模块具体参数配置 (示例) ---
        // Rs & DT
        pmsm_oid_cfg.cfg_rs_dt.max_current_pu = 0.3f;
        pmsm_oid_cfg.cfg_rs_dt.min_current_pu = 0.05f;
        pmsm_oid_cfg.cfg_rs_dt.steps = 5;
        pmsm_oid_cfg.cfg_rs_dt.align_time_s = 1.0f;
        pmsm_oid_cfg.cfg_rs_dt.measure_delay_s = 0.2f;
        pmsm_oid_cfg.cfg_rs_dt.measure_points = 100; // 采集 100 点求平均

        // Ld & Lq
        pmsm_oid_cfg.cfg_ld_lq.pulse_voltage_pu = 0.2f;
        pmsm_oid_cfg.cfg_ld_lq.max_bias_curr_pu = 0.2f;
        pmsm_oid_cfg.cfg_ld_lq.bias_steps = 5;
        pmsm_oid_cfg.cfg_ld_lq.align_current_pu = 0.5f;
        pmsm_oid_cfg.cfg_ld_lq.settle_time_s = 0.2f;
        pmsm_oid_cfg.cfg_ld_lq.pulse_time_s = 0.002f; // 2ms 极短脉冲
        pmsm_oid_cfg.cfg_ld_lq.cooldown_time_s = 0.05f;

        // Flux Linkage
        pmsm_oid_cfg.cfg_flux.min_target_speed_pu = 0.2f;
        pmsm_oid_cfg.cfg_flux.max_target_speed_pu = 0.4f;
        pmsm_oid_cfg.cfg_flux.steps = 3;
        pmsm_oid_cfg.cfg_flux.if_current_pu = 0.2f;
        pmsm_oid_cfg.cfg_flux.settle_time_s = 1.0f;
        pmsm_oid_cfg.cfg_flux.measure_points = 2000;

        // Mechanical
        pmsm_oid_cfg.cfg_mech.low_speed_pu = 0.2f;
        pmsm_oid_cfg.cfg_mech.high_speed_pu = 0.6f;
        pmsm_oid_cfg.cfg_mech.accel_iq_pu = 0.5f;
        pmsm_oid_cfg.cfg_mech.decel_iq_pu = -0.5f;
        pmsm_oid_cfg.cfg_mech.max_vbus_pu = 1.2f; // 120% OV 保护
        pmsm_oid_cfg.cfg_mech.if_current_pu = 0.2f;
        pmsm_oid_cfg.cfg_mech.settle_time_s = 2.0f;
        pmsm_oid_cfg.cfg_mech.transition_time_s = 0.5f;

        // 初始化辨识引擎
        ctl_init_pmsm_offline_id_sm(&pmsm_oid, &pmsm_oid_cfg, dsa_buffer, DSA_BUFFER_SIZE);

        pmsm_oid.enc = &pos_enc.encif;
}

void loop_pmsm_offline_id()
{
    // =========================================================================
        // Offline ID Phase 1: Prepare (ADC Calibration Handshake)
        // =========================================================================
        if (pmsm_oid.sm == PMSM_OFFLINE_ID_PREPARE)
        {

            // 监控 ADC 校准是否彻底完成 (根据你代码里 index > 13 就清零 flag 的逻辑)
                    if (flag_enable_adc_calibrator == 0 && index_adc_calibrator > 7) {
                        // 校准完成，手动将状态机推入下一个环节！
                        pmsm_oid.sm = ctl_oid_get_next_state(&pmsm_oid, PMSM_OFFLINE_ID_PREPARE);
                        // 这里声明一个外部可以调用的 ctl_oid_init_target_state
                        extern void ctl_oid_init_target_state(ctl_pmsm_offline_id_t* ctx);
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
