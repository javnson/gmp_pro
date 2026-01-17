/**
 * @file pwm_modulator.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for three-phase bridge modulation with dead-time compensation.
 * @version 1.0
 * @date 2026-01-15
 *
 * @copyright Copyright GMP(c) 2025
 */

/** 
 * @defgroup CTL_TP_MODULATION_API Three-Phase Modulation API
 * @{
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief Provides functions for generating three-phase PWM signals from voltage commands,
 * including dead-time compensation based on current direction.
 */

#ifndef _FILE_THREE_PHASE_MODULATION_H_
#define _FILE_THREE_PHASE_MODULATION_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////
// SPWM modulator
//

/**
 * @brief Data structure for the three-phase bridge SPWM modulation module.
 * @note Recommended Value for Current Deadband:
 * - Theoretical Minimum: I_min = 2 * Coss * Vdc / T_dt
 * - Practical Setting: Typically 1% to 3% of the rated current.
 * - Tuning Guide: Set higher if zero-crossing oscillation occurs; set lower if crossover distortion is observed.
 * Ensure value is strictly larger than the ADC noise floor.
 */
typedef struct _tag_spwm_modulator
{
    // --- Input ---
    ctl_vector3_t vab0_out; //!< RO: target output voltage v alpha beta 0 (Input must be Per-Unit: -1.0 to 1.0).
    ctl_vector3_t* iuvw;    //!< RO: Filtered three-phase currents. POSITIVE direction = Flowing OUT of inverter.

    // --- Output Ports ---
    pwm_gt pwm_out[3]; //!< Final PWM compare values {A, B, C}.

    // --- Parameters ---
    pwm_gt pwm_full_scale;           //!< PWM counter period value (100% duty cycle).
    pwm_gt pwm_deadband_comp_val;    //!< Dead-time compensation value (in PWM counts). Added/Subtracted from CMP value.
    ctrl_gt current_deadband;        //!< Base threshold for dead-time compensation (e.g., 0.02pu).
    ctrl_gt current_hysteresis_band; //!< Hysteresis band width (e.g., 0.005pu).

    // --- Internal State ---
    vector3_gt vabc_out;
    signed short last_current_dir[3];         //!< State memory: 1(Pos), -1(Neg), 0(None).
    fast_gt flag_enable_deadband_compensator; //!< Enable dead band compensator.
} spwm_modulator_t;

/**
 * @brief Clears the internal states of the modulation module.
 * @ingroup CTL_TP_MODULATION_API
 * @param[out] mod Pointer to the @ref spwm_modulator_t structure.
 */
GMP_STATIC_INLINE void ctl_clear_spwm_modulator(spwm_modulator_t* mod)
{
    mod->pwm_out[phase_A] = 0;
    mod->pwm_out[phase_B] = 0;
    mod->pwm_out[phase_C] = 0;

    ctl_vector3_clear(&mod->vabc_out);

    mod->last_current_dir[phase_A] = 0;
    mod->last_current_dir[phase_B] = 0;
    mod->last_current_dir[phase_C] = 0;
}

/**
 * @brief Initializes the three-phase bridge modulation module.
 * @ingroup CTL_TP_MODULATION_API
 *
 * @param[out] bridge Pointer to the `three_phase_bridge_modulation_t` structure.
 * @param[in] pwm_full_scale The maximum value of the PWM counter.
 * @param[in] pwm_deadband The total dead-time value in PWM timer counts.
 * @param[in] iuvw a pointer to inverter output current, point to ADC module or main controller.
 * @param[in] current_deadband The current threshold to enable dead-time compensation.
 * @param[in] current_hysteresis The current hysteresis to enable dead-time compensation.
 */
void ctl_init_spwm_modulator(spwm_modulator_t* mod, pwm_gt pwm_full_scale, pwm_gt pwm_deadband_comp_val,
                             ctl_vector3_t* iuvw, ctrl_gt current_deadband, ctrl_gt current_hysteresis);

/**
 * @brief Executes one step of the three-phase modulation algorithm.
 * @ingroup CTL_TP_MODULATION_API
 * @details Converts per-unit voltage commands (-1.0 to 1.0) to PWM compare values,
 * and applies dead-time compensation based on the direction of phase currents.
 *
 * @param[in,out] mod Pointer to the @ref spwm_modulator_t structure.
 */
GMP_STATIC_INLINE void ctl_step_spwm_modulator(spwm_modulator_t* mod)
{
    gmp_base_assert(mod);
    gmp_base_assert(mod->iuvw);
    gmp_base_assert(mod->pwm_out);

    // Temp variable to handle signed calculations safely
    pwm_gt pwm_value;
    int i;

    // Modulation, SPWM
    ctl_ct_iclarke(&mod->vab0_out, &mod->vabc_out);

    // --- Calculate Raw PWM Values ---
    // Convert voltage command (-1.0 to 1.0) to duty cycle (0 to 1.0), then to raw PWM value.
    for (i = 0; i < 3; ++i)
    {
        // Convert voltage command (-1.0 to 1.0) to duty cycle (0 to 1.0), then to raw PWM value.
        pwm_value = pwm_mul(ctl_div2(ctl_add(float2ctrl(1.0f), mod->vabc_out.dat[i])), mod->pwm_full_scale);

        //
        // NOTE: pwm_gt must be a signed int number.
        // Or dead-time compensator would get a wrong result, when low modulation ratio.
        //

        // dead-time compensation
        if (mod->flag_enable_deadband_compensator)
        {
            // ---------------------------------------------------------
            // ZONE 1: Strong Positive Current -> Apply Positive Comp
            // ---------------------------------------------------------
            if (mod->iuvw->dat[i] > mod->current_deadband + mod->current_hysteresis_band)
            {
                pwm_value += mod->pwm_deadband_comp_val;
                mod->last_current_dir[i] = 1;
            }

            // ---------------------------------------------------------
            // ZONE 2: Strong Negative Current -> Apply Negative Comp
            // ---------------------------------------------------------
            else if (mod->iuvw->dat[i] < (-mod->current_deadband - mod->current_hysteresis_band))
            {
                if (pwm_value >= mod->pwm_deadband_comp_val)
                {
                    pwm_value -= mod->pwm_deadband_comp_val;
                }
                else
                {
                    pwm_value = 0; // avoid unsigned number underflow.
                }

                mod->last_current_dir[i] = -1;
            }

            // ---------------------------------------------------------
            // ZONE 3: Inside Deadband (Parasitic Cap Zone) -> No Comp
            // ---------------------------------------------------------
            else if ((mod->iuvw->dat[i] > -mod->current_deadband) && (mod->iuvw->dat[i] < mod->current_deadband))
            {
                mod->last_current_dir[i] = 0;
            }

            // ---------------------------------------------------------
            // ZONE 4: Hysteresis Band -> Keep Last State
            // ---------------------------------------------------------
            else
            {
                // Current is in the gray area:
                // [Deadband, Deadband + Hysteresis] OR [-Deadband - Hysteresis, -Deadband]

                if (mod->last_current_dir[i] == 1)
                {
                    pwm_value += mod->pwm_deadband_comp_val;
                }
                else if (mod->last_current_dir[i] == -1)
                {
                    if (pwm_value >= mod->pwm_deadband_comp_val)
                    {
                        pwm_value -= mod->pwm_deadband_comp_val;
                    }
                    else
                    {
                        pwm_value = 0; // avoid unsigned number underflow.
                    }
                }
                else
                {
                    // last_current_dir == 0
                    // We came from the dead band zone, so we stay 0 (no comp)
                    // until we cross the outer hysteresis limit.
                }
            }
        }

        // Saturate final PWM values to ensure they are within valid range.
        pwm_value = pwm_sat(pwm_value, mod->pwm_full_scale, 0);

        // write back correct number
        mod->pwm_out[i] = pwm_value;
    }
}

//////////////////////////////////////////////////////////////////////////
// SVPWM modulator
//

/**
 * @brief Executes one step of the three-phase modulation algorithm.
 * @ingroup CTL_TP_MODULATION_API
 * @details Converts per-unit voltage commands (-1.0 to 1.0) to PWM compare values,
 * and applies dead-time compensation based on the direction of phase currents.
 * ����ƽSPWM���ƺ�SVPWM���ƹ���ͬһ�����ݽṹ��ֻ��Ҫ�������ƺ������ɡ�
 *
 * @param[in,out] mod Pointer to the @ref spwm_modulator_t structure.
 */
GMP_STATIC_INLINE void ctl_step_svpwm_modulator(spwm_modulator_t* mod)
{
    gmp_base_assert(mod);
    gmp_base_assert(mod->iuvw);
    gmp_base_assert(mod->pwm_out);

    // Temp variable to handle signed calculations safely
    pwm_gt pwm_value;
    int i;

    // Modulation, SVPWM
    //ctl_ct_iclarke(&mod->vab0_out, &mod->vabc_out);
    ctl_ct_svpwm_calc(&mod->vab0_out, &mod->vabc_out);

    // --- Calculate Raw PWM Values ---
    // Convert voltage command (-1.0 to 1.0) to duty cycle (0 to 1.0), then to raw PWM value.
    for (i = 0; i < 3; ++i)
    {
        // Convert voltage command (-1.0 to 1.0) to duty cycle (0 to 1.0), then to raw PWM value.
        pwm_value = pwm_mul(ctl_div2(ctl_add(float2ctrl(1.0f), mod->vabc_out.dat[i])), mod->pwm_full_scale);

        //
        // NOTE: pwm_gt must be a signed int number.
        // Or dead-time compensator would get a wrong result, when low modulation ratio.
        //

        // dead-time compensation
        if (mod->flag_enable_deadband_compensator)
        {
            // ---------------------------------------------------------
            // ZONE 1: Strong Positive Current -> Apply Positive Comp
            // ---------------------------------------------------------
            if (mod->iuvw->dat[i] > mod->current_deadband + mod->current_hysteresis_band)
            {
                pwm_value += mod->pwm_deadband_comp_val;
                mod->last_current_dir[i] = 1;
            }

            // ---------------------------------------------------------
            // ZONE 2: Strong Negative Current -> Apply Negative Comp
            // ---------------------------------------------------------
            else if (mod->iuvw->dat[i] < (-mod->current_deadband - mod->current_hysteresis_band))
            {
                if (pwm_value >= mod->pwm_deadband_comp_val)
                {
                    pwm_value -= mod->pwm_deadband_comp_val;
                }
                else
                {
                    pwm_value = 0; // avoid unsigned number underflow.
                }

                mod->last_current_dir[i] = -1;
            }

            // ---------------------------------------------------------
            // ZONE 3: Inside Deadband (Parasitic Cap Zone) -> No Comp
            // ---------------------------------------------------------
            else if ((mod->iuvw->dat[i] > -mod->current_deadband) && (mod->iuvw->dat[i] < mod->current_deadband))
            {
                mod->last_current_dir[i] = 0;
            }

            // ---------------------------------------------------------
            // ZONE 4: Hysteresis Band -> Keep Last State
            // ---------------------------------------------------------
            else
            {
                // Current is in the gray area:
                // [Deadband, Deadband + Hysteresis] OR [-Deadband - Hysteresis, -Deadband]

                if (mod->last_current_dir[i] == 1)
                {
                    pwm_value += mod->pwm_deadband_comp_val;
                }
                else if (mod->last_current_dir[i] == -1)
                {
                    if (pwm_value >= mod->pwm_deadband_comp_val)
                    {
                        pwm_value -= mod->pwm_deadband_comp_val;
                    }
                    else
                    {
                        pwm_value = 0; // avoid unsigned number underflow.
                    }
                }
                else
                {
                    // last_current_dir == 0
                    // We came from the dead band zone, so we stay 0 (no comp)
                    // until we cross the outer hysteresis limit.
                }
            }
        }

        // Saturate final PWM values to ensure they are within valid range.
        pwm_value = pwm_sat(pwm_value, mod->pwm_full_scale, 0);

        // write back correct number
        mod->pwm_out[i] = pwm_value;
    }
}

//////////////////////////////////////////////////////////////////////////
// Type-I NPC modulator
//

// --- NPC Output Channel Mapping Macros ---
// 物理含义：0, 2, 4 对应外管(S1/S4)，1, 3, 5 对应内管(S2/S3)
// 实际上S1和S3构成互补关系，S2和S4构成互补关系。
// 假设硬件连接顺序为：A相[S1, S2], B相[S1, S2], C相[S1, S2]
// 注意：这里的定义取决于您的具体硬件和PWM分组，以下为一种常见定义
#define NPC_IDX_PHASE_A_OUTER 0 // A相 S1 (Top) / S4 (Bot) - 取决于极性，此处指"外侧斩波管"
#define NPC_IDX_PHASE_A_INNER 1 // A相 S2 (Inner Top) / S3 (Inner Bot)
#define NPC_IDX_PHASE_B_OUTER 2
#define NPC_IDX_PHASE_B_INNER 3
#define NPC_IDX_PHASE_C_OUTER 4
#define NPC_IDX_PHASE_C_INNER 5

// 或者使用更通用的计算宏
#define NPC_OUTER_IDX(phase) ((phase) * 2)
#define NPC_INNER_IDX(phase) ((phase) * 2 + 1)

typedef struct _tag_npc_modulator
{
    // --- Input ---
    ctl_vector3_t vab0_out; //!< Voltage reference (-1.0 to 1.0)
    ctl_vector3_t* iuvw;    //!< Current feedback

    // --- Output Ports ---
    // [0]=A_Outer, [1]=A_Inner, [2]=B_Outer, [3]=B_Inner...
    pwm_gt pwm_out[6];

    // --- Parameters ---
    pwm_gt pwm_full_scale;        //!< ARR (Period)
    pwm_gt pwm_max_duty_limit;    //!< Bootstrap protection: Max allowed duty cycle (e.g., ARR - MinPulse)
    pwm_gt pwm_deadband_comp_val; //!< Dead-time compensation ticks
    ctrl_gt current_deadband;
    ctrl_gt current_hysteresis_band;

    // --- Internal State ---
    vector3_gt vabc_out;
    signed short last_current_dir[3];
    fast_gt flag_enable_deadband_compensator;
    //fast_gt flag_enable_bootstrap_protection; //!< Enable max duty limiting
} npc_modulator_t;

/**
 * @brief Get the computed PWM compare value for a specific switch.
 * @param mod Pointer to the modulator.
 * @param phase Phase index (0=A, 1=B, 2=C).
 * @param is_inner 0 for Outer Switch (S1/S4-like), 1 for Inner Switch (S2/S3-like).
 * @return The PWM compare value to be written to the register.
 */
GMP_STATIC_INLINE pwm_gt ctl_npc_get_pwm_cmp(const npc_modulator_t* mod, int phase, int is_inner)
{
    // 这里使用简单的位运算或者乘法来定位
    // 假设索引排列是: A_Out, A_In, B_Out, B_In, C_Out, C_In
    int idx = (phase << 1) + (is_inner ? 1 : 0);
    return mod->pwm_out[idx];
}

/**
 * @brief Determines the current direction zone for dead-time compensation.
 * * @param i_sample Current sample value (Per-Unit or Amps).
 * @param deadband The inner deadband threshold (Zone 3 limit).
 * @param hysteresis The hysteresis band width (Zone 4 width).
 * @param last_dir Pointer to the memory of the last state (-1, 0, 1).
 * * @return int The new current direction state:
 * 1 : Strong Positive (Current flows OUT)
 * -1: Strong Negative (Current flows IN)
 * 0 : Inside Deadband (Zero Crossing)
 */
GMP_STATIC_INLINE int get_current_zone(ctrl_gt i_sample, ctrl_gt deadband, ctrl_gt hysteresis, signed short* last_dir)
{
    int current_dir = *last_dir; // Default to keep history

    // Thresholds
    ctrl_gt upper_trip_point = deadband + hysteresis;
    ctrl_gt lower_trip_point = -deadband - hysteresis;
    ctrl_gt upper_reset_point = deadband;
    ctrl_gt lower_reset_point = -deadband;

    // --- ZONE 1: Strong Positive ---
    if (i_sample > upper_trip_point)
    {
        current_dir = 1;
    }
    // --- ZONE 2: Strong Negative ---
    else if (i_sample < lower_trip_point)
    {
        current_dir = -1;
    }
    // --- ZONE 3: Inside Deadband (Strict Zero) ---
    else if ((i_sample > lower_reset_point) && (i_sample < upper_reset_point))
    {
        current_dir = 0;
    }
    // --- ZONE 4: Hysteresis Area (Gray Zone) ---
    else
    {
        // Logic:
        // If we were 1, stay 1 until we drop below deadband.
        // If we were -1, stay -1 until we rise above -deadband.
        // If we were 0, stay 0 until we cross trip points (handled by Zone1/2 checks).
        
        // This 'else' block implicitly maintains 'last_dir' value
        // which implements the hysteresis hold.
    }

    // Update state memory
    *last_dir = current_dir;
    return current_dir;
}

/**
 * @brief Step to execute the modulator.
 * @param mod Pointer to the modulator.
 */
GMP_STATIC_INLINE void ctl_step_npc_modulator(npc_modulator_t* mod)
{
    gmp_base_assert(mod);

    int i;

    // Modulation, SPWM
    ctl_ct_iclarke(&mod->vab0_out, &mod->vabc_out);

    for (i = 0; i < 3; ++i)
    {
        ctrl_gt v_ref = mod->vabc_out.dat[i];
        pwm_gt cmp_outer = 0;
        pwm_gt cmp_inner = 0;

        // -------------------------------------------------------------
        // Step 1: Base Modulation (分段映射)
        // -------------------------------------------------------------
        if (v_ref >= 0)
        {
            // --- Positive Half (P <-> O) ---
            // S1 (Outer) toggles, S2 (Inner) is ON

            // Calculate S1 Duty
            cmp_outer = pwm_mul(v_ref, mod->pwm_full_scale);

            // S2 kept ON (100%)
            cmp_inner = mod->pwm_full_scale;
        }
        else
        {
            // --- Negative Half (O <-> N) ---
            // S1 (Outer) is OFF, S2 (Inner) toggles
            // Note: S2 duty logic is inverted relative to magnitude for N generation

            // S1 kept OFF
            cmp_outer = 0;

            // Calculate S2 Duty ( 1.0 - |v_ref| )
            ctrl_gt v_abs = ctl_abs(v_ref);
            cmp_inner = pwm_mul(ctl_sub(float2ctrl(1.0f), v_abs), mod->pwm_full_scale);
        }

        // ============================================================
        // B. 死区补偿逻辑 (Using get_current_zone)
        // ============================================================
        if (mod->flag_enable_deadband_compensator)
        {
            // 调用封装好的函数，传入当前相电流、参数和状态记忆
            int dir = get_current_zone(mod->iuvw->dat[i], 
                                       mod->current_deadband, 
                                       mod->current_hysteresis_band, 
                                       &mod->last_current_dir[i]);

            if (dir == 1) // Strong Positive (Out)
            {
                // Current flowing out -> Device drop reduces voltage -> Compensate by adding duty
                if (v_ref >= 0) cmp_outer += mod->pwm_deadband_comp_val;
                else            cmp_inner += mod->pwm_deadband_comp_val;
            }
            else if (dir == -1) // Strong Negative (In)
            {
                // Current flowing in -> Device drop increases voltage -> Compensate by reducing duty
                if (v_ref >= 0) cmp_outer -= mod->pwm_deadband_comp_val;
                else            cmp_inner -= mod->pwm_deadband_comp_val;
            }
            // dir == 0: Inside Deadband -> No compensation
        }

        // -------------------------------------------------------------
        // Step 3: Bootstrap Protection (Saturation / Max Duty Limit)
        // 这一步必须在死区补偿之后、最终赋值之前进行
        // -------------------------------------------------------------
        // if (mod->flag_enable_bootstrap_protection)
        // {
        //     // 限制 Outer 管 (S1) 的最大占空比，确保有时间充电
        //     if (cmp_outer > mod->pwm_max_duty_limit)
        //     {
        //         cmp_outer = mod->pwm_max_duty_limit;
        //     }

        //     // 限制 Inner 管 (S2) 的最大占空比
        //     // 注意：在负半周，S2 负责产生 O 状态。
        //     // 如果 S2 也是浮地驱动(取决于硬件)，也需要限制。
        //     // 通常 S2 也是需要限制的，防止 Inner 驱动过热或掉电。
        //     if (cmp_inner > mod->pwm_max_duty_limit)
        //     {
        //         cmp_inner = mod->pwm_max_duty_limit;
        //     }

        //     // 可选：最小脉宽限制 (防止窄脉冲干扰)
        //     // if (cmp_outer < MIN_PULSE) cmp_outer = 0;
        // }

        // -------------------------------------------------------------
        // Step 4: Write Output (Saturation & Assignment)
        // -------------------------------------------------------------
        // 最终的安全钳位，防止溢出 ARR
        mod->pwm_out[NPC_OUTER_IDX(i)] = pwm_sat(cmp_outer, mod->pwm_full_scale, 0);
        mod->pwm_out[NPC_INNER_IDX(i)] = pwm_sat(cmp_inner, mod->pwm_full_scale, 0);
    }
}


/**
 * @brief Step to execute the modulator.
 * @param mod Pointer to the modulator.
 */
GMP_STATIC_INLINE void ctl_step_npc_svpwm_modulator(npc_modulator_t* mod)
{
    gmp_base_assert(mod);

    int i;

    // Modulation, SPWM
    //ctl_ct_iclarke(&mod->vab0_out, &mod->vabc_out);
    ctl_ct_svpwm_calc(&mod->vab0_out, &mod->vabc_out);

    for (i = 0; i < 3; ++i)
    {
        ctrl_gt v_ref = mod->vabc_out.dat[i];
        pwm_gt cmp_outer = 0;
        pwm_gt cmp_inner = 0;

        // -------------------------------------------------------------
        // Step 1: Base Modulation (分段映射)
        // -------------------------------------------------------------
        if (v_ref >= 0)
        {
            // --- Positive Half (P <-> O) ---
            // S1 (Outer) toggles, S2 (Inner) is ON

            // Calculate S1 Duty
            cmp_outer = pwm_mul(v_ref, mod->pwm_full_scale);

            // S2 kept ON (100%)
            cmp_inner = mod->pwm_full_scale;
        }
        else
        {
            // --- Negative Half (O <-> N) ---
            // S1 (Outer) is OFF, S2 (Inner) toggles
            // Note: S2 duty logic is inverted relative to magnitude for N generation

            // S1 kept OFF
            cmp_outer = 0;

            // Calculate S2 Duty ( 1.0 - |v_ref| )
            ctrl_gt v_abs = ctl_abs(v_ref);
            cmp_inner = pwm_mul(ctl_sub(float2ctrl(1.0f), v_abs), mod->pwm_full_scale);
        }

        // ============================================================
        // B. 死区补偿逻辑 (Using get_current_zone)
        // ============================================================
        if (mod->flag_enable_deadband_compensator)
        {
            // 调用封装好的函数，传入当前相电流、参数和状态记忆
            int dir = get_current_zone(mod->iuvw->dat[i], 
                                       mod->current_deadband, 
                                       mod->current_hysteresis_band, 
                                       &mod->last_current_dir[i]);

            if (dir == 1) // Strong Positive (Out)
            {
                // Current flowing out -> Device drop reduces voltage -> Compensate by adding duty
                if (v_ref >= 0) cmp_outer += mod->pwm_deadband_comp_val;
                else            cmp_inner += mod->pwm_deadband_comp_val;
            }
            else if (dir == -1) // Strong Negative (In)
            {
                // Current flowing in -> Device drop increases voltage -> Compensate by reducing duty
                if (v_ref >= 0) cmp_outer -= mod->pwm_deadband_comp_val;
                else            cmp_inner -= mod->pwm_deadband_comp_val;
            }
            // dir == 0: Inside Deadband -> No compensation
        }

        // -------------------------------------------------------------
        // Step 3: Bootstrap Protection (Saturation / Max Duty Limit)
        // 这一步必须在死区补偿之后、最终赋值之前进行
        // -------------------------------------------------------------
        // if (mod->flag_enable_bootstrap_protection)
        // {
        //     // 限制 Outer 管 (S1) 的最大占空比，确保有时间充电
        //     if (cmp_outer > mod->pwm_max_duty_limit)
        //     {
        //         cmp_outer = mod->pwm_max_duty_limit;
        //     }

        //     // 限制 Inner 管 (S2) 的最大占空比
        //     // 注意：在负半周，S2 负责产生 O 状态。
        //     // 如果 S2 也是浮地驱动(取决于硬件)，也需要限制。
        //     // 通常 S2 也是需要限制的，防止 Inner 驱动过热或掉电。
        //     if (cmp_inner > mod->pwm_max_duty_limit)
        //     {
        //         cmp_inner = mod->pwm_max_duty_limit;
        //     }

        //     // 可选：最小脉宽限制 (防止窄脉冲干扰)
        //     // if (cmp_outer < MIN_PULSE) cmp_outer = 0;
        // }

        // -------------------------------------------------------------
        // Step 4: Write Output (Saturation & Assignment)
        // -------------------------------------------------------------
        // 最终的安全钳位，防止溢出 ARR
        mod->pwm_out[NPC_OUTER_IDX(i)] = pwm_sat(cmp_outer, mod->pwm_full_scale, 0);
        mod->pwm_out[NPC_INNER_IDX(i)] = pwm_sat(cmp_inner, mod->pwm_full_scale, 0);
    }
}



#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_MODULATION_H_

/**
 * @}
 */
