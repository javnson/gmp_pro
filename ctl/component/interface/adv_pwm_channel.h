/**
 * @file adv_pwm_channel.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Advanced Per-Unitized PWM Modulation Engine for Digital Power Electronics.
 * @details This module provides a unified Per-Unit framework for concurrent Period (Frequency), 
 * Phase, and Duty modulation. By systemically aligning all dimensions to the "Period/Time" domain, 
 * all scaling transformations maintain absolute perfect linearity while eliminating hardware runtime divisions.
 * @version 0.2
 * @date 2026-06-18
 *
 * @copyright Copyright GMP(c) 2026
 *
 * @note **Unified Period-Domain Scaling Verification Example:**
 * Consider a hardware timer clock where the nominal switching timebase corresponds 
 * to exactly 1000 clock ticks (e.g., 100 kHz at 100 MHz clock).
 * * 1. Setup Phase:
 * - Configure `period_base = 1000` (The hardware baseline clock ticks for 1.0 PU period).
 * * 2. Execution Step (Dynamic Modulation Ingestion):
 * - Input: `raw.period = 1.20 PU` (Commanding period expansion -> frequency scales down linearly to 83.33 kHz)
 * - Input: `raw.phase  = 0.25 PU` (Commanding a 90-degree phase shift based on the active period)
 * - Input: `raw.duty   = 0.50 PU` (Commanding a 50% pulse width based on the active period)
 * * 3. Mathematical Compilation Pipeline inside `ctl_step_adv_pwm_channel`:
 * - **Real-time Active Period (Ticks):** * `period = raw.period * period_base` = 1.20 * 1000 = 1200 ticks. 
 * - **Real-time Active Phase Offset (Ticks):** * `phase = raw.phase * period` = 0.25 * 1200 = 300 ticks.
 * - **Real-time Active Duty Threshold (Ticks):** * `duty = raw.duty * period` = 0.50 * 1200 = 600 ticks.
 * * 4. Final Output Mapping Matrix to Hardware Registers:
 * - Register Period (e.g., TBPRD / ARR) = `period` = 1200 Ticks
 * - Register Phase  (e.g., TBPHS / CNT) = `phase`  = 300 Ticks
 * - Register Compare(e.g., CMPA  / CCR) = `duty`   = 600 Ticks
 * Every single control parameter perfectly scales with the timebase without any harmonic or phase distortion.
 */

#ifndef _FILE_ADV_PWM_CHANNEL_H_
#define _FILE_ADV_PWM_CHANNEL_H_

#include <ctl/component/interface/interface_base.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup ADV_PWM_CHANNEL Advanced Per-Unit PWM Module
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief Unified Advanced PWM channel processing micro-kernels in Period-domain.
 * @{
 */

/**
 * @brief Normalized Per-Unit control interface bundle for advanced PWM tracking.
 * @details All components hold standard per-unit values (ctrl_gt) mapped to the period timebase.
 */
typedef struct _tag_adv_pwm_ift
{
    /**
     * @brief INPUT: Per-unit scaled period. 1.0 PU represents nominal period base. Higher values extend period (decrease frequency).
     */
    ctrl_gt period;
    /**
     * @brief INPUT: Per-unit phase shift offset based on current period timebase (1.0 PU = 360 degrees / 2*pi).
     */
    ctrl_gt phase;
    /**
     * INPUT: Per-unit unipolar active duty cycle command based on current period (1.0 PU = 100% / Full Period).
     */
    ctrl_gt duty;
    /**
     * @brief INPUT: Per-unit dead time based on the current period.
     * @details For example 200 ns at 100 kHz is 0.02 PU.  The hardware
     * adapter may write the result to both DBRED and DBFED.
     */
    ctrl_gt deadband;
} adv_pwm_ift;

/**
 * @brief Hard runtime instance structure for the advanced decoupled PWM channel.
 */
typedef struct _tag_adv_pwm_channel
{
    /**
     * @brief Internal copy of the continuous per-unit control reference commands.
     */
    adv_pwm_ift raw;

    /**
     * @brief PARAMETER: Fixed hardware tick period value corresponding to nominal 1.0 PU period.
     */
    pwm_gt period_base;

    /**
     * @brief OUTPUT: Computed definitive absolute integer period ticks for the hardware register (e.g., TBPRD).
     */
    pwm_gt period;
    /**
     * @brief OUTPUT: Computed definitive absolute integer phase shift sync ticks for hardware register (e.g., TBPHS).
     */
    pwm_gt phase;
    /**
     * @brief OUTPUT: Computed definitive absolute integer compare match threshold ticks for hardware register (e.g., CMPA). 
     */
    pwm_gt duty;
    /** @brief OUTPUT: Absolute dead-band ticks (DBRED/DBFED). */
    pwm_gt deadband;

} adv_pwm_channel_t;

/*---------------------------------------------------------------------------*/
/* Exported Advanced Architecture APIs                                       */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initializes the advanced decoupled PU PWM channel.
 * @param[out] pwm_obj Pointer to the advanced PWM channel instance.
 * @param[in] nominal_period_base Base period counts (ticks) defining the 1.0 PU period threshold.
 */
void ctl_init_adv_pwm_channel(adv_pwm_channel_t* pwm_obj, pwm_gt nominal_period_base);

/**
 * @brief Executes one advanced unipolar modulation step compiling raw PU commands into hardware registers ticks.
 * @details All calculations derive from the real-time compiled period to sustain absolute mathematical linearity.
 * @param[in,out] pwm_obj Pointer to the advanced PWM channel instance.
 * @param[in] input_ref Pointer to the upstream normalized triple-control command interface bundle.
 */
GMP_STATIC_INLINE void ctl_step_adv_pwm_channel(adv_pwm_channel_t* pwm_obj, const adv_pwm_ift* input_ref)
{
    /* 1. Ingest and safe boundary-clamp the period command */
    if (input_ref->period <= float2ctrl(0.01f))
    {
        pwm_obj->raw.period = float2ctrl(0.01f); /* Prevent zero-period singularity or infinite frequency faults */
    }
    else
    {
        pwm_obj->raw.period = input_ref->period;
    }

    /* 2. Compile real-time hardware period baseline (Ticks) */
    pwm_obj->period = pwm_mul(pwm_obj->raw.period, pwm_obj->period_base);

    /* 3. Wrap phase to [0, 1).  Negative commands are deliberately retained
       as an equivalent lag, which is required by bidirectional DAB control. */
    pwm_obj->raw.phase = input_ref->phase;
    while (pwm_obj->raw.phase < float2ctrl(0.0f))
        pwm_obj->raw.phase += float2ctrl(1.0f);
    while (pwm_obj->raw.phase >= float2ctrl(1.0f))
        pwm_obj->raw.phase -= float2ctrl(1.0f);
    pwm_obj->phase = pwm_mul(pwm_obj->raw.phase, pwm_obj->period);

    /* 4. Ingest and compile duty cycle threshold ticks based on the newly updated active period */
    if (input_ref->duty <= float2ctrl(0.0f))
    {
        pwm_obj->raw.duty = float2ctrl(0.0f);
        pwm_obj->duty = 0;
    }
    else if (input_ref->duty >= float2ctrl(1.0f))
    {
        pwm_obj->raw.duty = float2ctrl(1.0f);
        pwm_obj->duty = pwm_obj->period;
    }
    else
    {
        pwm_obj->raw.duty = input_ref->duty;
        pwm_obj->duty = pwm_mul(pwm_obj->raw.duty, pwm_obj->period);
    }

    /* 5. Direct hard saturation guard rails to seal hardware safety contract */
    pwm_obj->phase = pwm_sat(pwm_obj->phase, pwm_obj->period, 0);
    pwm_obj->duty = pwm_sat(pwm_obj->duty, pwm_obj->period, 0);

    /* Dead time is intentionally limited below half a period so invalid
       commands can never suppress both bridge transitions indefinitely. */
    pwm_obj->raw.deadband = ctl_sat(input_ref->deadband, float2ctrl(0.49f), float2ctrl(0.0f));
    pwm_obj->deadband = pwm_mul(pwm_obj->raw.deadband, pwm_obj->period);
}

/**
 * @brief Executes one advanced step compiling inverse complementary thresholds.
 * @details Mappings: Period = period, Phase = phase, Duty = (period - (duty * period)).
 * @param[in,out] pwm_obj Pointer to the advanced PWM channel instance.
 * @param[in] input_ref Pointer to the upstream normalized triple-control command interface bundle.
 */
GMP_STATIC_INLINE void ctl_step_adv_pwm_channel_inv(adv_pwm_channel_t* pwm_obj, const adv_pwm_ift* input_ref)
{
    ctl_step_adv_pwm_channel(pwm_obj, input_ref);
    pwm_obj->duty = pwm_obj->period - pwm_obj->duty;
}

/**
 * @brief Fetches control interface reference intercept port of the advanced channel.
 * @param[in] pwm Pointer to the advanced channel instance.
 * @return adv_pwm_ift* External normalized control port reference pointer.
 */
GMP_STATIC_INLINE adv_pwm_ift* ctl_get_adv_pwm_channel_ctrl_port(adv_pwm_channel_t* pwm)
{
    return &pwm->raw;
}

/** @} */

/**
 * @defgroup ADV_PWM_DUAL_CHANNEL Advanced Symmetrical Dual-Channel PWM Module
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief Advanced processing module for generalized full-bridge tridimensional modulation.
 * @{
 */

/**
 * @brief Fully symmetric Per-Unit control interface bundle for advanced dual-channel tracking.
 * @details Eliminates hard-coded functional naming dependencies to allow maximum mathematical scalability.
 */
typedef struct _tag_adv_pwm_dual_ift
{
    /**
     * @brief INPUT: Per-unit scaled period. 1.0 PU represents nominal period base.
     */
    ctrl_gt period;
    /**
     * @brief INPUT: Vectorized per-unit absolute phase shifts for Channel 0 and Channel 1 (1.0 PU = 360 degrees).
     */
    ctrl_gt phase[2];
    /**
     * @brief INPUT: Vectorized per-unit unipolar active duty cycles for Channel 0 and Channel 1 (1.0 PU = 100% / Full Period).
     */
    ctrl_gt duty[2];
    /** @brief INPUT: Shared per-unit dead time for both complementary legs. */
    ctrl_gt deadband;
} adv_pwm_dual_ift;

/**
 * @brief Fully symmetric runtime instance structure for the advanced dual-channel full-bridge module.
 * @details Mirrors the input interface array layout to achieve single-cycle structural assembly blocks moving.
 */
typedef struct _tag_adv_pwm_dual_channel
{
    /**
     * @brief Internal copy of the continuous vectorized per-unit control references.
     */
    adv_pwm_dual_ift raw;

    /**
     * @brief PARAMETER: Fixed hardware tick period value defining the 1.0 PU period baseline.
     */
    pwm_gt period_base;

    /**
     * @brief OUTPUT: Computed absolute integer period ticks for hardware register (TBPRD).
     */
    pwm_gt period;
    /**
     * @brief OUTPUT: Computed absolute integer phase synchronization ticks matrix for Leg 0 and Leg 1 (TBPHS).
     */
    pwm_gt phase[2];
    /**
     * @brief OUTPUT: Computed absolute integer compare thresholds matrix for Leg 0 and Leg 1 (CMPA).
     */
    pwm_gt duty[2];
    /** @brief OUTPUT: Shared absolute dead-band ticks. */
    pwm_gt deadband;
} adv_pwm_dual_channel_t;

/*---------------------------------------------------------------------------*/
/* Exported Advanced Dual-Channel Architecture APIs                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initializes the lean symmetric advanced dual-channel/full-bridge PWM entity.
 * @param[out] pwm_obj Pointer to the advanced dual PWM channel instance.
 * @param[in] nominal_period_base Base period counts (ticks) defining the 1.0 PU period threshold.
 */
void ctl_init_adv_pwm_dual_channel(adv_pwm_dual_channel_t* pwm_obj, pwm_gt nominal_period_base);

/**
 * @brief Executes one advanced matrix full-bridge modulation step compiling raw PU vectors into ticks matrices.
 * @details Linearly scales all phase and duty ticks concurrently based on the real-time calculated active period.
 * @param[in,out] pwm_obj Pointer to the advanced dual PWM channel instance.
 * @param[in] input_ref Pointer to the upstream normalized symmetric control vector interface bundle.
 */
GMP_STATIC_INLINE void ctl_step_adv_pwm_dual_channel(adv_pwm_dual_channel_t* pwm_obj, const adv_pwm_dual_ift* input_ref)
{
    int i;

    /* 1. Ingest and safe boundary-clamp the unified period command */
    if (input_ref->period <= float2ctrl(0.01f))
    {
        pwm_obj->raw.period = float2ctrl(0.01f);
    }
    else
    {
        pwm_obj->raw.period = input_ref->period;
    }
    pwm_obj->period = pwm_mul(pwm_obj->raw.period, pwm_obj->period_base);

    pwm_obj->raw.deadband = ctl_sat(input_ref->deadband, float2ctrl(0.49f), float2ctrl(0.0f));
    pwm_obj->deadband = pwm_mul(pwm_obj->raw.deadband, pwm_obj->period);

    /* 2. Concurrent Loop Compilation for both symmetrical channels */
    for (i = 0; i < 2; ++i)
    {
        /* --- Symmetrical Phase Vectors Matrix Compilation --- */
        ctrl_gt phase_cmd = input_ref->phase[i];
        while (phase_cmd < float2ctrl(0.0f))
            phase_cmd += float2ctrl(1.0f);
        while (phase_cmd >= float2ctrl(1.0f))
            phase_cmd -= float2ctrl(1.0f);
        pwm_obj->raw.phase[i] = phase_cmd;
        pwm_obj->phase[i] = pwm_mul(phase_cmd, pwm_obj->period);
        pwm_obj->phase[i] = pwm_sat(pwm_obj->phase[i], pwm_obj->period, 0);

        /* --- Symmetrical Duty Vectors Matrix Compilation --- */
        ctrl_gt duty_cmd = input_ref->duty[i];
        if (duty_cmd <= float2ctrl(0.0f))
        {
            pwm_obj->raw.duty[i] = float2ctrl(0.0f);
            pwm_obj->duty[i] = 0;
        }
        else if (duty_cmd >= float2ctrl(1.0f))
        {
            pwm_obj->raw.duty[i] = float2ctrl(1.0f);
            pwm_obj->duty[i] = pwm_obj->period;
        }
        else
        {
            pwm_obj->raw.duty[i] = duty_cmd;
            pwm_obj->duty[i] = pwm_mul(pwm_obj->raw.duty[i], pwm_obj->period);
        }
        pwm_obj->duty[i] = pwm_sat(pwm_obj->duty[i], pwm_obj->period, 0);
    }
}

/**
 * @brief Executes one advanced symmetric step compiling inverse complementary full-bridge thresholds matrix.
 * @details Formula: Output Period = period, Phase[i] = phase[i], Duty[i] = (period - forward_duty[i]).
 * @param[in,out] pwm_obj Pointer to the advanced dual PWM channel instance.
 * @param[in] input_ref Pointer to the upstream normalized symmetric control vector interface bundle.
 */
GMP_STATIC_INLINE void ctl_step_adv_pwm_dual_channel_inv(adv_pwm_dual_channel_t* pwm_obj,
                                                         const adv_pwm_dual_ift* input_ref)
{
    int i;

    /* 1. Synchronize base timeline properties via forward step pass */
    ctl_step_adv_pwm_dual_channel(pwm_obj, input_ref);

    /* 2. Invert compare thresholds concurrently for lower gate complement units */
    for (i = 0; i < 2; ++i)
    {
        pwm_gt forward_duty_ticks = pwm_obj->duty[i];

        if (pwm_obj->period >= forward_duty_ticks)
        {
            pwm_obj->duty[i] = pwm_obj->period - forward_duty_ticks;
        }
        else
        {
            pwm_obj->duty[i] = 0;
        }

        /* Direct safety boundary enclosure clamp */
        pwm_obj->duty[i] = pwm_sat(pwm_obj->duty[i], pwm_obj->period, 0);
    }
}

/**
 * @brief Fetches control ports intercept bundle reference of the symmetrical advanced channel.
 * @param[in] pwm Pointer to the advanced dual channel instance.
 * @return adv_pwm_dual_ift* External normalized symmetric control vector reference pointer.
 */
GMP_STATIC_INLINE adv_pwm_dual_ift* ctl_get_adv_pwm_dual_channel_ctrl_port(adv_pwm_dual_channel_t* pwm)
{
    return &pwm->raw;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ADV_PWM_CHANNEL_H_
