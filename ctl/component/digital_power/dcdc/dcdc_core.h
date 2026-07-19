/**
 * @file ctl_dcdc_core.h
 * @brief Universal Digital Power DCDC Controller Core.
 * @details This module implements a high-performance, cross-topology digital control 
 * core for Single-Input Single-Output (SISO) DC-DC converters (e.g., Buck, Boost, 
 * Buck-Boost, Phase-Shifted Full-Bridge, LLC, DAB). It abstracts the underlying 
 * hardware modulation via a "Formal Voltage" control command.
 */

#ifndef CTL_DCDC_CORE_H
#define CTL_DCDC_CORE_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/intrinsic/basic/hysteresis_controller.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>

/**
 * @brief Runtime execution modes for the DCDC Core.
 */
typedef enum
{
    CTL_DCDC_MODE_OPENLOOP = 0, /**< Voltage open-loop pass-through mode. */
    CTL_DCDC_MODE_CURRENTLOOP,  /**< Pure single current closed-loop mode. */
    CTL_DCDC_MODE_VOLTAGELOOP   /**< Full dual-loop voltage-governed mode. */
} ctl_dcdc_mode_e;

/**
 * @brief Runtime instance structure for the universal DCDC Core controller.
 * @details Contains memory intercepts, internal subsystem components, and execution states.
 */
typedef struct _tag_dcdc_core_t
{
    /* Hardware Data Access Links (Zero-Copy Intercepts) */
    adc_ift* v_in_fdbk;   /**< External reference link pointing to input voltage (PU). */
    adc_ift* v_out_fdbk;  /**< External reference link pointing to output terminal voltage (PU). */
    adc_ift* i_L_fdbk;    /**< External reference link pointing to main inductor current (PU). */
    adc_ift* i_load_fdbk; /**< External reference link pointing to consumption load current (PU). */

    /* Filter Submodules */
    ctl_filter_IIR1_t filter_v_in;   /**< Filter instance for input voltage. */
    ctl_filter_IIR1_t filter_v_out;  /**< Filter instance for output voltage. */
    ctl_filter_IIR1_t filter_i_L;    /**< Filter instance for inductor current. */
    ctl_filter_IIR1_t filter_i_load; /**< Filter instance for load current. */

    /* Reference Slope Limiters */
    ctl_slope_limiter_t ramp_v; /**< Rate limiter for voltage reference target. */
    ctl_slope_limiter_t ramp_i; /**< Rate limiter for current reference target. */

    /* Series-Form PID Controllers */
    ctl_pid_t voltage_pid; /**< Series-form voltage loop controller. */
    ctl_pid_t current_pid; /**< Series-form current loop controller. */

    /* Auxiliary Nonlinear Direction Monitor */
    ctl_hysteresis_controller_t i_dir_hcc; /**< Evaluates current direction polarity around zero. */

    /* Runtime Setpoints & Global Control States */
    ctl_dcdc_mode_e mode; /**< Active runtime control mode. */

    ctrl_gt v_target; /**< Raw user voltage input command (PU). */
    ctrl_gt i_target; /**< Raw user current input command (PU). */

    ctrl_gt v_ramp_ref; /**< Rate-limited voltage target applied to error loop (PU). */
    ctrl_gt i_ramp_ref; /**< Rate-limited current target applied to error loop (PU). */

    ctrl_gt v_out_formal;      /**< Final synthesized abstract voltage command for modulation (PU). */
    fast_gt is_current_dominant; /**< Low-overhead shadow register: 1=CC dominant, 0=CV dominant. */

    /* Safety Output Saturation Limits */
    ctrl_gt out_max; /**< Hard maximum limit clamp for the final formal voltage (PU). */
    ctrl_gt out_min; /**< Hard minimum limit clamp for the final formal voltage (PU). */
} ctl_dcdc_core_t;

/*---------------------------------------------------------------------------*/
/* Initialization and Interface Functions                                    */
/*---------------------------------------------------------------------------*/

/**
 * @brief Offline parameter configuration structure for DCDC Core initialization.
 * @details Allocates physical constraints and controller parameters using floating-point 
 * types for maximum quantization precision prior to fixed-point conversion.
 */
typedef struct _tag_dcdc_core_init_t
{
    parameter_gt fs; /**< System sampling and control loop execution frequency (Hz). */

    /* Filter Cutoff Frequency Configurations (Hz) */
    parameter_gt fc_v_in;   /**< Cutoff frequency for input voltage filter. */
    parameter_gt fc_v_out;  /**< Cutoff frequency for output terminal voltage filter. */
    parameter_gt fc_i_L;    /**< Cutoff frequency for inductor/resonant tank current filter. */
    parameter_gt fc_i_load; /**< Cutoff frequency for load consumption current filter. */

    /* Reference Target Slew-Rate Configurations (PU/s) */
    parameter_gt slope_v_pu_s; /**< Symmetric voltage target ramp rate. */
    parameter_gt slope_i_pu_s; /**< Symmetric current target ramp rate. */

    /* Series-Form Voltage PID Parameters & Saturation Limits (PU) */
    parameter_gt v_kp; /**< Proportional gain for voltage loop. */
    parameter_gt v_ki; /**< Integral time constant for voltage loop (seconds). */
    parameter_gt v_kd; /**< Derivative time constant for voltage loop (seconds). */
    ctrl_gt v_out_max; /**< Maximum limit of the voltage loop output. */
    ctrl_gt v_out_min; /**< Minimum limit of the voltage loop output. */

    /* Series-Form Current PID Parameters & Asymmetric Saturation Limits (PU) */
    parameter_gt i_kp; /**< Proportional gain for current loop. */
    parameter_gt i_ki; /**< Integral time constant for current loop (seconds). */
    parameter_gt i_kd; /**< Derivative time constant for current loop (seconds). */
    ctrl_gt i_out_max; /**< Maximum positive current capability boundary. */
    ctrl_gt i_out_min; /**< Maximum negative current/regeneration capability boundary. */
} ctl_dcdc_core_init_t;

/**
 * @brief Initializes the universal DCDC core based on offline user configuration.
 * @param[out] core Pointer to the DCDC core instance.
 * @param[in] init_config Pointer to the parameter configuration package.
 */
void ctl_init_dcdc_core(ctl_dcdc_core_t* core, const ctl_dcdc_core_init_t* init_config);

/**
 * @brief Clears all historical internal states and accumulators of the DCDC core.
 */
GMP_STATIC_INLINE void ctl_clear_dcdc_core(ctl_dcdc_core_t* core)
{
    ctl_clear_filter_iir1(&core->filter_v_in);
    ctl_clear_filter_iir1(&core->filter_v_out);
    ctl_clear_filter_iir1(&core->filter_i_L);
    ctl_clear_filter_iir1(&core->filter_i_load);

    ctl_clear_slope_limiter(&core->ramp_v);
    ctl_clear_slope_limiter(&core->ramp_i);

    ctl_clear_pid(&core->voltage_pid);
    ctl_clear_pid(&core->current_pid);

    core->v_ramp_ref = float2ctrl(0.0f);
    core->i_ramp_ref = float2ctrl(0.0f);
    core->v_out_formal = float2ctrl(0.0f);
    core->is_current_dominant = 0;
}

/**
 * @brief Configures safety protection limits for the final formal voltage output.
 */
GMP_STATIC_INLINE void ctl_set_dcdc_core_limits(ctl_dcdc_core_t* core, ctrl_gt max_limit, ctrl_gt min_limit)
{
    core->out_max = max_limit;
    core->out_min = min_limit;
}

/**
 * @brief Binds external hardware ADC interface channels to the DCDC core.
 * @param[out] core Pointer to the DCDC core instance.
 * @param[in] _v_in Pointer to input voltage ADC interface.
 * @param[in] _v_out Pointer to output voltage ADC interface.
 * @param[in] _i_L Pointer to inductor current ADC interface.
 * @param[in] _i_load Pointer to load current ADC interface.
 */
GMP_STATIC_INLINE void ctl_attach_dcdc_core(ctl_dcdc_core_t* core, adc_ift* _v_in, adc_ift* _v_out, adc_ift* _i_L,
                                            adc_ift* _i_load)
{
    core->v_in_fdbk = _v_in;
    core->v_out_fdbk = _v_out;
    core->i_L_fdbk = _i_L;
    core->i_load_fdbk = _i_load;
}

/**
 * @brief Low-overhead asynchronous query interface to fetch CC/CV dominant state.
 * @param[in] core Pointer to the DCDC core instance.
 * @return fast_gt 1 if current control is dominant (CC), 0 if voltage control is dominant (CV).
 */
GMP_STATIC_INLINE fast_gt ctl_dcdc_is_current_dominant(ctl_dcdc_core_t* core)
{
    return core->is_current_dominant;
}

/*---------------------------------------------------------------------------*/
/* Micro-Kernel Specialized Step Functions (Zero-Branch Production Path)     */
/*---------------------------------------------------------------------------*/

/**
 * @brief Executes raw hardware signal ingestion and internal filter pipeline updates.
 * @param[in,out] core Pointer to the DCDC core instance.
 */
GMP_STATIC_INLINE void ctl_dcdc_internal_ingest_and_filter(ctl_dcdc_core_t* core)
{
    ctrl_gt raw_v_in = ctl_get_adc_result(core->v_in_fdbk);
    ctrl_gt raw_v_out = ctl_get_adc_result(core->v_out_fdbk);
    ctrl_gt raw_i_L = ctl_get_adc_result(core->i_L_fdbk);
    ctrl_gt raw_i_load = ctl_get_adc_result(core->i_load_fdbk);

    ctl_step_filter_iir1(&core->filter_v_in, raw_v_in);
    ctl_step_filter_iir1(&core->filter_v_out, raw_v_out);
    ctl_step_filter_iir1(&core->filter_i_L, raw_i_L);
    ctl_step_filter_iir1(&core->filter_i_load, raw_i_load);

    ctl_step_hysteresis_controller(&core->i_dir_hcc, core->filter_i_L.out);
}

/**
 * @brief Executes one step of pure open-loop control path.
 * @param[in,out] core Pointer to the DCDC core instance.
 * @return ctrl_gt The bounded formal voltage output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_open_loop(ctl_dcdc_core_t* core)
{
    ctl_dcdc_internal_ingest_and_filter(core);
    core->is_current_dominant = 0;
    /* Open-loop commissioning must retain the configured soft-start. A raw
       voltage step can excite the output LC and trip protection before the
       current and voltage loops are introduced at later build levels. */
    core->v_ramp_ref = ctl_step_slope_limiter(&core->ramp_v, core->v_target);
    core->v_out_formal = ctl_sat(core->v_ramp_ref, core->out_max, core->out_min);
    return core->v_out_formal;
}

/**
 * @brief Executes one step of single current loop closed-control strategy path.
 * @param[in,out] core Pointer to the DCDC core instance.
 * @return ctrl_gt The calculated formal voltage output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_current_loop(ctl_dcdc_core_t* core)
{
    ctl_dcdc_internal_ingest_and_filter(core);
    core->is_current_dominant = 1;

    core->i_ramp_ref = ctl_step_slope_limiter(&core->ramp_i, core->i_target);
    ctrl_gt error_i = core->i_ramp_ref - core->filter_i_L.out;

    core->v_out_formal = ctl_step_pid_ser(&core->current_pid, error_i);
    core->v_out_formal = ctl_sat(core->v_out_formal, core->out_max, core->out_min);
    return core->v_out_formal;
}

/**
 * @brief Executes one step of single current loop closed-control strategy path.
 * @param[in,out] core Pointer to the DCDC core instance.
 * @return ctrl_gt The calculated formal voltage output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_output_current_loop(ctl_dcdc_core_t* core)
{
    ctl_dcdc_internal_ingest_and_filter(core);
    core->is_current_dominant = 1;

    core->i_ramp_ref = ctl_step_slope_limiter(&core->ramp_i, core->i_target);
    ctrl_gt error_i = core->i_ramp_ref - core->filter_i_load.out;

    core->v_out_formal = ctl_step_pid_ser(&core->current_pid, error_i);
    core->v_out_formal = ctl_sat(core->v_out_formal, core->out_max, core->out_min);
    return core->v_out_formal;
}

/**
 * @brief Executes one step of direct output-voltage closed-loop control.
 * @param[in,out] core Pointer to the DCDC core instance.
 * @return ctrl_gt The bounded formal modulation command.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_voltage_loop(ctl_dcdc_core_t* core)
{
    ctl_dcdc_internal_ingest_and_filter(core);
    core->is_current_dominant = 0;

    core->v_ramp_ref = ctl_step_slope_limiter(&core->ramp_v, core->v_target);
    ctrl_gt error_v = core->v_ramp_ref - core->filter_v_out.out;

    core->v_out_formal = ctl_step_pid_ser(&core->voltage_pid, error_v);
    core->v_out_formal = ctl_sat(core->v_out_formal, core->out_max, core->out_min);
    return core->v_out_formal;
}

/**
 * @brief Executes one step of dual-loop Cascade Control (Voltage Outer -> Current Inner).
 * @param[in,out] core Pointer to the DCDC core instance.
 * @return ctrl_gt The calculated formal voltage output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_cascade(ctl_dcdc_core_t* core)
{
    ctl_dcdc_internal_ingest_and_filter(core);

    /* 1. Execute Voltage Outer Loop */
    core->v_ramp_ref = ctl_step_slope_limiter(&core->ramp_v, core->v_target);
    ctrl_gt error_v = core->v_ramp_ref - core->filter_v_out.out;
    ctrl_gt inner_i_ref = ctl_step_pid_ser(&core->voltage_pid, error_v);

    /* 2. Route outer loop output as inner loop reference and execute */
    ctrl_gt error_i = inner_i_ref - core->filter_i_L.out;
    core->v_out_formal = ctl_step_pid_ser(&core->current_pid, error_i);
    core->v_out_formal = ctl_sat(core->v_out_formal, core->out_max, core->out_min);

    /* 3. Extract saturation state to update low-overhead shadow register */
    if ((inner_i_ref >= core->voltage_pid.out_max) || (inner_i_ref <= core->voltage_pid.out_min))
    {
        core->is_current_dominant = 1;
    }
    else
    {
        core->is_current_dominant = 0;
    }

    return core->v_out_formal;
}

/**
 * @brief Executes one step of dual-loop Parallel Control with anti-windup synchronization.
 * @param[in,out] core Pointer to the DCDC core instance.
 * @return ctrl_gt The calculated formal voltage output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_parallel(ctl_dcdc_core_t* core)
{
    ctl_dcdc_internal_ingest_and_filter(core);

    core->v_ramp_ref = ctl_step_slope_limiter(&core->ramp_v, core->v_target);
    core->i_ramp_ref = ctl_step_slope_limiter(&core->ramp_i, core->i_target);

    ctrl_gt error_v = core->v_ramp_ref - core->filter_v_out.out;
    ctrl_gt error_i = core->i_ramp_ref - core->filter_i_load.out;

    ctrl_gt out_v_pid = ctl_step_pid_ser(&core->voltage_pid, error_v);
    ctrl_gt out_i_pid = ctl_step_pid_ser(&core->current_pid, error_i);

    /* Competition Logic: Lower voltage demand wins to protect system from over-current */
    if (out_i_pid < out_v_pid)
    {
        core->v_out_formal = out_i_pid;
        core->is_current_dominant = 1;

        /* Enforce back-calculation anti-windup on the losing voltage integrator */
        ctl_pid_clamping_correction_using_real_output(&core->voltage_pid, core->v_out_formal);
    }
    else
    {
        core->v_out_formal = out_v_pid;
        core->is_current_dominant = 0;

        /* Enforce back-calculation anti-windup on the losing current integrator */
        ctl_pid_clamping_correction_using_real_output(&core->current_pid, core->v_out_formal);
    }

    core->v_out_formal = ctl_sat(core->v_out_formal, core->out_max, core->out_min);
    return core->v_out_formal;
}

/**
 * @brief Generic cascade branch dispatcher entry for evaluation/debugging phase.
 * @param[in,out] core Pointer to the DCDC core instance.
 * @return ctrl_gt The calculated formal voltage output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_core_cascade_generic(ctl_dcdc_core_t* core)
{
    switch (core->mode)
    {
    case CTL_DCDC_MODE_OPENLOOP:
        return ctl_step_dcdc_open_loop(core);

    case CTL_DCDC_MODE_CURRENTLOOP:
        return ctl_step_dcdc_current_loop(core);

    case CTL_DCDC_MODE_VOLTAGELOOP:
        /* Note: Production firmware calls ctl_step_dcdc_cascade or parallel directly 
               based on predefined topology to eliminate this branch cost. */
        return ctl_step_dcdc_cascade(core);

    default:
        return ctl_step_dcdc_open_loop(core);
    }
}

/**
 * @brief Generic parallel branch dispatcher entry for evaluation/debugging phase.
 * @param[in,out] core Pointer to the DCDC core instance.
 * @return ctrl_gt The calculated formal voltage output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_core_parallel_generic(ctl_dcdc_core_t* core)
{
    switch (core->mode)
    {
    case CTL_DCDC_MODE_OPENLOOP:
        return ctl_step_dcdc_open_loop(core);

    case CTL_DCDC_MODE_CURRENTLOOP:
        return ctl_step_dcdc_output_current_loop(core);

    case CTL_DCDC_MODE_VOLTAGELOOP:
        /* Note: Production firmware calls ctl_step_dcdc_cascade or parallel directly 
               based on predefined topology to eliminate this branch cost. */
        return ctl_step_dcdc_parallel(core);

    default:
        return ctl_step_dcdc_open_loop(core);
    }
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* CTL_DCDC_CORE_H */
