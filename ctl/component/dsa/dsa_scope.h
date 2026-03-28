/**
 * @file ctl_dsa_scope.h
 * @defgroup dsa_scope DSA Scope
 * @brief Digital Signal Analysis Oscilloscope with high-performance multi-channel recording.
 * @details Employs `ctl_mem_view_t` for dynamic memory slicing. 
 * Provides strongly-typed 1-4 channel injection functions to avoid float-to-double promotion overhead.
 * @{
 */

#include <ctl/math_block/utilities/mem_view.h>

#ifndef _FILE_CTL_DSA_SCOPE_H_
#define _FILE_CTL_DSA_SCOPE_H_

#include <stdarg.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _tag_ctl_dsa_scope
{
    ctl_mem_view_t mem;    /*!< Underlying memory view manager. */
    ctl_divider_t divider; /*!< Internal frequency divider. */

    parameter_gt isr_freq_hz;      /*!< Base execution frequency (Hz). */
    parameter_gt effective_dt_sec; /*!< Effective time step after sub-sampling. */

    uint16_t dims;        /*!< Currently configured number of dimensions. */
    uint32_t depth;       /*!< Currently configured depth per dimension. */
    uint32_t current_idx; /*!< Internal write pointer. */

} ctl_dsa_scope_t;

/**
 * @brief Initializes the DSA Scope globally (Called once during system boot).
 */
void ctl_init_dsa_scope(ctl_dsa_scope_t* scope, ctrl_gt* mem_pool, uint32_t capacity, parameter_gt isr_freq);

/**
 * @brief Configures the Scope for a specific test stage (Dynamic Slicing).
 */
void ctl_config_dsa_scope(ctl_dsa_scope_t* scope, uint16_t dims, uint32_t div);

/**
 * @brief Resets the write pointer to start a new recording, keeping existing configuration.
 */
GMP_STATIC_INLINE void ctl_reset_dsa_scope_tracker(ctl_dsa_scope_t* scope)
{
    scope->current_idx = 0;
    ctl_clear_divider(&scope->divider);
}

/**
 * @brief HARD WIPE: Overwrites the entire underlying memory pool with zero/null values.
 * @details Useful for ensuring no residual data corrupts visualization or debugging.
 * @param[in,out] scope Pointer to the scope instance.
 */
GMP_STATIC_INLINE void ctl_wipe_dsa_scope_memory(ctl_dsa_scope_t* scope)
{
    uint32_t i;

    for (i = 0; i < scope->mem.capacity; i++)
    {
        scope->mem.buffer[i] = float2ctrl(0.0f);
    }
    ctl_reset_dsa_scope_tracker(scope);
}

// ============================================================================
// Core Execution (Strongly-Typed Channels to avoid VA_ARGS overhead)
// ============================================================================

/**
 * @brief Records 1 Channel of data. Highly optimized, avoids VA_ARGS double promotion.
 */
GMP_STATIC_INLINE fast_gt ctl_step_dsa_scope_1ch(ctl_dsa_scope_t* scope, ctrl_gt ch0)
{
    if (ctl_step_divider(&scope->divider))
    {
        if (scope->current_idx < scope->depth)
        {
            if (scope->dims > 0)
                ctl_mem_set_2d_soa(&scope->mem, 0, scope->current_idx, scope->depth, ch0);
            scope->current_idx++;
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Records 2 Channels of data (e.g., Speed, Torque).
 */
GMP_STATIC_INLINE fast_gt ctl_step_dsa_scope_2ch(ctl_dsa_scope_t* scope, ctrl_gt ch0, ctrl_gt ch1)
{
    if (ctl_step_divider(&scope->divider))
    {
        if (scope->current_idx < scope->depth)
        {
            if (scope->dims > 0)
                ctl_mem_set_2d_soa(&scope->mem, 0, scope->current_idx, scope->depth, ch0);
            if (scope->dims > 1)
                ctl_mem_set_2d_soa(&scope->mem, 1, scope->current_idx, scope->depth, ch1);
            scope->current_idx++;
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Records 3 Channels of data (e.g., Id, Iq, Speed).
 */
GMP_STATIC_INLINE fast_gt ctl_step_dsa_scope_3ch(ctl_dsa_scope_t* scope, ctrl_gt ch0, ctrl_gt ch1, ctrl_gt ch2)
{
    if (ctl_step_divider(&scope->divider))
    {
        if (scope->current_idx < scope->depth)
        {
            if (scope->dims > 0)
                ctl_mem_set_2d_soa(&scope->mem, 0, scope->current_idx, scope->depth, ch0);
            if (scope->dims > 1)
                ctl_mem_set_2d_soa(&scope->mem, 1, scope->current_idx, scope->depth, ch1);
            if (scope->dims > 2)
                ctl_mem_set_2d_soa(&scope->mem, 2, scope->current_idx, scope->depth, ch2);
            scope->current_idx++;
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Records 4 Channels of data (e.g., Vd, Vq, Id, Iq).
 */
GMP_STATIC_INLINE fast_gt ctl_step_dsa_scope_4ch(ctl_dsa_scope_t* scope, ctrl_gt ch0, ctrl_gt ch1, ctrl_gt ch2,
                                                 ctrl_gt ch3)
{
    if (ctl_step_divider(&scope->divider))
    {
        if (scope->current_idx < scope->depth)
        {
            if (scope->dims > 0)
                ctl_mem_set_2d_soa(&scope->mem, 0, scope->current_idx, scope->depth, ch0);
            if (scope->dims > 1)
                ctl_mem_set_2d_soa(&scope->mem, 1, scope->current_idx, scope->depth, ch1);
            if (scope->dims > 2)
                ctl_mem_set_2d_soa(&scope->mem, 2, scope->current_idx, scope->depth, ch2);
            if (scope->dims > 3)
                ctl_mem_set_2d_soa(&scope->mem, 3, scope->current_idx, scope->depth, ch3);
            scope->current_idx++;
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Records N Channels using Variadic Arguments.
 * @warning C Standard Variadic functions implicitly promote 'float' to 'double'. 
 * If `ctrl_gt` is a float, this incurs significant FPU overhead. 
 * If `ctrl_gt` is an integer/fixed-point type, this function will FAIL silently due to type misalignment.
 * Use 1ch~4ch functions whenever possible!
 */
GMP_STATIC_INLINE fast_gt ctl_step_dsa_scope_varargs(ctl_dsa_scope_t* scope, uint16_t arg_count, ...)
{
    uint16_t i;

    if (ctl_step_divider(&scope->divider))
    {
        if (scope->current_idx < scope->depth)
        {
            va_list args;
            va_start(args, arg_count);
            uint16_t write_dims = (arg_count < scope->dims) ? arg_count : scope->dims;
            for (i = 0; i < write_dims; i++)
            {
                // Warning: Explicitly assumes passing floats promoted to double.
                ctrl_gt val = (ctrl_gt)va_arg(args, double);
                ctl_mem_set_2d_soa(&scope->mem, i, scope->current_idx, scope->depth, val);
            }
            va_end(args);
            scope->current_idx++;
            return 1;
        }
    }
    return 0;
}

// ============================================================================
// Utility Calculators for Test Planning
// ============================================================================

/**
 * @brief Calculates the minimum required frequency divider (prescaler).
 * @details Ensures that the buffer does not overflow before 'target_time_s' is reached.
 * * @param[in] capacity      Total memory capacity.
 * @param[in] dims          Number of dimensions requested.
 * @param[in] target_time_s The physical duration required for the test (Seconds).
 * @param[in] isr_freq_hz   Base ISR frequency (Hz).
 * @return uint32_t         The minimum safe divider value.
 */
GMP_STATIC_INLINE uint32_t ctl_dsa_calc_min_divider(uint32_t capacity, uint16_t dims, parameter_gt target_time_s,
                                                    parameter_gt isr_freq_hz)
{
    if (dims == 0)
        return 1;
    uint32_t depth = capacity / dims;
    if (depth == 0)
        return 1;

    parameter_gt total_ticks_needed = target_time_s * isr_freq_hz;
    uint32_t min_divider = (uint32_t)(total_ticks_needed / (parameter_gt)depth) + 1;

    return (min_divider > 0) ? min_divider : 1;
}

/**
 * @brief Calculates the maximum recording duration for a given configuration.
 * @param[in] capacity      Total memory capacity.
 * @param[in] dims          Number of dimensions requested.
 * @param[in] divider       The configured frequency divider.
 * @param[in] isr_freq_hz   Base ISR frequency (Hz).
 * @return parameter_gt     Maximum duration in seconds.
 */
GMP_STATIC_INLINE parameter_gt ctl_dsa_calc_max_duration(uint32_t capacity, uint16_t dims, uint32_t divider,
                                                         parameter_gt isr_freq_hz)
{
    if (dims == 0 || isr_freq_hz <= 0.001f)
        return 0.0f;
    uint32_t depth = capacity / dims;
    return (parameter_gt)(depth * divider) / isr_freq_hz;
}

/**
 * @brief Performs a linear regression of a specific dimension AGAINST TIME.
 * @details Solves the equation: Y = slope * t + intercept.
 * To maintain high floating-point precision, time is treated as relative (t = 0 at start_idx).
 * Therefore, the 'intercept' represents the calculated value of Y exactly at 'start_idx'.
 * * @param[in]  scope     Pointer to the scope instance.
 * @param[in]  dim_y     The dimension index to use as the dependent variable (Y).
 * @param[in]  start_idx The starting depth index of the data segment.
 * @param[in]  end_idx   The ending depth index of the data segment (inclusive).
 * @param[out] slope     Pointer to store the calculated slope (e.g., Acceleration in rad/s^2).
 * @param[out] intercept Pointer to store the calculated intercept (Value at start_idx).
 * @return fast_gt       Returns 1 if fitting is successful, 0 if data length is invalid or matrix is singular.
 */
fast_gt ctl_dsa_fit_vs_time(ctl_dsa_scope_t* scope, uint16_t dim_y, uint32_t start_idx, uint32_t end_idx,
                            parameter_gt* slope, parameter_gt* intercept);

/**
 * @brief Performs a linear regression of ONE DIMENSION AGAINST ANOTHER.
 * @details Solves the equation: Y = slope * X + intercept.
 * Extremely useful for determining system parameters like Resistance (U vs I) or Flux (|E| vs W).
 * * @param[in]  scope     Pointer to the scope instance.
 * @param[in]  dim_x     The dimension index to use as the independent variable (X).
 * @param[in]  dim_y     The dimension index to use as the dependent variable (Y).
 * @param[in]  start_idx The starting depth index of the data segment.
 * @param[in]  end_idx   The ending depth index of the data segment (inclusive).
 * @param[out] slope     Pointer to store the calculated slope (e.g., Rs, Ld, Psi_m).
 * @param[out] intercept Pointer to store the calculated intercept (e.g., Dead-time voltage).
 * @return fast_gt       Returns 1 if successful, 0 if invalid or singular.
 */
fast_gt ctl_dsa_fit_vs_dim(ctl_dsa_scope_t* scope, uint16_t dim_x, uint16_t dim_y, uint32_t start_idx, uint32_t end_idx,
                           parameter_gt* slope, parameter_gt* intercept);

#ifdef __cplusplus
}
#endif

#endif // _FILE_CTL_DSA_SCOPE_H_

/** @} */
