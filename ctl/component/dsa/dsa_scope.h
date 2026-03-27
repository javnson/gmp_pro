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
    for (uint32_t i = 0; i < scope->mem.capacity; i++)
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
    if (ctl_step_divider(&scope->divider))
    {
        if (scope->current_idx < scope->depth)
        {
            va_list args;
            va_start(args, arg_count);
            uint16_t write_dims = (arg_count < scope->dims) ? arg_count : scope->dims;
            for (uint16_t i = 0; i < write_dims; i++)
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

// (ctl_dsa_calc_min_divider, ctl_dsa_calc_max_duration, ctl_dsa_fit_vs_time, ctl_dsa_fit_vs_dim 保持不变)

#ifdef __cplusplus
}
#endif

#endif // _FILE_CTL_DSA_SCOPE_H_
       /** @} */
