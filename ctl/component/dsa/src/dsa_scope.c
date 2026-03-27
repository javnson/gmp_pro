

#include <ctl/component/dsa/dsa_scope.h>


/**
 * @brief Initializes the DSA Scope globally (Called once during system boot).
 */
void ctl_init_dsa_scope(ctl_dsa_scope_t* scope, ctrl_gt* mem_pool, uint32_t capacity, parameter_gt isr_freq)
{
    ctl_init_mem_view(&scope->mem, mem_pool, capacity);
    scope->isr_freq_hz = isr_freq;
    scope->dims = 0;
    scope->depth = 0;
    scope->current_idx = 0;
}

/**
 * @brief Configures the Scope for a specific test stage (Dynamic Slicing).
 */
void ctl_config_dsa_scope(ctl_dsa_scope_t* scope, uint16_t dims, uint32_t div)
{
    if (dims == 0)
        return;
    scope->dims = dims;
    scope->depth = scope->mem.capacity / dims;
    ctl_init_divider(&scope->divider, div);
    scope->current_idx = 0;

    if (scope->isr_freq_hz > 0.0f)
    {
        scope->effective_dt_sec = (parameter_gt)div / scope->isr_freq_hz;
    }
}

// ============================================================================
// Mathematical Fitting Engines (Least-Squares Regression)
// ============================================================================

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
                            parameter_gt* slope, parameter_gt* intercept)
{
    // 1. Boundary & Safety Checks
    if (start_idx >= end_idx || end_idx >= scope->depth || dim_y >= scope->dims)
    {
        return 0; // Invalid index range or dimension out of bounds
    }

    uint32_t n = end_idx - start_idx + 1;
    parameter_gt sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;

    // 2. Accumulate sums for Least-Squares method
    for (uint32_t i = 0; i < n; i++)
    {
        // X-axis: Relative physical time calculated from effective sample period
        parameter_gt x = (parameter_gt)i * scope->effective_dt_sec;

        // Y-axis: Safely fetch the data point from the memory view
        parameter_gt y = (parameter_gt)ctl_mem_get_2d_soa(&scope->mem, dim_y, start_idx + i, scope->depth);

        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }

    // 3. Denominator check to prevent Division by Zero (Singular Matrix)
    parameter_gt denominator = (n * sum_xx) - (sum_x * sum_x);
    if (denominator > -1e-6f && denominator < 1e-6f)
    {
        return 0; // Points form a vertical line or all X values are identical
    }

    // 4. Calculate Final Slope and Intercept
    *slope = ((n * sum_xy) - (sum_x * sum_y)) / denominator;
    *intercept = (sum_y - (*slope * sum_x)) / n;

    return 1;
}

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
                           parameter_gt* slope, parameter_gt* intercept)
{
    // 1. Boundary & Safety Checks
    if (start_idx >= end_idx || end_idx >= scope->depth || dim_x >= scope->dims || dim_y >= scope->dims)
    {
        return 0; // Invalid index range or dimension out of bounds
    }

    uint32_t n = end_idx - start_idx + 1;
    parameter_gt sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;

    // 2. Accumulate sums for Least-Squares method
    for (uint32_t i = start_idx; i <= end_idx; i++)
    {
        // Safely fetch X and Y points from the corresponding dimensions
        parameter_gt x = (parameter_gt)ctl_mem_get_2d_soa(&scope->mem, dim_x, i, scope->depth);
        parameter_gt y = (parameter_gt)ctl_mem_get_2d_soa(&scope->mem, dim_y, i, scope->depth);

        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }

    // 3. Denominator check to prevent Division by Zero
    parameter_gt denominator = (n * sum_xx) - (sum_x * sum_x);
    if (denominator > -1e-6f && denominator < 1e-6f)
    {
        return 0; // Singular matrix (e.g., X values are completely constant)
    }

    // 4. Calculate Final Slope and Intercept
    *slope = ((n * sum_xy) - (sum_x * sum_y)) / denominator;
    *intercept = (sum_y - (*slope * sum_x)) / n;

    return 1;
}
