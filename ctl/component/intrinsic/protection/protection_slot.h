/**
 * @file ctl_protection.h
 * @author GMP Library Contributors
 * @brief Specialized Power Electronics Protection Library.
 * 
 * @details
 * This module provides highly memory-optimized, strongly-typed protection nodes 
 * for power electronic systems. By categorizing protections into Single-threshold, 
 * Window, Vector, and Thermal/Inverse-time structures, it achieves 100% spatial 
 * utilization and maximum DSP execution efficiency.
 * 
 * @version 2.0 (Specialized Node Architecture)
 * @copyright Copyright GMP(c) 2024-2026
 */

#ifndef _CTL_PROTECTION_H_
#define _CTL_PROTECTION_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*===========================================================================*/
/* 1. Single Variable, Single Threshold Node (High, Low, Symmetric)          */
/*===========================================================================*/

/**
 * @brief Protection node for single-variable, single-threshold monitoring.
 */
typedef struct _tag_prot_single_t
{
    fast_gt is_enabled;        //!< Master enable flag.
    ctrl_gt threshold;         //!< The target threshold.
    uint16_t trip_limit_count; //!< Debounce limit.
    uint16_t current_count;    //!< Current violation counter.
    uint32_t status_bit;       //!< Global error bitmask.
    ctrl_gt fault_record_val;  //!< Recorded snapshot/peak value.
} ctl_prot_single_t;

/**
 * @brief Initializes a single-threshold protection node.
 */
void ctl_init_prot_single(ctl_prot_single_t* node, uint32_t status_bit, parameter_gt threshold, uint16_t trip_limit);

/**
 * @brief Step: High Limit Protection (Peak Mode).
 */
GMP_STATIC_INLINE uint32_t ctl_step_prot_single_high_peak(ctl_prot_single_t* node, ctrl_gt value)
{
    if (!node->is_enabled)
        return 0;

    if (value > node->threshold)
    {
        if (node->current_count < node->trip_limit_count)
            node->current_count++;
        if (node->current_count >= node->trip_limit_count)
        {
            if (ctl_abs(value) > ctl_abs(node->fault_record_val))
            {
                node->fault_record_val = value; // Peak hold
            }
            return node->status_bit;
        }
    }
    else
    {
        node->current_count = 0;
    }
    return 0;
}

/**
 * @brief Step: Low Limit Protection (Snapshot Mode).
 */
GMP_STATIC_INLINE uint32_t ctl_step_prot_single_low_snap(ctl_prot_single_t* node, ctrl_gt value)
{
    if (!node->is_enabled)
        return 0;

    if (value < node->threshold)
    {
        if (node->current_count < node->trip_limit_count)
            node->current_count++;
        if (node->current_count >= node->trip_limit_count)
        {
            if (node->fault_record_val == float2ctrl(0.0f))
            {
                node->fault_record_val = value; // Snapshot on first strike
            }
            return node->status_bit;
        }
    }
    else
    {
        node->current_count = 0;
    }
    return 0;
}

/**
 * @brief Step: Symmetric Absolute Limit Protection (Peak Mode).
 */
GMP_STATIC_INLINE uint32_t ctl_step_prot_single_sym_peak(ctl_prot_single_t* node, ctrl_gt value)
{
    if (!node->is_enabled)
        return 0;

    if (ctl_abs(value) > node->threshold)
    {
        if (node->current_count < node->trip_limit_count)
            node->current_count++;
        if (node->current_count >= node->trip_limit_count)
        {
            if (ctl_abs(value) > ctl_abs(node->fault_record_val))
            {
                node->fault_record_val = value;
            }
            return node->status_bit;
        }
    }
    else
    {
        node->current_count = 0;
    }
    return 0;
}

/*===========================================================================*/
/* 2. Single Variable, Double Threshold Node (Window Protection)             */
/*===========================================================================*/

/**
 * @brief Protection node for windowed monitoring [inf, sup].
 */
typedef struct _tag_prot_window_t
{
    fast_gt is_enabled;
    ctrl_gt sup; //!< Upper bound.
    ctrl_gt inf; //!< Lower bound.
    uint16_t trip_limit_count;
    uint16_t current_count;
    uint32_t status_bit;
    ctrl_gt fault_record_val;
} ctl_prot_window_t;

/**
 * @brief Initializes a window protection node.
 */
void ctl_init_prot_window(ctl_prot_window_t* node, uint32_t status_bit, parameter_gt sup, parameter_gt inf,
                          uint16_t trip_limit);

/**
 * @brief Step: Window Protection (Snapshot Mode).
 * @details Trips if the value wanders outside the [inf, sup] boundary.
 */
GMP_STATIC_INLINE uint32_t ctl_step_prot_window_snap(ctl_prot_window_t* node, ctrl_gt value)
{
    if (!node->is_enabled)
        return 0;

    if ((value > node->sup) || (value < node->inf))
    {
        if (node->current_count < node->trip_limit_count)
            node->current_count++;
        if (node->current_count >= node->trip_limit_count)
        {
            if (node->fault_record_val == float2ctrl(0.0f))
            {
                node->fault_record_val = value;
            }
            return node->status_bit;
        }
    }
    else
    {
        node->current_count = 0;
    }
    return 0;
}

/*===========================================================================*/
/* 3. Vector Variable Node (Circle, Square/Box Protection)                   */
/*===========================================================================*/

/**
 * @brief Protection node for 2D vectors.
 */
typedef struct _tag_prot_vector_t
{
    fast_gt is_enabled;
    ctrl_gt threshold;    //!< Radius (circle) or Half-width (box).
    ctrl_gt threshold_sq; //!< Pre-calculated square of threshold (DSP optimization).
    uint16_t trip_limit_count;
    uint16_t current_count;
    uint32_t status_bit;
    ctl_vector2_t fault_record_val; //!< Records the vector that triggered the fault.
} ctl_prot_vector_t;

/**
 * @brief Initializes a vector protection node.
 */
void ctl_init_prot_vector(ctl_prot_vector_t* node, uint32_t status_bit, parameter_gt threshold, uint16_t trip_limit);

/**
 * @brief Step: Circular Limit Protection (Magnitude).
 * @details Evaluates x^2 + y^2 > r^2. Avoids expensive sqrt() calculation.
 */
GMP_STATIC_INLINE uint32_t ctl_step_prot_vector_circle(ctl_prot_vector_t* node, const ctl_vector2_t* vector)
{
    if (!node->is_enabled)
        return 0;

    ctrl_gt sq_sum = ctl_mul(vector->dat[0], vector->dat[0]) + ctl_mul(vector->dat[1], vector->dat[1]);

    if (sq_sum > node->threshold_sq)
    {
        if (node->current_count < node->trip_limit_count)
            node->current_count++;
        if (node->current_count >= node->trip_limit_count)
        {
            // Snapshot the offending vector
            if (node->fault_record_val.dat[0] == float2ctrl(0.0f) && node->fault_record_val.dat[1] == float2ctrl(0.0f))
            {
                node->fault_record_val = *vector;
            }
            return node->status_bit;
        }
    }
    else
    {
        node->current_count = 0;
    }
    return 0;
}

/**
 * @brief Step: Square/Box Limit Protection.
 * @details Evaluates |x| > threshold OR |y| > threshold.
 */
GMP_STATIC_INLINE uint32_t ctl_step_prot_vector_box(ctl_prot_vector_t* node, const ctl_vector2_t* vector)
{
    if (!node->is_enabled)
        return 0;

    if ((ctl_abs(vector->dat[0]) > node->threshold) || (ctl_abs(vector->dat[1]) > node->threshold))
    {
        if (node->current_count < node->trip_limit_count)
            node->current_count++;
        if (node->current_count >= node->trip_limit_count)
        {
            if (node->fault_record_val.dat[0] == float2ctrl(0.0f) && node->fault_record_val.dat[1] == float2ctrl(0.0f))
            {
                node->fault_record_val = *vector;
            }
            return node->status_bit;
        }
    }
    else
    {
        node->current_count = 0;
    }
    return 0;
}

/*===========================================================================*/
/* 4. Thermal & Stress Node (I^2t, It Accumulators)                          */
/*===========================================================================*/

/**
 * @brief Protection node for inverse-time accumulations.
 */
typedef struct _tag_prot_thermal_t
{
    fast_gt is_enabled;
    ctrl_gt rated_value;   //!< Nominal rating (current or power).
    ctrl_gt thermal_limit; //!< Maximum accumulated integral before trip.
    ctrl_gt thermal_acc;   //!< Real-time running accumulator.
    uint32_t status_bit;
    ctrl_gt fault_record_val; //!< Peak physical value that pushed it over the edge.
} ctl_prot_thermal_t;

/**
 * @brief Initializes a thermal/stress protection node.
 * @note No trip_limit_count is used here, as the time aspect is embedded in the integral.
 */
void ctl_init_prot_thermal(ctl_prot_thermal_t* node, uint32_t status_bit, parameter_gt rated_value,
                           parameter_gt thermal_limit);

/**
 * @brief Step: Thermal Protection (I^2 * t).
 * @details Integrates the Joule heating equivalent: (I^2 - I_rated^2).
 */
GMP_STATIC_INLINE uint32_t ctl_step_prot_thermal_i2t(ctl_prot_thermal_t* node, ctrl_gt current)
{
    if (!node->is_enabled)
        return 0;

    ctrl_gt i_sq = ctl_mul(current, current);
    ctrl_gt rated_sq = ctl_mul(node->rated_value, node->rated_value);

    // Calculate thermal stress (+ means heating, - means cooling)
    ctrl_gt stress = i_sq - rated_sq;

    // Accumulate
    node->thermal_acc += stress;

    // Prevent negative accumulation (ambient temperature baseline)
    if (node->thermal_acc < float2ctrl(0.0f))
    {
        node->thermal_acc = float2ctrl(0.0f);
    }

    if (node->thermal_acc > node->thermal_limit)
    {
        if (ctl_abs(current) > ctl_abs(node->fault_record_val))
        {
            node->fault_record_val = current; // Record peak current
        }
        return node->status_bit;
    }
    return 0;
}

/**
 * @brief Step: Linear Stress Protection (I * t).
 * @details Integrates the absolute overload: (|I| - I_rated).
 */
GMP_STATIC_INLINE uint32_t ctl_step_prot_stress_it(ctl_prot_thermal_t* node, ctrl_gt current)
{
    if (!node->is_enabled)
        return 0;

    ctrl_gt stress = ctl_abs(current) - node->rated_value;

    node->thermal_acc += stress;

    if (node->thermal_acc < float2ctrl(0.0f))
    {
        node->thermal_acc = float2ctrl(0.0f);
    }

    if (node->thermal_acc > node->thermal_limit)
    {
        if (ctl_abs(current) > ctl_abs(node->fault_record_val))
        {
            node->fault_record_val = current;
        }
        return node->status_bit;
    }
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif // _CTL_PROTECTION_H_
