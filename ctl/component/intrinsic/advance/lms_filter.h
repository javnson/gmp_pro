/**
 * @file lms_filter.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a Least Mean Squares (LMS) adaptive FIR filter.
 * @version 1.0
 * @date 2025-08-09
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file implements an LMS adaptive filter. The filter continuously
 * adjusts its internal coefficients (weights) to minimize the mean square error
 * between its output and a desired reference signal. It is widely used for
 * applications like active noise cancellation, system identification, and echo
 * cancellation. The core of the algorithm is the update rule:
 * W(n+1) = W(n) + mu * e(n) * X(n)
 * reference: https://zhuanlan.zhihu.com/p/358236441
 */

#ifndef _LMS_FILTER_H_
#define _LMS_FILTER_H_

#include <ctl/math_block/gmp_math.h>
#include <stdint.h>
#include <stdlib.h> // Required for malloc and free

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup lms_adaptive_filter LMS Adaptive Filter
 * @brief A self-tuning filter that minimizes the mean square error.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* LMS Adaptive Filter                                                       */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the LMS adaptive filter.
 */
typedef struct _tag_lms_filter_t
{
    uint32_t order; //!< The order of the filter (number of taps/weights).
    ctrl_gt mu;     //!< The step-size parameter (learning rate).

    ctrl_gt* weights;      //!< Pointer to the array of filter coefficients (W).
    ctrl_gt* buffer;       //!< Pointer to the circular buffer for past inputs (X).
    uint32_t buffer_index; //!< Current index for the circular buffer.

    ctrl_gt output; //!< The last calculated filter output, y(n).
    ctrl_gt error;  //!< The last calculated error, e(n).
} ctl_lms_filter_t;

/**
 * @brief Initializes the LMS adaptive filter.
 * @details This function allocates memory for the filter's weights and data buffer.
 * The user is responsible for calling ctl_destroy_lms_filter to free this memory.
 * @param[out] lms Pointer to the LMS filter instance.
 * @param[in] order The number of filter taps (coefficients).
 * @param[in] mu The step-size (learning rate). Must be chosen carefully to ensure stability.
 * @return fast_gt Returns 1 on success (memory allocated), 0 on failure.
 */
fast_gt ctl_init_lms_filter(ctl_lms_filter_t* lms, uint32_t order, parameter_gt mu)
{
    lms->order = order;
    lms->mu = float2ctrl(mu);
    lms->output = 0.0f;
    lms->error = 0.0f;
    lms->buffer_index = 0;

    // --- Key Point Analysis 1: Dynamic Memory Allocation ---
    // The filter's memory requirement depends on its order. Dynamic allocation
    // provides flexibility. Ensure the MCU has enough heap space.
    lms->weights = (ctrl_gt*)malloc(order * sizeof(ctrl_gt));
    lms->buffer = (ctrl_gt*)malloc(order * sizeof(ctrl_gt));

    if (lms->weights == NULL || lms->buffer == NULL)
    {
        // Free any partially allocated memory before returning
        if (lms->weights)
            free(lms->weights);
        if (lms->buffer)
            free(lms->buffer);
        return 0; // Memory allocation failed
    }

    // Initialize weights and buffer to zero
    for (uint32_t i = 0; i < order; ++i)
    {
        lms->weights[i] = 0.0f;
        lms->buffer[i] = 0.0f;
    }

    return 1; // Success
}

/**
 * @brief Frees the memory allocated for the LMS filter.
 * @param[in,out] lms Pointer to the LMS filter instance.
 */
GMP_STATIC_INLINE void ctl_destroy_lms_filter(ctl_lms_filter_t* lms)
{
    if (lms->weights != NULL)
    {
        free(lms->weights);
        lms->weights = NULL;
    }
    if (lms->buffer != NULL)
    {
        free(lms->buffer);
        lms->buffer = NULL;
    }
}

/**
 * @brief Clears the internal states (weights and buffer) of the LMS filter.
 * @param[out] lms Pointer to the LMS filter instance.
 */
GMP_STATIC_INLINE void ctl_clear_lms_filter(ctl_lms_filter_t* lms)
{
    if (lms->weights != NULL && lms->buffer != NULL)
    {
        for (uint32_t i = 0; i < lms->order; i++)
        {
            lms->weights[i] = 0.0f;
            lms->buffer[i] = 0.0f;
        }
    }
    lms->output = 0.0f;
    lms->error = 0.0f;
    lms->buffer_index = 0;
}

/**
 * @brief Executes one step of the LMS adaptive filter algorithm.
 * @param[in,out] lms Pointer to the LMS filter instance.
 * @param[in] input The primary input signal to the filter, x(n).
 * @param[in] desired The desired (reference) signal, d(n).
 * @return ctrl_gt The calculated error signal, e(n) = d(n) - y(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_lms_filter(ctl_lms_filter_t* lms, ctrl_gt input, ctrl_gt desired)
{
    // --- Key Point Analysis 2: Update Input Buffer ---
    // A circular buffer is used to efficiently store the most recent 'order' input samples.
    // This avoids large memory shifts in every step.
    lms->buffer[lms->buffer_index] = input;

    // --- Key Point Analysis 3: Calculate Filter Output ---
    // The output y(n) is the dot product of the weights vector W(n) and the input vector X(n).
    // y(n) = W^T * X(n)
    lms->output = 0.0f;
    uint32_t j = lms->buffer_index;
    for (uint32_t i = 0; i < lms->order; i++)
    {
        lms->output += lms->weights[i] * lms->buffer[j];
        if (j == 0)
        {
            j = lms->order - 1;
        }
        else
        {
            j--;
        }
    }

    // --- Key Point Analysis 4: Calculate Error Signal ---
    // The error e(n) is the difference between the desired signal and the filter's output.
    // This error signal is what drives the adaptation process.
    // e(n) = d(n) - y(n)
    lms->error = desired - lms->output;

    // --- Key Point Analysis 5: Update Filter Weights ---
    // This is the core of the LMS algorithm. The weights are adjusted in the direction
    // that minimizes the error. The step-size 'mu' controls the speed and stability
    // of this convergence. A larger mu leads to faster adaptation but can cause instability.
    // W(n+1) = W(n) + mu * e(n) * X(n)
    j = lms->buffer_index;
    for (uint32_t i = 0; i < lms->order; i++)
    {
        lms->weights[i] += lms->mu * lms->error * lms->buffer[j];
        if (j == 0)
        {
            j = lms->order - 1;
        }
        else
        {
            j--;
        }
    }

    // Advance the circular buffer index for the next sample
    lms->buffer_index++;
    if (lms->buffer_index >= lms->order)
    {
        lms->buffer_index = 0;
    }

    return lms->error;
}

/**
 * @}
 */ // end of lms_adaptive_filter group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _LMS_FILTER_H_
