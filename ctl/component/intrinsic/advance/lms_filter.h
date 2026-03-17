/**
 * @file lms_filter.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a Least Mean Squares (LMS) adaptive FIR filter.
 * @version 1.0
 * @date 2025-08-09
 *
 * @copyright Copyright GMP(c) 2024
 *
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
 * @details This file implements an LMS adaptive filter. The filter continuously
 * adjusts its internal coefficients (weights) to minimize the mean square error
 * between its output and a desired reference signal. It is widely used for
 * applications like active noise cancellation, system identification, and echo
 * cancellation. The core of the algorithm is the update rule:
 * @f[ W(n+1) = W(n) + mu * e(n) * X(n) @f]
 * reference: https://zhuanlan.zhihu.com/p/358236441
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
    ctrl_gt* weights;      //!< Pointer to externally allocated filter coefficients array.
    ctrl_gt* buffer;       //!< Pointer to externally allocated input history buffer.
    uint32_t order;        //!< The number of weights (taps) in the filter.
    uint32_t buffer_index; //!< Current index for the circular buffer.

    ctrl_gt mu;     //!< Step-size parameter (learning rate).
    ctrl_gt output; //!< The filtered output signal y(n).
    ctrl_gt error;  //!< The error signal e(n).
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
fast_gt ctl_init_lms_filter(ctl_lms_filter_t* lms, uint32_t order, parameter_gt mu,
                            ctrl_gt* external_weights,
                            ctrl_gt* external_buffer);

/**
 * @brief Frees the memory allocated for the LMS filter.
 * @param[in,out] lms Pointer to the LMS filter instance.
 */
void ctl_destroy_lms_filter(ctl_lms_filter_t* lms);


/**
 * @brief Clears the internal states (weights and buffer) of the LMS filter.
 * @param[out] lms Pointer to the LMS filter instance.
 */
GMP_STATIC_INLINE void ctl_clear_lms_filter(ctl_lms_filter_t* lms)
{
    uint32_t i;

    lms->buffer_index = 0;
    lms->output = float2ctrl(0.0f);
    lms->error = float2ctrl(0.0f);

    if (lms->buffer != 0 && lms->weights != 0)
    {
        for (i = 0; i < lms->order; i++)
        {
            lms->buffer[i] = float2ctrl(0.0f);
            lms->weights[i] = float2ctrl(0.0f); // 初始化权重通常为 0
        }
    }
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
    uint32_t i;

    // 1. Update circular buffer
    lms->buffer[lms->buffer_index] = input;

    // 2. Calculate filter output (FIR Convolution)
    lms->output = float2ctrl(0.0f);
    uint32_t j = lms->buffer_index;
    for (i = 0; i < lms->order; i++)
    {
        // 修复 1：严格使用 ctl_mul 防止卷积溢出
        lms->output += ctl_mul(lms->weights[i], lms->buffer[j]);
        if (j == 0)
            j = lms->order - 1;
        else
            j--;
    }

    // 3. Calculate Error Signal
    // e(n) = d(n) - y(n)
    lms->error = desired - lms->output;

    // 4. Update Filter Weights (LMS Algorithm)
    // W(n+1) = W(n) + mu * e(n) * X(n)
    j = lms->buffer_index;
    for (i = 0; i < lms->order; i++)
    {
        // 修复 2：将三重乘法拆解为嵌套的 ctl_mul，彻底杜绝定点溢出灾难
        ctrl_gt error_x_input = ctl_mul(lms->error, lms->buffer[j]);
        ctrl_gt delta_w = ctl_mul(lms->mu, error_x_input);

        lms->weights[i] += delta_w;

        if (j == 0)
            j = lms->order - 1;
        else
            j--;
    }

    // 5. Advance buffer index
    lms->buffer_index++;
    if (lms->buffer_index >= lms->order)
    {
        lms->buffer_index = 0;
    }

    return lms->output;
}
/**
 * @}
 */ // end of lms_adaptive_filter group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _LMS_FILTER_H_
