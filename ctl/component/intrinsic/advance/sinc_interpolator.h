/**
 * @file sinc_interpolator.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a high-quality fractional delay filter using Windowed-Sinc interpolation.
 * @version 1.0
 * @date 2025-08-09
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file implements a Sinc interpolator, which is theoretically the
 * ideal method for reconstructing a band-limited signal between its discrete samples.
 * To make it practical, this implementation uses a finite-length, windowed version
 * of the Sinc function (specifically, a Blackman window) to create an FIR filter.
 * The filter coefficients are pre-calculated and stored in a look-up table to
 * achieve high performance on embedded systems. This module is ideal for high-precision
 * resampling and fractional delay applications.
 * reference: https://zhuanlan.zhihu.com/p/453094282
 */

#ifndef _SINC_INTERPOLATOR_H_
#define _SINC_INTERPOLATOR_H_

#include <ctl/math_block/gmp_math.h>
#include <math.h> // Required for sinf and cosf
#include <stdint.h>
#include <stdlib.h> // Required for malloc and free

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup sinc_interpolator Sinc Interpolator
 * @brief A high-quality resampling and fractional delay module.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Windowed-Sinc Interpolator                                                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Sinc interpolator.
 */
typedef struct _tag_sinc_interpolator_t
{
    uint32_t num_taps;   //!< The number of FIR filter taps (the length of the Sinc kernel). Must be even.
    uint32_t table_size; //!< The number of pre-calculated filter sets in the LUT (resolution).

    ctrl_gt** sinc_table;  //!< 2D LUT storing the pre-calculated windowed-sinc coefficients.
    ctrl_gt* buffer;       //!< Circular buffer for storing past input samples.
    uint32_t buffer_index; //!< Current index for the circular buffer.

    ctrl_gt output; //!< The last calculated interpolated output.
} ctl_sinc_interpolator_t;

/**
 * @brief Initializes the Sinc interpolator.
 * @details This function allocates memory and pre-calculates the entire windowed-sinc
 * coefficient table. The user is responsible for calling ctl_destroy_sinc_interpolator
 * to free this memory.
 * @param[out] sinc Pointer to the Sinc interpolator instance.
 * @param[in] num_taps The number of filter taps (e.g., 32, 64). Higher order gives better quality but requires more memory and CPU.
 * @param[in] table_size The resolution of the fractional delay (e.g., 256). Higher resolution gives smoother interpolation.
 * @return fast_gt Returns 1 on success, 0 on failure (memory allocation).
 */
fast_gt ctl_init_sinc_interpolator(ctl_sinc_interpolator_t* sinc, uint32_t num_taps, uint32_t table_size)
{
    sinc->num_taps = num_taps;
    sinc->table_size = table_size;
    sinc->buffer_index = 0;
    sinc->output = 0.0f;

    // Allocate memory for the data buffer
    sinc->buffer = (ctrl_gt*)malloc(num_taps * sizeof(ctrl_gt));
    if (sinc->buffer == NULL)
        return 0;

    // Allocate memory for the look-up table (array of pointers)
    sinc->sinc_table = (ctrl_gt**)malloc(table_size * sizeof(ctrl_gt*));
    if (sinc->sinc_table == NULL)
    {
        free(sinc->buffer);
        return 0;
    }

    // Allocate memory for each row of the table
    for (uint32_t i = 0; i < table_size; i++)
    {
        sinc->sinc_table[i] = (ctrl_gt*)malloc(num_taps * sizeof(ctrl_gt));
        if (sinc->sinc_table[i] == NULL)
        {
            // Clean up on failure
            for (uint32_t j = 0; j < i; j++)
                free(sinc->sinc_table[j]);
            free(sinc->sinc_table);
            free(sinc->buffer);
            return 0;
        }
    }

    // --- Key Point Analysis 1: Pre-calculate the Windowed-Sinc Coefficient Table ---
    // This is the core of the module. By pre-calculating all potentially needed FIR
    // filter coefficients at once, it avoids expensive sin/cos calculations in the real-time loop.
    for (uint32_t i = 0; i < table_size; i++)
    {
        // 'fractional_offset' represents the sub-sample offset (0.0 to 1.0) for the filter currently being calculated.
        float fractional_offset = (float)i / table_size;

        for (uint32_t j = 0; j < num_taps; j++)
        {
            // 't' is the time-axis variable for the Sinc function, shifted and centered.
            float t = (float)j - (float)(num_taps - 1) / 2.0f - fractional_offset;

            // Calculate the Sinc function value: sin(pi*t) / (pi*t)
            float sinc_val;
            if (t == 0.0f)
            {
                sinc_val = 1.0f;
            }
            else
            {
                sinc_val = sinf(PI * t) / (PI * t);
            }

            // Calculate the Blackman window value to smooth the truncation effects of the Sinc function
            // and reduce Gibbs phenomenon in the frequency domain.
            float window_val =
                0.42f - 0.5f * cosf(2.0f * PI * j / (num_taps - 1)) + 0.08f * cosf(4.0f * PI * j / (num_taps - 1));

            // The final filter coefficient is the product of the Sinc value and the window value.
            sinc->sinc_table[i][j] = (ctrl_gt)(sinc_val * window_val);
        }
    }

    // Initialize the data buffer
    ctl_clear_sinc_interpolator(sinc);
    return 1;
}

/**
 * @brief Frees the memory allocated for the Sinc interpolator.
 * @param[in,out] sinc Pointer to the Sinc interpolator instance.
 */
GMP_STATIC_INLINE void ctl_destroy_sinc_interpolator(ctl_sinc_interpolator_t* sinc)
{
    if (sinc->sinc_table != NULL)
    {
        for (uint32_t i = 0; i < sinc->table_size; i++)
        {
            if (sinc->sinc_table[i] != NULL)
            {
                free(sinc->sinc_table[i]);
            }
        }
        free(sinc->sinc_table);
        sinc->sinc_table = NULL;
    }
    if (sinc->buffer != NULL)
    {
        free(sinc->buffer);
        sinc->buffer = NULL;
    }
}

/**
 * @brief Clears the internal data buffer of the interpolator.
 * @param[out] sinc Pointer to the Sinc interpolator instance.
 */
GMP_STATIC_INLINE void ctl_clear_sinc_interpolator(ctl_sinc_interpolator_t* sinc)
{
    if (sinc->buffer != NULL)
    {
        for (uint32_t i = 0; i < sinc->num_taps; i++)
        {
            sinc->buffer[i] = 0.0f;
        }
    }
    sinc->output = 0.0f;
    sinc->buffer_index = 0;
}

/**
 * @brief Executes one step of the Sinc interpolation.
 * @param[in,out] sinc Pointer to the Sinc interpolator instance.
 * @param[in] input The new input sample, x(n).
 * @param[in] fractional_delay The desired fractional delay (0.0 to 1.0) to interpolate at.
 * @return ctrl_gt The interpolated output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_sinc_interpolator(ctl_sinc_interpolator_t* sinc, ctrl_gt input,
                                                     parameter_gt fractional_delay)
{
    // --- Key Point Analysis 2: Update the Circular Buffer for Input Samples ---
    sinc->buffer[sinc->buffer_index] = input;

    // --- Key Point Analysis 3: Select FIR Filter Coefficients Based on Fractional Delay ---
    // 'fractional_delay' is mapped to an index in the pre-calculated table, selecting
    // the best-matching set of Sinc coefficients.
    uint32_t table_index = (uint32_t)(fractional_delay * sinc->table_size);
    if (table_index >= sinc->table_size)
    {
        table_index = sinc->table_size - 1;
    }
    ctrl_gt* kernel = sinc->sinc_table[table_index];

    // --- Key Point Analysis 4: Perform the Convolution Operation ---
    // The output is calculated by convolving (dot product) the selected Sinc kernel
    // (FIR coefficients) with the samples in the input buffer.
    sinc->output = 0.0f;
    uint32_t j = sinc->buffer_index;
    for (uint32_t i = 0; i < sinc->num_taps; i++)
    {
        sinc->output += kernel[i] * sinc->buffer[j];
        if (j == 0)
        {
            j = sinc->num_taps - 1;
        }
        else
        {
            j--;
        }
    }

    // Advance the circular buffer index for the next sample.
    sinc->buffer_index++;
    if (sinc->buffer_index >= sinc->num_taps)
    {
        sinc->buffer_index = 0;
    }

    return sinc->output;
}

/**
 * @}
 */ // end of sinc_interpolator group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _SINC_INTERPOLATOR_H_
