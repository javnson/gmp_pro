/**
 * @file sine_analyzer.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a module to analyze a sine wave signal, calculating its RMS, average value, and frequency.
 * @details This module operates by detecting zero-crossings to identify complete cycles of the input waveform.
 * It is recommended to use this module when the control data type (ctrl_gt) is floating-point
 * to avoid potential calculation errors with fixed-point arithmetic.
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_SINE_ANALYZER_H_
#define _FILE_SINE_ANALYZER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup SINE_ANALYZER Sine Wave Analyzer
 * @brief A module for real-time analysis of sinusoidal signals.
 *
 * This module calculates the following properties of an input sine wave:
 * - **RMS Value**: @f$ V_{rms} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} v_i^2} @f$
 * - **Average Value** (of absolute signal): @f$ V_{avg} = \frac{1}{N} \sum_{i=1}^{N} |v_i|} @f$
 * - **Frequency**
 */

/*---------------------------------------------------------------------------*/
/* Sine Wave Analyzer                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup SINE_ANALYZER
 * @{
 */

/**
 * @brief Data structure for the sine wave analyzer.
 */
typedef struct _tag_sine_analyzer
{
    //
    // Outputs
    //
    ctrl_gt wave_rms;   /**< The calculated RMS value of the wave over one cycle. */
    ctrl_gt wave_avg;   /**< The calculated average of the absolute value of the wave over one cycle. */
    ctrl_gt ac_freq_pu; /**< The calculated frequency in per-unit, relative to the rated frequency. */
    fast_gt zero_cross; /**< A flag that is set to 1 for one cycle when a zero-crossing is detected. */

    //
    // Parameters
    //
    ctrl_gt threshold;    /**< The threshold used for zero-crossing detection to provide noise immunity. */
    ctrl_gt freq_sf;      /**< The scaling factor to convert the sample count into a per-unit frequency. */
    fast32_gt sample_min; /**< The minimum number of samples expected in a cycle, used for noise rejection. */
    fast32_gt sample_max; /**< The maximum number of samples expected in a cycle, used for timeout detection. */

    //
    // Internal State Variables
    //
    ctrl_gt wave_norm;        /**< The absolute value of the current input wave. */
    fast_gt prev_sign;        /**< The sign of the wave from the previous step. */
    fast_gt curr_sign;        /**< The sign of the wave in the current step. */
    ctrl_gt sumup;            /**< The accumulator for the sum of absolute values. */
    ctrl_gt sumup_sqr;        /**< The accumulator for the sum of squares of absolute values. */
    ctrl_gt inv_index;        /**< The pre-calculated inverse of the sample count (1/N). */
    ctrl_gt inv_index_sqrt;   /**< The pre-calculated inverse of the square root of the sample count (1/sqrt(N)). */
    fast32_gt sample_index;   /**< The counter for the number of samples in the current cycle. */
    fast32_gt jitter_counter; /**< A counter to handle jitter around the zero-crossing point. */

} sine_analyzer_t;

void ctl_init_sine_analyzer(sine_analyzer_t* sine, parameter_gt zcd_threshold, parameter_gt min_freq,
                            parameter_gt max_freq, parameter_gt rated_freq, parameter_gt f_ctrl);

/**
 * @brief Resets the sine analyzer's accumulators and output values.
 * @param sine Pointer to the `sine_analyzer_t` object.
 */
GMP_STATIC_INLINE void ctl_sine_clear_analyzer(sine_analyzer_t* sine)
{
    sine->zero_cross = 0;
    sine->wave_avg = 0;
    sine->wave_rms = 0;
    sine->ac_freq_pu = 0;
    sine->sample_index = 0;
    sine->sumup = 0;
    sine->sumup_sqr = 0;
}

/**
 * @brief Executes one step of the sine wave analysis.
 * @details This function should be called at a fixed rate (the controller frequency). It processes
 * one sample of the input wave and updates the RMS, average, and frequency values upon detecting
 * the completion of a full cycle.
 * @param sine Pointer to the `sine_analyzer_t` object.
 * @param wave_input The current sample of the input sine wave.
 * @return The latest calculated RMS value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_sine_analyzer(sine_analyzer_t* sine, ctrl_gt wave_input)
{
    sine->wave_norm = ctl_abs(wave_input);
    sine->curr_sign = (wave_input > sine->threshold) ? 1 : 0;
    sine->sample_index++;
    sine->sumup += sine->wave_norm;
    sine->sumup_sqr += (sine->wave_norm * sine->wave_norm);
    sine->zero_cross = 0;

    // Check for a positive-going zero-crossing, which marks the end of a full cycle.
    if (sine->prev_sign != sine->curr_sign && sine->curr_sign == 1)
    {
        // Check if the cycle duration is within the expected valid range.
        if (sine->sample_index > sine->sample_min)
        {
            sine->zero_cross = 1;

            // Pre-calculate inverse values for efficiency.
            sine->inv_index = float2ctrl(1.0f / sine->sample_index);
            sine->inv_index_sqrt = ctl_sqrt(sine->inv_index);

            // Calculate average, RMS, and frequency.
            sine->wave_avg = ctl_mul(sine->inv_index, sine->sumup);
            sine->wave_rms = ctl_mul(ctl_sqrt(sine->sumup_sqr), sine->inv_index_sqrt);
            sine->ac_freq_pu = ctl_mul(sine->freq_sf, sine->inv_index);

            // Reset accumulators for the next cycle.
            sine->sample_index = 0;
            sine->sumup = 0;
            sine->sumup_sqr = 0;
        }
        // Handle cases where a zero-crossing happens too quickly (likely due to noise/jitter).
        else
        {
            if (sine->jitter_counter <= 30)
            {
                sine->jitter_counter += 1;
            }
            sine->sample_index = 0;
        }
    }

    // Protection: If no valid wave is detected for too long, reset the analyzer.
    if (sine->sample_index > sine->sample_max || sine->jitter_counter > 20)
    {
        ctl_sine_clear_analyzer(sine);
    }

    // Update the sign for the next iteration.
    sine->prev_sign = sine->curr_sign;

    return sine->wave_rms;
}

/**
 * @brief Gets the last calculated RMS value.
 * @param sine Pointer to the `sine_analyzer_t` object.
 * @return The RMS value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_sa_rms(sine_analyzer_t* sine)
{
    return sine->wave_rms;
}

/**
 * @brief Gets the last calculated average value.
 * @param sine Pointer to the `sine_analyzer_t` object.
 * @return The average value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_sa_avg(sine_analyzer_t* sine)
{
    return sine->wave_avg;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SINE_ANALYZER_H_
