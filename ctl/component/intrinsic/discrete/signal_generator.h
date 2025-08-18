/**
 * @file signal_generator.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides several discrete signal generator modules.
 * @version 0.2
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _SIGNAL_GENERATOR_H_
#define _SIGNAL_GENERATOR_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup signal_generators Signal Generators
 * @brief A library of modules for generating standard test waveforms.
 * @details This file contains implementations for common signal generators used
 * for simulation, testing, and control system stimulation. It includes a
 * sine/cosine wave generator based on an iterative phasor rotation and a
 * ramp wave generator.

 * @{
 */

/*---------------------------------------------------------------------------*/
/* Sine/Cosine Wave Generator                                                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Sine/Cosine wave generator.
 * @details This module generates sine and cosine waves iteratively using phasor
 * rotation, which is computationally efficient. It avoids repeated calls to
 * `sin()` and `cos()` functions inside a loop.
 */
typedef struct _tag_sine_generator_t
{
    ctrl_gt ph_sin_delta; //!< The sine of the angle step per iteration.
    ctrl_gt ph_cos_delta; //!< The cosine of the angle step per iteration.
    ctrl_gt ph_sin;       //!< The current sine output value.
    ctrl_gt ph_cos;       //!< The current cosine output value.
} ctl_sine_generator_t;

/**
 * @brief Initializes the sine/cosine wave generator.
 * @param[out] sg Pointer to the sine generator instance.
 * @param[in] init_angle The initial angle in per-unit (pu), where 1.0 pu = 2*PI radians.
 * @param[in] step_angle The angle to advance in each step, in per-unit (pu).
 */
void ctl_init_sine_generator(ctl_sine_generator_t* sg, parameter_gt init_angle, parameter_gt step_angle);

/**
 * @brief Executes one step of the sine/cosine generator.
 * @details Updates the sine and cosine outputs by rotating the phasor by the step angle.
 * This is based on the angle sum identities:
 * sin(A+B) = sin(A)cos(B) + cos(A)sin(B)
 * cos(A+B) = cos(A)cos(B) - sin(A)sin(B)
 * @param[in,out] sg Pointer to the sine generator instance.
 */
GMP_STATIC_INLINE void ctl_step_sine_generator(ctl_sine_generator_t* sg)
{
    ctrl_gt sin_new = sg->ph_sin * sg->ph_cos_delta + sg->ph_cos * sg->ph_sin_delta;
    ctrl_gt cos_new = sg->ph_cos * sg->ph_cos_delta - sg->ph_sin * sg->ph_sin_delta;

    sg->ph_sin = sin_new;
    sg->ph_cos = cos_new;
}

/**
 * @brief Gets the current sine output.
 * @param[in] sg Pointer to the sine generator instance.
 * @return ctrl_gt The current sine value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_sine_generator_sin(ctl_sine_generator_t* sg)
{
    return sg->ph_sin;
}

/**
 * @brief Gets the current cosine output.
 * @param[in] sg Pointer to the sine generator instance.
 * @return ctrl_gt The current cosine value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_sine_generator_cos(ctl_sine_generator_t* sg)
{
    return sg->ph_cos;
}

/*---------------------------------------------------------------------------*/
/* Ramp Wave Generator                                                       */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Ramp wave generator.
 * @details Generates a sawtooth waveform that increases linearly from a minimum
 * to a maximum value, then resets.
 */
typedef struct _tag_ramp_generator_t
{
    ctrl_gt minimum; //!< The minimum value of the ramp (reset value).
    ctrl_gt maximum; //!< The maximum value of the ramp.
    ctrl_gt current; //!< The current output value.
    ctrl_gt slope;   //!< The amount to increment the output at each step.
} ctl_ramp_generator_t;

/**
 * @brief Initializes the ramp generator with a specified slope.
 * @param[out] rg Pointer to the ramp generator instance.
 * @param[in] slope The value to add to the output at each step.
 * @param[in] amp_pos The positive peak (maximum) value.
 * @param[in] amp_neg The negative peak (minimum) value.
 */
void ctl_init_ramp_generator(ctl_ramp_generator_t* rg, ctrl_gt slope, parameter_gt amp_pos, parameter_gt amp_neg);

/**
 * @brief Initializes the ramp generator based on frequency and amplitude.
 * @details Calculates the required slope to achieve a target frequency.
 * @param[out] rg Pointer to the ramp generator instance.
 * @param[in] isr_freq The frequency at which the step function is called (Hz).
 * @param[in] target_freq The desired frequency of the ramp wave (Hz).
 * @param[in] amp_pos The positive peak (maximum) value.
 * @param[in] amp_neg The negative peak (minimum) value.
 */
void ctl_init_ramp_generator_via_freq(ctl_ramp_generator_t* rg, parameter_gt isr_freq, parameter_gt target_freq,
                                      parameter_gt amp_pos, parameter_gt amp_neg);

/**
 * @brief Executes one step of the ramp generator.
 * @param[in,out] rg Pointer to the ramp generator instance.
 * @return ctrl_gt The new output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_ramp_generator(ctl_ramp_generator_t* rg)
{
    rg->current += rg->slope;

    if (rg->current > rg->maximum)
    {
        rg->current = rg->minimum;
    }

    return rg->current;
}

/**
 * @brief Gets the current output of the ramp generator.
 * @param[in] rg Pointer to the ramp generator instance.
 * @return ctrl_gt The current ramp value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ramp_generator_output(ctl_ramp_generator_t* rg)
{
    return rg->current;
}

/**
 * @brief Sets a new slope for the ramp generator.
 * @param[out] rg Pointer to the ramp generator instance.
 * @param[in] slope The new slope value.
 */
GMP_STATIC_INLINE void ctl_set_ramp_generator_slope(ctl_ramp_generator_t* rg, ctrl_gt slope)
{
    rg->slope = slope;
}

/*---------------------------------------------------------------------------*/
/* Square Wave Generator                                                     */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Square wave generator.
 */
typedef struct _tag_square_wave_generator_t
{
    ctrl_gt high_level; //!< The high output level (offset + amplitude).
    ctrl_gt low_level;  //!< The low output level (offset - amplitude).
    ctrl_gt phase;      //!< The internal phase accumulator (0 to 2*PI).
    ctrl_gt phase_step; //!< The phase increment per step.
    ctrl_gt output;     //!< The current output value.
} ctl_square_wave_generator_t;

/**
 * @brief Initializes the square wave generator.
 * @param[out] sq Pointer to the square wave generator instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] target_freq The desired frequency of the square wave (Hz).
 * @param[in] amplitude The amplitude of the wave (half of peak-to-peak).
 * @param[in] offset The DC offset of the wave.
 */
void ctl_init_square_wave_generator(ctl_square_wave_generator_t* sq, parameter_gt fs, parameter_gt target_freq,
                                    parameter_gt amplitude, parameter_gt offset);

/**
 * @brief Executes one step of the square wave generator.
 * @return ctrl_gt The new output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_square_wave_generator(ctl_square_wave_generator_t* sq)
{
    // Accumulate phase
    sq->phase += sq->phase_step;

    // Check for phase wrap-around
    if (sq->phase >= (2.0f * CTL_PARAM_CONST_PI))
    {
        sq->phase -= (2.0f * CTL_PARAM_CONST_PI);
    }

    // Determine output based on phase
    // Output is high for the first half of the cycle (0 to PI)
    // and low for the second half (PI to 2*PI).
    if (sq->phase < CTL_PARAM_CONST_PI)
    {
        sq->output = sq->high_level;
    }
    else
    {
        sq->output = sq->low_level;
    }

    return sq->output;
}

/*---------------------------------------------------------------------------*/
/* Triangle Wave Generator                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Triangle wave generator.
 */
typedef struct _tag_triangle_wave_generator_t
{
    ctrl_gt pos_peak; //!< The positive peak value.
    ctrl_gt neg_peak; //!< The negative peak value.
    ctrl_gt slope;    //!< The current slope (can be positive or negative).
    ctrl_gt output;   //!< The current output value.
} ctl_triangle_wave_generator_t;

/**
 * @brief Initializes the triangle wave generator.
 * @param[out] tri Pointer to the triangle wave generator instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] target_freq The desired frequency of the triangle wave (Hz).
 * @param[in] pos_peak The positive peak value.
 * @param[in] neg_peak The negative peak value.
 */
void ctl_init_triangle_wave_generator(ctl_triangle_wave_generator_t* tri, parameter_gt fs, parameter_gt target_freq,
                                      parameter_gt pos_peak, parameter_gt neg_peak);

/**
 * @brief Executes one step of the triangle wave generator.
 * @return ctrl_gt The new output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_triangle_wave_generator(ctl_triangle_wave_generator_t* tri)
{
    // Update output with the current slope
    tri->output += tri->slope;

    // Check for peak reversal
    if (tri->output >= tri->pos_peak)
    {
        tri->output = tri->pos_peak; // Clamp to peak
        tri->slope = -tri->slope;    // Reverse direction
    }
    else if (tri->output <= tri->neg_peak)
    {
        tri->output = tri->neg_peak; // Clamp to peak
        tri->slope = -tri->slope;    // Reverse direction
    }

    return tri->output;
}

/**
 * @}
 */ // end of signal_generators group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _SIGNAL_GENERATOR_H_
