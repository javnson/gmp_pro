/**
 * @file hysteresis_controller.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a hysteresis (bang-bang) controller.
 * @version 1.05
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */
#ifndef _HYSTERESIS_CONTROLLER_H_
#define _HYSTERESIS_CONTROLLER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup hysteresis_controller Hysteresis Controller
 * @brief A nonlinear controller that switches output based on a hysteresis band.
 * @details This module implements a hysteresis controller, also known as a
 * bang-bang controller or comparator with hysteresis. It is a nonlinear
 * controller that switches its output between two states based on whether the
 * input signal crosses the upper or lower bounds of a hysteresis band. This
 * method is common in applications like current control for power converters
 * due to its fast transient response.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Hysteresis Current Controller (HCC)                                       */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the hysteresis controller.
 */
typedef struct _tag_hysteresis_controller_t
{
    ctrl_gt target;        //!< The center of the hysteresis band.
    ctrl_gt half_width;    //!< The half-width of the hysteresis band (h).
    ctrl_gt current;       //!< The last input value, for reference.
    fast_gt switch_out;    //!< The current digital output state (0 or 1).
    fast_gt flag_polarity; //!< Determines the output state polarity (0 or 1).
} ctl_hysteresis_controller_t;

/**
 * @brief Initializes the hysteresis controller.
 * @param[out] hcc Pointer to the hysteresis controller instance.
 * @param[in] flag_polarity The output state when the input exceeds the upper band.
 * If 1, output becomes 1. If 0, output becomes 0.
 * @param[in] half_width The half-width of the hysteresis band.
 */
void ctl_init_hysteresis_controller(ctl_hysteresis_controller_t* hcc, fast_gt flag_polarity, ctrl_gt half_width);

/**
 * @brief Executes one step of the hysteresis controller.
 * @details Compares the input against the upper (target + half_width) and lower
 * (target - half_width) bounds and updates the switch output accordingly.
 * The output state is latched until the opposite boundary is crossed.
 * @param[in,out] hcc Pointer to the hysteresis controller instance.
 * @param[in] input The current signal to be controlled (e.g., measured current).
 * @return fast_gt The new digital output state (0 or 1).
 */
GMP_STATIC_INLINE fast_gt ctl_step_hysteresis_controller(ctl_hysteresis_controller_t* hcc, ctrl_gt input)
{
    hcc->current = input;

    if (hcc->current >= (hcc->target + hcc->half_width))
    {
        hcc->switch_out = hcc->flag_polarity;
    }
    else if (hcc->current <= (hcc->target - hcc->half_width))
    {
        hcc->switch_out = 1 - hcc->flag_polarity;
    }
    // If inside the band, the output remains unchanged.

    return hcc->switch_out;
}

/**
 * @brief Sets a new target (center) for the hysteresis band.
 * @param[out] hcc Pointer to the hysteresis controller instance.
 * @param[in] target The new target value.
 */
GMP_STATIC_INLINE void ctl_set_hysteresis_target(ctl_hysteresis_controller_t* hcc, ctrl_gt target)
{
    hcc->target = target;
}

/**
 * @brief Gets the current output state of the controller.
 * @param[in] hcc Pointer to the hysteresis controller instance.
 * @return fast_gt The current switch output (0 or 1).
 */
GMP_STATIC_INLINE fast_gt ctl_get_hysteresis_output(ctl_hysteresis_controller_t* hcc)
{
    return hcc->switch_out;
}

/**
 * @}
 */ // end of hysteresis_controller group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //_HYSTERESIS_CONTROLLER_H_
