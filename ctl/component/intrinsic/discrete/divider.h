/**
 * @file divider.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a simple counter-based frequency divider.
 * @version 0.2
 * @date 2024-10-25
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file implements a basic frequency divider (or prescaler). It is
 * used to generate a periodic trigger or event at a frequency that is an integer
 * fraction of a higher input frequency. For example, it can be used to execute
 * a slow control loop every N calls of a faster main ISR.
 */

#ifndef _DIVIDER_H_
#define _DIVIDER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup frequency_divider Frequency Divider
 * @brief A simple counter-based module to divide a clock or event frequency.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Counter-based Frequency Divider                                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the frequency divider.
 */
typedef struct _tag_divider_t
{
    uint32_t counter; //!< The internal counter, increments at each step.
    uint32_t target;  //!< The period of the divider. The output will trigger when counter reaches this value.
} ctl_divider_t;

/**
 * @brief Initializes the frequency divider module.
 * @param[out] obj Pointer to the divider instance.
 * @param[in] counter_period The desired division ratio. The output frequency will be
 * the input frequency divided by this value. A value of 1
 * means the output triggers on every call.
 */
void ctl_init_divider(ctl_divider_t* obj, uint32_t counter_period);

/**
 * @brief Executes one step of the frequency divider.
 * @details Increments the internal counter. When the counter reaches the target value,
 * it resets the counter and returns 1. Otherwise, it returns 0.
 * @param[in,out] obj Pointer to the divider instance.
 * @return fast_gt Returns 1 if the division period is complete (trigger event), otherwise 0.
 */
GMP_STATIC_INLINE fast_gt ctl_step_divider(ctl_divider_t* obj)
{
    obj->counter++;

    if (obj->counter >= obj->target)
    {
        obj->counter = 0;
        return 1; // Trigger the event
    }

    return 0;
}

/**
 * @brief Clears the internal counter of the divider.
 * @details Resets the counter to 0, effectively restarting the division cycle.
 * @param[out] obj Pointer to the divider instance.
 */
GMP_STATIC_INLINE void ctl_clear_divider(ctl_divider_t* obj)
{
    obj->counter = 0;
}

/**
 * @}
 */ // end of frequency_divider group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //_DIVIDER_H_
