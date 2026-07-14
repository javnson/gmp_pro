/**
 * @file dsa_trigger.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a basic data acquisition trigger for logging data based on a signal event.

 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/math_block/utilities/mem_view.h>

#ifndef _FILE_DSA_TRIGGER_H_
#define _FILE_DSA_TRIGGER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup DSA_TRIGGER Data Acquisition Trigger
 * @brief A module for event-based data logging.
 * @details This module implements a simple trigger mechanism that starts recording data
 * for a fixed duration when a monitored signal crosses zero from negative to positive.
 * It is useful for capturing transient events for debugging or analysis.
 *
 * @code
 * // Usage Demo:
 * #define MONITOR_BUFFER_SIZE 400
 * ctrl_gt monitor_buffer[MONITOR_BUFFER_SIZE];
 *
 * basic_trigger_t trigger;
 *
 * // Initialization
 * memset(monitor_buffer, 0, MONITOR_BUFFER_SIZE * sizeof(ctrl_gt));
 * dsa_init_basic_trigger(&trigger, MONITOR_BUFFER_SIZE);
 *
 * // In main ISR function
 * if (dsa_step_trigger(&trigger, channel1_data))
 * {
 * monitor_buffer[dsa_get_trigger_index(&trigger)] = channel1_data;
 * // other channels can be logged here...
 * // monitor_buffer2[dsa_get_trigger_index(&trigger)] = channel2_data;
 * }
 * @endcode
 */

/*---------------------------------------------------------------------------*/
/* Basic Data Acquisition Trigger                                            */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup DSA_TRIGGER
 * @{
 */

/*---------------------------------------------------------------------------*/
/* DSA Trigger Modules                                                       */
/*---------------------------------------------------------------------------*/

/**
 * @brief Options for DSA triggering behavior.
 */
typedef enum _tag_ctl_trigger_option
{
    DSA_TRIGGER_OPTION_CONTINUOUS = 0,       //!< Always triggered, ignores conditions.
    DSA_TRIGGER_OPTION_RISING_EDGE = 1,      //!< Triggers on rising edge only.
    DSA_TRIGGER_OPTION_FALLING_EDGE = 2,     //!< Triggers on falling edge only.
    DSA_TRIGGER_OPTION_RISING_EDGE_AUTO = 3, //!< Triggers on rising edge, or forces trigger on timeout.
    DSA_TRIGGER_OPTION_FALLING_EDGE_AUTO = 4 //!< Triggers on falling edge, or forces trigger on timeout.
} ctl_dsa_trigger_option_t;

/**
 * @brief Data structure for the DSA trigger module.
 */
typedef struct _tag_dsa_trigger
{
    ctl_dsa_trigger_option_t option; //!< Selected trigger option mode.
    ctrl_gt trigger_level;           //!< Target threshold value for edge detection.

    // --- Internal State Variables ---
    ctrl_gt prev_input;            //!< Cached signal value from the previous execution cycle.
    uint32_t auto_timeout_ticks;   //!< Maximum execution ticks to wait before forcing an AUTO trigger.
    uint32_t auto_timer;           //!< Internal counter tracking the timeout period.
    fast_gt flag_is_first_step;    //!< Guard flag to prevent false edge triggers on initialization.
    fast_gt flag_is_force_trigger; //!< Force trigger at next step, this variable will auto clear.
} ctl_dsa_trigger_t;

GMP_STATIC_INLINE void ctl_clear_dsa_trigger(ctl_dsa_trigger_t* obj)
{
    obj->prev_input = 0;
    obj->auto_timer = 0;
    obj->flag_is_first_step = 0;
}

GMP_STATIC_INLINE void ctl_init_dsa_trigger(ctl_dsa_trigger_t* obj, ctl_dsa_trigger_option_t trigger_option,
                                            parameter_gt trigger_level,
                                            parameter_gt trigger_suppression, // unit, s
                                            parameter_gt isr_freq)            // sample frequency
{
    obj->option = trigger_option;
    obj->trigger_level = float2ctrl(trigger_level);
    obj->auto_timeout_ticks = (uint32_t)trigger_suppression * isr_freq;

    ctl_clear_dsa_trigger(obj);
}

/**
 * @brief Evaluates the trigger logic based on the current signal input.
 * @param[in,out] obj Pointer to the DSA trigger instance.
 * @param[in]     input The current value of the source signal under observation.
 * @return fast_gt Returns 1 if a trigger condition (edge or timeout) is met, otherwise 0.
 */
GMP_STATIC_INLINE fast_gt ctl_step_dsa_trigger(ctl_dsa_trigger_t* obj, ctrl_gt input)
{
    fast_gt is_triggered = 0;

    if (obj->option == DSA_TRIGGER_OPTION_CONTINUOUS)
    {
        is_triggered = 1;
    }
    else
    {
        if (!obj->flag_is_first_step)
        {
            // Evaluate hardware edge matching conditions
            if (obj->option == DSA_TRIGGER_OPTION_RISING_EDGE || obj->option == DSA_TRIGGER_OPTION_RISING_EDGE_AUTO)
            {
                if ((obj->prev_input < obj->trigger_level) && (input >= obj->trigger_level))
                {
                    is_triggered = 1;
                }
            }
            else if (obj->option == DSA_TRIGGER_OPTION_FALLING_EDGE ||
                     obj->option == DSA_TRIGGER_OPTION_FALLING_EDGE_AUTO)
            {
                if ((obj->prev_input > obj->trigger_level) && (input <= obj->trigger_level))
                {
                    is_triggered = 1;
                }
            }

            // Evaluate software auto-timeout conditions
            if (!is_triggered && (obj->option == DSA_TRIGGER_OPTION_RISING_EDGE_AUTO ||
                                  obj->option == DSA_TRIGGER_OPTION_FALLING_EDGE_AUTO))
            {
                obj->auto_timer++;
                if (obj->auto_timer >= obj->auto_timeout_ticks)
                {
                    is_triggered = 1;
                }
            }
        }
        else
        {
            obj->flag_is_first_step = 0;
        }

        // Cache current input for the next cycle edge comparison
        obj->prev_input = input;
    }

    // Force trigger scenario
    if (obj->flag_is_force_trigger)
    {
        is_triggered = 1;
        obj->flag_is_force_trigger = 0;
    }

    if (is_triggered)
    {
        obj->auto_timer = 0;
    }

    return is_triggered;
}

/*---------------------------------------------------------------------------*/
/* DSA Logger Modules                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief State machine for the DSA data logger.
 * @note if @ctl_dsa_logger_t::flag_single_trigger is set after each sample the result 
 * will keep and waiting at DSA_LOGGER_COMPLETE state.  
 */
typedef enum _tag_ctl_dsa_logger_sm
{
    DSA_LOGGER_READY = 0,    //!< Waiting for a valid trigger event.
    DSA_LOGGER_SAMPLING = 1, //!< Triggered, actively filling the memory buffer.
    DSA_LOGGER_COMPLETE = 2  //!< Buffer is full, sampling complete.
} ctl_dsa_logger_sm_t;

/**
 * @brief Data structure for the DSA data logger module.
 */
typedef struct _tag_dsa_logger
{
    ctl_dsa_logger_sm_t sm;    //!< Current operational state of the logger machine.
    ctl_mem_view_t mem;        //!< Memory view manager bound to the data destination.
    ctl_divider_t divider;     //!< Frequency divider determining the down-sampling rate.
    ctl_dsa_trigger_t trigger; //!< Embedded hardware trigger instance.

    uint16_t channels;         //!< Active logging channel count (1 to 4).
    uint32_t channel_capacity; //!< Allocation limit size per continuous channel block.
    uint32_t current_position; //!< Local element tracking offset within a single channel segment.

    fast_gt flag_single_trigger; //!< Mode configuration: if true, holds in COMPLETE state.
} ctl_dsa_logger_t;

/**
 * @brief Initializes the segmented block layout multi-channel DSA logger module.
 */
void ctl_init_dsa_logger(ctl_dsa_logger_t* logger, ctrl_gt* mem_pool, uint32_t capacity, uint16_t channels,
                         uint32_t divider_step, ctl_dsa_trigger_option_t trigger_option, parameter_gt trigger_level,
                         parameter_gt trigger_suppression, parameter_gt isr_freq);

//void ctl_init_dsa_logger(ctl_dsa_logger_t* logger, ctrl_gt* mem_pool, uint32_t capacity, uint16_t channels,
//                         uint32_t divider_step, ctl_dsa_trigger_option_t trigger_option, parameter_gt trigger_level,
//                         parameter_gt trigger_suppression, // unit, s
//                         parameter_gt isr_freq)            // unit, Hz
//{
//    logger->sm = DSA_LOGGER_READY;
//    ctl_init_divider(&logger->divider, divider_step);
//    ctl_init_mem_view(&logger->mem, mem_pool, capacity);
//    ctl_init_dsa_trigger(&logger->trigger, trigger_option, trigger_level, trigger_suppression, isr_freq);
//
//    logger->channels = channels;
//    logger->current_position = 0;
//
//    logger->flag_single_trigger = 0;
//}

void ctl_set_dsa_logger_trigger(ctl_dsa_logger_t* logger, ctl_dsa_trigger_option_t trigger_option,
                                parameter_gt trigger_level)
{
    logger->trigger.option = trigger_option;
    logger->trigger.trigger_level = trigger_level;
}

/*---------------------------------------------------------------------------*/
/* Multi-Channel Inline Steppers (Segmented Continuous Block Architecture)   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Step handler for 1-channel signal logging.
 */
GMP_STATIC_INLINE void ctl_step_dsa_logger_1ch(ctl_dsa_logger_t* logger, ctrl_gt trig_signal, ctrl_gt log_signal)
{
    fast_gt trigger_fired = ctl_step_dsa_trigger(&logger->trigger, trig_signal);

    switch (logger->sm)
    {
    case DSA_LOGGER_READY:
        if (trigger_fired)
        {
            logger->current_position = 0;
            ctl_clear_divider(&logger->divider);

            ctl_mem_set_1d(&logger->mem, logger->current_position, log_signal);
            logger->current_position++;

            logger->sm =
                (logger->current_position >= logger->channel_capacity) ? DSA_LOGGER_COMPLETE : DSA_LOGGER_SAMPLING;
        }
        break;

    case DSA_LOGGER_SAMPLING:
        if (ctl_step_divider(&logger->divider))
        {
            ctl_mem_set_1d(&logger->mem, logger->current_position, log_signal);
            logger->current_position++;

            if (logger->current_position >= logger->channel_capacity)
            {
                logger->sm = DSA_LOGGER_COMPLETE;
            }
        }
        break;

    case DSA_LOGGER_COMPLETE:
        // continuous trigger
        if (logger->flag_single_trigger == 0)
            logger->sm = DSA_LOGGER_READY;

        break;
    default:
        break;
    }
}

/**
 * @brief Step handler for 2-channel segmented continuous block logging.
 */
GMP_STATIC_INLINE void ctl_step_dsa_logger_2ch(ctl_dsa_logger_t* logger, ctrl_gt trig_signal, ctrl_gt log_signal_ch1,
                                               ctrl_gt log_signal_ch2)
{
    fast_gt trigger_fired = ctl_step_dsa_trigger(&logger->trigger, trig_signal);

    switch (logger->sm)
    {
    case DSA_LOGGER_READY:
        if (trigger_fired)
        {
            logger->current_position = 0;
            ctl_clear_divider(&logger->divider);

            // Write to continuous separate segments
            ctl_mem_set_1d(&logger->mem, logger->current_position, log_signal_ch1);
            ctl_mem_set_1d(&logger->mem, logger->channel_capacity + logger->current_position, log_signal_ch2);
            logger->current_position++;

            logger->sm =
                (logger->current_position >= logger->channel_capacity) ? DSA_LOGGER_COMPLETE : DSA_LOGGER_SAMPLING;
        }
        break;

    case DSA_LOGGER_SAMPLING:
        if (ctl_step_divider(&logger->divider))
        {
            ctl_mem_set_1d(&logger->mem, logger->current_position, log_signal_ch1);
            ctl_mem_set_1d(&logger->mem, logger->channel_capacity + logger->current_position, log_signal_ch2);
            logger->current_position++;

            if (logger->current_position >= logger->channel_capacity)
            {
                logger->sm = DSA_LOGGER_COMPLETE;
            }
        }
        break;

    case DSA_LOGGER_COMPLETE:
        // continuous trigger
        if (logger->flag_single_trigger == 0)
            logger->sm = DSA_LOGGER_READY;

        break;
    default:
        break;
    }
}

/**
 * @brief Step handler for 3-channel segmented continuous block logging.
 */
GMP_STATIC_INLINE void ctl_step_dsa_logger_3ch(ctl_dsa_logger_t* logger, ctrl_gt trig_signal, ctrl_gt log_signal_ch1,
                                               ctrl_gt log_signal_ch2, ctrl_gt log_signal_ch3)
{
    fast_gt trigger_fired = ctl_step_dsa_trigger(&logger->trigger, trig_signal);
    uint32_t ch_cap = logger->channel_capacity;

    switch (logger->sm)
    {
    case DSA_LOGGER_READY:
        if (trigger_fired)
        {
            logger->current_position = 0;
            ctl_clear_divider(&logger->divider);

            ctl_mem_set_1d(&logger->mem, logger->current_position, log_signal_ch1);
            ctl_mem_set_1d(&logger->mem, ch_cap + logger->current_position, log_signal_ch2);
            ctl_mem_set_1d(&logger->mem, (ch_cap << 1) + logger->current_position, log_signal_ch3);
            logger->current_position++;

            logger->sm = (logger->current_position >= ch_cap) ? DSA_LOGGER_COMPLETE : DSA_LOGGER_SAMPLING;
        }
        break;

    case DSA_LOGGER_SAMPLING:
        if (ctl_step_divider(&logger->divider))
        {
            ctl_mem_set_1d(&logger->mem, logger->current_position, log_signal_ch1);
            ctl_mem_set_1d(&logger->mem, ch_cap + logger->current_position, log_signal_ch2);
            ctl_mem_set_1d(&logger->mem, (ch_cap << 1) + logger->current_position, log_signal_ch3);
            logger->current_position++;

            if (logger->current_position >= ch_cap)
            {
                logger->sm = DSA_LOGGER_COMPLETE;
            }
        }
        break;

    case DSA_LOGGER_COMPLETE:
        // continuous trigger
        if (logger->flag_single_trigger == 0)
            logger->sm = DSA_LOGGER_READY;

        break;
    default:
        break;
    }
}

/**
 * @brief Step handler for 4-channel segmented continuous block logging.
 */
GMP_STATIC_INLINE void ctl_step_dsa_logger_4ch(ctl_dsa_logger_t* logger, ctrl_gt trig_signal, ctrl_gt log_signal_ch1,
                                               ctrl_gt log_signal_ch2, ctrl_gt log_signal_ch3, ctrl_gt log_signal_ch4)
{
    fast_gt trigger_fired = ctl_step_dsa_trigger(&logger->trigger, trig_signal);
    uint32_t ch_cap = logger->channel_capacity;

    switch (logger->sm)
    {
    case DSA_LOGGER_READY:
        if (trigger_fired)
        {
            logger->current_position = 0;
            ctl_clear_divider(&logger->divider);

            ctl_mem_set_1d(&logger->mem, logger->current_position, log_signal_ch1);
            ctl_mem_set_1d(&logger->mem, ch_cap + logger->current_position, log_signal_ch2);
            ctl_mem_set_1d(&logger->mem, (ch_cap << 1) + logger->current_position, log_signal_ch3);
            ctl_mem_set_1d(&logger->mem, (ch_cap * 3) + logger->current_position, log_signal_ch4);
            logger->current_position++;

            logger->sm = (logger->current_position >= ch_cap) ? DSA_LOGGER_COMPLETE : DSA_LOGGER_SAMPLING;
        }
        break;

    case DSA_LOGGER_SAMPLING:
        if (ctl_step_divider(&logger->divider))
        {
            ctl_mem_set_1d(&logger->mem, logger->current_position, log_signal_ch1);
            ctl_mem_set_1d(&logger->mem, ch_cap + logger->current_position, log_signal_ch2);
            ctl_mem_set_1d(&logger->mem, (ch_cap << 1) + logger->current_position, log_signal_ch3);
            ctl_mem_set_1d(&logger->mem, (ch_cap * 3) + logger->current_position, log_signal_ch4);
            logger->current_position++;

            if (logger->current_position >= ch_cap)
            {
                logger->sm = DSA_LOGGER_COMPLETE;
            }
        }
        break;

    case DSA_LOGGER_COMPLETE:
    default:
        break;
    }
}

GMP_STATIC_INLINE void dsa_enable_single_trigger(ctl_dsa_logger_t* dsa)
{
    dsa->flag_single_trigger = 1;
}

GMP_STATIC_INLINE void dsa_disable_single_triigger(ctl_dsa_logger_t* dsa)
{
    dsa->flag_single_trigger = 0;
}

/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the basic data acquisition trigger.
 */
typedef struct _tag_basic_trigger
{
    addr32_gt target_index; /**< The current index for writing into the data buffer. */
    addr32_gt cell_size;    /**< The total size of the data buffer. */
    ctrl_gt last_data;      /**< The value of the monitored signal from the previous step. */
    fast_gt flag_triggered; /**< A flag indicating if the trigger is currently active (recording). */
} basic_trigger_t;

/**
 * @brief Initializes the basic trigger object.
 * @param trigger Pointer to the `basic_trigger_t` object.
 * @param cell_size The size of the buffer that will be used for logging.
 */
GMP_STATIC_INLINE void dsa_init_basic_trigger(basic_trigger_t* trigger, addr32_gt cell_size)
{
    trigger->target_index = 0;
    trigger->cell_size = cell_size;
    trigger->last_data = 0;
    trigger->flag_triggered = 0;
}

/**
 * @brief Executes one step of the trigger logic.
 * @details This function should be called in every control cycle. It checks for the
 * trigger condition (negative-to-positive zero crossing) and manages the recording process.
 * @param trigger Pointer to the `basic_trigger_t` object.
 * @param monitor The current value of the signal to be monitored.
 * @return 1 if the trigger is active (i.e., data should be recorded), 0 otherwise.
 */
GMP_STATIC_INLINE fast_gt dsa_step_trigger(basic_trigger_t* trigger, ctrl_gt monitor)
{
    // If already triggered, continue recording.
    if (trigger->flag_triggered)
    {
        trigger->target_index += 1;

        // Check if the recording sequence is complete.
        if (trigger->target_index > trigger->cell_size)
        {
            // Reset the trigger
            trigger->flag_triggered = 0;
            trigger->target_index = 0;
            trigger->last_data = monitor;
        }
    }
    // If not triggered, check for the trigger condition.
    else
    {
        // Trigger condition: negative-to-positive zero crossing.
        if (trigger->last_data < 0 && monitor >= 0)
        {
            trigger->flag_triggered = 1;
        }

        trigger->last_data = monitor;
    }

    return trigger->flag_triggered;
}

/**
 * @brief Gets the current array index for data logging.
 * @warning This function has a side effect: it may reset the `target_index` if it
 * has overrun the buffer size. This is generally not good practice for a 'get' function.
 * @param trigger Pointer to the `basic_trigger_t` object.
 * @return The 0-based index for the logging buffer.
 */
GMP_STATIC_INLINE addr32_gt dsa_get_trigger_index(basic_trigger_t* trigger)
{
    if (trigger->target_index > trigger->cell_size)
    {
        trigger->target_index = 1;
    }
    else if (trigger->target_index <= 0)
    {
        return 0;
    }

    return trigger->target_index - 1;
}

/**
 * @}
 */

//// typedef struct tag_trigger_memory_2ch
////{
////     // This memory will contain all the logged information
////     // This channel is used to trigger the output
////     ctrl_gt *memory_ch1;
////     ctrl_gt *memory_ch2;
//
////    // monitor target
////    ctrl_gt *monitor_target_ch1;
////    ctrl_gt *monitor_target_ch2;
//
////    // memory length
////    uint32_t log_length;
//
////    // cell size
////    uint32_t cell_size;
//
////    // trigger flag
////    fast_gt flag_trigger;
////    ctrl_gt last_target1;
////    uint32_t current_index;
//
////} trigger_memory_log_t;
//
//// void dsa_init_trigger_memory(
////     // log object
////     trigger_memory_log_t *log,
////     // monitor target 2 channel for all
////     ctrl_gt *target1, ctrl_gt *target2,
////     // memory block
////     ctrl_gt *memory_block1, ctrl_gt *memory_block2,
////     // type size
////     uint32_t type_size,
////     // memory size
////     uint32_t memory_size);
//
////// Set Monitor Target
//// GMP_STATIC_INLINE
//// void dsa_set_trigger_memory_target(
////     // log object
////     trigger_memory_log_t *log,
////     // monitor target 2 channel for all
////     ctrl_gt *target1, ctrl_gt *target2)
////{
////     log->monitor_target_ch1 = target1;
////     log->monitor_target_ch2 = target2;
//// }
//
////// step monitor
//// GMP_STATIC_INLINE
//// void dsa_step_trigger_memory_target(
////     // log object
////     trigger_memory_log_t *log)
//// {
////     if (log->flag_trigger || (log->last_target1 < 0 && (*log->monitor_target_ch1) > 0))
////     {
////         log->flag_trigger = 1;
//
////        log->memory_ch1[log->current_index] = *log->monitor_target_ch1;
////        log->memory_ch2[log->current_index] = *log->monitor_target_ch2;
//
////        log->current_index += 1;
////        if (log->current_index >= log->log_length)
////        {
////            log->current_index = 0;
////            log->flag_trigger = 0;
////        }
////    }
////    else
////    {
////        log->last_target1 = *log->monitor_target_ch1;
////    }
////}
//
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_DSA_TRIGGER_H_
