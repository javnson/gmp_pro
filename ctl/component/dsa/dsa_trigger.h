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

// typedef struct tag_trigger_memory_2ch
//{
//     // This memory will contain all the logged information
//     // This channel is used to trigger the output
//     ctrl_gt *memory_ch1;
//     ctrl_gt *memory_ch2;

//    // monitor target
//    ctrl_gt *monitor_target_ch1;
//    ctrl_gt *monitor_target_ch2;

//    // memory length
//    uint32_t log_length;

//    // cell size
//    uint32_t cell_size;

//    // trigger flag
//    fast_gt flag_trigger;
//    ctrl_gt last_target1;
//    uint32_t current_index;

//} trigger_memory_log_t;

// void dsa_init_trigger_memory(
//     // log object
//     trigger_memory_log_t *log,
//     // monitor target 2 channel for all
//     ctrl_gt *target1, ctrl_gt *target2,
//     // memory block
//     ctrl_gt *memory_block1, ctrl_gt *memory_block2,
//     // type size
//     uint32_t type_size,
//     // memory size
//     uint32_t memory_size);

//// Set Monitor Target
// GMP_STATIC_INLINE
// void dsa_set_trigger_memory_target(
//     // log object
//     trigger_memory_log_t *log,
//     // monitor target 2 channel for all
//     ctrl_gt *target1, ctrl_gt *target2)
//{
//     log->monitor_target_ch1 = target1;
//     log->monitor_target_ch2 = target2;
// }

//// step monitor
// GMP_STATIC_INLINE
// void dsa_step_trigger_memory_target(
//     // log object
//     trigger_memory_log_t *log)
// {
//     if (log->flag_trigger || (log->last_target1 < 0 && (*log->monitor_target_ch1) > 0))
//     {
//         log->flag_trigger = 1;

//        log->memory_ch1[log->current_index] = *log->monitor_target_ch1;
//        log->memory_ch2[log->current_index] = *log->monitor_target_ch2;

//        log->current_index += 1;
//        if (log->current_index >= log->log_length)
//        {
//            log->current_index = 0;
//            log->flag_trigger = 0;
//        }
//    }
//    else
//    {
//        log->last_target1 = *log->monitor_target_ch1;
//    }
//}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_DSA_TRIGGER_H_
