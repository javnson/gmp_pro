/**
 * @file protection.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a generic boundary protection monitoring system.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _PROTECTION_MONITOR_H_
#define _PROTECTION_MONITOR_H_

#include <stdint.h> // For uint32_t, int32_t

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup protection_monitor Protection Monitor
 * @brief A module for checking multiple variables against their boundaries.
 * @details This file implements a configurable protection monitor. It is designed
 * to check a set of variables against their predefined upper (supremum) and
 * lower (infimum) limits. If any variable goes out of bounds, the monitor
 * flags a fault and records which variable caused it. This is a fundamental
 * component for ensuring safe operation in any control system.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Boundary Protection Monitor                                               */
/*---------------------------------------------------------------------------*/

/**
 * @brief Defines a single item to be monitored.
 */
typedef struct _tag_protection_item_t
{
    const ctrl_gt* source; //!< Pointer to the variable to be monitored.
    ctrl_gt supremum;      //!< The upper limit for the variable.
    ctrl_gt infimum;       //!< The lower limit for the variable.
} ctl_protection_item_t;

/**
 * @brief Data structure for the protection monitor.
 */
typedef struct _tag_protection_monitor_t
{
    uint32_t num_items;                    //!< The number of items in the protection set.
    const ctl_protection_item_t* item_set; //!< Pointer to the array of items to monitor.
    int32_t fault_index; //!< Stores the index of the first item that failed the check (-1 if no fault).
} ctl_protection_monitor_t;

/**
 * @brief Initializes the protection monitor.
 * @param[out] mon Pointer to the protection monitor instance.
 * @param[in] item_set A pointer to an array of protection items.
 * @param[in] num_items The number of items in the item_set array.
 */
void ctl_init_protection_monitor(ctl_protection_monitor_t* mon, const ctl_protection_item_t* item_set,
                                 uint32_t num_items);

/**
 * @brief Executes one step of the protection check.
 * @details Iterates through all monitored items. If an item's value is outside
 * its [infimum, supremum] range, it records the index of that item and returns
 * true. The check stops at the first fault found. If a fault already exists,
 * it is not cleared by this function.
 * @param[in,out] mon Pointer to the protection monitor instance.
 * @return fast_gt Returns 1 if a new fault is detected, 0 otherwise.
 */
GMP_STATIC_INLINE fast_gt ctl_step_protection_monitor(ctl_protection_monitor_t* mon)
{
    uint32_t i;

    // Do not check if a fault is already latched
    if (mon->fault_index != -1)
    {
        return 0; // No *new* fault
    }

    for (i = 0; i < mon->num_items; ++i)
    {
        const ctl_protection_item_t* item = &mon->item_set[i];
        if ((*item->source > item->supremum) || (*item->source < item->infimum))
        {
            mon->fault_index = (int32_t)i;
            return 1; // New fault detected
        }
    }

    return 0; // No fault
}

/**
 * @brief Clears any latched fault in the monitor.
 * @param[out] mon Pointer to the protection monitor instance.
 */
GMP_STATIC_INLINE void ctl_clear_protection_fault(ctl_protection_monitor_t* mon)
{
    mon->fault_index = -1;
}

/**
 * @brief Checks if a fault has occurred.
 * @param[in] mon Pointer to the protection monitor instance.
 * @return fast_gt Returns 1 if a fault is active, 0 otherwise.
 */
GMP_STATIC_INLINE fast_gt ctl_is_protection_fault_active(const ctl_protection_monitor_t* mon)
{
    return (mon->fault_index != -1);
}

/**
 * @brief Gets the index of the item that caused the fault.
 * @param[in] mon Pointer to the protection monitor instance.
 * @return int32_t The index of the faulted item, or -1 if no fault is active.
 */
GMP_STATIC_INLINE int32_t ctl_get_protection_fault_index(const ctl_protection_monitor_t* mon)
{
    return mon->fault_index;
}

/**
 * @}
 */ // end of protection_monitor group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _PROTECTION_MONITOR_H_
