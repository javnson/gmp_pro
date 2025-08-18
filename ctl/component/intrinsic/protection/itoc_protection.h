/**
 * @file trip_protector.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a three-stage inverse-time trip protector (circuit breaker model).
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _TRIP_PROTECTOR_H_
#define _TRIP_PROTECTOR_H_

#include <ctl/math_block/gmp_math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup trip_protector Three-Stage Trip Protector
 * @brief An inverse-time overcurrent protection module.
 * @details This file implements a three-stage overcurrent protection module that
 * mimics the behavior of a circuit breaker with an inverse-time trip curve.
 * It monitors a signal (e.g., current) and triggers a fault based on three levels:
 * 1.  **Instantaneous (INST):** Immediate trip for very high currents.
 * 2.  **Short-Time Delay (STD):** A fast, delayed trip for moderate fault currents.
 * 3.  **Long-Time Delay (LTD):** A slow, delayed trip for minor overload conditions.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Three-Stage Inverse-Time Trip Protector                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Enumeration for the fault status and trip reason.
 */
typedef enum
{
    TRIP_STATUS_OK = 0,        //!< No fault detected.
    TRIP_STATUS_LTD_TRIP = 1,  //!< Fault triggered by Long-Time Delay.
    TRIP_STATUS_STD_TRIP = 2,  //!< Fault triggered by Short-Time Delay.
    TRIP_STATUS_INST_TRIP = 3, //!< Fault triggered by Instantaneous level.
} ctl_trip_status_t;

/**
 * @brief Data structure for the three-stage trip protector.
 */
typedef struct _tag_trip_protector_t
{
    // Source
    const ctrl_gt* source; //!< Pointer to the variable to be monitored (e.g., current).

    // Configuration
    ctrl_gt level_ltd;          //!< Current threshold for Long-Time Delay trip.
    uint32_t delay_ltd_samples; //!< Trip delay for LTD in samples.
    ctrl_gt level_std;          //!< Current threshold for Short-Time Delay trip.
    uint32_t delay_std_samples; //!< Trip delay for STD in samples.
    ctrl_gt level_inst;         //!< Current threshold for Instantaneous trip.

    // State variables
    uint32_t ltd_counter;           //!< Counter for LTD duration.
    uint32_t std_counter;           //!< Counter for STD duration.
    ctl_trip_status_t fault_status; //!< The current latched fault status.

} ctl_trip_protector_t;

/**
 * @brief Initializes the three-stage trip protector.
 * @param[out] prot Pointer to the trip protector instance.
 * @param[in] source Pointer to the variable to monitor.
 * @param[in] level_ltd The current level for LTD trip.
 * @param[in] delay_ltd_ms The trip delay for LTD in milliseconds.
 * @param[in] level_std The current level for STD trip.
 * @param[in] delay_std_ms The trip delay for STD in milliseconds.
 * @param[in] level_inst The current level for instantaneous trip.
 * @param[in] fs The sampling frequency in Hz.
 */
void ctl_init_trip_protector(ctl_trip_protector_t* prot, const ctrl_gt* source, parameter_gt level_ltd,
                             parameter_gt delay_ltd_ms, parameter_gt level_std, parameter_gt delay_std_ms,
                             parameter_gt level_inst, parameter_gt fs);

/**
 * @brief Clears the internal states and any latched fault.
 * @param[out] prot Pointer to the trip protector instance.
 */
GMP_STATIC_INLINE void ctl_clear_trip_protector_fault(ctl_trip_protector_t* prot)
{
    prot->ltd_counter = 0;
    prot->std_counter = 0;
    prot->fault_status = TRIP_STATUS_OK;
}

/**
 * @brief Executes one step of the trip protection logic.
 * @details Checks the source value against the three trip levels in order of priority:
 * INST -> STD -> LTD. A fault, once triggered, is latched.
 * @param[in,out] prot Pointer to the trip protector instance.
 * @return ctl_trip_status_t The current fault status.
 */
GMP_STATIC_INLINE ctl_trip_status_t ctl_step_trip_protector(ctl_trip_protector_t* prot)
{
    // If a fault is already latched, do nothing further.
    if (prot->fault_status != TRIP_STATUS_OK)
    {
        return prot->fault_status;
    }

    // Use the absolute value for overcurrent checks.
    ctrl_gt current_abs = ctl_abs(*prot->source);

    // 1. Check for Instantaneous Trip
    if (current_abs >= prot->level_inst)
    {
        prot->fault_status = TRIP_STATUS_INST_TRIP;
        return prot->fault_status;
    }

    // 2. Check for Short-Time Delay Trip
    if (current_abs >= prot->level_std)
    {
        prot->std_counter++;
        if (prot->std_counter >= prot->delay_std_samples)
        {
            prot->fault_status = TRIP_STATUS_STD_TRIP;
        }
    }
    else
    {
        prot->std_counter = 0; // Reset counter if current drops below level
    }

    // 3. Check for Long-Time Delay Trip
    if (current_abs >= prot->level_ltd)
    {
        prot->ltd_counter++;
        if (prot->ltd_counter >= prot->delay_ltd_samples)
        {
            prot->fault_status = TRIP_STATUS_LTD_TRIP;
        }
    }
    else
    {
        prot->ltd_counter = 0; // Reset counter if current drops below level
    }

    return prot->fault_status;
}

/**
 * @}
 */ // end of trip_protector group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _TRIP_PROTECTOR_H_
