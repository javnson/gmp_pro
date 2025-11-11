/**
 * @file voltage_event_detector.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a detector for voltage sag and swell events on a sinusoidal signal.
 * @version 0.1
 * @date 2025-08-07
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _VOLTAGE_EVENT_DETECTOR_H_
#define _VOLTAGE_EVENT_DETECTOR_H_

#include <ctl/component/intrinsic/discrete/discrete_sogi.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup voltage_event_detector Voltage Sag/Swell Detector
 * @brief A module for detecting voltage sag and swell power quality events.
 * @details This module monitors a sinusoidal input voltage to detect power quality
 * events such as sags (undervoltage) and swells (overvoltage). It uses a
 * Second-Order Generalized Integrator (SOGI) to extract the fundamental
 * amplitude of the signal. A fault is flagged if the amplitude remains outside
 * the configured thresholds for a specified duration.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Voltage Sag and Swell Detector                                            */
/*---------------------------------------------------------------------------*/

/**
 * @brief Enumeration for the detected fault status.
 */
typedef enum
{
    VOLTAGE_EVENT_OK = 0,    //!< No fault detected.
    VOLTAGE_EVENT_SAG = 1,   //!< A voltage sag event is active.
    VOLTAGE_EVENT_SWELL = 2, //!< A voltage swell event is active.
} ctl_voltage_event_status_t;

/**
 * @brief Data structure for the voltage sag/swell detector.
 */
typedef struct _tag_voltage_event_detector_t
{
    // Core component for amplitude extraction
    discrete_sogi_t sogi; //!< SOGI to extract fundamental amplitude.

    // Configuration parameters
    ctrl_gt sag_threshold;        //!< Voltage amplitude below which a sag is detected.
    ctrl_gt swell_threshold;      //!< Voltage amplitude above which a swell is detected.
    uint32_t sag_delay_samples;   //!< Number of samples the sag must persist to trigger a fault.
    uint32_t swell_delay_samples; //!< Number of samples the swell must persist to trigger a fault.

    // State variables
    ctrl_gt amplitude;                       //!< The calculated instantaneous amplitude of the input signal.
    uint32_t sag_counter;                    //!< Counter for sag duration.
    uint32_t swell_counter;                  //!< Counter for swell duration.
    ctl_voltage_event_status_t fault_status; //!< The current latched fault status.

} ctl_voltage_event_detector_t;

/**
 * @brief Initializes the voltage sag/swell detector.
 * @param[out] ved Pointer to the voltage event detector instance.
 * @param[in] sag_threshold The sag voltage threshold.
 * @param[in] swell_threshold The swell voltage threshold.
 * @param[in] sag_delay_ms The required duration of a sag event in milliseconds to trigger a fault.
 * @param[in] swell_delay_ms The required duration of a swell event in milliseconds to trigger a fault.
 * @param[in] nominal_freq The nominal frequency of the input signal in Hz.
 * @param[in] fs The sampling frequency in Hz.
 */
void ctl_init_voltage_event_detector(ctl_voltage_event_detector_t* ved, parameter_gt sag_threshold,
                                     parameter_gt swell_threshold, parameter_gt sag_delay_ms,
                                     parameter_gt swell_delay_ms, parameter_gt nominal_freq, parameter_gt fs);

/**
 * @brief Clears the internal states and latched faults of the detector.
 * @param[out] ved Pointer to the voltage event detector instance.
 */
GMP_STATIC_INLINE void ctl_clear_voltage_event_detector(ctl_voltage_event_detector_t* ved)
{
    ctl_clear_discrete_sogi(&ved->sogi);
    ved->sag_counter = 0;
    ved->swell_counter = 0;
    ved->amplitude = 0;
    ved->fault_status = VOLTAGE_EVENT_OK;
}

/**
 * @brief Executes one step of the voltage event detection logic.
 * @param[in,out] ved Pointer to the voltage event detector instance.
 * @param[in] input The instantaneous input voltage sample.
 * @return ctl_voltage_event_status_t The current fault status.
 */
GMP_STATIC_INLINE ctl_voltage_event_status_t ctl_step_voltage_event_detector(ctl_voltage_event_detector_t* ved,
                                                                             ctrl_gt input)
{
    // Step the SOGI to get the fundamental components
    ctl_step_discrete_sogi(&ved->sogi, input);
    ctrl_gt d_out = ctl_get_discrete_sogi_ds(&ved->sogi);
    ctrl_gt q_out = ctl_get_discrete_sogi_qs(&ved->sogi);

    // Calculate the amplitude
    ved->amplitude = ctl_sqrt(ctl_mul(d_out, d_out) + ctl_mul(q_out, q_out));

    // Do not check for new faults if one is already latched
    if (ved->fault_status != VOLTAGE_EVENT_OK)
    {
        return ved->fault_status;
    }

    // Check for swell condition
    if (ved->amplitude > ved->swell_threshold)
    {
        ved->swell_counter++;
        if (ved->swell_counter >= ved->swell_delay_samples)
        {
            ved->fault_status = VOLTAGE_EVENT_SWELL;
        }
    }
    else
    {
        ved->swell_counter = 0;
    }

    // Check for sag condition
    if (ved->amplitude < ved->sag_threshold)
    {
        ved->sag_counter++;
        if (ved->sag_counter >= ved->sag_delay_samples)
        {
            ved->fault_status = VOLTAGE_EVENT_SAG;
        }
    }
    else
    {
        ved->sag_counter = 0;
    }

    return ved->fault_status;
}

/**
 * @}
 */ // end of voltage_event_detector group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _VOLTAGE_EVENT_DETECTOR_H_
