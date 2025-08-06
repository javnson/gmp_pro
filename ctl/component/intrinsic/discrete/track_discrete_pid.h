/**
 * @file tracking_pid.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief A composite PID controller with trajectory generation and execution rate division.
 * @version 0.1
 * @date 2024-10-26
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file implements a tracking PID controller. It is a higher-level
 * control block that combines three intrinsic modules:
 * 1.  A Frequency Divider: To execute the controller at a lower rate than the main ISR.
 * 2.  A Slope Limiter: To generate a smooth trajectory (ramp) for the setpoint.
 * 3.  A Discrete PID: To calculate the control output based on the error between
 * the trajectory and the feedback signal.
 * This structure is ideal for reference tracking applications where smooth transitions
 * and different control rates are required.
 */

#ifndef _TRACKING_PID_H_
#define _TRACKING_PID_H_

#include <ctl/component/intrinsic/discrete/discrete_pid.h>
#include <ctl/component/intrinsic/discrete/divider.h>
#include <ctl/component/intrinsic/discrete/slope_limiter.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup tracking_pid Tracking PID Controller
 * @brief A composite PID controller for smooth setpoint tracking.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Tracking Discrete PID Controller                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the tracking PID controller.
 */
typedef struct _tag_tracking_pid_t
{
    discrete_pid_t pid;       //!< The core discrete PID controller instance.
    ctl_divider_t div;        //!< The frequency divider instance.
    ctl_slope_limiter_t traj; //!< The slope limiter for trajectory generation.
} ctl_tracking_pid_t;

/**
 * @brief Initializes the tracking PID controller.
 * @param[out] tp Pointer to the tracking PID instance.
 * @param[in] kp Proportional gain of the PID.
 * @param[in] Ti Integral time constant of the PID (in seconds).
 * @param[in] Td Derivative time constant of the PID (in seconds).
 * @param[in] sat_max Maximum output saturation limit for the PID.
 * @param[in] sat_min Minimum output saturation limit for the PID.
 * @param[in] slope_max Maximum positive rate of change for the setpoint (units/sec).
 * @param[in] slope_min Maximum negative rate of change for the setpoint (-units/sec).
 * @param[in] division The factor by which to divide the execution frequency.
 * @param[in] fs The main sampling frequency (Hz) at which this module's step function is called.
 */
void ctl_init_tracking_pid(ctl_tracking_pid_t* tp, parameter_gt kp, parameter_gt Ti, parameter_gt Td, ctrl_gt sat_max,
                           ctrl_gt sat_min, parameter_gt slope_max, parameter_gt slope_min, uint32_t division,
                           parameter_gt fs);

/**
 * @brief Clears the internal states of the tracking PID controller.
 * @param[out] tp Pointer to the tracking PID instance.
 */
GMP_STATIC_INLINE void ctl_clear_tracking_pid(ctl_tracking_pid_t* tp)
{
    ctl_clear_discrete_pid(&tp->pid);
    ctl_clear_divider(&tp->div);
    ctl_clear_slope_limiter(&tp->traj);
}

/**
 * @brief Executes one step of the tracking PID controller.
 * @details This function should be called at the main system frequency (fs). It uses an
 * internal divider to decide when to execute the core control logic. When the
 * divider triggers, it updates the setpoint trajectory and calculates a new PID
 * output. On other steps, it holds the last calculated PID output.
 * @param[in,out] tp Pointer to the tracking PID instance.
 * @param[in] target The ultimate target value for the setpoint.
 * @param[in] fbk The feedback value from the system.
 * @return ctrl_gt The latest control output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_tracking_pid(ctl_tracking_pid_t* tp, ctrl_gt target, ctrl_gt fbk)
{
    // Check if it's time to execute the controller based on the division factor
    if (ctl_step_divider(&tp->div))
    {
        // If so, first update the trajectory by one step towards the target
        ctl_step_slope_limiter(&tp->traj, target);
        // Then, execute the PID controller with the error between the current
        // trajectory point and the feedback value.
        return ctl_step_discrete_pid(&tp->pid, tp->traj.out - fbk);
    }

    // If it's not time to execute, just return the last held PID output value.
    return ctl_get_discrete_pid_output(&tp->pid);
}

/**
 * @}
 */ // end of tracking_pid group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _TRACKING_PID_H_
