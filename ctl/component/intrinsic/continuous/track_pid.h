/**
 * @file tracking_continuous_pid.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief A composite continuous-form PID with trajectory generation and rate division.
 * @version 0.2
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file implements a tracking PID controller based on the continuous-form
 * PID implementation. It combines three intrinsic modules:
 * 1.  A Frequency Divider: To execute the controller at a lower rate.
 * 2.  A Slope Limiter: To generate a smooth trajectory (ramp) for the setpoint.
 * 3.  A Continuous-Form PID: To calculate the control output.
 * This is ideal for setpoint tracking where smooth transitions are needed.
 */

#ifndef _TRACKING_CONTINUOUS_PID_H_
#define _TRACKING_CONTINUOUS_PID_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup tracking_continuous_pid Tracking Continuous PID Controller
 * @brief A composite PID controller for smooth setpoint tracking using a continuous-form PID.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Tracking Continuous PID Controller                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the tracking continuous PID controller.
 */
typedef struct _tag_tracking_continuous_pid_t
{
    ctl_pid_t pid;            //!< The core continuous-form PID controller instance.
    ctl_divider_t div;        //!< The frequency divider instance.
    ctl_slope_limiter_t traj; //!< The slope limiter for trajectory generation.
} ctl_tracking_continuous_pid_t;

/**
 * @brief Initializes the tracking continuous PID controller.
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
void ctl_init_tracking_continuous_pid(ctl_tracking_continuous_pid_t* tp, parameter_gt kp, parameter_gt Ti,
                                      parameter_gt Td, ctrl_gt sat_max, ctrl_gt sat_min, parameter_gt slope_max,
                                      parameter_gt slope_min, uint32_t division, parameter_gt fs);

/**
 * @brief Clears the internal states of the tracking continuous PID controller.
 * @param[out] tp Pointer to the tracking PID instance.
 */
GMP_STATIC_INLINE void ctl_clear_tracking_continuous_pid(ctl_tracking_continuous_pid_t* tp)
{
    ctl_clear_pid(&tp->pid);
    ctl_clear_divider(&tp->div);
    ctl_clear_slope_limiter(&tp->traj);
}

/**
 * @brief Executes one step of the tracking continuous PID controller.
 * @details This function should be called at the main system frequency (fs). It uses an
 * internal divider to decide when to execute the core control logic. When the
 * divider triggers, it updates the setpoint trajectory and calculates a new PID
 * output. On other steps, it holds the last calculated PID output.
 * @param[in,out] tp Pointer to the tracking PID instance.
 * @param[in] target The ultimate target value for the setpoint.
 * @param[in] fbk The feedback value from the system.
 * @return ctrl_gt The latest control output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_tracking_continuous_pid(ctl_tracking_continuous_pid_t* tp, ctrl_gt target,
                                                           ctrl_gt fbk)
{
    // Check if it's time to execute the controller based on the division factor
    if (ctl_step_divider(&tp->div))
    {
        // If so, first update the trajectory by one step towards the target
        ctl_step_slope_limiter(&tp->traj, target);
        // Then, execute the PID controller with the error between the current
        // trajectory point and the feedback value. The series form is used here.
        return ctl_step_pid_ser(&tp->pid, tp->traj.out - fbk);
    }

    // If it's not time to execute, just return the last held PID output value.
    return tp->pid.out;
}

/**
 * @brief Gets the current output of the tracking PID controller.
 * @param[in] tp Pointer to the tracking PID instance.
 * @return ctrl_gt The last calculated output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_tracking_continuous_pid_output(ctl_tracking_continuous_pid_t* tp)
{
    return tp->pid.out;
}

/**
 * @}
 */ // end of tracking_continuous_pid group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _TRACKING_CONTINUOUS_PID_H_
