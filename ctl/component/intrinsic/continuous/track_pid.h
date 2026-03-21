/**
 * @file tracking_continuous_pid.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief A composite continuous-form PID with trajectory generation and rate division.
 * @version 0.3
 * @date 2026-03-16
 *
 * @copyright Copyright GMP(c) 2026
 *
 */

#ifndef _TRACKING_CONTINUOUS_PID_H_
#define _TRACKING_CONTINUOUS_PID_H_

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/continuous/continuous_pid_aw.h> // Éý¼¶Îª PID_AW

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
    ctl_pid_aw_t pid;         //!< The core anti-windup PID controller instance. (Upgraded)
    ctl_divider_t div;        //!< The frequency divider instance.
    ctl_slope_limiter_t traj; //!< The slope limiter for trajectory generation.
} ctl_tracking_continuous_pid_t;

/**
 * @brief Initializes the tracking continuous PID controller.
 * @note The inner PID and Trajectory are automatically configured using the EFFECTIVE 
 * sampling frequency (fs / division).
 * @param[out] tp Pointer to the tracking PID instance.
 * @param[in] kp Proportional gain.
 * @param[in] Ti Integral time constant (seconds).
 * @param[in] Td Derivative time constant (seconds).
 * @param[in] sat_max Maximum output saturation limit for the PID.
 * @param[in] sat_min Minimum output saturation limit for the PID.
 * @param[in] slope_max Maximum positive rate of change for the setpoint (units/sec).
 * @param[in] slope_min Maximum negative rate of change for the setpoint (-units/sec).
 * @param[in] division The factor by which to divide the execution frequency.
 * @param[in] fs The MAIN sampling frequency (Hz) of the caller loop.
 */
void ctl_init_tracking_continuous_pid(ctl_tracking_continuous_pid_t* tp, parameter_gt kp, parameter_gt Ti,
                                      parameter_gt Td, ctrl_gt sat_max, ctrl_gt sat_min, parameter_gt slope_max,
                                      parameter_gt slope_min, uint32_t division, parameter_gt fs);

/**
 * @brief Manually overrides the derivative low-pass filter time constant (Tf).
 * @param[out] tp Pointer to the tracking PID instance.
 * @param[in] Tf New derivative filter time constant (seconds). Set to 0 to disable filtering.
 * @param[in] division The original division factor.
 * @param[in] fs The MAIN sampling frequency (Hz).
 */
void ctl_set_tracking_continuous_pid_filter(ctl_tracking_continuous_pid_t* tp, parameter_gt Tf, uint32_t division,
                                            parameter_gt fs);

/**
 * @brief Clears the internal states of the tracking continuous PID controller.
 */
GMP_STATIC_INLINE void ctl_clear_tracking_continuous_pid(ctl_tracking_continuous_pid_t* tp)
{
    ctl_clear_pid_aw(&tp->pid);
    ctl_clear_divider(&tp->div);
    ctl_clear_slope_limiter(&tp->traj);
}

/**
 * @brief Executes one step of the tracking continuous PID controller.
 * @return ctrl_gt The latest control output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_tracking_continuous_pid(ctl_tracking_continuous_pid_t* tp, ctrl_gt target,
                                                           ctrl_gt fbk)
{
    if (ctl_step_divider(&tp->div))
    {
        ctl_step_slope_limiter(&tp->traj, target);
        // Using AW Series Step
        return ctl_step_pid_aw_ser(&tp->pid, tp->traj.out - fbk);
    }
    return tp->pid.out;
}

/**
 * @brief Gets the current output of the tracking PID controller.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_tracking_continuous_pid_output(ctl_tracking_continuous_pid_t* tp)
{
    return tp->pid.out;
}

/** @} */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _TRACKING_CONTINUOUS_PID_H_
