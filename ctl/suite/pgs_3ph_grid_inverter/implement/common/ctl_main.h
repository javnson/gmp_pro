/**
 * @file ctl_main.cpp
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// const F & VF controller
#include <ctl/component/motor_control/basic/vf_generator.h>

// speed encoder
#include <ctl\component\motor_control\basic\encoder.h>


#include <ctl/component/interface/pwm_channel.h>

#include <ctl/component/interface/adc_channel.h>

#include <xplt.peripheral.h>

#include <ctl/component/intrinsic/discrete/signal_generator.h>

#include <ctl/component/digital_power/single_phase/single_phase_dc_ac.h>

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

extern volatile fast_gt flag_system_enable;
extern volatile fast_gt flag_system_running;

// controller objects
extern inv_ctrl_t inv_ctrl;

// periodic callback function things.
GMP_STATIC_INLINE
void ctl_dispatch(void)
{

    ctl_step_inv_ctrl(&inv_ctrl);
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_
