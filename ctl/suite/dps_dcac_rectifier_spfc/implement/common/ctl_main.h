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

// motor controller
#include <ctl/suite/mcs_pmsm/pmsm_ctrl.h>

#include <ctl/component/interface/pwm_channel.h>

#include <ctl/component/interface/adc_channel.h>

#include <ctl\component\digital_power\basic/boost.h>

#include <ctl/component/digital_power/mppt/PnO_algorithm.h>

#include <xplt.peripheral.h>

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// ADC RESULT INDEX
typedef enum _tag_adc_result_nameplate
{
    ADC_RESULT_IL = 0,
    ADC_RESULT_UIN = 1,
    ADC_RESULT_UOUT = 2,
    ADC_BOOST_CHANNEL_NUM = 3
} adc_result_nameplate_t;

extern adc_bias_calibrator_t adc_calibrator;
extern fast_gt flag_enable_adc_calibrator;
extern fast_gt index_adc_calibrator;

extern volatile fast_gt flag_system_enable;
extern volatile fast_gt flag_system_running;

// Boost Controller Suite
extern boost_ctrl_t boost_ctrl;

#if BUILD_LEVEL >= 4
extern ctrl_gt mppt_set;
extern mppt_PnO_algo_t mppt;
#endif // BUILD_LEVEL

// periodic callback function things.
GMP_STATIC_INLINE
void ctl_dispatch(void)
{
#if BUILD_LEVEL >= 4
    mppt_set = ctl_step_mppt_PnO_algo(&mppt, boost_ctrl.lpf_ui.out, boost_ctrl.lpf_il.out);
    ctl_set_boost_ctrl_voltage_openloop(&boost_ctrl, 1.0f - mppt_set);
#endif // BUILD_LEVEL

    ctl_step_boost_ctrl(&boost_ctrl);
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_
