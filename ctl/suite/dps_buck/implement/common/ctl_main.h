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

#include <ctl/component/interface/pwm_channel.h>

#include <ctl/component/interface/adc_channel.h>

#include <ctl/component/digital_power/basic/buck.h>

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
    ADC_BUCK_CHANNEL_NUM = 3
} adc_result_nameplate_t;

extern adc_bias_calibrator_t adc_calibrator;
extern fast_gt flag_enable_adc_calibrator;
extern fast_gt index_adc_calibrator;


extern volatile fast_gt flag_system_enable;
extern volatile fast_gt flag_system_running;


// Boost Controller Suite
extern buck_ctrl_t buck_ctrl;

// periodic callback function things.
GMP_STATIC_INLINE
void ctl_dispatch(void)
{
    ctl_step_buck_ctrl(&buck_ctrl);
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_
