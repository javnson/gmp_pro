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
#include <ctl/component/motor_control/pmsm_controller/pmsm_ctrl.h>

#include <ctl/component/interface/pwm_channel.h>

#include <ctl/component/interface/adc_channel.h>

#include <ctl/component/intrinsic/discrete/biquad_filter.h>

#include <xplt.peripheral.h>

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// speed encoder
extern spd_calculator_t spd_enc;

// PMSM servo objects
// extern pmsm_fm_t pmsm;

#if defined OPENLOOP_CONST_FREQUENCY
// PMSM const frequency controller
extern ctl_const_f_controller rg;
#else // OPENLOOP_CONST_FREQUENCY
// PMSM const frequency slope controller
extern ctl_slope_f_controller rg;
#endif // OPENLOOP_CONST_FREQUENCY

#ifdef PMSM_CTRL_USING_QEP_ENCODER
// Auto - turn encoder
extern pos_autoturn_encoder_t pos_enc;
#endif // PMSM_CTRL_USING_QEP_ENCODER

// PMSM controller
extern pmsm_controller_t pmsm_ctrl;

extern adc_bias_calibrator_t adc_calibrator;
extern fast_gt flag_enable_adc_calibrator;
extern fast_gt index_adc_calibrator;

typedef enum _tag_adc_index
{
    MTR_ADC_UA = 0,
    MTR_ADC_UB,
    MTR_ADC_UC,
    MTR_ADC_IA,
    MTR_ADC_IB,
    MTR_ADC_IC,
    MTR_ADC_UDC,
    MTR_ADC_IDC
} adc_index_t;

// periodic callback function things.
GMP_STATIC_INLINE
void ctl_dispatch(void)
{
    if (flag_enable_adc_calibrator)
    {
        if (index_adc_calibrator == 3)
            ctl_step_adc_calibrator(&adc_calibrator, pmsm_ctrl.mtr_interface.idc->value);
        else
            ctl_step_adc_calibrator(&adc_calibrator, pmsm_ctrl.mtr_interface.iabc->value.dat[index_adc_calibrator]);
    }
    else
    {
#if defined OPENLOOP_CONST_FREQUENCY
        ctl_step_const_f_controller(&rg);
#else  // OPENLOOP_CONST_FREQUENCY
        ctl_step_slope_f(&rg);
#endif // OPENLOOP_CONST_FREQUENCY
    }

    ctl_step_spd_calc(&spd_enc);

    ctl_step_pmsm_ctrl(&pmsm_ctrl);
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_
