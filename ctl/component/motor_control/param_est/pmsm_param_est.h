// This file will implement a tiny PMSM controller for identifier.

#include <ctl/component/intrinsic/discrete/signal_generator.h>
#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/current_loop/motor_current_ctrl.h>
#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h>

#ifndef _FILE_PMSM_PARAM_EST_H_
#define _FILE_PMSM_PARAM_EST_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef struct _tag_pmsm_offline_identifier_type
{
    // motor interface
    mtr_ift mtr_interface; /**< @brief Universal motor input interface (ADC, encoder, etc.). */
    tri_pwm_ift* pwm_out;  /**< @brief Three-phase PWM output interface. */

    // inner controller

    // angle generator
    ctl_ramp_generator_t rg;

    // sine generator, for high frequency injection
    ctl_sine_generator_t hfi;

    // current controller, IF controller
    ctl_current_controller_t inner_controller;

    // state identifier.
    enum
    {
        PMSM_ID_READY = 0,
//        PMSM_ID_ADC_CALIBRATE = 1,
        PMSM_ID_STATIC, // Rs and encoder calibrate
        //PMSM_ID_ENCODER_CALIBRATE = 2,
        //PMSM_ID_IDENTIFY_RS = 3,
        PMSM_ID_IDENTIFY_LDQ = 4,
        PMSM_ID_IDENTIFY_PSIF = 5,
        PMSM_ID_IDENTIFY_MECHANICAL = 6,
        PMSM_ID_COMPLETE
    } identifier_state;

    // parameters

    // motor positioning current
    ctrl_gt positioning_current;

    // motor HFI voltage
    ctrl_gt hfi_voltage;



    // flag
    fast_gt flag_enable_identifier;
    fast_gt flag_identify_adc_bias;
    fast_gt flag_identify_encoder_bias;
    fast_gt flag_identify_Rs;
    fast_gt flag_identify_Ldq;
    fast_gt flag_identify_psif;
    fast_gt flag_identify_mechanical;

} pmsm_offline_identifier_t;

typedef struct _tag_pmsm_offline_identifier_init_type
{

    // controller frequency
    parameter_gt fs;

    // motor pole pairs
    uint16_t pole_pair;

} pmsm_offline_identifier_init_t;

void ctl_init_pmsm_offline_identifier(pmsm_offline_identifier_t* mtr_id, pmsm_offline_identifier_init_t* init)
{
}

// This function will run in main ISR.
void ctl_step_pmsm_offline_identifier(pmsm_offline_identifier_t* mtr_id)
{
}

// This function will run in mainloop,
// return 0 means the identifier is running,
// return 1 means the identifier is complete.
fast_gt ctl_loop_pmsm_offline_identifier(pmsm_offline_identifier_t* mtr_id)
{
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_PARAM_EST_H_
