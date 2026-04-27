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

#include <gmp_core.h>

#include <ctl/ctl_core.h>


#include "ctl_main.h"

// User may set (get handle) TX content via `gmp_csp_sl_get_tx_buffer`
// User may get (get handle) RX content via `gmp_csp_sl_get_rx_buffer`

// BUILD_LEVEL 1: Voltage Open loop
// BUILD_LEVEL 2: Current Open loop
// BUILD_LEVEL 3: Current Open loop with actual position
// BUILD_LEVEL 4: Speed Close loop
#define BUILD_LEVEL (4)

// position encoder
ctl_pos_encoder_t pos_enc;

// speed encoder
ctl_spd_calculator_t spd_enc;

// PMSM servo objects
pmsm_servo_fm_t pmsm_servo;

// PMSM const frequency controller
ctl_const_f_controller const_f;

#define CONTROLLER_FREQUENCY (10000)

// CTL initialize routine
void ctl_init()
{
    // position encoder & speed encoder
    ctl_init_pos_encoder(&pos_enc);
    ctl_init_spd_calculator(&spd_enc);

    // PMSM servo controller framework based on Control Nano
    ctl_init_pmsm_servo_framework(&pmsm_servo);

    // constant frequency virtual encoder
    ctl_init_const_f_controller(&const_f);
    ctl_setup_const_f_controller(&const_f, 50, CONTROLLER_FREQUENCY);

    // setup position encoder & speed encoder
    ctl_setup_pos_encoder(&pos_enc, 1, ((uint32_t)1 << 17) - 1);
    ctl_setup_spd_calculator(&spd_enc, CONTROLLER_FREQUENCY, 5, 3000, 1, 20, &pos_enc.encif);

    // setup PMSM servo controller framework
    ctl_setup_pmsm_servo_framework(
        // link PMSM servo and encoder
        // PMSM servo object, position encoder, speed encoder, control law frequency
        &pmsm_servo, CTL_POSITION_IF(&const_f), CTL_SPEED_IF(&spd_enc), CONTROLLER_FREQUENCY,
        // current controller PID parameter
        // P, I, D, sat_min, sat_max
        float2ctrl(0.8), float2ctrl(0.01), 0, float2ctrl(-0.5), float2ctrl(0.5),
        // speed controller PID parameter
        // P, I, D, sat_min, sat_max
        float2ctrl(1.5), float2ctrl(0.001), 0, float2ctrl(-0.5), float2ctrl(0.5),
        // acceleration
        // negative acceleration, positive acceleration, speed controller division
        float2ctrl(-0.05), float2ctrl(0.05), 5,
        // ADC parameters
        // ADC resolution, iqn, current gain, current bias, voltage gain, voltage bias
        // NOTE iqn parameter is meaningless for float environment
        // NOTE gain is negative value means ADC result is negative
        16, 24, float2ctrl(2.0), float2ctrl(1.65/2.5), float2ctrl(0.1), float2ctrl(0.1),
        // PWM parameters
        10000);

    gmp_base_print("PMSM SERVO struct has been inited, size :%d\r\n", sizeof(pmsm_servo_fm_t));

    // Specify ctl nanao object
    ctl_setup_default_ctl_nano_obj(&pmsm_servo.base);

#if BUILD_LEVEL == 1
    // VF Control, voltage Open-loop
    ctl_set_pmm_servo_pos_enc(&pmsm_servo, CTL_POSITION_IF(&const_f));

    ctl_vector2_t vdq_set = {float2ctrl(0.2), float2ctrl(0.2)};
    ctl_set_pmsm_servo_voltage_mode(&pmsm_servo);
    ctl_set_pmsm_servo_ff_voltage(&pmsm_servo, &vdq_set);

#elif BUILD_LEVEL == 2
    // Current open-loop
    ctl_set_pmm_servo_pos_enc(&pmsm_servo, CTL_POSITION_IF(&const_f));

    ctl_vector2_t idq_set = {float2ctrl(0.0), float2ctrl(0.2)};
    ctl_set_pmsm_servo_current_mode(&pmsm_servo);
    ctl_set_pmsm_servo_ff_current(&pmsm_servo, &idq_set);

#elif BUILD_LEVEL == 3
    // IF Control, current close-loop
    ctl_set_pmm_servo_pos_enc(&pmsm_servo, CTL_POSITION_IF(&pos_enc));

    ctl_vector2_t idq_set = {float2ctrl(0.0), float2ctrl(0.2)};
    ctl_set_pmsm_servo_current_mode(&pmsm_servo);
    ctl_set_pmsm_servo_ff_current(&pmsm_servo, &idq_set);

#elif BUILD_LEVEL == 4
    // speed control, speed close-loop
    ctl_set_pmm_servo_pos_enc(&pmsm_servo, CTL_POSITION_IF(&pos_enc));

    ctl_set_pmsm_servo_spd_mode(&pmsm_servo);
    ctl_set_pmsm_servo_spd(&pmsm_servo, float2ctrl(0.5));

#endif // BUILD_LEVEL

    // Debug mode online the controller
    //ctl_fm_force_online(&pmsm_servo.base);

    ctl_fm_force_calibrate(&pmsm_servo.base);
}

//////////////////////////////////////////////////////////////////////////
// endless loop function here

void ctl_mainloop(void)
{
    // User Controller logic here.
    if (gmp_base_get_ctrl_tick() > 2000)
    {
        ctl_set_pmsm_servo_spd(&pmsm_servo, float2ctrl(-0.5));
    }

    else if (gmp_base_get_ctrl_tick() > 1000)
    {
        ctl_set_pmsm_servo_spd(&pmsm_servo, float2ctrl(0.1));
    }

    return;
}


#ifdef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO


void ctl_fmif_monitor_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

// return value:
// 1 change to next progress
// 0 keep the same state
fast_gt ctl_fmif_sm_pending_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 0;
}

// return value:
// 1 change to next progress
// 0 keep the same state
fast_gt ctl_fmif_sm_calibrate_routine(ctl_object_nano_t *pctl_obj)
{
    return ctl_cb_pmsm_servo_frmework_current_calibrate(&pmsm_servo);
}

fast_gt ctl_fmif_sm_ready_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 0;
}

// Main relay close, power on the main circuit
fast_gt ctl_fmif_sm_runup_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 1;
}

fast_gt ctl_fmif_sm_online_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 0;
}

fast_gt ctl_fmif_sm_fault_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 0;
}




#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
