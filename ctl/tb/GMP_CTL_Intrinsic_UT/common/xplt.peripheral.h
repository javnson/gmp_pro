
//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all necessary GMP config macro in this file.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#ifndef _FILE_XPLT_PERIPHERAL_H_
#define _FILE_XPLT_PERIPHERAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <gmp_core.h>

// controller settings
#include "ctrl_settings.h"

// select ADC PTR interface
#include <ctl/component/interface/adc_ptr_channel.h>

// select default encoder
#include <ctl/component/motor_control/basic/encoder.h>

// select SIL package
#include <ctl/component/digital_power/basic/std_sil_dp_interface.h>

// buffer for rx & tx
extern gmp_pc_simulink_rx_buffer_t simulink_rx_buffer;
extern gmp_pc_simulink_tx_buffer_t simulink_tx_buffer;

//extern tri_ptr_adc_channel_t uabc;
//extern tri_ptr_adc_channel_t iabc;
//
//extern ptr_adc_channel_t udc;
//extern ptr_adc_channel_t idc;
//
//extern pos_autoturn_encoder_t pos_enc;
//
//extern pwm_tri_channel_t pwm_out;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PERIPHERAL_H_
