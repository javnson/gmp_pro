
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

// select SIL package
#include <ctl/component/digital_power/basic/std_sil_dp_interface.h>

// Three phase DC/AC
#include <ctl/component/digital_power/three_phase/three_phase_dc_ac.h>

// buffer for rx & tx
extern gmp_pc_simulink_rx_buffer_t simulink_rx_buffer;
extern gmp_pc_simulink_tx_buffer_t simulink_tx_buffer;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PERIPHERAL_H_
