
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

// SDPE-generated controller settings
#include <sdpe_dps_fsbb_iris_settings.h>

#ifndef CONTROLLER_LED
#define CONTROLLER_LED SYSTEM_LED
#endif // CONTROLLER_LED

// select ADC PTR interface
#include <ctl/component/interface/adc_ptr_channel.h>

#include <core/dev/datalink.h>

//=================================================================================================
// definitions of peripheral

extern adc_channel_t adc_v_in;
extern adc_channel_t adc_v_out;
extern adc_channel_t adc_i_L;
extern adc_channel_t adc_i_load;

// dlog DSA objects
//extern basic_trigger_t trigger;

#define DLOG_MEM_LENGTH 100
extern ctrl_gt dlog_mem1[DLOG_MEM_LENGTH];
extern ctrl_gt dlog_mem2[DLOG_MEM_LENGTH];

void reset_controller(void);
void flush_dl_tx_buffer(void);
void flush_dl_rx_buffer(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PERIPHERAL_H_
