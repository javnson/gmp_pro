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
#endif

#include <gmp_core.h>
#include <ctrl_settings.h>
#include <ctl/component/interface/adc_ptr_channel.h>
#include <core/dev/datalink/datalink.h>

/** @brief Place the application-owned Scope storage in combined GS2/GS3 RAM. */
#define GMP_DL_SCOPE_STORAGE_RAMGS2_3

extern tri_ptr_adc_channel_t uuvw;
extern adc_gt uuvw_src[3];
extern tri_ptr_adc_channel_t iuvw;
extern adc_gt iuvw_src[3];
extern ptr_adc_channel_t udc;
extern adc_gt udc_src;
extern ptr_adc_channel_t idc;
extern adc_gt idc_src;

#ifdef __cplusplus
}
#endif

#endif // _FILE_XPLT_PERIPHERAL_H_
