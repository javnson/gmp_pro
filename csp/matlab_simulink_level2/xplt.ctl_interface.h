//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of controller objects in this file.
//
// User should implement the Main ISR of the controller tasks.
//
// User should ensure that all the controller codes here is platform-independent.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include <ctl/component/motor_control/basic/std_sil_motor_interface.h>

#include <xplt.peripheral.h>

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////
// device related functions
// Controller interface
//

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_enable_output()
{
}

// Disable Output
GMP_STATIC_INLINE void ctl_disable_output()
{
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
