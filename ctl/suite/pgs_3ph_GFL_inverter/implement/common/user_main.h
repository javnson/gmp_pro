//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of user objects in this file.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include <core/dev/at_device.h>

#include <core/pm/function_scheduler.h>

#ifndef _FILE_USER_MAIN_H_
#define _FILE_USER_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

//=================================================================================================
// global variables

extern at_device_entity_t at_dev;
extern at_device_cmd_t at_cmds[];

extern cia402_sm_t cia402_sm;

#ifndef SPECIFY_PC_TEST_ENV

#endif // SPECIFY_PC_TEST_ENV

//=================================================================================================
// global functions

//
// User should implement this 3 functions at least
//
void init(void);
void mainloop(void);
void setup_peripheral(void);

//
// For Controller projects user should implement the following functions
//
void ctl_init(void);
void ctl_mainloop(void);

#ifdef __cplusplus
}
#endif

#endif // _FILE_USER_MAIN_H_
