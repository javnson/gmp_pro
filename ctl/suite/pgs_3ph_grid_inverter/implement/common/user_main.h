//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of user objects in this file.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#ifndef _FILE_USER_MAIN_H_
#define _FILE_USER_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

//////////////////////////////////////////////////////////////////////////
// Handle Type definition, copy from <main.c>
//
//

#ifndef SPECIFY_PC_TEST_ENV

#endif // SPECIFY_PC_TEST_ENV

//////////////////////////////////////////////////////////////////////////
// functions

// User should implement this 3 functions at least
//
void init(void);
void mainloop(void);
void setup_peripheral(void);

// For Controller projects user should implement the following functions
//
void ctl_init(void);
void ctl_mainloop(void);
fast_gt ctl_ready_mainloop(void);

//////////////////////////////////////////////////////////////////////////
// Additionally functions prototypes

#ifdef __cplusplus
}
#endif

#endif // _FILE_USER_MAIN_H_
