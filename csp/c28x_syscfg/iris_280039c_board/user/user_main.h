//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of user objects in this file.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include <core/dev/at_device.h>
#include <core/dev/datalink.h>

#include <core/pm/function_scheduler.h>

// peripheral
#include <core/dev/display/ht16k33.h>
#include <core/dev/gpio/pca9555.h>
#include <core/dev/sensor/hdc1080.h>

#ifndef _FILE_USER_MAIN_H_
#define _FILE_USER_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

//=================================================================================================
// global variables

#ifndef SPECIFY_PC_TEST_ENV

#endif // SPECIFY_PC_TEST_ENV

extern iic_halt iic_bus;
extern ht16k33_dev_t ht16k33;
extern hdc1080_dev_t hdc1080;

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

gmp_task_status_t tsk_startup(gmp_task_t* tsk);
gmp_task_status_t tsk_key_flush(gmp_task_t* tsk);
gmp_task_status_t tsk_LED_flush(gmp_task_t* tsk);
gmp_task_status_t fpga_test_task(gmp_task_t* tsk);


// peripheral function
void beep_on();
void beep_off();



#ifdef __cplusplus
}
#endif

#endif // _FILE_USER_MAIN_H_
