/** @file user_main.h @brief GMP lifecycle declarations for mcs_acim_nt. */

#ifndef _FILE_MCS_ACIM_NT_USER_MAIN_H_
#define _FILE_MCS_ACIM_NT_USER_MAIN_H_

#include <core/dev/at_device.h>
#include <core/pm/function_scheduler/function_scheduler.h>

#ifdef __cplusplus
extern "C"
{
#endif

void init(void);
void mainloop(void);
void setup_peripheral(void);
void ctl_init(void);
void ctl_mainloop(void);
gmp_task_status_t tsk_startup(gmp_task_t* tsk);

#ifdef __cplusplus
}
#endif

#endif // _FILE_MCS_ACIM_NT_USER_MAIN_H_
