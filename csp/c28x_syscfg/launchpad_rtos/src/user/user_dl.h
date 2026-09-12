/**
 * @file user_dl.h
 * @brief Data Link composition boundary for the LaunchPad reference app.
 */

#ifndef GMP_LAUNCHPAD_USER_DL_H
#define GMP_LAUNCHPAD_USER_DL_H

#include <gmp_core.h>
#include <core/dev/datalink/datalink.h>
#include <core/pm/function_scheduler/function_scheduler.h>
#include <ctl/component/dsa/dsa_dl_scope.h>

void user_dl_init(void);
void user_dl_background(void);
gmp_task_status_t user_dl_task(gmp_task_t* task);
void user_dl_apply_signal_parameters(void);
void user_dl_dsa_timer_step(void);

/** Debug-visible ownership roots; nested state is inspected through these. */
extern gmp_datalink_t datalink;
extern ctl_dsa_dl_scope_t dl_scope;
extern volatile uint32_t dl_facility_init_errors;
extern float signal_frequency_hz;
extern float signal_gain;
extern float signal_dc_offset;

#endif /* GMP_LAUNCHPAD_USER_DL_H */
