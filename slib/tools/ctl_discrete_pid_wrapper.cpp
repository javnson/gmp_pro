
/*
 * Include Files
 *
 */
#include "simstruc.h"



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */

#include <math.h>
#include <gmp_core.hpp>
#include <ctl/component/intrinsic/discrete/discrete_pid.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */

typedef discrete_pid_t ctrl_dut_t;
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void ctl_discrete_pid_Start_wrapper(real_T *xD,
			void **pW,
			const real_T *pid_param, const int_T p_width0,
			const real_T *pid_minmax, const int_T p_width1,
			const real_T *ctrl_freq, const int_T p_width2,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */

// pid_param[0]:  kp
    // pr_param[1]: Ti
    // pr_param[2]: Td
    ctrl_dut_t* ctrl = new ctrl_dut_t;
    if(ctrl_freq < 0)
        ssSetLocalErrorStatus(S, "controller frequency should be a positive value.");
    
    // TODO: add CTL controller init function here.
    ctl_init_discrete_pid(ctrl, pid_param[0], pid_param[1], pid_param[2], *ctrl_freq);

    if(p_width1 == 2)
        ctl_set_discrete_pid_limit(ctrl, pid_minmax[1], pid_minmax[0]);

    pW[0] = static_cast<void*>(ctrl);
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void ctl_discrete_pid_Outputs_wrapper(const real_T *u0,
			real_T *y0,
			const real_T *xD,
			void **pW,
			const real_T *pid_param, const int_T p_width0,
			const real_T *pid_minmax, const int_T p_width1,
			const real_T *ctrl_freq, const int_T p_width2,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */

ctrl_dut_t* ctrl = static_cast<ctrl_dut_t*>(pW[0]);

    *y0 = *xD;
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void ctl_discrete_pid_Update_wrapper(const real_T *u0,
			real_T *y0,
			real_T *xD,
			void **pW,
			const real_T *pid_param, const int_T p_width0,
			const real_T *pid_minmax, const int_T p_width1,
			const real_T *ctrl_freq, const int_T p_width2,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */

ctrl_dut_t* ctrl = static_cast<ctrl_dut_t*>(pW[0]);

    // TODO: add CTL controller step function.
    *xD = ctl_step_discrete_pid(ctrl, float2ctrl(*u0));
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Terminate function
 *
 */
void ctl_discrete_pid_Terminate_wrapper(real_T *xD,
			void **pW,
			const real_T *pid_param, const int_T p_width0,
			const real_T *pid_minmax, const int_T p_width1,
			const real_T *ctrl_freq, const int_T p_width2,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */

ctrl_dut_t* ctrl = static_cast<ctrl_dut_t*>(pW[0]);
    delete ctrl;
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

