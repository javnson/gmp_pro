
/*
 * Include Files
 *
 */
#include "simstruc.h"



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include <gmp_core.hpp>
#include <ctl/component/intrinsic/discrete/proportional_resonant.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
typedef resonant_ctrl_t ctrl_dut_t;
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void ctl_discrete_resonant_Start_wrapper(real_T *xD,
			void **pW,
			const real_T *pr_param, const int_T p_width0,
			const real_T *ctrl_freq, const int_T p_width1,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
// pr_param[0]:  kr
    // pr_param[1]: resonant frequency
    ctrl_dut_t* ctrl = new ctrl_dut_t;
    if(ctrl_freq < 0)
        ssSetLocalErrorStatus(S, "controller frequency should be a positive value.");
    
    // TODO: add CTL controller init function here.
    ctl_init_resonant_controller(ctrl, pr_param[0], pr_param[1], *ctrl_freq);

    pW[0] = static_cast<void*>(ctrl);
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void ctl_discrete_resonant_Outputs_wrapper(const real_T *u0,
			real_T *y0,
			const real_T *xD,
			void **pW,
			const real_T *pr_param, const int_T p_width0,
			const real_T *ctrl_freq, const int_T p_width1,
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
void ctl_discrete_resonant_Update_wrapper(const real_T *u0,
			real_T *y0,
			real_T *xD,
			void **pW,
			const real_T *pr_param, const int_T p_width0,
			const real_T *ctrl_freq, const int_T p_width1,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
ctrl_dut_t* ctrl = static_cast<ctrl_dut_t*>(pW[0]);

    // TODO: add CTL controller step function.
    *xD = ctl_step_resonant_controller(ctrl, float2ctrl(*u0));
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Terminate function
 *
 */
void ctl_discrete_resonant_Terminate_wrapper(real_T *xD,
			void **pW,
			const real_T *pr_param, const int_T p_width0,
			const real_T *ctrl_freq, const int_T p_width1,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
ctrl_dut_t* ctrl = static_cast<ctrl_dut_t*>(pW[0]);
    delete ctrl;
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

