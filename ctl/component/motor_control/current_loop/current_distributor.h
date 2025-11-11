// Current Allocation using Lookup Table
// Distributes current idq based on the input im and alpha using a lookup table(im-alpha)
// The im isometric distribution of the lookup table is required


#ifndef _FILE_CURRENT_DISTRIBUTOR_H_
#define _FILE_CURRENT_DISTRIBUTOR_H_

#include <ctl/component/motor_control/controller_preset/CURRENT_DISTRIBUTOR_LUT.h>



#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus


typedef enum
{
    CONST_ALPHA, // Constant alpha
    LUT_LINEAR   // LUT, linear interpolation
} distribution_mode_t;

typedef struct _tag_idq_current_distributor
{
    // input parameters
    ctrl_gt im;    // Input current magnitude
    ctrl_gt alpha; // Current angle (alpha)

    // output parameters
    ctrl_gt id; // d-axis current
    ctrl_gt iq; // q-axis current

    // Configuration for current distribution
    distribution_mode_t mode;             // Mode for alpha calculation
    ctrl_gt alpha_coef[CURRENT_DISTRIBUTION_LUT_SIZE - 1];        // Alpha interpolation coefficients

    // Field weakening
    fast_gt FW_ENB;
    ctrl_gt spd_set;
    ctrl_gt ud_set;
    ctrl_gt uq_set;
    ctrl_gt u_amp;
    ctrl_gt vs_limit;
    pid_regular_t fw_pid;

} idq_current_distributor_t;

//GMP_STATIC_INLINE
//void ctl_attach_idq_distributor(pmsm_bare_controller_t *ctrl, idq_current_distributor_t *distributor)
//{
//    ctrl->distributor = distributor;
//}


// Function to calculate the interpolation coefficients for the lookup table
GMP_STATIC_INLINE
void calc_interpolation_coefficients(idq_current_distributor_t *distributor)
{
    for (uint32_t i = 0; i < CURRENT_DISTRIBUTION_LUT_SIZE - 1; i++) {
        ctrl_gt im_lower = CURRENT_DISTRIBUTION_LUT[i].im;
        ctrl_gt im_upper = CURRENT_DISTRIBUTION_LUT[i + 1].im;
        ctrl_gt alpha_lower = CURRENT_DISTRIBUTION_LUT[i].alpha;
        ctrl_gt alpha_upper = CURRENT_DISTRIBUTION_LUT[i + 1].alpha;

        // ·ÀÖ¹³ýÁã£¬È·±£ im_upper != im_lower
        if (im_upper != im_lower) {
            distributor->alpha_coef[i] = (alpha_upper - alpha_lower) / (im_upper - im_lower);
        } else {
            distributor->alpha_coef[i] = (alpha_upper - alpha_lower) / 2.0;
        }
    }
}




// Function to perform linear interpolation based on input im
GMP_STATIC_INLINE
ctrl_gt linear_interpolation(idq_current_distributor_t *distributor)
{
    ctrl_gt im_search = distributor->im;
    int index = (im_search - CURRENT_DISTRIBUTION_LUT[0].im) / CURRENT_DISTRIBUTION_LUT_STEP;

    // Prevent out-of-bound access
    if (index < 0)
        index = 0;
    if (index >= CURRENT_DISTRIBUTION_LUT_SIZE - 1)
        index = CURRENT_DISTRIBUTION_LUT_SIZE - 2;

    if (im_search < CURRENT_DISTRIBUTION_LUT[0].im)
        im_search = CURRENT_DISTRIBUTION_LUT[0].im;
    if (im_search > CURRENT_DISTRIBUTION_LUT[CURRENT_DISTRIBUTION_LUT_SIZE - 1].im)
        im_search = CURRENT_DISTRIBUTION_LUT[CURRENT_DISTRIBUTION_LUT_SIZE - 1].im;

    ctrl_gt im_lower = CURRENT_DISTRIBUTION_LUT[index].im;
    ctrl_gt im_upper = CURRENT_DISTRIBUTION_LUT[index + 1].im;
    ctrl_gt alpha_lower = CURRENT_DISTRIBUTION_LUT[index].alpha;
    ctrl_gt alpha_upper = CURRENT_DISTRIBUTION_LUT[index + 1].alpha;

    // Perform linear interpolation: alpha = alpha_lower + (input_im - im_lower) * alpha_coef
//    return (alpha_lower + (distributor->im - im_lower) * (distributor->alpha_coef[index]));
    ctrl_gt scale = distributor->alpha_coef[index];
    ctrl_gt inc = (im_search - im_lower) * scale;
    return (alpha_lower + inc);
}




// Initialize the current distributor configuration
GMP_STATIC_INLINE
void ctl_init_idq_current_distributor(idq_current_distributor_t *distributor, distribution_mode_t mode)
{
    distributor->mode = mode;
    distributor->im = 0;
    distributor->alpha = CURRENT_DISTRIBUTION_ALPHA;
    distributor->id = 0;
    distributor->iq = 0;
    distributor->spd_set=0;
    distributor->ud_set=0;
    distributor->uq_set=0;
    distributor->u_amp=0;

    //---FW init
    distributor->FW_ENB=1;
    distributor->vs_limit=294.5f/MTR_CTRL_VOLTAGE_BASE;    // FW limit voltage, 95%*537V/sqrt(3)
    ctrl_gt fw_kp = 0.05f;
    ctrl_gt fw_ki = 0.005f;
    ctl_init_pid(
           // fw pid for current angle(rad)
            &distributor->fw_pid,
           // parameters for pid controller: Kd Ti Td
            fw_kp, fw_kp/fw_ki/CONTROLLER_FREQUENCY, 0,
           // controller frequency
           CONTROLLER_FREQUENCY);
    ctl_set_pid_limit(&distributor->fw_pid, 0.055, 0);  // 30deg/360deg=0.083
//    distributor->fw_pid.integral_min = 0;
//    distributor->fw_pid.integral_max = 0.083;
    ctl_clear_pid(&distributor->fw_pid);

    // If LUT_LINEAR mode is selected, calculate interpolation coefficients
    if (mode == LUT_LINEAR)
    {
        calc_interpolation_coefficients(distributor);
    }
}



// Step function to calculate id and iq currents based on the given im
GMP_STATIC_INLINE
void ctl_step_idq_current_distributor(idq_current_distributor_t *distributor)
{

    // Select the method for alpha calculation based on the mode
    if (distributor->mode == CONST_ALPHA)
    {
        // If using constant alpha, set alpha to a predefined value
        distributor->alpha = CURRENT_DISTRIBUTION_ALPHA; // Example: constant alpha of 30 degrees
    }
    else if (distributor->mode == LUT_LINEAR)
    {
        // If using LUT_LINEAR mode, calculate alpha using linear interpolation
        distributor->alpha = linear_interpolation(distributor);
    }

    // Field weakening
    if (distributor->FW_ENB)
    {
        ctrl_gt vs_diff = distributor->u_amp-distributor->vs_limit;
        distributor->alpha = ctl_sat ( (distributor->alpha + ctl_step_pid_ser(&distributor->fw_pid, vs_diff) ), 0.472f, 0);
    }

    // Calculate id and iq based on the computed alpha and input im
    if (distributor->im < 0) // speed down
    {
        distributor->id = distributor->im * ctl_cos((0.625f)); // 225deg for negative im
        distributor->iq = distributor->im * ctl_sin((0.625f)); // 225deg for negative im
    }
    else
    {
        distributor->id = distributor->im * ctl_cos(distributor->alpha); // Convert alpha to radians
        distributor->iq = distributor->im * ctl_sin(distributor->alpha); // Convert alpha to radians
    }

}

GMP_STATIC_INLINE
void distributor_set_im(idq_current_distributor_t *distributor, ctrl_gt im)
{
    distributor->im = im;
}


GMP_STATIC_INLINE
void distributor_set_spd(idq_current_distributor_t *distributor, ctrl_gt spd)
{
    distributor->spd_set=spd;
}


GMP_STATIC_INLINE
void distributor_set_fw(idq_current_distributor_t *distributor, ctrl_gt ud, ctrl_gt uq)
{
    distributor->ud_set=ud;
    distributor->uq_set=uq;
    distributor->u_amp=ctl_sqrt(ctl_mul(ud,ud)+ctl_mul(uq,uq));
}



#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CURRENT_DISTRIBUTOR_H_
