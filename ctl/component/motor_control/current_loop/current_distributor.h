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
    ctrl_gt *alpha_coef;                  // Alpha interpolation coefficients

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
    distributor->alpha_coef = (ctrl_gt *)malloc(sizeof(ctrl_gt) * (CURRENT_DISTRIBUTION_LUT_SIZE - 1));

    for (uint32_t i = 0; i < CURRENT_DISTRIBUTION_LUT_SIZE - 1; i++)
    {
        // Calculate interpolation coefficients for each interval
        // alpha_coef = (alpha_upper - alpha_lower) / (im_upper - im_lower)
        ctrl_gt im_lower = CURRENT_DISTRIBUTION_LUT[i].im;
        ctrl_gt im_upper = CURRENT_DISTRIBUTION_LUT[i + 1].im;
        ctrl_gt alpha_lower = CURRENT_DISTRIBUTION_LUT[i].alpha;
        ctrl_gt alpha_upper = CURRENT_DISTRIBUTION_LUT[i + 1].alpha;

        distributor->alpha_coef[i] = (alpha_upper - alpha_lower) / (im_upper - im_lower);
    }
}



// Function to perform linear interpolation based on input im
GMP_STATIC_INLINE
ctrl_gt linear_interpolation(idq_current_distributor_t *distributor)
{
    int index = (int)((distributor->im - CURRENT_DISTRIBUTION_LUT[0].im) / CURRENT_DISTRIBUTION_LUT_STEP);

    // Prevent out-of-bound access
    if (index < 0)
        index = 0;
    if (index >= CURRENT_DISTRIBUTION_LUT_SIZE - 1)
        index = CURRENT_DISTRIBUTION_LUT_SIZE - 2;

    ctrl_gt im_lower = CURRENT_DISTRIBUTION_LUT[index].im;
    ctrl_gt im_upper = CURRENT_DISTRIBUTION_LUT[index + 1].im;
    ctrl_gt alpha_lower = CURRENT_DISTRIBUTION_LUT[index].alpha;
    ctrl_gt alpha_upper = CURRENT_DISTRIBUTION_LUT[index + 1].alpha;

    // Perform linear interpolation: alpha = alpha_lower + (input_im - im_lower) * alpha_coef
    return (alpha_lower + (distributor->im - im_lower) * (distributor->alpha_coef[index]));
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

    // Calculate id and iq based on the computed alpha and input im
    distributor->id = distributor->im * ctl_cos(distributor->alpha); // Convert alpha to radians
    distributor->iq = distributor->im * ctl_sin(distributor->alpha); // Convert alpha to radians
}

GMP_STATIC_INLINE
void distributor_set_im(idq_current_distributor_t *distributor, ctrl_gt im)
{
    distributor->im = im;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CURRENT_DISTRIBUTOR_H_
