#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Z transfer function

#include <ctl/component/intrinsic/discrete/z_function.h>

void ctl_init_z_function(ctl_z_function_t* obj, int32_t num_order, const ctrl_gt* num_coeffs, int32_t den_order,
                         const ctrl_gt* den_coeffs, ctrl_gt* input_buffer, ctrl_gt* output_buffer)
{
    // Assign orders
    obj->num_order = num_order;
    obj->den_order = den_order;

    // Assign pointers to coefficients and state buffers
    obj->num_coeffs = num_coeffs;
    obj->den_coeffs = den_coeffs;
    obj->input_buffer = input_buffer;
    obj->output_buffer = output_buffer;

    // Clear all state buffers to ensure a clean start
    ctl_clear_z_function(obj);
}
