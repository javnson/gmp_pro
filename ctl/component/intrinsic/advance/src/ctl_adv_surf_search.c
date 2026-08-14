#include <ctl/math_block/gmp_math.h>


//////////////////////////////////////////////////////////////////////////
// surf_search, surface search function

#include <ctl/component/intrinsic/advance/surf_search.h>

void ctl_init_lut1d(ctl_lut1d_t* lut, const ctrl_gt* axis, uint32_t size)
{
    lut->axis = axis;
    lut->size = size;
}

void ctl_init_lut2d(ctl_lut2d_t* lut, const ctrl_gt* axis1, uint32_t size1, const ctrl_gt* axis2, uint32_t size2,
                    const ctrl_gt* surface) // Surface data uses flattened row-major storage.
{
    ctl_init_lut1d(&lut->dim1_axis, axis1, size1);
    ctl_init_lut1d(&lut->dim2_axis, axis2, size2);
    lut->surface = surface;
}

void ctl_init_uniform_lut2d(ctl_uniform_lut2d_t* lut, ctrl_gt x_min, ctrl_gt x_max, uint32_t x_size, ctrl_gt y_min,
                            ctrl_gt y_max, uint32_t y_size,
                            const ctrl_gt* surface) // Surface data uses flattened row-major storage.
{
    lut->x_min = x_min;
    lut->x_size = x_size;

    // Calculate the normalized X coordinate in the parameter domain.
    parameter_gt x_delta = ctrl2param(x_max) - ctrl2param(x_min);
    if (param_abs(x_delta) < 1e-9f)
    {
        lut->x_step_inv = CTL_CTRL_CONST_ZERO;
    }
    else
    {
        // Convert the final normalized coordinate back to the control domain.
        lut->x_step_inv = real2ctrl((parameter_gt)(lut->x_size - 1) / x_delta);
    }

    lut->y_min = y_min;
    lut->y_size = y_size;

    // Apply the same normalization to the Y axis.
    parameter_gt y_delta = ctrl2param(y_max) - ctrl2param(y_min);
    if (param_abs(y_delta) < 1e-9f)
    {
        lut->y_step_inv = CTL_CTRL_CONST_ZERO;
    }
    else
    {
        lut->y_step_inv = real2ctrl((parameter_gt)(lut->y_size - 1) / y_delta);
    }

    lut->surface = surface;
}
