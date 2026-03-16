#include <gmp_core.h>


//////////////////////////////////////////////////////////////////////////
// surf_search, surface search function

#include <ctl/component/intrinsic/advance/surf_search.h>

void ctl_init_lut1d(ctl_lut1d_t* lut, const ctrl_gt* axis, uint32_t size)
{
    lut->axis = axis;
    lut->size = size;
}

void ctl_init_lut2d(ctl_lut2d_t* lut, const ctrl_gt* axis1, uint32_t size1, const ctrl_gt* axis2, uint32_t size2,
                    const ctrl_gt* surface) // 注意：改为一维指针
{
    ctl_init_lut1d(&lut->dim1_axis, axis1, size1);
    ctl_init_lut1d(&lut->dim2_axis, axis2, size2);
    lut->surface = surface;
}

void ctl_init_uniform_lut2d(ctl_uniform_lut2d_t* lut, ctrl_gt x_min, ctrl_gt x_max, uint32_t x_size, ctrl_gt y_min,
                            ctrl_gt y_max, uint32_t y_size, const ctrl_gt* surface) // 注意：改为一维指针
{
    lut->x_min = x_min;
    lut->x_size = x_size;

    // 修复：必须先通过 ctrl2float 转换到物理域后再做减法，否则定点数相减会产生巨大整型垃圾
    parameter_gt x_delta = ctrl2float(x_max) - ctrl2float(x_min);
    if (fabs(x_delta) < 1e-9)
    {
        lut->x_step_inv = float2ctrl(0.0f);
    }
    else
    {
        // 浮点除法完成后安全转回控制域
        lut->x_step_inv = float2ctrl((parameter_gt)(lut->x_size - 1) / x_delta);
    }

    lut->y_min = y_min;
    lut->y_size = y_size;

    // 同理修复 Y 轴
    parameter_gt y_delta = ctrl2float(y_max) - ctrl2float(y_min);
    if (fabs(y_delta) < 1e-9)
    {
        lut->y_step_inv = float2ctrl(0.0f);
    }
    else
    {
        lut->y_step_inv = float2ctrl((parameter_gt)(lut->y_size - 1) / y_delta);
    }

    lut->surface = surface;
}
