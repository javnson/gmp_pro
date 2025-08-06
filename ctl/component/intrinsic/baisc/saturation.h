/**
 * @file saturation.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_CTL_SATURATION_H_
#define _FILE_CTL_SATURATION_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef struct _tag_saturation_t
{
    // output
    ctrl_gt out;

    // parameters
    // output limit
    ctrl_gt out_min;
    ctrl_gt out_max;

} ctl_saturation_t;

GMP_STATIC_INLINE
ctrl_gt ctl_step_saturation(ctl_saturation_t *obj, ctrl_gt input)
{
    obj->out = ctl_sat(input, obj->out_max, obj->out_min);

    return obj->out;
}

void ctl_init_saturation(ctl_saturation_t *obj, ctrl_gt out_min, ctrl_gt out_max);

GMP_STATIC_INLINE
void ctl_set_saturation(ctl_saturation_t *obj, ctrl_gt out_min, ctrl_gt out_max)
{
    obj->out_min = out_min;
    obj->out_max = out_max;
}

// limit output in [-max, -min] or [min, max] range
typedef struct _tag_bipolar_saturation_t
{
    // output
    ctrl_gt out;

    // parameters
    // output limit, both of these param should greater than 0
    ctrl_gt out_min;
    ctrl_gt out_max;
} ctl_bipolar_saturation_t;

GMP_STATIC_INLINE
ctrl_gt ctl_step_bipolar_saturation(ctl_bipolar_saturation_t *obj, ctrl_gt input)
{
    if (input > 0)
        obj->out = ctl_sat(input, obj->out_max, obj->out_min);
    else
        obj->out = ctl_sat(input, -obj->out_min, -obj->out_max);

    return obj->out;
}

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _FILE_CTL_SATURATION_H_
