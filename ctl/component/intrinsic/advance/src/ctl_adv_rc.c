#include <ctl/math_block/gmp_math.h>

//////////////////////////////////////////////////////////////////////////
// repetitive controller
#include <ctl/component/intrinsic/advance/repetitive_controller.h>

void ctl_init_rc(ctl_rc_t* obj, ctrl_gt* buffer, uint32_t capacity, parameter_gt fs, parameter_gt f_min,
                 parameter_gt q_gain, parameter_gt k_rc, int32_t phase_lead_k)
{
    gmp_ctl_assert(buffer != NULL);
    gmp_ctl_assert(fs > 0.0f);
    gmp_ctl_assert(f_min > 0.0f);
    gmp_ctl_assert(capacity >= CTL_RC_CALC_MIN_CAPACITY(fs, f_min));

    obj->buffer = buffer;
    obj->buffer_capacity = capacity;
    obj->phase_lead_k = phase_lead_k;

    obj->q_gain = real2ctrl(q_gain);
    obj->k_rc = real2ctrl(k_rc);

    obj->out_max = CTL_CTRL_CONST_1;
    obj->out_min = (-CTL_CTRL_CONST_1);

    obj->fs = fs;
    obj->f_min_rated = f_min;
    ctl_set_rc_frequency(obj, f_min);

    ctl_enable_rc_integrating(obj);
    ctl_clear_rc(obj);
}
