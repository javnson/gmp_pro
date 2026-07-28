#include <gmp_core.h>

#include <ctl/component/digital_power/inv/inv_voltage_ctrl.h>
#include <ctl/component/digital_power/inv/inv_zero_ctrl.h>
#include <ctl/math_block/coordinate/svpwm_3d.h>

int main()
{
    ctl_vector3_t reference = {};
    ctl_vector4_t duty = {};
    inv_voltage_ctrl_t voltage = {};
    inv_zero_ctrl_t zero = {};

    ctl_ct_svpwm_3d_calc(&reference, &duty);
    return (voltage.flag_enable != 0) || (zero.flag_enable != 0) || (duty.dat[phase_N] != 0.5f);
}
