/**
 * @file numeric_type_contract_test.c
 * @brief Runtime checks for CTL numeric domains, angles, constants, and capabilities.
 */

#include <math.h>
#include <stdio.h>

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/intrinsic/advance/fdrc.h>

#if (SPECIFY_CTRL_GT_TYPE == USING_FIXED_TI_IQ_LIBRARY) && CTL_FDRC_SUPPORTED
#error FDRC_must_be_disabled_for_fixed_point_ctrl_gt
#endif

/** @brief Compares two parameter-domain values using an explicit test tolerance. */
static int expect_near(const char* label, parameter_gt actual, parameter_gt expected, parameter_gt tolerance)
{
    parameter_gt error = param_abs(actual - expected);
    if (error > tolerance)
    {
        printf("FAIL %s: actual=%.12g expected=%.12g error=%.12g\n", label, (double)actual, (double)expected,
               (double)error);
        return 1;
    }
    return 0;
}

int main(void)
{
#if (SPECIFY_CTRL_GT_TYPE == USING_FIXED_TI_IQ_LIBRARY)
    const parameter_gt ctrl_tolerance = real2param(2.0e-4);
#else
    const parameter_gt ctrl_tolerance = real2param(2.0e-6);
#endif
    const parameter_gt param_tolerance = (sizeof(parameter_gt) == sizeof(double))
                                             ? CTL_PARAM_CONST_EPSILON
                                             : CTL_PARAM_CONST_EPSILON * CTL_PARAM_CONST_2;
    int failures = 0;
#if !CTL_FDRC_SUPPORTED
    ctl_fdrc_t unsupported_fdrc = {0};
    ctl_enable_fdrc_integrating(&unsupported_fdrc);
    if (unsupported_fdrc.flag_enable_rc_integrating != 0 ||
        ctl_step_fdrc(&unsupported_fdrc, CTL_CTRL_CONST_1) != CTL_CTRL_CONST_ZERO)
    {
        printf("FAIL fixed-point FDRC capability guard\n");
        failures++;
    }
#endif
    ctrl_gt quarter_turn = real2ctrl(0.25);
    ctrl_gt half_pi = param2ctrl(CTL_PARAM_CONST_PI * CTL_PARAM_CONST_1_OVER_2);

    failures += expect_near("real2param", real2param(0.125), real2param(0.125), param_tolerance);
    failures += expect_near("real2ctrl/ctrl2param", ctrl2param(real2ctrl(0.375)), real2param(0.375), ctrl_tolerance);
    failures += expect_near("param2ctrl/ctrl2param", ctrl2param(param2ctrl(real2param(-0.625))),
                            real2param(-0.625), ctrl_tolerance);

    failures += expect_near("ctrl pi", ctrl2param(CTL_CTRL_CONST_PI), CTL_PARAM_CONST_PI, ctrl_tolerance);
    failures += expect_near("ctrl sqrt3", ctrl2param(CTL_CTRL_CONST_SQRT_3), CTL_PARAM_CONST_SQRT3,
                            ctrl_tolerance);
    failures += expect_near("param sqrt", param_sqrt(real2param(9.0)), real2param(3.0), param_tolerance);
    failures += expect_near("param pow", param_pow(real2param(2.0), real2param(3.0)), real2param(8.0),
                            param_tolerance);

    failures += expect_near("param sin rad", param_sin(CTL_PARAM_CONST_PI * CTL_PARAM_CONST_1_OVER_2),
                            CTL_PARAM_CONST_1, param_tolerance);
    failures += expect_near("param cos rad", param_cos(CTL_PARAM_CONST_PI), -CTL_PARAM_CONST_1, param_tolerance);
    failures += expect_near("param sin pu", param_sin_pu(CTL_PARAM_CONST_1_OVER_4), CTL_PARAM_CONST_1,
                            param_tolerance);
    failures += expect_near("param cos pu", param_cos_pu(CTL_PARAM_CONST_1_OVER_2), -CTL_PARAM_CONST_1,
                            param_tolerance);

    failures += expect_near("ctl sin pu", ctrl2param(ctl_sin(quarter_turn)), CTL_PARAM_CONST_1, ctrl_tolerance);
    failures += expect_near("ctl cos pu", ctrl2param(ctl_cos(CTL_CTRL_CONST_1_OVER_2)), -CTL_PARAM_CONST_1,
                            ctrl_tolerance);
    failures += expect_near("ctl sin rad", ctrl2param(ctl_sin_rad(half_pi)), CTL_PARAM_CONST_1, ctrl_tolerance);
    failures += expect_near("ctl cos rad", ctrl2param(ctl_cos_rad(CTL_CTRL_CONST_PI)), -CTL_PARAM_CONST_1,
                            ctrl_tolerance);
    failures += expect_near("ctl pow", ctrl2param(ctl_pow(real2ctrl(2.0), real2ctrl(3.0))), real2param(8.0),
                            real2param(2.0e-3));
    failures += expect_near("ctl atan2 rad", ctrl2param(ctl_atan2(CTL_CTRL_CONST_1, CTL_CTRL_CONST_ZERO)),
                            CTL_PARAM_CONST_PI * CTL_PARAM_CONST_1_OVER_2, real2param(2.0e-3));
    failures += expect_near("ctl atan2 pu", ctrl2param(ctl_atan2_pu(CTL_CTRL_CONST_1, CTL_CTRL_CONST_ZERO)),
                            CTL_PARAM_CONST_1_OVER_4, real2param(2.0e-3));

    if (failures == 0)
        printf("PASS ctrl=%d parameter=%d\n", SPECIFY_CTRL_GT_TYPE, SPECIFY_PARAMETER_GT_TYPE);
    return failures == 0 ? 0 : 1;
}
