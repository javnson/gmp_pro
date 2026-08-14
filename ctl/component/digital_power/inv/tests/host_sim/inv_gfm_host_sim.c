#include <math.h>
#include <stdio.h>

#include <ctl/math_block/gmp_math.h>

#include <ctl/component/digital_power/inv/inv_gfm_droop_ctrl.h>
#include <ctl/component/digital_power/inv/inv_gfm_transition.h>
#include <ctl/component/digital_power/inv/inv_gfm_virtual_impedance.h>
#include <ctl/component/digital_power/inv/inv_gfm_vsm_ctrl.h>
#include <ctl/component/digital_power/inv/gfl_pq_droop_ctrl.h>

static int near_value(ctrl_gt actual, float expected, float tolerance)
{
    return fabsf(ctrl2param(actual) - expected) <= tolerance;
}

static int test_droop_reference_generator(void)
{
    inv_gfm_droop_ctrl_t droop;
    inv_gfm_droop_init_t init = {
        10000.0f, 50.0f, 0.5f, 100.0f, 1.0f, 0.1f, 2.0f, 0.4f, 0.6f};
    ctl_vector2_t vdq = {{CTL_CTRL_CONST_1_OVER_2, 0}};
    ctl_vector2_t idq = {{real2ctrl(0.2f), real2ctrl(-0.2f)}};
    int k;

    ctl_init_inv_gfm_droop(&droop, &init);
    ctl_attach_inv_gfm_droop(&droop, &vdq, &idq);
    ctl_set_inv_gfm_droop_power_reference(&droop, 0, 0);
    ctl_enable_inv_gfm_droop(&droop);
    for (k = 0; k < 1000; ++k)
        ctl_step_inv_gfm_droop(&droop);

    if (!near_value(droop.pq_filt.dat[0], 0.1f, 2e-3f))
        return 1;
    if (!near_value(droop.pq_filt.dat[1], 0.1f, 2e-3f))
        return 2;
    if (!near_value(droop.frequency_ref_hz, 49.9f, 2e-3f))
        return 3;
    if (!near_value(droop.vdq_ref.dat[phase_d], 0.49f, 2e-3f))
        return 4;
    return 0;
}

static int test_bumpless_transition(void)
{
    inv_gfm_transition_t transition;
    inv_gfm_transition_init_t init = {1000.0f, 0.01f, 50.0f};
    ctrl_gt pll_angle = real2ctrl(0.25f);
    ctl_vector2_t pll_phasor;
    ctl_vector2_t tracking = {{0, 0}};
    ctl_vector2_t forming = {{CTL_CTRL_CONST_1, (-CTL_CTRL_CONST_1)}};
    ctl_vector2_t output;
    int k;

    ctl_set_phasor_via_angle(pll_angle, &pll_phasor);
    ctl_init_inv_gfm_transition(&transition, &init);
    ctl_attach_inv_gfm_transition(&transition, &pll_angle, &pll_phasor);
    ctl_step_inv_gfm_transition(&transition, real2ctrl(50.0f));
    if (!near_value(transition.angle_gfm, 0.25f, 1e-5f) ||
        !near_value(transition.blend, 0.0f, 1e-6f))
        return 1;

    ctl_request_forming_inv_gfm_transition(&transition);
    for (k = 0; k < 5; ++k)
        ctl_step_inv_gfm_transition(&transition, real2ctrl(50.0f));
    ctl_blend_inv_gfm_transition(&transition, &tracking, &forming, &output);
    if (!near_value(transition.blend, 0.5f, 2e-3f) ||
        !near_value(output.dat[phase_d], 0.5f, 2e-3f) ||
        !near_value(output.dat[phase_q], -0.5f, 2e-3f))
        return 2;

    for (k = 0; k < 5; ++k)
        ctl_step_inv_gfm_transition(&transition, real2ctrl(50.0f));
    if (!ctl_is_forming_inv_gfm_transition(&transition) ||
        !near_value(transition.blend, 1.0f, 2e-3f) ||
        fabsf(ctrl2param(ctl_vector2_mag(&transition.phasor_out)) - 1.0f) > 2e-3f)
        return 3;
    return 0;
}

static int test_vsm_reference_generator(void)
{
    inv_gfm_vsm_ctrl_t vsm;
    inv_gfm_vsm_init_t init = {
        1000.0f, 50.0f, 0.5f, 50.0f, 0.2f, 2.0f, 0.1f, 2.0f, 0.4f, 0.6f};
    ctl_vector2_t vdq = {{CTL_CTRL_CONST_1_OVER_2, 0}};
    ctl_vector2_t idq = {{real2ctrl(0.2f), real2ctrl(-0.2f)}};
    int k;

    ctl_init_inv_gfm_vsm(&vsm, &init);
    ctl_attach_inv_gfm_vsm(&vsm, &vdq, &idq);
    ctl_set_inv_gfm_vsm_power_reference(&vsm, 0, 0);
    ctl_enable_inv_gfm_vsm(&vsm);
    for (k = 0; k < 2000; ++k)
        ctl_step_inv_gfm_vsm(&vsm);

    if (!near_value(vsm.frequency_ref_hz, 49.95f, 3e-3f))
        return 1;
    if (!near_value(vsm.vdq_ref.dat[phase_d], 0.49f, 3e-3f))
        return 2;
    return 0;
}

static int test_virtual_impedance(void)
{
    inv_gfm_virtual_impedance_t impedance;
    inv_gfm_virtual_impedance_init_t init = {0.1f, 0.2f, 1.0f};
    ctl_vector2_t idq = {{real2ctrl(0.2f), real2ctrl(-0.1f)}};

    ctl_init_inv_gfm_virtual_impedance(&impedance, &init);
    ctl_attach_inv_gfm_virtual_impedance(&impedance, &idq);
    ctl_set_inv_gfm_virtual_impedance_base(&impedance, CTL_CTRL_CONST_1_OVER_2, 0);
    ctl_enable_inv_gfm_virtual_impedance(&impedance);
    ctl_step_inv_gfm_virtual_impedance(&impedance);

    if (!near_value(impedance.vdq_ref.dat[phase_d], 0.46f, 2e-3f))
        return 1;
    if (!near_value(impedance.vdq_ref.dat[phase_q], -0.03f, 2e-3f))
        return 2;
    return 0;
}

static int test_gfl_pq_droop(void)
{
    gfl_pq_droop_ctrl_t droop;
    gfl_pq_droop_init_t init = {
        1000.0f, 50.0f, 50.0f, 0.5f, 0.1f, 0.5f,
        -0.8f, 0.8f, -0.8f, 0.8f};
    ctrl_gt frequency_hz = real2ctrl(49.0f);
    ctl_vector2_t vdq = {{real2ctrl(0.3f), 0}};
    int k;

    ctl_init_gfl_pq_droop(&droop, &init);
    ctl_attach_gfl_pq_droop(&droop, &frequency_hz, &vdq);
    ctl_set_gfl_pq_droop_base(&droop, real2ctrl(0.1f), 0);
    ctl_enable_gfl_pq_droop(&droop);
    for (k = 0; k < 1000; ++k)
        ctl_step_gfl_pq_droop(&droop);

    if (!near_value(droop.pq_ref.dat[0], 0.2f, 2e-3f))
        return 1;
    if (!near_value(droop.pq_ref.dat[1], 0.1f, 2e-3f))
        return 2;
    return 0;
}

int main(void)
{
    int result = test_droop_reference_generator();
    if (result != 0)
    {
        fprintf(stderr, "droop test failed: %d\n", result);
        return 10 + result;
    }

    result = test_bumpless_transition();
    if (result != 0)
    {
        fprintf(stderr, "transition test failed: %d\n", result);
        return 20 + result;
    }

    result = test_vsm_reference_generator();
    if (result != 0)
    {
        fprintf(stderr, "VSM test failed: %d\n", result);
        return 30 + result;
    }

    result = test_virtual_impedance();
    if (result != 0)
    {
        fprintf(stderr, "virtual impedance test failed: %d\n", result);
        return 40 + result;
    }

    result = test_gfl_pq_droop();
    if (result != 0)
    {
        fprintf(stderr, "GFL PQ droop test failed: %d\n", result);
        return 50 + result;
    }

    puts("GFM/GFL outer-loop host tests passed.");
    return 0;
}
