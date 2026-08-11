#include <math.h>
#include <stdio.h>

#include <gmp_core.h>

#include <ctl/component/intrinsic/complex/dq_ladrc1.h>
#include <ctl/component/intrinsic/complex/dq_pi.h>

static int near_value(ctrl_gt actual, ctrl_gt expected, ctrl_gt tolerance)
{
    return fabsf(ctrl2float(actual - expected)) <= ctrl2float(tolerance);
}

static int test_vector_limiters(void)
{
    ctl_vector2_t vec = {{float2ctrl(3.0f), float2ctrl(4.0f)}};
    ctl_vector2_t approx_vec = {{float2ctrl(1.2f), float2ctrl(1.6f)}};
    ctl_vector2_t far_vec = {{float2ctrl(2.0f), float2ctrl(0.0f)}};
    ctl_vector2_t zero_limit_vec = {{float2ctrl(1.0f), float2ctrl(0.0f)}};
    ctl_vector2_t max = {{float2ctrl(1.0f), float2ctrl(2.0f)}};
    ctl_vector2_t min = {{float2ctrl(-1.0f), float2ctrl(-2.0f)}};

    ctl_vector2_sat_circle_sq(&vec, &vec, float2ctrl(4.0f));
    if (!near_value(vec.dat[0], float2ctrl(1.2f), float2ctrl(1e-5f)) ||
        !near_value(vec.dat[1], float2ctrl(1.6f), float2ctrl(1e-5f)))
        return 1;

    ctl_vector2_sat_circle_sq_taylor(&approx_vec, &approx_vec, float2ctrl(3.24f));
    if (ctl_vector2_mag_sq(&approx_vec) > float2ctrl(3.24f) + float2ctrl(1e-5f) ||
        approx_vec.dat[0] <= float2ctrl(0.0f) ||
        !near_value(ctl_div(approx_vec.dat[1], approx_vec.dat[0]),
                    float2ctrl(4.0f / 3.0f), float2ctrl(1e-5f)))
        return 2;

    ctl_vector2_sat_circle_sq_taylor(&far_vec, &far_vec, float2ctrl(1.0f));
    ctl_vector2_sat_circle_sq(&zero_limit_vec, &zero_limit_vec, float2ctrl(0.0f));
    if (!near_value(far_vec.dat[0], float2ctrl(0.0f), float2ctrl(1e-5f)) ||
        !near_value(zero_limit_vec.dat[0], float2ctrl(0.0f), float2ctrl(1e-5f)))
        return 3;

    ctl_vector2_sat_rect(&vec, &vec, &max, &min);
    if (!near_value(vec.dat[0], float2ctrl(1.0f), float2ctrl(1e-5f)) ||
        !near_value(vec.dat[1], float2ctrl(1.6f), float2ctrl(1e-5f)))
        return 4;

    ctl_vector2_sat_square(&vec, &vec, float2ctrl(0.5f));
    if (!near_value(vec.dat[0], float2ctrl(0.5f), float2ctrl(1e-5f)) ||
        !near_value(vec.dat[1], float2ctrl(0.5f), float2ctrl(1e-5f)))
        return 5;

    return 0;
}

static int test_scalar_compatibility(void)
{
    ctl_pid_t pid;
    ctl_ladrc1_t ladrc;

    ctl_init_pid(&pid, 2.0f, 0.0f, 0.0f, 10000.0f);
    if (!near_value(ctl_step_pid_par(&pid, float2ctrl(1.0f)), float2ctrl(1.0f), float2ctrl(1e-5f)))
        return 1;

    ctl_init_ladrc1(&ladrc, 1.0f, 100.0f, 500.0f, 10000.0f);
    if (!near_value(ctl_step_ladrc1(&ladrc, float2ctrl(10.0f), float2ctrl(0.0f)),
                    float2ctrl(1.0f), float2ctrl(1e-5f)))
        return 2;

    return 0;
}

static int test_dq_pi(void)
{
    ctl_dq_pi_t dq;
    ctl_vector2_t target = {{float2ctrl(3.0f), float2ctrl(4.0f)}};
    ctl_vector2_t feedback = {{float2ctrl(0.0f), float2ctrl(0.0f)}};
    ctl_vector2_t feedforward = {{float2ctrl(0.5f), float2ctrl(0.0f)}};
    ctl_vector2_t max = {{float2ctrl(1.0f), float2ctrl(2.0f)}};
    ctl_vector2_t min = {{float2ctrl(-1.0f), float2ctrl(-2.0f)}};
    ctl_vector2_t output;

    ctl_init_dq_pi(&dq, 1.0f, 0.0f, 1.0f, 0.0f, 10000.0f);
    ctl_set_pid_int_limit(&dq.axis[0], float2ctrl(10.0f), float2ctrl(-10.0f));
    ctl_set_pid_int_limit(&dq.axis[1], float2ctrl(10.0f), float2ctrl(-10.0f));
    ctl_set_dq_pi_circle_limit_sq(&dq, float2ctrl(4.0f));
    ctl_set_dq_pi_rect_limit(&dq, &max, &min);
    ctl_enable_dq_pi_feedforward(&dq);
    ctl_enable_dq_pi_circle_limit(&dq);

    ctl_step_dq_pi(&dq, &target, &feedback, &feedforward, &output);

    if (!near_value(output.dat[0], float2ctrl(1.0f), float2ctrl(1e-5f)))
        return 1;
    if (ctl_vector2_mag(&output) > float2ctrl(2.0f) + float2ctrl(1e-5f))
        return 2;
    if (!near_value(dq.axis[0].p_term + dq.axis[0].i_term,
                    output.dat[0] - feedforward.dat[0], float2ctrl(1e-5f)))
        return 3;

    ctl_disable_dq_pi_feedforward(&dq);
    ctl_step_dq_pi(&dq, &target, &feedback, NULL, NULL);
    if (!near_value(dq.ff_out.dat[0], float2ctrl(0.0f), float2ctrl(1e-5f)))
        return 4;

    return 0;
}

static int test_dq_ladrc(void)
{
    ctl_dq_ladrc1_t dq;
    ctl_vector2_t zero = {{float2ctrl(0.0f), float2ctrl(0.0f)}};
    ctl_vector2_t feedforward = {{float2ctrl(1.0f), float2ctrl(0.0f)}};
    ctl_vector2_t max = {{float2ctrl(0.4f), float2ctrl(1.0f)}};
    ctl_vector2_t min = {{float2ctrl(-0.4f), float2ctrl(-1.0f)}};
    ctl_vector2_t output;

    ctl_init_dq_ladrc1(&dq, 1.0f, 100.0f, 500.0f, 1.0f, 100.0f, 500.0f, 10000.0f);
    ctl_set_dq_ladrc1_circle_limit(&dq, float2ctrl(0.5f));
    ctl_set_dq_ladrc1_rect_limit(&dq, &max, &min);
    ctl_enable_dq_ladrc1_feedforward(&dq);
    ctl_enable_dq_ladrc1_circle_limit(&dq);

    ctl_step_dq_ladrc1(&dq, &zero, &zero, &feedforward, &output);

    if (!near_value(output.dat[0], float2ctrl(0.4f), float2ctrl(1e-5f)))
        return 1;
    if (!near_value(dq.axis[0].u_prev, output.dat[0], float2ctrl(1e-5f)))
        return 2;

    return 0;
}

int main(void)
{
    int result;

    result = test_vector_limiters();
    if (result)
    {
        printf("vector limiter test failed: %d\n", result);
        return 10 + result;
    }

    result = test_scalar_compatibility();
    if (result)
    {
        printf("scalar compatibility test failed: %d\n", result);
        return 20 + result;
    }

    result = test_dq_pi();
    if (result)
    {
        printf("d-q PI test failed: %d\n", result);
        return 30 + result;
    }

    result = test_dq_ladrc();
    if (result)
    {
        printf("d-q LADRC test failed: %d\n", result);
        return 40 + result;
    }

    puts("d-q controller host tests passed");
    return 0;
}
