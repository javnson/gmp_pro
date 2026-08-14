/**
 * @file intrinsic_gtests.cpp
 * @brief Portable Google Test coverage for representative CTL intrinsic components.
 */

#include <gtest/gtest.h>

#include <ctl/portable/gmp_ctl_portable.h>
#include <ctl/component/intrinsic/basic/hysteresis_controller.h>
#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/continuous/continuous_pi.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>

namespace
{
constexpr double tolerance = 2.0e-5;

/** @brief Convert a control value to the hosted comparison type. */
double value_of(ctrl_gt value)
{
    return static_cast<double>(ctrl2param(value));
}
} // namespace

/** @brief Return a deterministic portable tick for hosted tests. */
time_gt gmp_ctl_portable_get_tick(void)
{
    return 0U;
}

/** @brief Verify the basic limiter components. */
TEST(BasicIntrinsic, SaturationAndSlopeLimitsAreApplied)
{
    ctl_saturation_t saturation{};
    ctl_init_saturation(&saturation, real2ctrl(-0.5), real2ctrl(0.5));
    EXPECT_NEAR(value_of(ctl_step_saturation(&saturation, real2ctrl(0.75))), 0.5, tolerance);
    EXPECT_NEAR(value_of(ctl_step_saturation(&saturation, real2ctrl(-0.75))), -0.5, tolerance);

    ctl_slope_limiter_t slope{};
    ctl_init_slope_limiter(&slope, real2param(10.0), real2param(-20.0), real2param(100.0));
    EXPECT_NEAR(value_of(ctl_step_slope_limiter(&slope, CTL_CTRL_CONST_1)), 0.1, tolerance);
    EXPECT_NEAR(value_of(ctl_step_slope_limiter(&slope, real2ctrl(-1.0))), -0.1, tolerance);
}

/** @brief Verify hysteresis transitions and state retention. */
TEST(BasicIntrinsic, HysteresisStateIsLatchedInsideTheBand)
{
    ctl_hysteresis_controller_t controller{};
    ctl_init_hysteresis_controller(&controller, 1, real2ctrl(0.1));
    ctl_set_hysteresis_target(&controller, real2ctrl(0.5));
    EXPECT_EQ(ctl_step_hysteresis_controller(&controller, real2ctrl(0.7)), 1);
    EXPECT_EQ(ctl_step_hysteresis_controller(&controller, real2ctrl(0.5)), 1);
    EXPECT_EQ(ctl_step_hysteresis_controller(&controller, real2ctrl(0.3)), 0);
}

/** @brief Verify representative continuous-controller contracts. */
TEST(ContinuousIntrinsic, PiAndPidRespectTheirCoefficientsAndLimits)
{
    ctl_pi_t pi{};
    ctl_init_pi(&pi, real2param(2.0), real2param(4.0), real2param(1000.0));
    EXPECT_NEAR(value_of(pi.ki), 0.004, tolerance);
    EXPECT_NEAR(value_of(ctl_step_pi_par(&pi, real2ctrl(0.1))), 0.2004, tolerance);

    ctl_pid_t pid{};
    ctl_init_pid(&pid, CTL_PARAM_CONST_1, CTL_PARAM_CONST_1, CTL_PARAM_CONST_ZERO, real2param(100.0));
    ctl_set_pid_limit(&pid, CTL_CTRL_CONST_1_OVER_2, -CTL_CTRL_CONST_1_OVER_2);
    EXPECT_NEAR(value_of(ctl_step_pid_par(&pid, CTL_CTRL_CONST_1)), 0.5, tolerance);
}

/** @brief Verify the low-pass step and steady-state response. */
TEST(DiscreteIntrinsic, LowPassStepResponseConvergesMonotonically)
{
    ctl_low_pass_filter_t filter{};
    ctl_init_lp_filter(&filter, real2param(1000.0), real2param(10.0));
    double previous = 0.0;
    for (int iteration = 0; iteration < 200; ++iteration)
    {
        const double current = value_of(ctl_step_lowpass_filter(&filter, CTL_CTRL_CONST_1));
        EXPECT_GE(current + tolerance, previous);
        previous = current;
    }
    EXPECT_GT(previous, 0.99);
    EXPECT_LE(previous, 1.0 + tolerance);
}

/** @brief Ramp frequency is signed: zero is stationary and negative values run in reverse. */
TEST(DiscreteIntrinsic, RampGeneratorSupportsStationaryAndReverseFrequencies)
{
    ctl_ramp_generator_t ramp{};

    ctl_init_ramp_generator_via_freq(&ramp, real2param(4.0), real2param(1.0),
                                     CTL_PARAM_CONST_1, -CTL_PARAM_CONST_1);
    EXPECT_NEAR(value_of(ramp.slope), 0.5, tolerance);
    EXPECT_NEAR(value_of(ctl_step_ramp_generator(&ramp)), 0.5, tolerance);
    EXPECT_NEAR(value_of(ctl_step_ramp_generator(&ramp)), 1.0, tolerance);
    EXPECT_NEAR(value_of(ctl_step_ramp_generator(&ramp)), -1.0, tolerance);

    ctl_init_ramp_generator_via_freq(&ramp, real2param(4.0), CTL_PARAM_CONST_ZERO,
                                     CTL_PARAM_CONST_1, -CTL_PARAM_CONST_1);
    EXPECT_NEAR(value_of(ramp.slope), 0.0, tolerance);
    EXPECT_NEAR(value_of(ctl_step_ramp_generator(&ramp)), 0.0, tolerance);

    ctl_init_ramp_generator_via_freq(&ramp, real2param(4.0), real2param(-1.0),
                                     CTL_PARAM_CONST_1, -CTL_PARAM_CONST_1);
    EXPECT_NEAR(value_of(ramp.slope), -0.5, tolerance);
    EXPECT_NEAR(value_of(ctl_step_ramp_generator(&ramp)), -0.5, tolerance);
    EXPECT_NEAR(value_of(ctl_step_ramp_generator(&ramp)), -1.0, tolerance);
    EXPECT_NEAR(value_of(ctl_step_ramp_generator(&ramp)), 1.0, tolerance);
}
