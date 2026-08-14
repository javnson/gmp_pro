/**
 * @file intrinsic_tests.cpp
 * @brief Visual Studio tests for representative CTL intrinsic components.
 */

#include "vs_test_support.h"

#include <ctl/portable/gmp_ctl_portable.h>
#include <ctl/component/intrinsic/basic/hysteresis_controller.h>
#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/continuous/continuous_pi.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_ctl_unit_test
{
namespace
{
constexpr double intrinsic_tolerance = 2.0e-5;

double intrinsic_value(ctrl_gt value)
{
    return static_cast<double>(ctrl2param(value));
}
} // namespace

TEST_CLASS(BasicIntrinsicTests)
{
  public:
    /** @brief Verify saturation behavior inside and outside both limits. */
    TEST_METHOD(SaturationClampsBothLimitsAndPassesInteriorValues)
    {
        ctl_saturation_t limiter{};
        ctl_init_saturation(&limiter, real2ctrl(-0.5), real2ctrl(0.5));
        expect_near(intrinsic_value(ctl_step_saturation(&limiter, real2ctrl(0.25))), 0.25, intrinsic_tolerance,
                    L"saturation changed an interior value");
        expect_near(intrinsic_value(ctl_step_saturation(&limiter, real2ctrl(0.75))), 0.5, intrinsic_tolerance,
                    L"saturation did not apply its upper limit");
        expect_near(intrinsic_value(ctl_step_saturation(&limiter, real2ctrl(-0.75))), -0.5, intrinsic_tolerance,
                    L"saturation did not apply its lower limit");
    }

    /** @brief Verify asymmetric positive and negative slope constraints. */
    TEST_METHOD(SlopeLimiterConstrainsPositiveAndNegativeChanges)
    {
        ctl_slope_limiter_t limiter{};
        ctl_init_slope_limiter(&limiter, real2param(10.0), real2param(-20.0), real2param(100.0));
        expect_near(intrinsic_value(ctl_step_slope_limiter(&limiter, CTL_CTRL_CONST_1)), 0.1,
                    intrinsic_tolerance, L"positive slope limit failed");
        expect_near(intrinsic_value(ctl_step_slope_limiter(&limiter, real2ctrl(-1.0))), -0.1,
                    intrinsic_tolerance, L"negative slope limit failed");
    }

    /** @brief Verify hysteresis transitions and in-band state retention. */
    TEST_METHOD(HysteresisControllerLatchesInsideItsBand)
    {
        ctl_hysteresis_controller_t controller{};
        ctl_init_hysteresis_controller(&controller, 1, real2ctrl(0.1));
        ctl_set_hysteresis_target(&controller, real2ctrl(0.5));
        Assert::AreEqual<fast_gt>(1, ctl_step_hysteresis_controller(&controller, real2ctrl(0.7)));
        Assert::AreEqual<fast_gt>(1, ctl_step_hysteresis_controller(&controller, real2ctrl(0.5)));
        Assert::AreEqual<fast_gt>(0, ctl_step_hysteresis_controller(&controller, real2ctrl(0.3)));
    }
};

TEST_CLASS(ContinuousIntrinsicTests)
{
  public:
    /** @brief Verify PI coefficient initialization and step output. */
    TEST_METHOD(PiInitializationAndStepUseTheExpectedCoefficients)
    {
        ctl_pi_t pi{};
        ctl_init_pi(&pi, real2param(2.0), real2param(4.0), real2param(1000.0));
        expect_near(intrinsic_value(pi.kp), 2.0, intrinsic_tolerance, L"PI proportional gain is wrong");
        expect_near(intrinsic_value(pi.ki), 0.004, intrinsic_tolerance, L"PI integral gain is wrong");
        expect_near(intrinsic_value(ctl_step_pi_par(&pi, real2ctrl(0.1))), 0.2004, intrinsic_tolerance,
                    L"PI step result is wrong");
    }

    /** @brief Verify PID output and integrator limiting. */
    TEST_METHOD(PidLimitsOutputAndIntegratorState)
    {
        ctl_pid_t pid{};
        ctl_init_pid(&pid, real2param(1.0), real2param(1.0), CTL_PARAM_CONST_ZERO, real2param(100.0));
        ctl_set_pid_limit(&pid, CTL_CTRL_CONST_1_OVER_2, -CTL_CTRL_CONST_1_OVER_2);
        expect_near(intrinsic_value(ctl_step_pid_par(&pid, CTL_CTRL_CONST_1)), 0.5, intrinsic_tolerance,
                    L"PID output limit failed");
        for (int iteration = 0; iteration < 200; ++iteration)
        {
            (void)ctl_step_pid_par(&pid, CTL_CTRL_CONST_1);
        }
        Assert::IsTrue(intrinsic_value(pid.i_term) <= intrinsic_value(pid.integral_max) + intrinsic_tolerance);
    }
};

TEST_CLASS(DiscreteIntrinsicTests)
{
  public:
    /** @brief Ramp frequency is signed: zero is stationary and negative values run in reverse. */
    TEST_METHOD(RampGeneratorSupportsStationaryAndReverseFrequencies)
    {
        ctl_ramp_generator_t ramp{};

        ctl_init_ramp_generator_via_freq(&ramp, real2param(4.0), real2param(1.0),
                                         CTL_PARAM_CONST_1, -CTL_PARAM_CONST_1);
        expect_near(intrinsic_value(ramp.slope), 0.5, intrinsic_tolerance,
                    L"positive frequency did not create a forward slope");
        expect_near(intrinsic_value(ctl_step_ramp_generator(&ramp)), 0.5, intrinsic_tolerance,
                    L"forward ramp did not accumulate toward one");
        expect_near(intrinsic_value(ctl_step_ramp_generator(&ramp)), 1.0, intrinsic_tolerance,
                    L"forward ramp did not reach its upper boundary");
        expect_near(intrinsic_value(ctl_step_ramp_generator(&ramp)), -1.0, intrinsic_tolerance,
                    L"forward ramp did not wrap through one to minus one");

        ctl_init_ramp_generator_via_freq(&ramp, real2param(4.0), CTL_PARAM_CONST_ZERO,
                                         CTL_PARAM_CONST_1, -CTL_PARAM_CONST_1);
        expect_near(intrinsic_value(ramp.slope), 0.0, intrinsic_tolerance,
                    L"zero-frequency ramp did not start stationary");
        expect_near(intrinsic_value(ctl_step_ramp_generator(&ramp)), 0.0, intrinsic_tolerance,
                    L"zero-frequency ramp advanced unexpectedly");

        ctl_init_ramp_generator_via_freq(&ramp, real2param(4.0), real2param(-1.0),
                                         CTL_PARAM_CONST_1, -CTL_PARAM_CONST_1);
        expect_near(intrinsic_value(ramp.slope), -0.5, intrinsic_tolerance,
                    L"negative frequency did not create a reverse slope");
        expect_near(intrinsic_value(ctl_step_ramp_generator(&ramp)), -0.5, intrinsic_tolerance,
                    L"reverse ramp did not accumulate toward minus one");
        expect_near(intrinsic_value(ctl_step_ramp_generator(&ramp)), -1.0, intrinsic_tolerance,
                    L"reverse ramp did not reach its lower boundary");
        expect_near(intrinsic_value(ctl_step_ramp_generator(&ramp)), 1.0, intrinsic_tolerance,
                    L"reverse ramp did not wrap through minus one to one");
    }

    /** @brief Verify low-pass transient and steady-state behavior. */
    TEST_METHOD(LowPassFilterConvergesMonotonicallyToAStep)
    {
        ctl_low_pass_filter_t filter{};
        ctl_init_lp_filter(&filter, real2param(1000.0), real2param(10.0));
        double previous = 0.0;
        for (int iteration = 0; iteration < 200; ++iteration)
        {
            const double current = intrinsic_value(ctl_step_lowpass_filter(&filter, CTL_CTRL_CONST_1));
            Assert::IsTrue(current + intrinsic_tolerance >= previous, L"low-pass step response is not monotonic");
            previous = current;
        }
        Assert::IsTrue(previous > 0.99 && previous <= 1.0 + intrinsic_tolerance,
                       L"low-pass filter did not converge to its steady-state value");
        ctl_clear_lowpass_filter(&filter);
        expect_near(intrinsic_value(ctl_get_lowpass_filter_result(&filter)), 0.0, intrinsic_tolerance,
                    L"low-pass clear operation failed");
    }
};
} // namespace gmp_ctl_unit_test
