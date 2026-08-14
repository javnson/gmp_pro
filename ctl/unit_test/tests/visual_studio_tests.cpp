/**
 * @file visual_studio_tests.cpp
 * @brief Visual Studio Test Explorer wrappers for core CTL contracts.
 */

#include <CppUnitTest.h>

#include <cmath>
#include <type_traits>

#include <ctl/portable/gmp_ctl_portable.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/math_block/matrix_lite/matrix.hpp>
#include <ctl/math_block/vector_lite/vector.hpp>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_ctl_unit_test
{
namespace
{
#if defined(GMP_CTL_TEST_DOUBLE)
constexpr double ctrl_tolerance = 1.0e-12;
#else
constexpr double ctrl_tolerance = 1.0e-6;
#endif

/** @brief Convert a control-domain value to the host comparison type. */
double ctrl_value(ctrl_gt value)
{
    return static_cast<double>(ctrl2param(value));
}

/** @brief Assert that two host values differ by no more than a tolerance. */
void expect_near(double actual, double expected, double tolerance, const wchar_t* message)
{
    Assert::IsTrue(std::abs(actual - expected) <= tolerance, message);
}
} // namespace

TEST_CLASS(NumericDomainTests)
{
  public:
    TEST_METHOD(SelectedTypesMatchTheProjectContract)
    {
        static_assert(std::is_same<parameter_gt, double>::value, "parameter_gt must be double in hosted tests");
#if defined(GMP_CTL_TEST_DOUBLE)
        static_assert(std::is_same<ctrl_gt, double>::value, "double project must use double ctrl_gt");
#else
        static_assert(std::is_same<ctrl_gt, float>::value, "float project must use float ctrl_gt");
#endif
    }

    TEST_METHOD(ExplicitConversionsPreserveTheirMeaning)
    {
        const parameter_gt parameter = real2param(0.123456789012345);
        const ctrl_gt control = param2ctrl(parameter);
        expect_near(static_cast<double>(ctrl2param(control)), static_cast<double>(parameter), ctrl_tolerance,
                    L"parameter/control round trip exceeded the selected-domain tolerance");
        expect_near(ctrl_value(real2ctrl(0.75)), 0.75, ctrl_tolerance, L"real2ctrl produced the wrong value");
    }
};

TEST_CLASS(MathContractTests)
{
  public:
    TEST_METHOD(ConstantsRemainConsistentAcrossDomains)
    {
        expect_near(ctrl_value(CTL_CTRL_CONST_PI), static_cast<double>(CTL_PARAM_CONST_PI), ctrl_tolerance,
                    L"control and parameter PI constants differ");
        expect_near(ctrl_value(CTL_CTRL_CONST_SQRT_3), std::sqrt(3.0), ctrl_tolerance,
                    L"sqrt(3) constant is inaccurate");
        Assert::IsTrue(static_cast<double>(CTL_PARAM_CONST_EPSILON) > 0.0);
        Assert::IsTrue(ctrl_value(CTL_CTRL_CONST_EPSILON) > 0.0);
    }

    TEST_METHOD(RadianAndPerUnitAnglesAreExplicit)
    {
        expect_near(ctrl_value(ctl_sin_rad(param2ctrl(CTL_PARAM_CONST_PI * CTL_PARAM_CONST_1_OVER_2))), 1.0,
                    ctrl_tolerance * 4.0, L"ctl_sin_rad does not interpret its input as radians");
        expect_near(static_cast<double>(param_sin_pu(real2param(0.25))), 1.0, 1.0e-12,
                    L"param_sin_pu does not interpret one quarter turn correctly");
    }
};

TEST_CLASS(LiteAlgebraTests)
{
  public:
    TEST_METHOD(VectorArithmeticAndDotProductAreCorrect)
    {
        const double left_data[3] = {1.0, 2.0, 3.0};
        const double right_data[3] = {4.0, 5.0, 6.0};
        ctl::math::vector_lite<double, 3> left(left_data);
        ctl::math::vector_lite<double, 3> right(right_data);
        const auto sum = left + right;
        Assert::AreEqual(5.0, sum[0]);
        Assert::AreEqual(9.0, sum[2]);
        Assert::AreEqual(32.0, ctl::math::dot(left, right));
    }

    TEST_METHOD(MatrixProductsAreCorrect)
    {
        const double matrix_data[4] = {1.0, 2.0, 3.0, 4.0};
        const double vector_data[2] = {2.0, 1.0};
        ctl::math::matrix_lite<double, 2, 2> matrix(matrix_data);
        ctl::math::vector_lite<double, 2> vector(vector_data);
        const auto product = matrix * vector;
        Assert::AreEqual(4.0, product[0]);
        Assert::AreEqual(10.0, product[1]);
        const auto unchanged = matrix * ctl::math::matrix_lite<double, 2, 2>::identity();
        Assert::AreEqual(2.0, unchanged(0, 1));
        Assert::AreEqual(3.0, unchanged(1, 0));
    }
};

TEST_CLASS(ContinuousPidTests)
{
  public:
    TEST_METHOD(InitializationCachesParameterDomainCalculations)
    {
        ctl_pid_t pid{};
        ctl_init_pid(&pid, real2param(2.0), real2param(4.0), real2param(0.1), real2param(1000.0));
        expect_near(ctrl_value(pid.kp), 2.0, ctrl_tolerance, L"PID kp is wrong");
        expect_near(ctrl_value(pid.ki), 0.004, ctrl_tolerance, L"PID ki is wrong");
        expect_near(ctrl_value(pid.kd), 100.0, ctrl_tolerance, L"PID kd is wrong");
    }

    TEST_METHOD(StepAppliesOutputAndIntegratorLimits)
    {
        ctl_pid_t pid{};
        ctl_init_pid(&pid, CTL_PARAM_CONST_1, CTL_PARAM_CONST_1, CTL_PARAM_CONST_ZERO, real2param(100.0));
        ctl_set_pid_limit(&pid, CTL_CTRL_CONST_1_OVER_2, -CTL_CTRL_CONST_1_OVER_2);
        expect_near(ctrl_value(ctl_step_pid_par(&pid, CTL_CTRL_CONST_1)), 0.5, ctrl_tolerance,
                    L"PID output limit was not applied");
        for (int iteration = 0; iteration < 200; ++iteration)
            (void)ctl_step_pid_par(&pid, CTL_CTRL_CONST_1);
        Assert::IsTrue(ctrl_value(pid.i_term) <= ctrl_value(pid.integral_max) + ctrl_tolerance);
    }
};
} // namespace gmp_ctl_unit_test

/** @brief Supply the portable CTL test clock. */
time_gt gmp_ctl_portable_get_tick(void)
{
    return 0U;
}
