/**
 * @file math_double_tests.cpp
 * @brief Visual Studio contracts for the double control-math backend.
 */

#include "vs_test_support.h"

#include <type_traits>

#include <ctl/portable/gmp_ctl_portable.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_ctl_unit_test
{
namespace
{
constexpr double double_tolerance = 1.0e-12;
} // namespace

TEST_CLASS(DoubleMathTests)
{
  public:
    /** @brief Verify the selected double numeric types. */
    TEST_METHOD(SelectsDoubleControlAndParameterDomains)
    {
        static_assert(std::is_same<ctrl_gt, double>::value, "double test must select double ctrl_gt");
        static_assert(std::is_same<parameter_gt, double>::value, "parameter_gt must retain double precision");
    }

    /** @brief Verify double-domain conversion precision. */
    TEST_METHOD(ConversionsRetainDoublePrecision)
    {
        const parameter_gt parameter = real2param(0.123456789012345);
        const ctrl_gt control = param2ctrl(parameter);
        expect_near(static_cast<double>(ctrl2param(control)), 0.123456789012345, double_tolerance,
                    L"double parameter/control conversion lost precision");
    }

    /** @brief Verify representative double nonlinear operations. */
    TEST_METHOD(NonlinearOperationsAreCorrect)
    {
        expect_near(ctl_sqrt(real2ctrl(2.25)), 1.5, double_tolerance, L"double square root failed");
        expect_near(ctl_pow(real2ctrl(2.0), real2ctrl(3.0)), 8.0, double_tolerance, L"double power failed");
        expect_near(ctl_ln(ctl_exp(real2ctrl(0.5))), 0.5, double_tolerance, L"double exp/log round trip failed");
        expect_near(ctl_atan2(real2ctrl(1.0), real2ctrl(1.0)), static_cast<double>(CTL_PARAM_CONST_PI) / 4.0,
                    double_tolerance, L"double atan2 failed");
    }

    /** @brief Verify explicit radian and per-unit angle domains. */
    TEST_METHOD(RadianAndPerUnitAnglesRemainDistinct)
    {
        expect_near(ctl_cos(CTL_CTRL_CONST_1_OVER_2), -1.0, double_tolerance, L"double per-unit cosine failed");
        expect_near(ctl_cos_rad(param2ctrl(CTL_PARAM_CONST_PI)), -1.0, double_tolerance,
                    L"double radian cosine failed");
        expect_near(static_cast<double>(param_sin_pu(real2param(0.25))), 1.0, double_tolerance,
                    L"parameter per-unit sine failed");
    }
};
} // namespace gmp_ctl_unit_test
