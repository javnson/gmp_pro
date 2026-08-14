/**
 * @file math_iqmath_tests.cpp
 * @brief Visual Studio contracts for the emulated TI IQ24 control-math backend.
 */

#include "vs_test_support.h"

#include <type_traits>

#include <ctl/portable/gmp_ctl_portable.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_ctl_unit_test
{
namespace
{
constexpr double iq24_tolerance = 2.0e-5;

double iq_value(ctrl_gt value)
{
    return static_cast<double>(ctrl2param(value));
}
} // namespace

TEST_CLASS(IqmathTests)
{
  public:
    /** @brief Verify IQ24 storage and parameter-domain selection. */
    TEST_METHOD(SelectsTheIq24ControlDomain)
    {
        static_assert(std::is_integral<ctrl_gt>::value, "IQmath ctrl_gt must use integral storage");
        static_assert(GLOBAL_IQ == 24, "the hosted fixed-point contract uses IQ24");
        static_assert(std::is_same<parameter_gt, double>::value, "IQmath tests retain double initialization precision");
    }

    /** @brief Verify IQ24 conversion quantization. */
    TEST_METHOD(ConversionsQuantizeWithinOneIq24Scale)
    {
        const parameter_gt parameter = real2param(0.123456789012345);
        const ctrl_gt control = param2ctrl(parameter);
        expect_near(iq_value(control), static_cast<double>(parameter), iq24_tolerance,
                    L"IQ24 conversion exceeded its quantization tolerance");
        expect_near(iq_value(real2ctrl(-0.75)), -0.75, iq24_tolerance, L"negative IQ24 conversion failed");
    }

    /** @brief Verify basic IQ24 arithmetic and square root. */
    TEST_METHOD(MultiplyDivideAndSquareRootAreCorrect)
    {
        expect_near(iq_value(ctl_mul(real2ctrl(1.5), real2ctrl(0.5))), 0.75, iq24_tolerance,
                    L"IQ24 multiplication failed");
        expect_near(iq_value(ctl_div(real2ctrl(0.75), real2ctrl(0.5))), 1.5, iq24_tolerance,
                    L"IQ24 division failed");
        expect_near(iq_value(ctl_sqrt(real2ctrl(2.25))), 1.5, iq24_tolerance,
                    L"IQ24 square root failed");
    }

    /** @brief Verify IQ24 per-unit and radian trigonometry. */
    TEST_METHOD(PerUnitAndRadianTrigonometryAreCorrect)
    {
        expect_near(iq_value(ctl_sin(CTL_CTRL_CONST_1_OVER_4)), 1.0, iq24_tolerance,
                    L"IQ24 per-unit sine failed");
        expect_near(iq_value(ctl_cos(CTL_CTRL_CONST_1_OVER_2)), -1.0, iq24_tolerance,
                    L"IQ24 per-unit cosine failed");
        expect_near(iq_value(ctl_sin_rad(param2ctrl(CTL_PARAM_CONST_PI * CTL_PARAM_CONST_1_OVER_2))), 1.0,
                    iq24_tolerance, L"IQ24 radian sine failed");
    }

    /** @brief Verify IQ24 nonlinear functions. */
    TEST_METHOD(Atan2ExpLogAndPowerAreCorrect)
    {
        expect_near(iq_value(ctl_atan2(real2ctrl(1.0), real2ctrl(1.0))),
                    static_cast<double>(CTL_PARAM_CONST_PI) / 4.0, iq24_tolerance,
                    L"IQ24 atan2 failed");
        expect_near(iq_value(ctl_ln(ctl_exp(real2ctrl(0.5)))), 0.5, iq24_tolerance * 4.0,
                    L"IQ24 exp/log round trip failed");
        expect_near(iq_value(ctl_pow(real2ctrl(2.0), real2ctrl(3.0))), 8.0, iq24_tolerance * 8.0,
                    L"IQ24 power failed");
    }
};
} // namespace gmp_ctl_unit_test
