/**
 * @file math_float_tests.cpp
 * @brief Visual Studio contracts for the float control-math backend.
 */

#include "vs_test_support.h"

#include <type_traits>

#include <ctl/portable/gmp_ctl_portable.h>
#include <ctl/math_block/matrix_lite/matrix.hpp>
#include <ctl/math_block/vector_lite/vector.hpp>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_ctl_unit_test
{
namespace
{
constexpr double float_tolerance = 2.0e-5;

double float_ctrl_value(ctrl_gt value)
{
    return static_cast<double>(ctrl2param(value));
}
} // namespace

TEST_CLASS(FloatMathTests)
{
  public:
    /** @brief Verify the selected hosted numeric types. */
    TEST_METHOD(SelectsFloatControlAndDoubleParameterDomains)
    {
        static_assert(std::is_same<ctrl_gt, float>::value, "float test must select float ctrl_gt");
        static_assert(std::is_same<parameter_gt, double>::value, "parameter_gt must retain double precision");
    }

    /** @brief Verify explicit conversions and shared constants. */
    TEST_METHOD(ConversionsAndConstantsPreserveTheirContracts)
    {
        const parameter_gt parameter = real2param(0.123456789012345);
        expect_near(static_cast<double>(parameter), 0.123456789012345, 1.0e-14, L"real2param lost precision");
        expect_near(float_ctrl_value(param2ctrl(parameter)), static_cast<double>(parameter), float_tolerance,
                    L"param2ctrl produced an unexpected float value");
        expect_near(float_ctrl_value(CTL_CTRL_CONST_PI), static_cast<double>(CTL_PARAM_CONST_PI), float_tolerance,
                    L"PI constants disagree across numeric domains");
    }

    /** @brief Verify representative linear and nonlinear operations. */
    TEST_METHOD(LinearAndNonlinearOperationsAreCorrect)
    {
        expect_near(float_ctrl_value(ctl_mul(real2ctrl(1.5), real2ctrl(2.0))), 3.0, float_tolerance,
                    L"float multiplication failed");
        expect_near(float_ctrl_value(ctl_div(real2ctrl(3.0), real2ctrl(2.0))), 1.5, float_tolerance,
                    L"float division failed");
        expect_near(float_ctrl_value(ctl_sqrt(real2ctrl(2.25))), 1.5, float_tolerance,
                    L"float square root failed");
        expect_near(float_ctrl_value(ctl_pow(real2ctrl(2.0), real2ctrl(3.0))), 8.0, float_tolerance,
                    L"float power failed");
        expect_near(float_ctrl_value(ctl_ln(ctl_exp(real2ctrl(0.5)))), 0.5, float_tolerance,
                    L"float exp/log round trip failed");
    }

    /** @brief Verify explicit radian and per-unit angle domains. */
    TEST_METHOD(RadianAndPerUnitAnglesRemainDistinct)
    {
        expect_near(float_ctrl_value(ctl_sin(CTL_CTRL_CONST_1_OVER_4)), 1.0, float_tolerance,
                    L"float per-unit sine failed");
        expect_near(float_ctrl_value(ctl_sin_rad(param2ctrl(CTL_PARAM_CONST_PI * CTL_PARAM_CONST_1_OVER_2))), 1.0,
                    float_tolerance, L"float radian sine failed");
        expect_near(static_cast<double>(param_cos_pu(real2param(0.5))), -1.0, 1.0e-12,
                    L"parameter per-unit cosine failed");
    }

    /** @brief Verify generic vector-lite and matrix-lite arithmetic. */
    TEST_METHOD(LiteVectorAndMatrixArithmeticIsGeneric)
    {
        const float left_data[3] = {1.0F, 2.0F, 3.0F};
        const float right_data[3] = {4.0F, 5.0F, 6.0F};
        ctl::math::vector_lite<float, 3> left(left_data);
        ctl::math::vector_lite<float, 3> right(right_data);
        Assert::AreEqual(32.0F, ctl::math::dot(left, right));

        const float matrix_data[4] = {1.0F, 2.0F, 3.0F, 4.0F};
        const float vector_data[2] = {2.0F, 1.0F};
        ctl::math::matrix_lite<float, 2, 2> matrix(matrix_data);
        ctl::math::vector_lite<float, 2> vector(vector_data);
        const auto product = matrix * vector;
        Assert::AreEqual(4.0F, product[0]);
        Assert::AreEqual(10.0F, product[1]);
    }
};
} // namespace gmp_ctl_unit_test
