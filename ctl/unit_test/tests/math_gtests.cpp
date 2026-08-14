/**
 * @file math_gtests.cpp
 * @brief Hosted regression tests for CTL floating-point math domains.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <type_traits>

#include <ctl/portable/gmp_ctl_portable.h>
#include <ctl/math_block/matrix_lite/matrix.hpp>
#include <ctl/math_block/vector_lite/vector.hpp>

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
} // namespace

/** @brief Supply the portable CTL test clock. */
time_gt gmp_ctl_portable_get_tick(void)
{
    return 0U;
}

/** @brief Run all Google Test cases in this executable. */
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(NumericDomains, SelectedTypesMatchTheProjectContract)
{
    static_assert(std::is_same<parameter_gt, double>::value, "parameter_gt must retain initialization precision");
#if defined(GMP_CTL_TEST_DOUBLE)
    static_assert(std::is_same<ctrl_gt, double>::value, "double project must use a double ctrl_gt");
#else
    static_assert(std::is_same<ctrl_gt, float>::value, "float project must use a float ctrl_gt");
#endif
    SUCCEED();
}

TEST(NumericDomains, ExplicitConversionsPreserveTheirMeaning)
{
    const parameter_gt parameter = real2param(0.123456789012345);
    const ctrl_gt control = param2ctrl(parameter);
    const parameter_gt round_trip = ctrl2param(control);

    EXPECT_NEAR(static_cast<double>(parameter), 0.123456789012345, 1.0e-14);
    EXPECT_NEAR(static_cast<double>(round_trip), static_cast<double>(parameter), ctrl_tolerance);
    EXPECT_NEAR(ctrl_value(real2ctrl(0.75)), 0.75, ctrl_tolerance);
}

TEST(MathConstants, ControlAndParameterConstantsAreConsistent)
{
    EXPECT_NEAR(ctrl_value(CTL_CTRL_CONST_PI), static_cast<double>(CTL_PARAM_CONST_PI), ctrl_tolerance);
    EXPECT_NEAR(ctrl_value(CTL_CTRL_CONST_SQRT_3), std::sqrt(3.0), ctrl_tolerance);
    EXPECT_GT(static_cast<double>(CTL_PARAM_CONST_EPSILON), 0.0);
    EXPECT_GT(ctrl_value(CTL_CTRL_CONST_EPSILON), 0.0);
}

TEST(AngleDomains, RadianAndPerUnitFunctionsAreExplicit)
{
    EXPECT_NEAR(ctrl_value(ctl_sin_rad(param2ctrl(CTL_PARAM_CONST_PI * CTL_PARAM_CONST_1_OVER_2))), 1.0,
                ctrl_tolerance * 4.0);
    EXPECT_NEAR(ctrl_value(ctl_cos_rad(param2ctrl(CTL_PARAM_CONST_PI))), -1.0, ctrl_tolerance * 4.0);
    EXPECT_NEAR(static_cast<double>(param_sin_pu(real2param(0.25))), 1.0, 1.0e-12);
    EXPECT_NEAR(static_cast<double>(param_cos_pu(real2param(0.5))), -1.0, 1.0e-12);
}

TEST(VectorLite, GenericArithmeticUsesInlineStorage)
{
    const double left_data[3] = {1.0, 2.0, 3.0};
    const double right_data[3] = {4.0, 5.0, 6.0};
    ctl::math::vector_lite<double, 3> left(left_data);
    ctl::math::vector_lite<double, 3> right(right_data);

    const auto sum = left + right;
    EXPECT_DOUBLE_EQ(sum[0], 5.0);
    EXPECT_DOUBLE_EQ(sum[2], 9.0);
    EXPECT_DOUBLE_EQ(ctl::math::dot(left, right), 32.0);
    EXPECT_EQ(sizeof(left), sizeof(double) * 3U);
}

TEST(MatrixLite, MatrixVectorAndMatrixMatrixProductsAreCorrect)
{
    const double matrix_data[4] = {1.0, 2.0, 3.0, 4.0};
    const double vector_data[2] = {2.0, 1.0};
    ctl::math::matrix_lite<double, 2, 2> matrix(matrix_data);
    ctl::math::vector_lite<double, 2> vector(vector_data);

    const auto product = matrix * vector;
    EXPECT_DOUBLE_EQ(product[0], 4.0);
    EXPECT_DOUBLE_EQ(product[1], 10.0);

    const auto identity = ctl::math::matrix_lite<double, 2, 2>::identity();
    const auto unchanged = matrix * identity;
    EXPECT_DOUBLE_EQ(unchanged(0, 1), 2.0);
    EXPECT_DOUBLE_EQ(unchanged(1, 0), 3.0);
}
