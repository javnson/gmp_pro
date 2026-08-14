/**
 * @file lite_algebra_test.cpp
 * @brief Validates lite vector and matrix templates with a custom arithmetic type.
 */

#include <cmath>
#include <cstdio>

#include <ctl/math_block/vector_lite/vector.hpp>
#include <ctl/math_block/matrix_lite/matrix.hpp>

class test_number
{
public:
    test_number(double value = 0.0) : value_(value) {}
    test_number& operator+=(const test_number& rhs) { value_ += rhs.value_; return *this; }
    test_number& operator-=(const test_number& rhs) { value_ -= rhs.value_; return *this; }
    test_number& operator*=(const test_number& rhs) { value_ *= rhs.value_; return *this; }
    test_number& operator/=(const test_number& rhs) { value_ /= rhs.value_; return *this; }
    double value() const { return value_; }
private:
    double value_;
};

static test_number operator+(test_number lhs, const test_number& rhs) { return lhs += rhs; }
static test_number operator-(test_number lhs, const test_number& rhs) { return lhs -= rhs; }
static test_number operator*(test_number lhs, const test_number& rhs) { return lhs *= rhs; }
static test_number operator/(test_number lhs, const test_number& rhs) { return lhs /= rhs; }

static const double test_epsilon = 1.0e-12;
static bool near(double lhs, double rhs) { return std::fabs(lhs - rhs) < test_epsilon; }

int main()
{
    ctl::math::vector_lite<test_number, 2> vector;
    ctl::math::matrix_lite<test_number, 2, 2> matrix;
    ctl::math::matrix_lite<test_number, 2, 2> identity =
        ctl::math::matrix_lite<test_number, 2, 2>::identity();

    vector[0] = test_number(2.0);
    vector[1] = test_number(3.0);
    matrix(0, 0) = test_number(1.0);
    matrix(0, 1) = test_number(2.0);
    matrix(1, 0) = test_number(3.0);
    matrix(1, 1) = test_number(4.0);

    ctl::math::vector_lite<test_number, 2> result = matrix * vector;
    ctl::math::matrix_lite<test_number, 2, 2> unchanged = matrix * identity;

    if (!near(result[0].value(), 8.0) || !near(result[1].value(), 18.0) ||
        !near(unchanged(1, 0).value(), 3.0) || !near(ctl::math::dot(vector, vector).value(), 13.0))
        return 1;

    std::puts("PASS vector_lite/matrix_lite custom arithmetic type");
    return 0;
}
