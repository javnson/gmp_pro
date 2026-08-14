/**
 * @file vs_test_support.h
 * @brief Shared helpers for Visual Studio native CTL unit tests.
 */

#ifndef GMP_CTL_UNIT_TEST_VS_TEST_SUPPORT_H
#define GMP_CTL_UNIT_TEST_VS_TEST_SUPPORT_H

#include <CppUnitTest.h>

#include <cmath>

namespace gmp_ctl_unit_test
{
/** @brief Assert that two host values differ by no more than a tolerance. */
inline void expect_near(double actual, double expected, double tolerance, const wchar_t* message)
{
    Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(std::abs(actual - expected) <= tolerance, message);
}
} // namespace gmp_ctl_unit_test

#endif /* GMP_CTL_UNIT_TEST_VS_TEST_SUPPORT_H */
