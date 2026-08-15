/**
 * @file test_base_checksum.cpp
 * @brief Visual Studio native tests for CRC16-CCITT.
 */

#include "vs_test_support.h"

#include <core/base/checksum/crc16.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_core_unit_test
{
TEST_CLASS(BaseChecksumTests)
{
  public:
    TEST_METHOD(EmptyInputKeepsTheInitialValue)
    {
        Assert::AreEqual<uint16_t>(0xFFFFu, gmp_base_calculate_crc16(nullptr, 0));
    }

    TEST_METHOD(StandardCheckVectorMatchesCrc16CcittFalse)
    {
        static const byte_gt input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        Assert::AreEqual<uint16_t>(0x29B1u, gmp_base_calculate_crc16(input, sizeof(input)));
    }

    TEST_METHOD(BinaryInputMatchesAnIndependentReferenceValue)
    {
        static const byte_gt input[] = {0x00, 0x01, 0x02, 0x03, static_cast<byte_gt>(0xFF)};
        Assert::AreEqual<uint16_t>(0x427Bu, gmp_base_calculate_crc16(input, sizeof(input)));
    }
};
} // namespace gmp_core_unit_test

