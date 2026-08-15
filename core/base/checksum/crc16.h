#ifndef _FILE_CRC16_H_
#define _FILE_CRC16_H_

#include <gmp_type.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief  External function to calculate the CRC16-CCITT of a given data array.
 * @param  data          Pointer to the data array.
 * @param  len           Number of elements to calculate.
 * @return uint16_t      The calculated CRC16 value.
 */
extern uint16_t gmp_base_calculate_crc16(const byte_gt* data, size_gt len);

#ifdef __cplusplus
}
#endif

#endif // _FILE_CRC16_H_

