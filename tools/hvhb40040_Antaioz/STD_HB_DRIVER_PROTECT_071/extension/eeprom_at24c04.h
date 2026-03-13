#ifndef INC_EEPROM_AT24C04_H_
#define INC_EEPROM_AT24C04_H_

#include "main.h" // 包含您的主 STM32 HAL 头文件
#include <stdint.h>

// 声明由 CubeMX 生成的 I2C2 句柄
extern I2C_HandleTypeDef hi2c2;

// --- 配置定义 ---
#define EEPROM_I2C_HANDLE       (&hi2c2)
#define AT24C04_ADDR_0_255      0xA0  // 访问 0x000 - 0x0FF
#define AT24C04_ADDR_256_511    0xA2  // 访问 0x100 - 0x1FF
#define AT24C04_PAGE_SIZE       16    // 16 字节页大小
#define AT24C04_TIMEOUT         100   // HAL 超时时间 (ms)
#define AT24C04_MAX_MEM_ADDR    0x1FF // 511


/**
 * @brief  向 EEPROM 写入一个字节
 * @param  MemAddress: 要写入的内存地址 (0x000 - 0x1FF)
 * @param  pData: 要写入的字节
 * @retval HAL 状态 (HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT)
 */
HAL_StatusTypeDef EEPROM_Write_Byte(uint16_t MemAddress, uint8_t Data);

/**
 * @brief  从 EEPROM 读取一个字节
 * @param  MemAddress: 要读取的内存地址 (0x000 - 0x1FF)
 * @param  pData: (输出) 用于存储读取数据的指针
 * @retval HAL 状态
 */
HAL_StatusTypeDef EEPROM_Read_Byte(uint16_t MemAddress, uint8_t *pData);

/**
 * @brief  向 EEPROM 写入一个缓冲区
 * @note   此函数自动处理跨页写入和跨 256 字节设备地址边界
 * @param  MemAddress: 起始内存地址 (0x000 - 0x1FF)
 * @param  pBuffer: 要写入的数据缓冲区
 * @param  len: 要写入的数据长度
 * @retval HAL 状态
 */
HAL_StatusTypeDef EEPROM_Write_Buffer(uint16_t MemAddress, uint8_t* pBuffer, uint16_t len);

/**
 * @brief  从 EEPROM 读取一个缓冲区
 * @note   此函数自动处理跨 256 字节设备地址边界
 * @param  MemAddress: 起始内存地址 (0x000 - 0x1FF)
 * @param  pBuffer: (输出) 存储读取数据的缓冲区
 * @param  len: 要读取的数据长度
 * @retval HAL 状态
 */
HAL_StatusTypeDef EEPROM_Read_Buffer(uint16_t MemAddress, uint8_t* pBuffer, uint16_t len);

#endif /* INC_EEPROM_AT24C04_H_ */
