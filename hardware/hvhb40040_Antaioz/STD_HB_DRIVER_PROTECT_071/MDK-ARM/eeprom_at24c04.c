#include "eeprom_at24c04.h"
#include <string.h> // for NULL

/**
 * @brief  辅助函数：根据内存地址获取正确的I2C设备地址
 */
static uint16_t EEPROM_Get_I2C_Addr(uint16_t MemAddress)
{
    // 如果地址大于等于 256 (0x100)，使用第二个设备地址
    return (MemAddress >= 0x100) ? AT24C04_ADDR_256_511 : AT24C04_ADDR_0_255;
}

/**
 * @brief  向 EEPROM 写入一个字节
 */
HAL_StatusTypeDef EEPROM_Write_Byte(uint16_t MemAddress, uint8_t Data)
{
    // 调用缓冲区写入函数，长度为 1
    return EEPROM_Write_Buffer(MemAddress, &Data, 1);
}

/**
 * @brief  从 EEPROM 读取一个字节
 */
HAL_StatusTypeDef EEPROM_Read_Byte(uint16_t MemAddress, uint8_t *pData)
{
    // 调用缓冲区读取函数，长度为 1
    return EEPROM_Read_Buffer(MemAddress, pData, 1);
}


/**
 * @brief  向 EEPROM 写入一个缓冲区 (核心函数)
 */
HAL_StatusTypeDef EEPROM_Write_Buffer(uint16_t MemAddress, uint8_t* pBuffer, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pBuffer == NULL || (MemAddress + len > AT24C04_MAX_MEM_ADDR + 1))
    {
        return HAL_ERROR; // 检查无效指针或内存溢出
    }

    uint16_t current_addr = MemAddress;
    uint8_t* current_data = pBuffer;
    uint16_t remaining_len = len;

    while (remaining_len > 0)
    {
        // 1. 获取当前地址对应的I2C设备地址
        uint16_t i2c_addr = EEPROM_Get_I2C_Addr(current_addr);
        
        // 2. I2C内存地址总是8位的 (高位在I2C设备地址中)
        uint8_t mem_addr_8bit = (uint8_t)(current_addr & 0xFF);

        // 3. 计算本次可以写入多少数据
        // 限制 1: 不能超过页边界 (16 字节)
        uint16_t bytes_to_page_end = AT24C04_PAGE_SIZE - (current_addr % AT24C04_PAGE_SIZE);
        
        // 限制 2: 不能超过 256 字节的设备地址边界
        uint16_t bytes_to_boundary;
        if (current_addr < 0x100)
        {
            bytes_to_boundary = 0x100 - current_addr;
        }
        else
        {
            bytes_to_boundary = 0x200 - current_addr;
        }

        // 限制 3: 不能超过剩余的总长度
        uint16_t chunk_size = remaining_len;

        if (chunk_size > bytes_to_page_end)
        {
            chunk_size = bytes_to_page_end;
        }
        if (chunk_size > bytes_to_boundary)
        {
            chunk_size = bytes_to_boundary;
        }
        
        // 4. 执行 I2C 内存写入
        status = HAL_I2C_Mem_Write(EEPROM_I2C_HANDLE, 
                                   i2c_addr, 
                                   mem_addr_8bit, 
                                   I2C_MEMADD_SIZE_8BIT, 
                                   current_data, 
                                   chunk_size, 
                                   AT24C04_TIMEOUT);
                                   
        if (status != HAL_OK)
        {
            return status;
        }

        // 5. 等待写入完成 (关键!)
        // AT24C04 在写入时不会 ACK 它的地址。我们使用这个特性来轮询。
        // 我们必须轮询刚才写入的那个地址。
        status = HAL_I2C_IsDeviceReady(EEPROM_I2C_HANDLE, i2c_addr, 5, 100);
        while(status != HAL_OK)
        {
            // 如果持续失败，返回超时或错误
            status = HAL_I2C_IsDeviceReady(EEPROM_I2C_HANDLE, i2c_addr, 5, 100);
            // 也可以在这里加一个 HAL_Delay(1) 来降低总线占用
            // HAL_Delay(1); 
        }

        // 6. 更新指针和计数器
        current_addr += chunk_size;
        current_data += chunk_size;
        remaining_len -= chunk_size;
    }

    return HAL_OK;
}

/**
 * @brief  从 EEPROM 读取一个缓冲区 (核心函数)
 */
HAL_StatusTypeDef EEPROM_Read_Buffer(uint16_t MemAddress, uint8_t* pBuffer, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pBuffer == NULL || (MemAddress + len > AT24C04_MAX_MEM_ADDR + 1))
    {
        return HAL_ERROR; // 检查无效指针或内存溢出
    }

    uint16_t current_addr = MemAddress;
    uint8_t* current_data = pBuffer;
    uint16_t remaining_len = len;

    while (remaining_len > 0)
    {
        // 1. 获取I2C设备地址
        uint16_t i2c_addr = EEPROM_Get_I2C_Addr(current_addr);
        uint8_t mem_addr_8bit = (uint8_t)(current_addr & 0xFF);

        // 2. 计算本次可以读取多少数据 (只需考虑 256 字节的设备地址边界)
        uint16_t bytes_to_boundary;
        if (current_addr < 0x100)
        {
            bytes_to_boundary = 0x100 - current_addr;
        }
        else
        {
            bytes_to_boundary = 0x200 - current_addr;
        }
        
        uint16_t chunk_size = remaining_len;
        if (chunk_size > bytes_to_boundary)
        {
            chunk_size = bytes_to_boundary;
        }

        // 3. 执行 I2C 内存读取
        status = HAL_I2C_Mem_Read(EEPROM_I2C_HANDLE, 
                                  i2c_addr, 
                                  mem_addr_8bit, 
                                  I2C_MEMADD_SIZE_8BIT, 
                                  current_data, 
                                  chunk_size, 
                                  AT24C04_TIMEOUT);
                                  
        if (status != HAL_OK)
        {
            return status;
        }

        // 4. 更新指针和计数器
        current_addr += chunk_size;
        current_data += chunk_size;
        remaining_len -= chunk_size;
    }

    return HAL_OK;
}
