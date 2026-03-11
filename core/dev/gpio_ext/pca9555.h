#ifndef PCA9555_H
#define PCA9555_H

#include "driverlib.h"

// 芯片 IIC 地址
#define PCA9555_I2C_ADDR 0x20

// 寄存器定义
#define PCA9555_REG_IN_0  0x00 // 输入端口 0
#define PCA9555_REG_IN_1  0x01 // 输入端口 1
#define PCA9555_REG_OUT_0 0x02 // 输出端口 0
#define PCA9555_REG_OUT_1 0x03 // 输出端口 1
#define PCA9555_REG_POL_0 0x04 // 极性反转 0
#define PCA9555_REG_POL_1 0x05 // 极性反转 1
#define PCA9555_REG_CFG_0 0x06 // 方向配置 0 (1=输入, 0=输出)
#define PCA9555_REG_CFG_1 0x07 // 方向配置 1 (1=输入, 0=输出)

// IO0 强制约束掩码
#define IO0_JOYSTICK_MASK 0x1F // Bit 0~4 必须为 1 (输入)
#define IO0_BUZZER_MASK   0x20 // Bit 5 必须为 0 (输出)

#endif // PCA9555_H


