// Device interface: IIC
// Device Description: Keyboard & LED matrix manager
// Update Date: 2026.03.11

#ifndef _GMP_DEV_HT16K33_H_
#define _GMP_DEV_HT16K33_H_

#define GMP_DEV_HT16K33_DESC

#define GMP_DEV_IIC_DEV_ADDR 0x70

//
// Related functions, these function should be implemented in CSP folder
//

ec_gt gmp_hal_iic_write_cmd(iic_halt* h, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len);

ec_gt gmp_hal_iic_write_mem(iic_halt* h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                           size_gt mem_len);

ec_gt gmp_hal_iic_read_mem(iic_halt* h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                          size_gt mem_len);


ec_gt gmp_hal_iic_write_cmd(iic_halt* h, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len);

ec_gt gmp_hal_iic_write_reg(iic_halt* h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t reg_data,
                           size_gt reg_len);

ec_gt gmp_hal_iic_write_mem(iic_halt* h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                           size_gt mem_len);

uint32_t gmp_hal_iic_read_reg(iic_halt* h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len,
                              size_gt reg_len, ec_gt* error_code_ret);

ec_gt gmp_hal_iic_read_mem(iic_halt* h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                          size_gt mem_len);


#endif // _GMP_DEV_HT16K33_H_
