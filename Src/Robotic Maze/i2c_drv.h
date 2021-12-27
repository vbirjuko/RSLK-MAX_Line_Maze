/*
 * i2c_drv.h
 *
 *  Created on: 24 мар. 2019 г.
 *      Author: wl
 */

#ifndef I2C_DRV_H_
#define I2C_DRV_H_

#include <stdint.h>

//Результат выполнения операции с i2c
typedef enum {
    I2C_SUCCESS = 0,
    I2C_TIMEOUT,
    I2C_ERROR,
    I2C_NAK_ADDR,
    I2C_BUSY
} t_i2c_status;

void i2c_master_init(void);
t_i2c_status i2c_wr      (uint8_t address,                    unsigned char * data, unsigned int length);
t_i2c_status i2c_wr_reg  (uint8_t address, uint8_t  reg_addr, unsigned char * data, unsigned int length);
t_i2c_status i2c_wr_reg16(uint8_t address, uint16_t reg_addr, unsigned char * data, unsigned int length);
t_i2c_status i2c_rd      (uint8_t address,                    unsigned char * data, unsigned int length);
t_i2c_status i2c_rd_reg  (uint8_t address, uint8_t  reg_addr, unsigned char * data, unsigned int length);
t_i2c_status i2c_rd_reg16(uint8_t address, uint16_t reg_addr, unsigned char * data, unsigned int length);
/*
t_i2c_status i2c_display_init(void);
void i2c_update_display(void);
*/

#endif /* I2C_DRV_H_ */
