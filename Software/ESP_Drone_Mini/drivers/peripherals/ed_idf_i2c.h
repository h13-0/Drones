#ifndef __ED_IDF_I2C_H__
#define __ED_IDF_I2C_H__

#include "driver/gpio.h"
#include "driver/i2c.h"

int ed_idf_i2c_init(i2c_port_t i2c_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num);

int ed_idf_i2c_read_reg(i2c_port_t i2c_num, uint8_t address, uint8_t reg, uint8_t len, uint8_t *buf);

int ed_idf_i2c_write_reg(i2c_port_t i2c_num, uint8_t address, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
