#ifndef I2C_DR_H
#define I2C_DR_H

#include "esp_err.h"
#include "driver/i2c.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define I2C_NUM I2C_NUM_0

typedef unsigned char byte;

esp_err_t mpuReadfromReg(byte Reg, byte* ReadBuffer, size_t len, byte addr);
esp_err_t mpuWriteReg(byte Reg, byte data, byte addr);
esp_err_t i2c_master_init(void);
void setup_i2c(void);

#endif
