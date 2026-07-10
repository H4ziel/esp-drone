#include "i2c_dr.h"

esp_err_t i2c_master_init(){
    i2c_config_t i2c_config = {
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .mode       = I2C_MODE_MASTER,
        .master.clk_speed = 4e5,
    };

    i2c_param_config(I2C_NUM, &i2c_config);
    return (i2c_driver_install(I2C_NUM,  i2c_config.mode,  0,  0, 0));
}

void setup_i2c(){
    ESP_ERROR_CHECK(i2c_master_init());
}

esp_err_t mpuReadfromReg(byte Reg, byte* ReadBuffer, size_t len, byte addr){
    return (i2c_master_write_read_device(I2C_NUM, addr, &Reg, 1, ReadBuffer,
																    len, 2000));
}

esp_err_t mpuWriteReg(byte Reg, byte data, byte addr){
    uint8_t writeBuf[2];
    writeBuf[0] = Reg;
    writeBuf[1] = data;
    return (i2c_master_write_to_device(I2C_NUM, addr, writeBuf, 2, 1000));
}
