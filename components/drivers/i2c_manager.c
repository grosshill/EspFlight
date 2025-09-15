#include "i2c_manager.h"
#include "string.h"



esp_err_t i2c_write(i2c_master_dev_handle_t dev_handle, const uint8_t reg_addr, const uint8_t* data_word, const uint16_t data_length)
{   
    
    uint8_t write_buffer[data_length + 1];
    write_buffer[0] = reg_addr;
    memcpy(&write_buffer[1], data_word, data_length);

    EF_ERR_CHECK(i2c_master_transmit(dev_handle, write_buffer, sizeof(uint8_t) * (data_length + 1), I2C_TIMEOUT), I2C_TAG);

    return ESP_OK;
}

esp_err_t i2c_read(i2c_master_dev_handle_t dev_handle, const uint8_t reg_addr, uint8_t *data, const uint16_t data_length)
{   
    EF_ERR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, sizeof(reg_addr), data, data_length * sizeof(uint8_t), I2C_TIMEOUT), I2C_TAG);

    return ESP_OK;
}