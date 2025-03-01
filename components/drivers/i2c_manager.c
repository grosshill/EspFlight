#include "i2c_manager.h"
#include "esp_err.h"
#include "esp_log.h"
#include "string.h"

#define I2C_TIMEOUT pdMS_TO_TICKS(100)


esp_err_t i2c_write(i2c_master_dev_handle_t dev_handle, const uint8_t reg_addr, const uint8_t* data_word, const uint8_t data_length)
{   
    
    uint8_t write_buffer[data_length + 1];
    write_buffer[0] = reg_addr;
    memcpy(&write_buffer[1], data_word, data_length);

    esp_err_t ret = i2c_master_transmit(dev_handle, write_buffer, sizeof(uint8_t) * (data_length + 1), I2C_TIMEOUT);
    if (ret != ESP_OK) 
    {
        ESP_LOGI("I2C", "I2C write failed with error: %d", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t i2c_read(i2c_master_dev_handle_t dev_handle, const uint8_t reg_addr, uint8_t *data, const uint8_t data_length)
{   
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, sizeof(reg_addr), data, data_length * sizeof(uint8_t), I2C_TIMEOUT);

    if (ret != ESP_OK)
    {
        ESP_LOGI("I2C", "I2C read failed with error: %d", ret);
        return ret;
    }
    
    return ESP_OK;
}