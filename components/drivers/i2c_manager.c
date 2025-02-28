#include "i2c_manager.h"
#include "esp_err.h"
#include "esp_log.h"
#include "string.h"

#define I2C_TIMEOUT pdMS_TO_TICKS(100)


esp_err_t i2c_init(i2c_general_config_t* gen_cfg, i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* dev_handle)
{
    if (gen_cfg == NULL || bus_handle == NULL || dev_handle == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = i2c_new_master_bus(&gen_cfg->bus_cfg, bus_handle);
    if (ret != ESP_OK) 
    {
        ESP_LOGE("I2C", "Failed to initialize I2C bus: %d", ret);
        return ret;
    }

    ret = i2c_master_bus_add_device(*bus_handle, &gen_cfg->dev_cfg, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to add I2C device: %d", ret);
        i2c_del_master_bus(*bus_handle);
        return ret;
    }

    return ESP_OK;
}


esp_err_t i2c_add_device(i2c_device_config_t* dev_cfg, i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* dev_handle)
{
    if (dev_cfg == NULL || bus_handle == NULL || dev_handle == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = i2c_master_bus_add_device(*bus_handle, dev_cfg, dev_handle);
    if (ret != ESP_OK) 
    {
        ESP_LOGE("I2C", "Failed to add I2C device: %d", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t i2c_write(i2c_master_dev_handle_t* dev_handle, uint8_t reg_addr, const uint8_t* data_word, uint8_t data_length)
{   
    uint8_t write_buffer[data_length + 1];
    write_buffer[0] = reg_addr;
    memcpy(&write_buffer[1], data_word, data_length);

    esp_err_t ret = i2c_master_transmit(*dev_handle, write_buffer, sizeof(uint8_t) * (data_length + 1), I2C_TIMEOUT);
    if (ret != ESP_OK) 
    {
        ESP_LOGI("I2C", "I2C write failed with error: %d", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t i2c_read(i2c_master_dev_handle_t* dev_handle, uint8_t reg_addr, uint8_t *data, uint8_t data_length)
{   
    esp_err_t ret = i2c_master_transmit_receive(*dev_handle, &reg_addr, sizeof(reg_addr), data, data_length * sizeof(uint8_t), I2C_TIMEOUT);

    if (ret != ESP_OK)
    {
        ESP_LOGI("I2C", "I2C read failed with error: %d", ret);
        return ret;
    }
    
    return ESP_OK;
}