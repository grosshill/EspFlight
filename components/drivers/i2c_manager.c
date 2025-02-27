#include "i2c_manager.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_TIMEOUT pdMS_TO_TICKS(100)


esp_err_t i2c_init(i2c_general_config_t* gen_cfg, i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* dev_handle)
{
    if (gen_cfg == NULL || bus_handle == NULL || dev_handle == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = i2c_new_master_bus(&gen_cfg->bus_cfg, bus_handle);
    if (ret != ESP_OK) {
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


esp_err_t i2c_read(i2c_io_handle_t* io_handle, i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* dev_handle)
{   
    if(io_handle == NULL || io_handle->write_len <= 0 || io_handle->read_len <= 0) return ESP_FAIL;

    uint8_t* read_data = malloc(io_handle->read_len * sizeof(uint8_t));
    if (read_data == NULL) return ESP_ERR_NO_MEM;

    esp_err_t ret = i2c_master_transmit_receive(*dev_handle, io_handle->write_seq, io_handle->write_len * sizeof(uint8_t), read_data, io_handle->read_len, I2C_TIMEOUT);
    if(ret != ESP_OK) {
        free(read_data);
        ESP_LOGE("I2C", "Failed to read I2C data: %d", ret);
        return ret;
    }

    io_handle->read_seq = read_data;
    return ESP_OK;
}


esp_err_t i2c_write(i2c_io_handle_t* io_handle, i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* dev_handle)
{
    if(io_handle == NULL || io_handle->write_len <= 0 || io_handle->read_len > 0) return ESP_FAIL;

    return i2c_master_transmit(*dev_handle, io_handle->write_seq, io_handle->write_len * sizeof(uint8_t), I2C_TIMEOUT);
}