#include "mpu6050.h"
#include "i2c_manager.h"
#include "esp_err.h"
#include "esp_log.h"

esp_err_t mpu6050_init(i2c_master_dev_handle_t* dev_handle, enum mpu6050_clk_setup clk_setup)
{
    uint8_t data;
    esp_err_t ret = i2c_read(dev_handle, MPU6050_PWR_MGMT_1, &data, 1);
    if (ret != ESP_OK) return ret;
    
    data &= 0x08;
    data |= (uint8_t)(clk_setup & 0x07);
    ret = i2c_write(dev_handle, MPU6050_PWR_MGMT_1, &data, 1);
    if (ret != ESP_OK) return ret;

    ESP_LOGI("MPU6050", "MPU6050 initialized.");
    
    return ESP_OK;
}