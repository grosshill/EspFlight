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

    ESP_LOGI("MPU6050", "initialized.");
    
    return ESP_OK;
}


esp_err_t mpu6050_set_freq(i2c_master_dev_handle_t* dev_handle, uint16_t freq) 
{
    if (freq > 8000) return ESP_ERR_INVALID_ARG;

    uint8_t data = (8000 / freq) - 1;
    
    esp_err_t ret = i2c_write(dev_handle, MPU6050_SMPLRT_DIV, &data, 1);
    if (ret != ESP_OK) return ret;

    ESP_LOGI("MPU6050", "frequency set to %d Hz.", freq);
    
    return ESP_OK;
}


esp_err_t mpu6050_device_setup(i2c_master_dev_handle_t* dev_handle, enum mpu6050_device_setup setup)
{
    uint8_t data = (uint8_t)(setup & 0x07);
    
    esp_err_t ret = i2c_write(dev_handle, MPU6050_CONFIG, &data, 1);
    if (ret != ESP_OK) return ret;

    ESP_LOGI("MPU6050", "device setup complete.");
    
    return ESP_OK;
}


esp_err_t mpu6050_gyro_range_setup(i2c_master_dev_handle_t *dev_handle, enum mpu6050_gyro_setup setup)
{
    uint8_t data = (uint8_t)((setup & 0x03) << 3);
    
    esp_err_t ret = i2c_write(dev_handle, MPU6050_ACCEL_CONFIG, &data, 1);
    if (ret != ESP_OK) return ret;    

    ESP_LOGI("MPU6050", "gyro setup complete.");
    
    return ESP_OK;
}


esp_err_t mpu6050_gyro_read(i2c_master_dev_handle_t *dev_handle, mpu6050_gyro_pack *gyro_pack)
{
    uint8_t data_pack[6];
    
    esp_err_t ret = i2c_read(dev_handle, MPU6050_ACCEL_XOUT_H, data_pack, 6);
    if (ret != ESP_OK) return ret;

    gyro_pack->rgx = (data_pack[0] << 8) | data_pack[1];
    gyro_pack->rgy = (data_pack[2] << 8) | data_pack[3];
    gyro_pack->rgz = (data_pack[4] << 8) | data_pack[5];

    return ESP_OK;
}


esp_err_t mpu6050_accel_range_setup(i2c_master_dev_handle_t *dev_handle, enum mpu6050_accel_setup setup)
{
    uint8_t data;
    
    esp_err_t ret = i2c_read(dev_handle, MPU6050_ACCEL_CONFIG, &data, 1);
    if (ret != ESP_OK) return ret;

    data &= 0x07;
    data |= (uint8_t)((setup & 0x03) << 3);
    
    ret = i2c_write(dev_handle, MPU6050_ACCEL_CONFIG, &data, 1);
    if (ret != ESP_OK) return ret;

    ESP_LOGI("MPU6050", "accel range setup complete.");
    
    return ESP_OK;
}


esp_err_t mpu6050_accel_hpf_setup(i2c_master_dev_handle_t *dev_handle, enum mpu6050_accel_hpf_setup setup)
{
    uint8_t data;
    esp_err_t ret = i2c_read(dev_handle, MPU6050_ACCEL_CONFIG, &data, 1);
    if (ret != ESP_OK) return ret;

    data &= 0x18;
    data |= (uint8_t)(setup & 0x07);
    
    ret = i2c_write(dev_handle, MPU6050_ACCEL_CONFIG, &data, 1);
    if (ret != ESP_OK) return ret;

    ESP_LOGI("MPU6050", "accel hpf setup complete.");

    return ESP_OK;
}