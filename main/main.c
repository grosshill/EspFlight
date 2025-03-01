#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include "i2c_manager.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "hmc5883l.h"
#include "pid.h"

esp_err_t ret;

i2c_master_bus_config_t i2c_master_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = i2c_internal_port,
    .scl_io_num = I2C_INTERNAL_SCL,
    .sda_io_num = I2C_INTERNAL_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true
};

mpu6050_accel_pack_t mpu6050_accel_pack;
mpu6050_gyro_pack_t mpu6050_gyro_pack;

i2c_master_bus_handle_t i2c_bus_handle;

void mpu6050_task(void* param);
void bmp180_task(void* param);

void app_main(void)
{   
    i2c_device_config_t mpu6050_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_DEVICE_L,
        .scl_speed_hz = I2C_CLK_FREQ,
    };

    i2c_device_config_t bmp180_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP180_DEVICE,
        .scl_speed_hz = I2C_CLK_FREQ,
    };

    i2c_master_dev_handle_t mpu6050_handle;
    i2c_master_dev_handle_t bmp180_handle;

    ret = i2c_new_master_bus(&i2c_master_bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) return;
    ESP_LOGI("EspFlight", "I2C bus created successfully");

    i2c_master_bus_add_device(i2c_bus_handle, &bmp180_cfg, &bmp180_handle);
    i2c_master_bus_add_device(i2c_bus_handle, &mpu6050_cfg, &mpu6050_handle);

    mpu6050_init(mpu6050_handle);

    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // xTaskCreatePinnedToCore(mpu6050_task, "MPU6050", 10240, (void*)mpu6050_handle, 5, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    xTaskCreatePinnedToCore(bmp180_task, "BMP180", 10240, (void*)bmp180_handle, 5, NULL, 0);

    ESP_LOGI("EspFlight", "Initialize I2C");
}


void mpu6050_task(void* param)
{
    i2c_master_dev_handle_t mpu6050_handle = (i2c_master_dev_handle_t) param;
    ESP_LOGI("LOG1" , "handle got!");
    while(1)
    {
        mpu6050_accel_read(mpu6050_handle, &mpu6050_accel_pack);
        mpu6050_gyro_read(mpu6050_handle, &mpu6050_gyro_pack);
        
        if (ret != ESP_OK) return;
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI("MPU6050", "GX: %d GY: %d GZ: %d", mpu6050_accel_pack.rax, mpu6050_accel_pack.ray, mpu6050_accel_pack.raz);
        
        ESP_LOGI("MPU6050", "GX: %d GY: %d GZ: %d", mpu6050_gyro_pack.rgx, mpu6050_gyro_pack.rgy, mpu6050_gyro_pack.rgz);
    }
}


void bmp180_task(void* param)
{
    i2c_master_dev_handle_t bmp180_handle = (i2c_master_dev_handle_t) param;
    bmp_colab_params_t colab = bmp180_init(bmp180_handle);
    ESP_LOGI("LOG2", "handle got!");
    while(1)
    {
        bmp180_read(bmp180_handle, colab);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}