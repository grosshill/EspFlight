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
#include "bmp280.h"
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

hmc5883l_pack_t hmc5883l_pack;

i2c_master_bus_handle_t i2c_bus_handle;

void hmc5883l_task(void* param);

void app_main(void)
{   
    i2c_device_config_t mpu6050_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_DEVICE_L,
        .scl_speed_hz = I2C_CLK_FREQ,
    };

    i2c_device_config_t bmp280_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP280_DEVICE,
        .scl_speed_hz = I2C_CLK_FREQ,
    };

    i2c_device_config_t hmc5883l_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = HMC5883L_DEVICE,
        .scl_speed_hz = I2C_CLK_FREQ,
    };

    i2c_master_dev_handle_t mpu6050_handle;
    i2c_master_dev_handle_t bmp280_handle;
    i2c_master_dev_handle_t hmc5883l_handle;

    ret = i2c_bus_init(&i2c_master_bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) return;
    ESP_LOGI("EspFlight", "I2C bus created successfully");

    // ret = i2c_add_device(&mpu6050_cfg, i2c_bus_handle, &mpu6050_handle);
    // if (ret != ESP_OK) return;
    // ESP_LOGI("EspFlight", "I2C bus added mpu6050");

    ret = i2c_add_device(&hmc5883l_cfg, i2c_bus_handle, &hmc5883l_handle);
    if (ret != ESP_OK) return;
    ESP_LOGI("EspFlight", "I2C bus added hmm5883l");

    // ret = i2c_add_device(&bmp280_cfg, i2c_bus_handle, &bmp280_handle);
    // if (ret != ESP_OK) return;
    // ESP_LOGI("EspFlight", "I2C bus added bmp280");

    vTaskDelay(pdMS_TO_TICKS(1000));

    xTaskCreatePinnedToCore(hmc5883l_task, "HMC5883L", 4096, (void*)hmc5883l_handle, 5, NULL, 0);

    ESP_LOGI("EspFlight", "Initialize I2C");
}


void hmc5883l_task(void* param)
{
    i2c_master_dev_handle_t hmc5883l_handle = (i2c_master_dev_handle_t) param;
    ret = hmc5883l_init(hmc5883l_handle);
    if (ret != ESP_OK) return;
    while(1)
    {
        ret = hmc5883l_read_raw(hmc5883l_handle, &hmc5883l_pack);
        if (ret != ESP_OK) return;
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI("HMC5883L", "GX: %5f GY: %5f GZ: %5f", hmc5883l_pack.gx, hmc5883l_pack.gy, hmc5883l_pack.gz);
    }
}