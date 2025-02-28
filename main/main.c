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

i2c_master_bus_config_t i2c_master_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = i2c_internal_port,
    .scl_io_num = I2C_INTERNAL_SCL,
    .sda_io_num = I2C_INTERNAL_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true
};

i2c_general_config_t i2c_general_config;
i2c_master_bus_handle_t i2c_bus_handle;

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

    i2c_general_config.bus_cfg = i2c_master_bus_config;
    i2c_general_config.dev_cfg = mpu6050_cfg;

    i2c_init(&i2c_general_config, &i2c_bus_handle, &mpu6050_handle);
    i2c_add_device(&bmp280_cfg, &i2c_bus_handle, &bmp280_handle);
    i2c_add_device(&hmc5883l_cfg, &i2c_bus_handle, &hmc5883l_handle);

    ESP_LOGI("EspFlight", "Initialize I2C");
}