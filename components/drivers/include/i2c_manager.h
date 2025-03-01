#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#define I2C_INTERNAL_SCL GPIO_NUM_12
#define I2C_INTERNAL_SDA GPIO_NUM_14
#define I2C_EXTERNAL_SCL GPIO_NUM_2
#define I2C_EXTERNAL_SDA GPIO_NUM_3

#define I2C_TIMEOUT pdMS_TO_TICKS(100)  // 100ms timeout
#define I2C_CLK_FREQ 400000

enum i2c_port_id {
    i2c_internal_port = 0,
    i2c_external_port
};
/*
  According to the documentation of ESP-IDF, i2c transmit and read
  functions are equiped with bulit in mutex, so there is no need
  for users to add SemaphoreMutex manually.

typedef struct {
    i2c_master_dev_handle_t dev_handle;
    SemaphoreHandle_t mutex;
} i2c_device_handle_t;
*/

esp_err_t i2c_write(i2c_master_dev_handle_t dev_handle, const uint8_t reg_addr, const uint8_t* data_word, const uint8_t data_length);

esp_err_t i2c_read(i2c_master_dev_handle_t dev_handle, const uint8_t reg_addr, uint8_t* data, const uint8_t data_length);

#endif