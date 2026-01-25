#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "debug_utils.h"
#include "esp_err.h"

#define I2C_INTERNAL_SCL GPIO_NUM_12
#define I2C_INTERNAL_SDA GPIO_NUM_14
#define I2C_EXTERNAL_SCL GPIO_NUM_2
#define I2C_EXTERNAL_SDA GPIO_NUM_3
#define I2C_ONBOARD_SCL GPIO_NUM_5
#define I2C_ONBOARD_SDA GPIO_NUM_4

/*
  According to the doc of ESP-IDF, the default value of this time_out is 100ms.
  However, for some devices, for example the bmi270 gyro, we need to burst write a
  config file with length 8192 bytes, which takes about 185ms for a I2C bus with
  clock frequency 400kHz, this will cause seriout problem. So we recommend to set
  this time_out value up to at least 400ms, this won't influence performance.
*/

#define I2C_TIMEOUT pdMS_TO_TICKS(400)
#define I2C_CLK_FREQ (500000)
#define I2C_TAG "I2C"
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

esp_err_t i2c_write(i2c_master_dev_handle_t dev_handle, const uint8_t reg_addr, const uint8_t* data_word, const uint16_t data_length);

esp_err_t i2c_read(i2c_master_dev_handle_t dev_handle, const uint8_t reg_addr, uint8_t* data, const uint16_t data_length);

#endif