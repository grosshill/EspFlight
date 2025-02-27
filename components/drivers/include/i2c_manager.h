#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#define I2C_INTERNAL_SCL GPIO_NUM_0
#define I2C_INTERNAL_SDA GPIO_NUM_1
#define I2C_EXTERNAL_SCL GPIO_NUM_2
#define I2C_EXTERNAL_SDA GPIO_NUM_3

#define I2C_TIMEOUT pdMS_TO_TICKS(100)  // 100ms timeout

enum i2c_port_id {
    i2c_internal_port = 0,
    i2c_external_port
};

typedef struct {
    i2c_master_bus_config_t bus_cfg;  /* ESP-IDF i2c config */
    i2c_device_config_t dev_cfg;
    enum i2c_port_id port_id;             /* i2c port num */
} i2c_general_config_t;

typedef struct {
    enum i2c_port_id port_id;
    uint8_t* write_seq;
    uint32_t write_len;
    uint8_t* read_seq;
    uint32_t read_len;    
} i2c_io_handle_t;

esp_err_t i2c_init(i2c_general_config_t* gen_cfg, i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* dev_handle);

esp_err_t i2c_read(i2c_io_handle_t* io_handle, i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* dev_handle);

esp_err_t i2c_write(i2c_io_handle_t* io_handle, i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* dev_handle);

#endif